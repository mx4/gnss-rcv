use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rayon::prelude::*;
use rustfft::num_complex::Complex32;
use serde::Serialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::channel::{AcqFftCache, Channel, ChannelConfig, MAX_CONCURRENT_ACQ};
use crate::code::Signal;
use crate::device::RtlSdrDevice;
use crate::ephemeris::{Ephemeris as RxEphemeris, Measurement};
use crate::network::RtlSdrTcp;
use crate::osnma::OsnmaVerifier;
use crate::recording::IQFileType;
use crate::recording::IQRecording;
use crate::scheduler::Scheduler;
use crate::solver::PositionSolver;
use crate::state::GnssState;
use std::collections::HashSet;

pub trait IQReader {
    fn read_iq_block(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>>;

    /// Total length in seconds, if known (a file). `None` for a live device, which
    /// has no end — the UI then shows no progress bar.
    fn duration_sec(&self) -> Option<f64> {
        None
    }
}

/// In-memory IQ source: serves slices of a pre-loaded sample buffer and reports
/// "end of file" past the end, matching the `IQRecording` contract. Lets tests
/// and synthetic-signal harnesses drive `Receiver`/`Channel` without a file.
pub struct MockIQReader {
    samples: Vec<Complex32>,
}

impl MockIQReader {
    pub fn new(samples: Vec<Complex32>) -> Self {
        Self { samples }
    }
}

impl IQReader for MockIQReader {
    fn read_iq_block(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        let end = off_samples + num_samples;
        if end > self.samples.len() {
            return Err("end of file".into());
        }
        Ok(self.samples[off_samples..end].to_vec())
    }
}

/// All the value configuration `Receiver::new` needs, in one place. Implements
/// `Default` (file-less, 2.046 MHz / zero-IF L1CA over every GPS PRN), so a
/// caller sets only what differs instead of passing a dozen positional args.
pub struct ReceiverConfig {
    pub use_device: bool,
    pub hostname: String,
    pub file: PathBuf,
    pub iq_file_type: IQFileType,
    pub fs: f64,
    pub fi: f64,
    /// The recording's IF spectrum is inverted (high-side-LO mixing, e.g. the
    /// ION IFEN SX3 capture). Flips the carrier->code aiding sign in tracking.
    pub invert_spectrum: bool,
    pub off_msec: usize,
    pub sig: Signal,
    /// All signal families this session runs (each on its own scheduler
    /// grid). Empty = just `sig`. Populated from a comma list in --sig
    /// ("L1CA,E1B") for mixed sessions.
    pub families: Vec<Signal>,
    pub sats: String,
    pub sbas: bool,
    pub qzss: bool,
    pub plots: bool,
    /// Keep full per-channel diagnostic history and publish the UI snapshot.
    /// True for --plots or when the egui UI runs; false headless (the loops then
    /// keep only a tiny history ring — see channel.rs HISTORY_MIN).
    pub diagnostics: bool,
    pub exit_on_fix: bool,
    /// Write an end-of-run JSON summary to this path (`-` = stdout). None = off.
    pub json: Option<PathBuf>,
    /// Experimental Galileo E1-C pilot tracking (`--e1c`): on an E1-C channel,
    /// sync the CS25 secondary code and integrate coherently past the 4 ms
    /// primary period with a 4-quadrant PLL. Off by default — a bring-up gate
    /// for assessing the pilot's tracking-quality benefit vs E1-B. No effect on
    /// non-E1-C signals.
    pub e1c_pilot: bool,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            use_device: false,
            hostname: String::new(),
            file: PathBuf::new(),
            iq_file_type: IQFileType::TypePairFloat32,
            fs: 2_046_000.0,
            fi: 0.0,
            invert_spectrum: false,
            off_msec: 0,
            sig: Signal::L1ca,
            families: Vec::new(),
            sats: String::new(),
            sbas: false,
            qzss: false,
            plots: false,
            diagnostics: false,
            exit_on_fix: false,
            json: None,
            e1c_pilot: false,
        }
    }
}

pub struct Receiver {
    iq_feed: Box<dyn IQReader>,
    /// The session's signal families, in scheduler registration order.
    families: Vec<Signal>,
    /// Each family's shared acquisition carrier replicas (same order).
    fam_carriers: Vec<std::sync::Arc<Vec<Vec<Complex32>>>>,
    fft_planner: rustfft::FftPlanner<f32>,
    code_period_sec: f64, // family-0 code period (the data-time stat unit)
    fs: f64,              // sample rate, for the data-time / progress calc
    off_samples: usize,
    scheduler: Scheduler,
    channels: HashMap<SV, Channel>,
    /// Shared UI/state handle, so the receiver can publish OSNMA verification
    /// status (the per-channel verifier status the SV table renders).
    pub_state: Arc<Mutex<GnssState>>,
    solver: PositionSolver,
    last_fix_sec: f64,
    exit_on_fix: bool,
    exit_req: Arc<AtomicBool>,
    /// Pause flag, shared with the UI (default: a never-paused flag, so the CLI
    /// and tests are unaffected). See [`Receiver::set_pause_flag`].
    paused: Arc<AtomicBool>,
    stats: RunStats,
    json_out: Option<PathBuf>,
    /// Galileo OSNMA: on automatically for an E1B signal (it's where the OSNMA
    /// bits are, and the overhead is ~5%). The verifier is built lazily once the
    /// first decoded GST week reveals the epoch (so the right 2023/2024/2025 trust
    /// anchor is chosen), then fed every channel's decoded I/NAV pages each step.
    /// `osnma_authenticated` is the set of PRNs already reported, logged once each.
    osnma_enabled: bool,
    osnma: Option<OsnmaVerifier>,
    osnma_authenticated: HashSet<u8>,
}

/// Run-level work/perf counters, printed as a summary at end of `run_loop`.
struct RunStats {
    start: std::time::Instant,
    msec_processed: usize, // 1 ms steps fed through process_step
    fix_attempts: usize,   // solver called (>= 4 SVs after xcorr rejection)
    fix_ok: usize,
    fix_fail: usize,
    xcorr_rejections: usize, // duplicate-ephemeris SVs dropped before solving
    /// Time-to-first-fix in *data* time (seconds of signal consumed before the
    /// first successful solve), latched once. The receiver-performance number —
    /// independent of how fast we replay the file. None until the first fix.
    ttff_sec: Option<f64>,
}

impl Default for RunStats {
    fn default() -> Self {
        Self {
            start: std::time::Instant::now(),
            msec_processed: 0,
            fix_attempts: 0,
            fix_ok: 0,
            fix_fail: 0,
            xcorr_rejections: 0,
            ttff_sec: None,
        }
    }
}

/// Machine-readable end-of-run summary — the `--json` twin of the `print_stats`
/// block. Built once in `run_loop`; serialized to a file or stdout and used to
/// drive the human print, so the two never drift.
#[derive(Serialize)]
struct RunSummary {
    /// The last computed position fix, if any was solved during the run.
    fix: Option<JsonFix>,
    funnel: JsonFunnel,
    stats: JsonStats,
    /// One entry per PRN that locked at least once (the per-SV table).
    sats: Vec<JsonSat>,
}

#[derive(Serialize)]
struct JsonFix {
    lat: f64,
    lon: f64,
    alt_m: f64,
    n_sv: usize,
}

#[derive(Serialize)]
struct JsonFunnel {
    searched: usize,
    acquired: usize,
    tracked: usize,
    ephemeris: usize,
    used_in_fix: usize,
}

#[derive(Serialize)]
struct JsonStats {
    data_sec: f64,
    wall_sec: f64,
    real_time_x: f64,
    fix_attempts: usize,
    fix_ok: usize,
    fix_fail: usize,
    xcorr_rejected: usize,
    /// Data-time seconds to the first fix; None if no fix was solved.
    ttff_sec: Option<f64>,
    acq_attempts: u64,
    acq_correlations: u64,
    tracking_periods: u64,
    subframes: u64,
    parity_errors: u64,
}

#[derive(Serialize)]
struct JsonSat {
    sv: String,
    prn: u8,
    locks: u64,
    losses: u64,
    tracked_s: f64,
    max_lock_s: f64,
    /// Time-to-first-lock (s); None if the PRN never locked long enough.
    ttfl_s: Option<f64>,
    /// Sustained C/N0 (slow EMA of the tracked C/N0) — the honest "how strong is
    /// this SV while tracking" figure; decays when an SV fades after acquisition.
    cn0: f64,
    /// Peak C/N0 ever seen while tracking; can far overstate a faded SV.
    peak_cn0: f64,
    /// RMS steady-state carrier-loop phase error (milliradians) — the carrier
    /// tracking-quality figure for the E1-B vs E1-C-pilot A/B (lower = cleaner).
    phase_rms_mrad: f64,
    subframes: u64,
    parity_errors: u64,
    ephemeris: bool,
    used_in_fix: bool,
}

/// Render `summary` as JSON to `path` (`-` = stdout).
fn write_json_summary(summary: &RunSummary, path: &Path) -> std::io::Result<()> {
    let json = serde_json::to_string_pretty(summary).expect("serialize run summary");
    if path == Path::new("-") {
        println!("{json}");
    } else {
        std::fs::write(path, format!("{json}\n"))?;
        log::warn!("wrote JSON summary to {}", path.display());
    }
    Ok(())
}

/// Build the channel list, tagging each PRN's constellation by its number:
/// 193-202 are QZSS, 120-158 are SBAS, the rest GPS. All three share the L1 C/A
/// Gold-code/LNAV machinery, so they acquire and decode through the same path;
/// the tag drives the log/plot label (G/S/Q) and the solver. (SBAS never
/// completes a GPS ephemeris so the solver ignores it; QZSS does and `gnss-rtk`
/// solves it.)
///
/// The base selection comes from `--sats`:
///   - Empty: the default block for `sig` (GPS 1-32, or Galileo 1-36 on E1).
///   - A comma-separated list, each token either a prefixed SV identifier
///     (`G3`, `E11`, `J193`, `S120`) — constellation explicit, works regardless
///     of `--sig`, case-insensitive — or a bare PRN number (`3`, `11`) whose
///     constellation is inferred from `sig` (GPS 1-32, QZSS 193-202, SBAS ≥120).
///
/// `--sbas` then appends the legacy SBAS L1 block (PRN 120-138) and `--qzss` the
/// QZSS block (PRN 193-202) on top of whatever was selected — the explicit
/// `--sats` list included, so `--sats 1 --sbas` searches PRN 1 plus the GEOs.
fn build_sat_list(sats: &str, sig: Signal, sbas: bool, qzss: bool) -> Vec<SV> {
    let mut svs: Vec<SV> = if sats.is_empty() {
        let (base_cons, range): (Constellation, std::ops::RangeInclusive<u8>) = match sig {
            Signal::GalileoE1b | Signal::GalileoE1c => (Constellation::Galileo, 1..=36),
            _ => (Constellation::GPS, 1..=32),
        };
        range.map(|prn| SV::new(base_cons, prn)).collect()
    } else {
        // Constellation to fall back to when a bare PRN is given.
        let fallback_cons = |prn: u8| match sig {
            Signal::GalileoE1b | Signal::GalileoE1c => Constellation::Galileo,
            _ => {
                if (193..=202).contains(&prn) {
                    Constellation::QZSS
                } else if prn >= 120 {
                    Constellation::SBAS
                } else {
                    Constellation::GPS
                }
            }
        };

        sats.split(',')
            .map(|token| {
                let token = token.trim();
                if token
                    .chars()
                    .next()
                    .is_some_and(|c| c.is_ascii_alphabetic())
                {
                    // Prefixed: delegate to the gnss_rs parser ("G3", "E11", …).
                    token
                        .parse::<SV>()
                        .unwrap_or_else(|_| panic!("invalid SV: {token}"))
                } else {
                    let prn = token
                        .parse::<u8>()
                        .unwrap_or_else(|_| panic!("invalid PRN: {token}"));
                    SV::new(fallback_cons(prn), prn)
                }
            })
            .collect()
    };

    // --sbas / --qzss append their blocks on top of whatever was selected.
    if sbas {
        svs.extend((120..=138_u8).map(|prn| SV::new(Constellation::SBAS, prn)));
    }
    if qzss {
        svs.extend((193..=202_u8).map(|prn| SV::new(Constellation::QZSS, prn)));
    }
    // SBAS and QZSS are L1 C/A signals: on a Galileo E1 session there is no
    // spreading code for them (and no multi-signal stepping yet), so building
    // their channels would panic in Channel::new. Drop them with a note — the
    // UI carries a sticky --sbas flag across signal switches.
    if matches!(sig, Signal::GalileoE1b | Signal::GalileoE1c) {
        let before = svs.len();
        svs.retain(|sv| !matches!(sv.constellation, Constellation::SBAS | Constellation::QZSS));
        if svs.len() != before {
            log::warn!(
                "dropping {} SBAS/QZSS satellites: L1 C/A signals, not decodable in an E1 session",
                before - svs.len()
            );
        }
    }
    svs
}

/// Reject cross-correlation false locks.
///
/// A weak channel can lock onto a strong SV's signal via C/A code
/// cross-correlation (peaks ~21 dB below the auto-correlation, at Doppler
/// offsets of k*1 kHz). It then decodes that strong SV's navigation data, so its
/// ephemeris is bit-identical to the strong SV's while its pseudorange is a
/// biased duplicate. Distinct GPS satellites never broadcast identical
/// orbital+clock parameters, so when two channels report the same ephemeris we
/// keep only the highest-C/N0 (true) one and drop the cross-correlation(s).
fn families_has_e1b(families: &[Signal]) -> bool {
    families.iter().any(|s| matches!(s, Signal::GalileoE1b))
}

fn reject_cross_correlations(
    mut snaps: Vec<(Measurement, RxEphemeris)>,
) -> Vec<(Measurement, RxEphemeris)> {
    snaps.sort_by(|a, b| b.0.cn0.total_cmp(&a.0.cn0));
    let mut kept: Vec<(Measurement, RxEphemeris)> = Vec::with_capacity(snaps.len());
    for (m, e) in snaps {
        if let Some((dm, dup)) = kept
            .iter()
            .find(|(_, k)| k.m0 == e.m0 && k.omg0 == e.omg0 && k.f0 == e.f0)
        {
            log::warn!(
                "{}: dropping cross-correlation lock (duplicate ephemeris of {}, cn0 {:.1} < {:.1})",
                e.sv,
                dup.sv,
                m.cn0,
                dm.cn0,
            );
        } else {
            kept.push((m, e));
        }
    }
    kept
}

fn open_iq_feed(
    use_device: bool,
    hostname: &str,
    sig: Signal,
    fs: f64,
    file: &Path,
    iq_file_type: &IQFileType,
    exit_req: Arc<AtomicBool>,
) -> Result<Box<dyn IQReader>, Box<dyn std::error::Error>> {
    if use_device {
        let dev = RtlSdrDevice::new(sig, fs).map_err(|_| "failed to open rtl-sdr device")?;
        Ok(Box::new(dev))
    } else if !hostname.is_empty() {
        let net = RtlSdrTcp::new(hostname, exit_req.clone(), sig, fs)
            .map_err(|_| format!("failed to connect rtl_tcp backend {hostname}"))?;
        log::warn!("Using rtl_tcp backend: {}", hostname);
        Ok(Box::new(net))
    } else {
        Ok(Box::new(IQRecording::new(file, fs, iq_file_type)?))
    }
}

impl Receiver {
    /// Usual entry point: open the file/device/tcp IQ source described by `cfg`.
    pub fn new(
        cfg: &ReceiverConfig,
        exit_req: Arc<AtomicBool>,
        state: Arc<Mutex<GnssState>>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let iq_feed = open_iq_feed(
            cfg.use_device,
            &cfg.hostname,
            cfg.sig,
            cfg.fs,
            &cfg.file,
            &cfg.iq_file_type,
            exit_req.clone(),
        )?;
        Ok(Self::with_feed(iq_feed, cfg, exit_req, state))
    }

    /// Build a receiver around an already-constructed IQ source. Used by tests
    /// (with `MockIQReader`) and any caller feeding samples from a non-file
    /// source; `new` wraps this with the file/device/tcp source.
    pub fn with_feed(
        iq_feed: Box<dyn IQReader>,
        cfg: &ReceiverConfig,
        exit_req: Arc<AtomicBool>,
        state: Arc<Mutex<GnssState>>,
    ) -> Self {
        // Ingest decimation: wideband file captures are filtered down to the
        // lowest rate the signals need before anything else sees them — the
        // entire receiver below simply runs at the lower fs (see decimate.rs).
        // Streaming feeds (device/network) lack the random access the
        // overlap reads need, so they pass through; GNSS_DECIM=off for A/B.
        let mut iq_feed = iq_feed;
        let mut fs = cfg.fs;
        let decim_ok = !cfg.use_device
            && cfg.hostname.is_empty()
            && !std::env::var("GNSS_DECIM").is_ok_and(|v| v == "off");
        if decim_ok {
            let k = crate::decimate::decimation_factor(cfg.fs, cfg.fi);
            if k > 1 {
                log::warn!(
                    "decimating {:.3} -> {:.3} Msps (factor {k}); the signal band \
                     (|fi| + 2.046 MHz BOC lobes) fits with filter margin",
                    cfg.fs / 1e6,
                    cfg.fs / k as f64 / 1e6
                );
                iq_feed = Box::new(crate::decimate::DecimatingReader::new(iq_feed, k));
                fs = cfg.fs / k as f64;
            }
        }

        let families: Vec<Signal> = if cfg.families.is_empty() {
            vec![cfg.sig]
        } else {
            cfg.families.clone()
        };
        // Kept for the off_msec quirk and the data-time stat; family 0 is the
        // session's primary signal.
        let code_period_sec = families[0].code_period_sec();
        let period_sp = (code_period_sec * fs) as usize;
        // Per family: one shared acquisition carrier-replica set (the set
        // depends on the code period, so a mixed session needs one per
        // family — duplicating it per channel was the 50 MHz OOM), and the
        // channels of the constellations that family carries.
        let mut channels = HashMap::<SV, Channel>::new();
        let mut fam_carriers = Vec::new();
        for &fam_sig in &families {
            let fam_sp = (fam_sig.code_period_sec() * fs) as usize;
            let carriers = Channel::build_carriers(fs, cfg.fi, fam_sp);
            fam_carriers.push(carriers.clone());
            // SBAS/QZSS are C/A-family blocks: offer them only to that
            // family's list — handing them to the E1 pass just made its
            // guard drop them with a spurious "not decodable" warning while
            // the GEOs were alive in the session via the C/A family.
            let ca = !fam_sig.is_boc11();
            for sv in build_sat_list(&cfg.sats, fam_sig, cfg.sbas && ca, cfg.qzss && ca) {
                // In a mixed session each family contributes only its own
                // constellations (build_sat_list is per-signal).
                if families.len() > 1 {
                    let is_gal = sv.constellation == Constellation::Galileo;
                    if is_gal != fam_sig.is_boc11() {
                        continue;
                    }
                }
                channels.insert(
                    sv,
                    Channel::new(
                        ChannelConfig {
                            sig: fam_sig,
                            sv,
                            fs,
                            fi: cfg.fi,
                            invert_spectrum: cfg.invert_spectrum,
                            plots: cfg.plots,
                            diagnostics: cfg.diagnostics,
                            e1c_pilot: cfg.e1c_pilot,
                        },
                        state.clone(),
                        carriers.clone(),
                    ),
                );
            }
        }

        Self {
            iq_feed,
            code_period_sec,
            // The post-decimation rate the receiver actually runs at (off_samples
            // counts decimated samples), so off_samples/fs is the true data time.
            fs,
            off_samples: cfg.off_msec * period_sp,
            scheduler: Scheduler::new(fs, &families),
            families,
            fam_carriers,
            fft_planner: rustfft::FftPlanner::new(),
            channels,
            pub_state: state.clone(),
            solver: PositionSolver::new(state),
            last_fix_sec: 0.0,
            exit_on_fix: cfg.exit_on_fix,
            exit_req,
            paused: Arc::new(AtomicBool::new(false)),
            stats: RunStats::default(),
            json_out: cfg.json.clone(),
            osnma_enabled: families_has_e1b(if cfg.families.is_empty() {
                std::slice::from_ref(&cfg.sig)
            } else {
                &cfg.families
            }),
            osnma: None,
            osnma_authenticated: HashSet::new(),
        }
    }

    fn compute_fix(&mut self, ts_sec: f64) {
        // Fix-attempt cadence, in data time. A solver call costs ~ms, so this is
        // a logging/UI rate choice, not a perf one — but with -x it quantizes
        // the time-to-first-fix.
        if ts_sec - self.last_fix_sec < 2.0 {
            return;
        }

        let ephs: Vec<_> = self
            .channels
            .values()
            .filter(|&ch| ch.is_state_tracking())
            .filter(|&ch| ch.is_ephemeris_complete())
            // tx_anchored alone suffices: the anchor pins only once the
            // ephemeris completes (>= 3 clean subframes, ~30 s of continuous
            // tracking), so the tracking loops are long settled by then.
            .filter(|&ch| ch.nav.meas.tx_anchored)
            .map(|ch| (ch.nav.meas, ch.nav.eph))
            .collect();

        let n_raw = ephs.len();
        let ephs = reject_cross_correlations(ephs);
        self.stats.xcorr_rejections += n_raw - ephs.len();

        if ephs.len() < 4 {
            return;
        }

        log::warn!(
            "t={ts_sec:.3} -- {}",
            format!("attempting fix with {} SVs", ephs.len()).red()
        );

        self.stats.fix_attempts += 1;
        if self.solver.compute_position(ts_sec, &ephs) {
            self.stats.fix_ok += 1;
            // Time-to-first-fix: the data-time of the first solve that succeeds.
            self.stats.ttff_sec.get_or_insert(ts_sec);
            for (_, eph) in &ephs {
                if let Some(ch) = self.channels.get_mut(&eph.sv) {
                    ch.stats.used_in_fix = true;
                }
            }
            // The SVs in this fix, for the UI's per-SV "contributing" check
            // (overwritten each fix, so it tracks the current pool, unlike the
            // sticky per-channel stat above).
            if let Ok(mut st) = self.pub_state.lock() {
                st.fix_svs = ephs.iter().map(|(_, eph)| eph.sv).collect();
            }
        } else {
            self.stats.fix_fail += 1;
        }
        self.last_fix_sec = ts_sec;
    }

    /// Advance the stream by one 1 ms base block (the scheduler's grid).
    /// Returns Ok(true) when the session's family stepped — its channels
    /// processed a full code period — which is what `run_loop` counts (so
    /// `--num-msec` keeps its historical per-period meaning).
    fn process_step(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let block = self
            .iq_feed
            .read_iq_block(self.off_samples, self.scheduler.block_sp())?;
        self.off_samples += self.scheduler.block_sp();
        let due = self.scheduler.ingest(block);
        if due.is_empty() {
            return Ok(false);
        }

        let mut latest_ts = 0.0f64;
        for fam in due {
            let (window, ts_sec) = self.scheduler.window(fam);
            // Hand channels their own copy of the window (same cost as the
            // old per-fetch to_vec); the ring itself stays with the
            // scheduler. Only the due family's channels step — matched by
            // their code period.
            let iq_vec = window.to_vec();
            let fam_period = self.families[fam].code_period_sec();

            // Acquisition admission: cap how many of this family's channels
            // search at once (each holds a sum_p grid — see MAX_CONCURRENT_ACQ).
            // Channels never self-promote, so this is the only Idle -> Acquisition
            // path: count in-flight searchers, then admit backoff-ready idle
            // channels, longest-waiting first (fair, no starvation), up to the
            // cap. Runs before the FFT-cache union so admittees are covered now.
            let in_flight = self
                .channels
                .values()
                .filter(|ch| ch.code_period_sec() == fam_period && ch.is_acquiring())
                .count();
            let free = MAX_CONCURRENT_ACQ.saturating_sub(in_flight);
            if free > 0 {
                let mut ready: Vec<(SV, usize)> = self
                    .channels
                    .iter()
                    .filter(|(_, ch)| ch.code_period_sec() == fam_period)
                    .filter_map(|(sv, ch)| ch.acq_wait().map(|waited| (*sv, waited)))
                    .collect();
                // Longest wait first; ties broken by SV so admission — and thus
                // the lock order and the resulting fix — is reproducible, not a
                // function of the channel map's per-process-random iteration order.
                ready.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
                for (sv, _) in ready.into_iter().take(free) {
                    if let Some(ch) = self.channels.get_mut(&sv) {
                        ch.admit_acquisition();
                    }
                }
            }

            // Shared acquisition FFT cache: the per-bin carrier-mixed forward
            // FFT is PRN-independent, so compute each searched bin once per
            // family step and let every searching channel reuse it. The
            // searched range is the union over this family's searchers.
            let range = self
                .channels
                .values()
                .filter(|ch| ch.code_period_sec() == fam_period)
                .filter_map(|ch| ch.acq_search_range())
                .fold(None::<(usize, usize)>, |acc, (lo, hi)| {
                    Some(acc.map_or((lo, hi), |(alo, ahi)| (alo.min(lo), ahi.max(hi))))
                });
            let cache = range.map(|(lo, hi)| {
                let half = iq_vec.len() / 2;
                let slice = &iq_vec[half..];
                let carriers = &self.fam_carriers[fam];
                let fft = self.fft_planner.plan_fft_forward(slice.len());
                let mut bins: Vec<Option<Vec<Complex32>>> = vec![None; carriers.len()];
                bins[lo..hi].par_iter_mut().enumerate().for_each(|(k, b)| {
                    let mut buf: Vec<Complex32> = slice.to_vec();
                    for (s, c) in buf.iter_mut().zip(carriers[lo + k].iter()) {
                        *s *= *c;
                    }
                    fft.process(&mut buf);
                    *b = Some(buf);
                });
                AcqFftCache { bins }
            });

            self.channels
                .par_iter_mut()
                .filter(|(_, ch)| ch.code_period_sec() == fam_period)
                .for_each(|(_id, channel)| {
                    channel.process_samples_cached(&iq_vec, ts_sec, cache.as_ref())
                });
            latest_ts = latest_ts.max(ts_sec);
        }
        self.stats.msec_processed += 1;

        self.compute_fix(latest_ts);
        self.feed_osnma();

        Ok(true)
    }

    /// Feed every channel's freshly decoded I/NAV pages into the OSNMA verifier
    /// (draining their buffers), then log any satellite that has newly reached an
    /// authenticated state. When OSNMA is off, the buffers are just drained so
    /// they don't grow unbounded. Runs after the parallel channel step, in the
    /// same sequential aggregation phase as the position fix.
    fn feed_osnma(&mut self) {
        if !self.osnma_enabled {
            for ch in self.channels.values_mut() {
                ch.nav.osnma_pages.clear();
            }
            return;
        }
        // Build the verifier lazily from the GST week of the first page that
        // actually carries OSNMA bits, so the trust anchor matches the capture's
        // epoch (2023/2024/2025) and a pre-OSNMA E1B stream (e.g. 2013 IOV) never
        // spuriously activates it.
        if self.osnma.is_none() {
            let week = self
                .channels
                .values()
                .flat_map(|ch| ch.nav.osnma_pages.iter())
                .find(|p| p.word.osnma.iter().any(|&b| b != 0))
                .map(|p| p.week);
            let Some(week) = week else {
                // No OSNMA bits seen yet; drain so buffers don't grow unbounded.
                for ch in self.channels.values_mut() {
                    ch.nav.osnma_pages.clear();
                }
                return;
            };
            log::warn!(
                "OSNMA: GST week {week} -> {} trust anchor",
                OsnmaVerifier::anchor_name(week)
            );
            self.osnma = Some(OsnmaVerifier::for_gst_week(week));
        }
        let verifier = self.osnma.as_mut().unwrap();
        for ch in self.channels.values_mut() {
            let prn = ch.sv.prn;
            let galileo = ch.sv.constellation == Constellation::Galileo;
            for page in ch.nav.osnma_pages.drain(..) {
                if galileo {
                    verifier.feed(prn, page.week, page.tow, &page.word);
                }
            }
        }
        // Log satellites that have newly become authenticated (once each).
        let verifier = self.osnma.as_ref().unwrap();
        let newly: Vec<SV> = self
            .channels
            .values()
            .map(|ch| ch.sv)
            .filter(|sv| sv.constellation == Constellation::Galileo)
            .filter(|sv| verifier.is_authenticated(sv.prn))
            .filter(|sv| !self.osnma_authenticated.contains(&sv.prn))
            .collect();
        for sv in newly {
            self.osnma_authenticated.insert(sv.prn);
            if let Some(cs) = self.pub_state.lock().unwrap().channels.get_mut(&sv) {
                cs.osnma_verified = true;
            }
            log::warn!("{}: {}", sv, "OSNMA authenticated".green());
        }
        // Surface DSM-KROOT assembly progress (the gate to full authentication).
        self.pub_state.lock().unwrap().osnma_kroot = verifier.kroot_progress();
    }

    /// Share the pause flag (set by the UI before `run_loop`). While it is `true`,
    /// `run_loop` suspends; the default flag is never set, so headless runs (CLI,
    /// tests) ignore pausing entirely.
    pub fn set_pause_flag(&mut self, paused: Arc<AtomicBool>) {
        self.paused = paused;
    }

    /// Push run progress (fraction of the recording processed) and the real-time
    /// factor to the shared state for the UI's progress bar. No-op for a live
    /// device (no known total).
    fn publish_progress(&self) {
        let Some(total) = self.iq_feed.duration_sec() else {
            return;
        };
        if total <= 0.0 {
            return;
        }
        let data_sec = self.off_samples as f64 / self.fs;
        let wall = self.stats.start.elapsed().as_secs_f64();
        let realtime = if wall > 1e-3 { data_sec / wall } else { 0.0 };
        let mut st = self.pub_state.lock().unwrap();
        st.run_progress = Some((data_sec / total).clamp(0.0, 1.0) as f32);
        st.realtime_x = realtime as f32;
    }

    pub fn run_loop(&mut self, num_msec: usize) {
        let mut n = 0;
        loop {
            // Pause (set by the UI): suspend processing without consuming samples
            // or tearing down state. The receiver clock is sample-count-based, so
            // freezing the feed freezes time — acquisition, tracking, OSNMA and the
            // current fix all resume exactly where they were.
            while self.paused.load(Ordering::SeqCst) && !self.exit_req.load(Ordering::SeqCst) {
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
            let stepped = match self.process_step() {
                Ok(stepped) => stepped,
                Err(_) => break,
            };
            if self.exit_req.load(Ordering::SeqCst) {
                log::info!("exit requested");
                break;
            }
            if self.exit_on_fix && self.solver.has_fix() {
                log::warn!("position fix obtained, exiting");
                break;
            }
            if !stepped {
                continue;
            }
            n += 1;
            if n % 32 == 0 {
                self.publish_progress();
            }
            if num_msec != 0 && n >= num_msec {
                log::info!("{num_msec} msecs of iq-data processed");
                break;
            }
        }
        let summary = self.build_summary();
        // `--json -` emits *only* the JSON object on stdout (pipe-friendly, e.g.
        // `| jq`); a file target keeps the human stats on stdout and writes the
        // JSON alongside.
        let json_stdout = matches!(&self.json_out, Some(p) if p.as_path() == Path::new("-"));
        if !json_stdout {
            print_summary(&summary);
        }
        if let Some(path) = &self.json_out
            && let Err(e) = write_json_summary(&summary, path)
        {
            log::error!("failed to write JSON summary to {}: {e}", path.display());
        }
    }

    /// Compute the end-of-run summary (funnel, work counters, per-SV table, last
    /// fix) once; both the human print and the JSON output render from it, so the
    /// two never drift.
    fn build_summary(&self) -> RunSummary {
        let s = &self.stats;
        let data_sec = s.msec_processed as f64 * self.code_period_sec;
        let wall = s.start.elapsed().as_secs_f64();
        let rtf = if wall > 0.0 { data_sec / wall } else { 0.0 };

        let mut chans: Vec<&Channel> = self.channels.values().collect();
        chans.sort_by_key(|c| c.sv.prn);

        let acquired = chans.iter().filter(|c| c.stats.locks > 0).count();
        // "tracked" = held a *continuous* lock for >1s, to distinguish real SVs
        // from the brief false locks every PRN gets during the search (those
        // accumulate tracked time but never sustain a long streak).
        let tracked = chans.iter().filter(|c| c.max_lock_secs() > 1.0).count();
        let with_eph = chans.iter().filter(|c| c.is_ephemeris_complete()).count();
        let used = chans.iter().filter(|c| c.stats.used_in_fix).count();

        let sum = |f: fn(&Channel) -> u64| -> u64 { chans.iter().map(|c| f(c)).sum() };

        // Per-SV detail, only for PRNs that acquired at least once.
        let sats = chans
            .iter()
            .filter(|c| c.stats.locks > 0)
            .map(|c| {
                let st = &c.stats;
                JsonSat {
                    sv: c.sv.to_string(),
                    prn: c.sv.prn,
                    locks: st.locks,
                    losses: st.lock_losses,
                    tracked_s: c.tracked_secs(),
                    max_lock_s: c.max_lock_secs(),
                    ttfl_s: (st.first_lock_ts > 0.0).then_some(st.first_lock_ts),
                    cn0: st.mean_cn0,
                    peak_cn0: st.peak_cn0,
                    phase_rms_mrad: st.phase_rms_rad() * 1e3,
                    subframes: st.subframes,
                    parity_errors: st.parity_errors,
                    ephemeris: c.is_ephemeris_complete(),
                    used_in_fix: st.used_in_fix,
                }
            })
            .collect();

        let fix = self
            .solver
            .last_fix_geodetic()
            .map(|(lat, lon, alt_m)| JsonFix {
                lat,
                lon,
                alt_m,
                n_sv: used,
            });

        RunSummary {
            fix,
            funnel: JsonFunnel {
                searched: chans.len(),
                acquired,
                tracked,
                ephemeris: with_eph,
                used_in_fix: used,
            },
            stats: JsonStats {
                data_sec,
                wall_sec: wall,
                real_time_x: rtf,
                fix_attempts: s.fix_attempts,
                fix_ok: s.fix_ok,
                fix_fail: s.fix_fail,
                xcorr_rejected: s.xcorr_rejections,
                ttff_sec: s.ttff_sec,
                acq_attempts: sum(|c| c.stats.acq_attempts),
                acq_correlations: sum(|c| c.stats.acq_corrs),
                tracking_periods: sum(|c| c.stats.trk_periods),
                subframes: sum(|c| c.stats.subframes),
                parity_errors: sum(|c| c.stats.parity_errors),
            },
            sats,
        }
    }
}

/// Render the run summary as the human `===== run stats =====` block.
fn print_summary(sum: &RunSummary) {
    let st = &sum.stats;
    let f = &sum.funnel;
    println!("\n===== run stats =====");
    println!(
        "data {:.1}s   wall {:.1}s   real-time {:.1}x",
        st.data_sec, st.wall_sec, st.real_time_x
    );
    println!(
        "funnel: searched {} -> acquired {} -> tracked {} -> ephemeris {} -> used-in-fix {}",
        f.searched, f.acquired, f.tracked, f.ephemeris, f.used_in_fix
    );
    let ttff = st
        .ttff_sec
        .map(|v| format!("{v:.1}s"))
        .unwrap_or_else(|| "-".to_string());
    println!(
        "fixes: {} attempts, {} ok, {} failed   xcorr-rejected {}   ttff {ttff}",
        st.fix_attempts, st.fix_ok, st.fix_fail, st.xcorr_rejected
    );
    println!(
        "work: {} acq-attempts, {} acq-correlations, {} tracking-periods, {} subframes, {} parity-errors",
        st.acq_attempts, st.acq_correlations, st.tracking_periods, st.subframes, st.parity_errors
    );

    // cn0 = sustained (EMA) C/N0, pk = peak ever seen while tracking.
    println!("  SV    locks losses  trk(s) maxlk(s) ttfl(s)  cn0   pk phRMS subfr parity eph fix");
    for s in &sum.sats {
        let ttfl = s
            .ttfl_s
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "-".to_string());
        println!(
            "  {:<5} {:>5} {:>6} {:>7.1} {:>8.1} {:>7} {:>4.1} {:>4.1} {:>5.0} {:>5} {:>6} {:>3} {:>3}",
            s.sv,
            s.locks,
            s.losses,
            s.tracked_s,
            s.max_lock_s,
            ttfl,
            s.cn0,
            s.peak_cn0,
            s.phase_rms_mrad,
            s.subframes,
            s.parity_errors,
            if s.ephemeris { "y" } else { "-" },
            if s.used_in_fix { "y" } else { "-" },
        );
    }
}

#[cfg(test)]
#[path = "receiver_tests.rs"]
mod tests;
