use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use serde::Serialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::channel::Channel;
use crate::code::Signal;
use crate::device::RtlSdrDevice;
use crate::ephemeris::Ephemeris as RxEphemeris;
use crate::network::RtlSdrTcp;
use crate::osnma::OsnmaVerifier;
use crate::recording::IQFileType;
use crate::recording::IQRecording;
use crate::solver::PositionSolver;
use crate::state::GnssState;
use std::collections::HashSet;

pub trait IQReader {
    fn get_iq_data(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>>;
}

/// In-memory IQ source: serves slices of a pre-loaded sample buffer and reports
/// "end of file" past the end, matching the `IQRecording` contract. Lets tests
/// and synthetic-signal harnesses drive `Receiver`/`Channel` without a file.
pub struct MockIQReader {
    samples: Vec<Complex64>,
}

impl MockIQReader {
    pub fn new(samples: Vec<Complex64>) -> Self {
        Self { samples }
    }
}

impl IQReader for MockIQReader {
    fn get_iq_data(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
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
    pub off_msec: usize,
    pub sig: Signal,
    pub sats: String,
    pub sbas: bool,
    pub qzss: bool,
    pub plots: bool,
    pub exit_on_fix: bool,
    /// Write an end-of-run JSON summary to this path (`-` = stdout). None = off.
    pub json: Option<PathBuf>,
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
            off_msec: 0,
            sig: Signal::L1ca,
            sats: String::new(),
            sbas: false,
            qzss: false,
            plots: false,
            exit_on_fix: false,
            json: None,
        }
    }
}

pub struct Receiver {
    iq_feed: Box<dyn IQReader>,
    period_sp: usize, // samples per code period (signal-dependent)
    fs: f64,
    code_period_sec: f64, // one spreading-code period (1 ms L1CA, 4 ms E1)
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
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
    cn0: f64,
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
fn get_sat_list(sats: &str, sig: Signal, sbas: bool, qzss: bool) -> Vec<SV> {
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
fn reject_cross_correlations(mut ephs: Vec<RxEphemeris>) -> Vec<RxEphemeris> {
    ephs.sort_by(|a, b| b.cn0.total_cmp(&a.cn0));
    let mut kept: Vec<RxEphemeris> = Vec::with_capacity(ephs.len());
    for e in ephs {
        if let Some(dup) = kept
            .iter()
            .find(|k| k.m0 == e.m0 && k.omg0 == e.omg0 && k.f0 == e.f0)
        {
            log::warn!(
                "{}: dropping cross-correlation lock (duplicate ephemeris of {}, cn0 {:.1} < {:.1})",
                e.sv,
                dup.sv,
                e.cn0,
                dup.cn0,
            );
        } else {
            kept.push(e);
        }
    }
    kept
}

fn get_iq_feed(
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
        let iq_feed = get_iq_feed(
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
        // The receiver steps one spreading-code period at a time; that period is
        // the signal's, not a hardcoded 1 ms (E1 is 4 ms).
        let code_period_sec = cfg.sig.code_period_sec();
        let period_sp = (code_period_sec * cfg.fs) as usize;
        // Build the acquisition carrier replicas once and share them across every
        // channel (period_sp == code_sp). PRN-independent, so this is the same data
        // for all 36 — duplicating it per channel was the 50 MHz OOM.
        let carriers = Channel::build_carriers(cfg.fs, cfg.fi, period_sp);
        let mut channels = HashMap::<SV, Channel>::new();
        for sv in get_sat_list(&cfg.sats, cfg.sig, cfg.sbas, cfg.qzss) {
            channels.insert(
                sv,
                Channel::new(
                    cfg.sig,
                    sv,
                    cfg.fs,
                    cfg.fi,
                    cfg.plots,
                    state.clone(),
                    carriers.clone(),
                ),
            );
        }

        Self {
            iq_feed,
            period_sp,
            fs: cfg.fs,
            code_period_sec,
            off_samples: cfg.off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels,
            pub_state: state.clone(),
            solver: PositionSolver::new(state),
            last_fix_sec: 0.0,
            exit_on_fix: cfg.exit_on_fix,
            exit_req,
            paused: Arc::new(AtomicBool::new(false)),
            stats: RunStats::default(),
            json_out: cfg.json.clone(),
            osnma_enabled: matches!(cfg.sig, Signal::GalileoE1b),
            osnma: None,
            osnma_authenticated: HashSet::new(),
        }
    }

    fn fetch_samples_msec(&mut self) -> Result<(Vec<Complex64>, f64), Box<dyn std::error::Error>> {
        let num_samples = if self.cached_iq_vec.is_empty() {
            2 * self.period_sp
        } else {
            self.period_sp
        };

        let mut iq_vec = self.iq_feed.get_iq_data(self.off_samples, num_samples)?;

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut iq_vec);
        self.cached_ts_sec_tail += num_samples as f64 / self.fs;

        if self.cached_iq_vec.len() > 2 * self.period_sp {
            let num_samples = self.period_sp;
            let _ = self.cached_iq_vec.drain(0..num_samples);
        }
        let len = self.cached_iq_vec.len();

        // we pass 2 code worth of iq data back
        // the timestamp given corresponds to the beginning of the last code
        // [...code...][...code...]
        //             ^

        Ok((
            self.cached_iq_vec[len - 2 * self.period_sp..].to_vec(),
            self.cached_ts_sec_tail - self.code_period_sec,
        ))
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
            .filter(|&ch| ch.nav.eph.tx_anchored)
            .map(|ch| ch.nav.eph)
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
            for eph in &ephs {
                if let Some(ch) = self.channels.get_mut(&eph.sv) {
                    ch.stats.used_in_fix = true;
                }
            }
        } else {
            self.stats.fix_fail += 1;
        }
        self.last_fix_sec = ts_sec;
    }

    fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let (iq_vec, ts_sec) = self.fetch_samples_msec()?;
        self.stats.msec_processed += 1;

        self.channels
            .par_iter_mut()
            .for_each(|(_id, channel)| channel.process_samples(&iq_vec, ts_sec));

        self.compute_fix(ts_sec);
        self.feed_osnma();

        Ok(())
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
            if self.process_step().is_err() {
                break;
            }
            if self.exit_req.load(Ordering::SeqCst) {
                log::info!("exit requested");
                break;
            }
            if self.exit_on_fix && self.solver.has_fix() {
                log::warn!("position fix obtained, exiting");
                break;
            }
            n += 1;
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
                    cn0: st.peak_cn0,
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
    println!(
        "fixes: {} attempts, {} ok, {} failed   xcorr-rejected {}",
        st.fix_attempts, st.fix_ok, st.fix_fail, st.xcorr_rejected
    );
    println!(
        "work: {} acq-attempts, {} acq-correlations, {} tracking-periods, {} subframes, {} parity-errors",
        st.acq_attempts, st.acq_correlations, st.tracking_periods, st.subframes, st.parity_errors
    );

    println!("  SV    locks losses  trk(s) maxlk(s) ttfl(s)  cn0 subfr parity eph fix");
    for s in &sum.sats {
        let ttfl = s
            .ttfl_s
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "-".to_string());
        println!(
            "  {:<5} {:>5} {:>6} {:>7.1} {:>8.1} {:>7} {:>4.1} {:>5} {:>6} {:>3} {:>3}",
            s.sv,
            s.locks,
            s.losses,
            s.tracked_s,
            s.max_lock_s,
            ttfl,
            s.cn0,
            s.subframes,
            s.parity_errors,
            if s.ephemeris { "y" } else { "-" },
            if s.used_in_fix { "y" } else { "-" },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{
        GeoFeed, SynthE1Sv, SynthSv, pick_geo_constellation, pick_geo_constellation_gal, synth_e1,
        synth_l1ca,
    };

    #[test]
    fn mock_reader_serves_slices_and_eof() {
        let samples: Vec<Complex64> = (0..100).map(|i| Complex64::new(i as f64, 0.0)).collect();
        let mut r = MockIQReader::new(samples);

        let a = r.get_iq_data(0, 10).unwrap();
        assert_eq!(a.len(), 10);
        assert_eq!(a[0], Complex64::new(0.0, 0.0));

        let b = r.get_iq_data(10, 10).unwrap();
        assert_eq!(b[0], Complex64::new(10.0, 0.0));

        assert!(
            r.get_iq_data(95, 10).is_err(),
            "reading past the end is EOF"
        );
    }

    // End-to-end DSP unit test with no recording or network: a synthetic clean
    // signal must drive a channel from Acquisition into Tracking. This is the
    // hermetic regression the file-based integration tests can't be in CI.
    #[test]
    fn synthetic_signal_acquires_and_tracks() {
        let (fs, fi) = (2_046_000.0, 0.0);
        let prn = 5u8;
        // noiseless single SV at code phase 200 chips, zero Doppler.
        let svs = [SynthSv::new(prn, 0.0, 200.0, 0.0)];
        let sig = synth_l1ca(&svs, fs, fi, 60, None); // 60 ms: enough to acquire + track

        let state = Arc::new(Mutex::new(GnssState::new()));
        let cfg = ReceiverConfig {
            sats: prn.to_string(),
            fs,
            fi,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(MockIQReader::new(sig)),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            state,
        );
        rx.run_loop(0); // until the mock feed is exhausted

        let sv = SV::new(Constellation::GPS, prn);
        let ch = &rx.channels[&sv];
        assert!(
            ch.is_state_tracking(),
            "synthetic clean signal for {sv} should reach Tracking"
        );
        assert!(
            ch.get_cn0() > 45.0,
            "a noiseless signal should track at very high C/N0, got {:.1}",
            ch.get_cn0()
        );
    }

    // Point-4 regression at a realistic SNR: three SVs at different Doppler,
    // code phase and strength share one AWGN realization; each must still
    // acquire and hold tracking. Catches DSP regressions the noiseless test can
    // hide, with no recording (runs in CI in well under a second).
    #[test]
    fn synthetic_noisy_multi_sv_acquires_and_tracks() {
        let (fs, fi) = (2_046_000.0, 0.0);
        let svs = [
            SynthSv::new(5, 1200.0, 137.0, 48.0),
            SynthSv::new(12, -3400.0, 512.0, 45.0),
            SynthSv::new(20, 700.0, 900.0, 50.0),
        ];
        let sig = synth_l1ca(&svs, fs, fi, 150, Some(0xC0FFEE)); // 150 ms in AWGN

        let cfg = ReceiverConfig {
            sats: "5,12,20".to_string(),
            fs,
            fi,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(MockIQReader::new(sig)),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        );
        rx.run_loop(0);

        for s in &svs {
            let sv = SV::new(Constellation::GPS, s.prn);
            let ch = &rx.channels[&sv];
            assert!(
                ch.is_state_tracking(),
                "{sv} ({} dB-Hz synth) should reach Tracking in AWGN",
                s.cn0_dbhz
            );
            assert!(
                ch.get_cn0() > 35.0,
                "{sv} C/N0 {:.1} should be above the lock threshold",
                ch.get_cn0()
            );
        }
    }

    // Hermetic Galileo E1-B regression: a synthetic BOC(1,1) signal must drive a
    // channel from Acquisition into Tracking — the E1 analogue of
    // `synthetic_signal_acquires_and_tracks`, exercising the BOC correlator and
    // the signal-aware 4 ms code period with no recording.
    #[test]
    fn synthetic_e1_acquires_and_tracks() {
        let (fs, fi) = (4_092_000.0, 0.0); // 2 samples / BOC sub-chip
        let prn = 1u8;
        let svs = [SynthE1Sv::new(prn, 800.0, 1000.0, 0.0)]; // noiseless, some Doppler
        let sig = synth_e1(&svs, fs, fi, 400, None); // 400 ms: acquire + track E1's 4 ms code

        let cfg = ReceiverConfig {
            sats: prn.to_string(),
            sig: Signal::GalileoE1b,
            fs,
            fi,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(MockIQReader::new(sig)),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        );
        rx.run_loop(0);

        let sv = SV::new(Constellation::Galileo, prn);
        let ch = &rx.channels[&sv];
        assert!(
            ch.is_state_tracking(),
            "synthetic E1 signal for {sv} should reach Tracking"
        );
        assert!(
            ch.get_cn0() > 45.0,
            "a noiseless E1 signal should track at very high C/N0, got {:.1}",
            ch.get_cn0()
        );
    }

    // Regression: a code phase of exactly 0 makes the first tracking step wrap
    // `code_off_sec` below zero while corr_p is still empty. That used to panic
    // (`corr_p.back().unwrap()` in get_code_and_carrier_phase); it must now track.
    #[test]
    fn tracks_at_code_phase_zero_without_panicking() {
        let (fs, fi) = (2_046_000.0, 0.0);
        let prn = 5u8;
        let svs = [SynthSv::new(prn, 0.0, 0.0, 0.0)]; // code phase exactly 0
        let sig = synth_l1ca(&svs, fs, fi, 60, None);
        let cfg = ReceiverConfig {
            sats: prn.to_string(),
            fs,
            fi,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(MockIQReader::new(sig)),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        );
        rx.run_loop(0);

        let sv = SV::new(Constellation::GPS, prn);
        assert!(rx.channels[&sv].is_state_tracking());
    }

    /// Build the standard 6-SV Geneva scene (synth::GeoFeed +
    /// pick_geo_constellation), run the full real pipeline on it until the
    /// first fix, and return the 3D fix error vs truth in metres. Shared by
    /// the clean and noisy hermetic positioning regressions below.
    fn run_geo_scene_fix_error_m(label: &str, cn0_dbhz: f64, seed: Option<u64>) -> f64 {
        let (fs, fi) = (2_046_000.0, 0.0);
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0]; // the gpssim Geneva site
        let (week, toe) = (2348u32, 36_000u32);

        let ephs = pick_geo_constellation(truth, week, toe, 6);
        assert!(ephs.len() >= 5, "picked only {} SVs", ephs.len());
        let sats = ephs
            .iter()
            .map(|e| e.sv.prn.to_string())
            .collect::<Vec<_>>()
            .join(",");

        let feed = GeoFeed::new(&ephs, truth, fs, fi, 30_000, cn0_dbhz, seed);
        let state = Arc::new(Mutex::new(GnssState::new()));
        let cfg = ReceiverConfig {
            sats,
            fs,
            fi,
            exit_on_fix: true,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(feed),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            state.clone(),
        );
        rx.run_loop(0);

        let st = state.lock().unwrap();
        assert!(
            st.longitude != 0.0,
            "no fix from the {label} synthetic scene"
        );
        let (x, y, z) = map_3d::geodetic2ecef(
            st.latitude.to_radians(),
            st.longitude.to_radians(),
            st.height,
            map_3d::Ellipsoid::WGS84,
        );
        let (tlat, tlon, talt) =
            map_3d::ecef2geodetic(truth[0], truth[1], truth[2], map_3d::Ellipsoid::WGS84);
        let horiz = ((st.latitude.to_radians() - tlat) * 6.378e6)
            .hypot((st.longitude.to_radians() - tlon) * 6.378e6 * tlat.cos());
        let err = ((x - truth[0]).powi(2) + (y - truth[1]).powi(2) + (z - truth[2]).powi(2)).sqrt();
        eprintln!(
            "{label} synthetic fix error vs truth: {err:.2} m (horiz {horiz:.2} m, vert {:+.2} m)",
            st.height - talt
        );
        err
    }

    // The hermetic end-to-end positioning regression: a geometry-consistent
    // synthetic scene (per-SV code phase, Doppler and LNAV timing all derived
    // from true ranges — see synth::GeoFeed) must drive the full real pipeline
    // (acquisition → tracking → bit/frame sync → ephemeris decode → tx anchor
    // → gnss-rtk solve) to a position fix within metres of the known truth.
    // No recording, no gps-sdr-sim, no network — the recording-based fix
    // tests' hermetic twin, runnable anywhere including CI. Clean scene:
    // measures the pipeline's systematic error alone (~2.5 m).
    #[test]
    #[ignore] // ~5 s — runs via `just test-all` / `cargo test -- --ignored`
    fn synthetic_geometry_solves_to_truth() {
        let err = run_geo_scene_fix_error_m("clean", 45.0, None);
        assert!(err < 15.0, "fix error {err:.1} m vs truth");
    }

    // The SBAS corrections regression, on the only judge with exact truth:
    // the synthetic scene broadcasts per-SV clock errors the signal itself
    // does not have (GeoFeed::new_diverged — the real signal-in-space error
    // situation SBAS measures), which corrupts the fix; feeding MT1+MT2 with
    // PRC = −c·ε through the production path (GnssState → solver pr += PRC)
    // must recover clean-scene accuracy. Locks the application sign: flipping
    // it doubles the damage instead.
    #[test]
    #[ignore] // ~10 s — runs via `just test-all` / `cargo test -- --ignored`
    fn sbas_fast_corrections_recover_broadcast_clock_errors() {
        use crate::constants::SPEED_OF_LIGHT;
        use crate::sbas_corr::test_msgs;

        let (fs, fi) = (2_046_000.0, 0.0);
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0];
        let ephs = pick_geo_constellation(truth, 2348, 36_000, 6);
        assert!(ephs.len() >= 5);

        // Broadcast clock errors ±~10 m, differential across the SVs (a
        // common part would just fold into the receiver clock).
        let eps_ns: [f64; 6] = [30.0, -42.0, 18.0, -24.0, 36.0, -12.0];
        let mut bcast = ephs.clone();
        for (b, eps) in bcast.iter_mut().zip(eps_ns) {
            b.f0 += eps * 1e-9;
        }

        let run = |with_corrections: bool| -> f64 {
            let feed = GeoFeed::new_diverged(&ephs, &bcast, truth, fs, fi, 30_000, 45.0, None);
            let state = Arc::new(Mutex::new(GnssState::new()));
            if with_corrections {
                // MT1 mask slots are in ascending PRN order; PRC = −c·ε.
                let mut by_prn: Vec<(u16, f64)> = ephs
                    .iter()
                    .zip(eps_ns)
                    .map(|(e, eps)| (e.sv.prn as u16, -eps * 1e-9 * SPEED_OF_LIGHT))
                    .collect();
                by_prn.sort_unstable_by_key(|(prn, _)| *prn);
                let mask: Vec<u16> = by_prn.iter().map(|(p, _)| *p).collect();
                let prcs: Vec<f64> = by_prn.iter().map(|(_, c)| *c).collect();
                let mut st = state.lock().unwrap();
                st.sbas_corr.feed(&test_msgs::mt1(&mask), 0.0);
                // Receipt time mid-scene: fresh (±30 s) for the whole run.
                st.sbas_corr.feed(&test_msgs::mt_fast(2, &prcs), 15.0);
            }
            let sats = ephs
                .iter()
                .map(|e| e.sv.prn.to_string())
                .collect::<Vec<_>>()
                .join(",");
            let cfg = ReceiverConfig {
                sats,
                fs,
                fi,
                exit_on_fix: true,
                ..Default::default()
            };
            let mut rx = Receiver::with_feed(
                Box::new(feed),
                &cfg,
                Arc::new(AtomicBool::new(false)),
                state.clone(),
            );
            rx.run_loop(0);

            let st = state.lock().unwrap();
            assert!(st.longitude != 0.0, "no fix from the diverged scene");
            let (x, y, z) = map_3d::geodetic2ecef(
                st.latitude.to_radians(),
                st.longitude.to_radians(),
                st.height,
                map_3d::Ellipsoid::WGS84,
            );
            ((x - truth[0]).powi(2) + (y - truth[1]).powi(2) + (z - truth[2]).powi(2)).sqrt()
        };

        let uncorrected = run(false);
        let corrected = run(true);
        eprintln!(
            "broadcast clock errors ±10 m: fix error {uncorrected:.2} m uncorrected, \
             {corrected:.2} m with SBAS fast corrections"
        );
        assert!(
            uncorrected > 6.0,
            "injected SIS errors should corrupt the fix (got {uncorrected:.2} m)"
        );
        assert!(
            corrected < 3.5,
            "SBAS corrections should recover clean accuracy (got {corrected:.2} m)"
        );
    }

    // The Galileo twin of the hermetic positioning regression: a
    // geometry-consistent E1-B scene (BOC(1,1) codes, full I/NAV pages with
    // ICD-convention word-5 TOWs) must drive the real pipeline — including the
    // I/NAV decoder's 2.0 s anchor-latency correction — to a Galileo-only fix
    // within metres of the truth. Without the latency correction this scene
    // carries a ~2 s common transmit-time error: the solve still converges (a
    // common offset folds into the clock) but the absolute t_tx is wrong —
    // exactly what broke the GPS+Galileo merge.
    #[test]
    #[ignore] // ~12 s — runs via `just test-all` / `cargo test -- --ignored`
    fn synthetic_e1_geometry_solves_to_truth() {
        let (fs, fi) = (4_092_000.0, 0.0);
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0];
        let ephs = pick_geo_constellation_gal(truth, 1300, 36_000, 6);
        assert!(ephs.len() >= 5, "picked only {} SVs", ephs.len());
        let sats = ephs
            .iter()
            .map(|e| e.sv.prn.to_string())
            .collect::<Vec<_>>()
            .join(",");

        let feed = GeoFeed::new_e1(&ephs, truth, fs, fi, 25_000, 45.0, None);
        let state = Arc::new(Mutex::new(GnssState::new()));
        let cfg = ReceiverConfig {
            sats,
            sig: Signal::GalileoE1b,
            fs,
            fi,
            exit_on_fix: true,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(feed),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            state.clone(),
        );
        rx.run_loop(0);

        let st = state.lock().unwrap();
        assert!(st.longitude != 0.0, "no fix from the synthetic E1 scene");
        let (x, y, z) = map_3d::geodetic2ecef(
            st.latitude.to_radians(),
            st.longitude.to_radians(),
            st.height,
            map_3d::Ellipsoid::WGS84,
        );
        let err = ((x - truth[0]).powi(2) + (y - truth[1]).powi(2) + (z - truth[2]).powi(2)).sqrt();
        eprintln!("synthetic E1 fix error vs truth: {err:.2} m");
        assert!(err < 15.0, "E1 fix error {err:.1} m vs truth");
    }

    // The same scene in AWGN at a realistic open-sky 44 dB-Hz (deterministic
    // seed): adds acquisition/tracking/DLL noise on top of the systematic
    // error, so the gate is looser. Locks the pipeline's noise robustness —
    // a tracking-loop regression that only hurts in noise shows up here, not
    // in the clean twin.
    #[test]
    #[ignore] // ~8 s — runs via `just test-all` / `cargo test -- --ignored`
    fn synthetic_geometry_solves_to_truth_in_noise() {
        let err = run_geo_scene_fix_error_m("noisy", 44.0, Some(0x6E55));
        assert!(err < 30.0, "noisy fix error {err:.1} m vs truth");
    }

    /// Build a geometric scene for `ephs`, run the full receiver over it, and
    /// measure each anchored channel's transmit-time error directly against
    /// the generator's own ground truth: δ = true_t_tx − receiver_t_tx at the
    /// channel's final snapshot. Solver-free — no least-squares absorption can
    /// hide an anchor offset here.
    fn anchor_deltas(
        ephs: &[crate::ephemeris::Ephemeris],
        truth: [f64; 3],
        sig: Signal,
        fs: f64,
        num_msec: usize,
    ) -> Vec<(SV, f64)> {
        use gnss_rtk::prelude::Duration;
        let mk = || match sig {
            Signal::L1ca => GeoFeed::new(ephs, truth, fs, 0.0, num_msec, 45.0, None),
            _ => GeoFeed::new_e1(ephs, truth, fs, 0.0, num_msec, 45.0, None),
        };
        let (feed, twin) = (mk(), mk());
        let sats = ephs
            .iter()
            .map(|e| e.sv.prn.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let cfg = ReceiverConfig {
            sats,
            sig,
            fs,
            fi: 0.0,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(feed),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        );
        rx.run_loop(0);

        let mut out: Vec<(SV, f64)> = rx
            .channels
            .values()
            .filter(|ch| ch.is_state_tracking() && ch.nav.eph.tx_anchored)
            .map(|ch| {
                let e = &ch.nav.eph;
                let rx_tx = e.tx_tow_gpst
                    + Duration::from_seconds((e.trk_phase - e.code_off_sec) - e.tow_trk_phase);
                // The snapshot phase corresponds to the end of the channel's
                // last prompt window, at ts + code_off (sub-ms past ts).
                let t_snap = ch.ts_sec;
                let truth_tx = twin.true_transmit_time(ch.sv, t_snap).unwrap();
                (ch.sv, (truth_tx - rx_tx).to_seconds())
            })
            .collect();
        out.sort_by_key(|(sv, _)| *sv);
        out
    }

    // Direct, solver-free measurement of both nav decoders' transmit-time
    // anchor conventions against synthetic ground truth. This is the
    // instrument that resolves the 1.84 s inter-constellation offset the
    // two-pass LimeSDR experiment measured: each decoder's absolute anchor
    // latency, per SV, with no least-squares absorption in the way.
    #[test]
    #[ignore] // ~15 s — runs via `just test-all` / `cargo test -- --ignored`
    fn tx_anchor_latency_measured_against_synthetic_truth() {
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0];

        // The pipeline's reference convention: every decoder leaves t_tx early
        // by the LNAV decode latency (0.160 s) — deliberately uncorrected (see
        // LNAV_DECODE_LATENCY_SEC). What the GPS+Galileo merge requires is that
        // both decoders sit at the SAME offset; the I/NAV anchor is aligned to
        // LNAV by INAV_DECODE_LATENCY_SEC (= 2.000 − 0.160 s).
        let expect = crate::gps_lnav::LNAV_DECODE_LATENCY_SEC;

        let gps = pick_geo_constellation(truth, 2348, 36_000, 5);
        let dg = anchor_deltas(&gps, truth, Signal::L1ca, 2_046_000.0, 30_000);
        for (sv, d) in &dg {
            eprintln!("LNAV anchor delta {sv}: {:+.4} s", d);
            assert!(
                (d - expect).abs() < 0.01,
                "{sv}: LNAV anchor at {d:+.4} s (convention is {expect:+.3})"
            );
        }

        let gal = pick_geo_constellation_gal(truth, 1300, 36_000, 5);
        let de = anchor_deltas(&gal, truth, Signal::GalileoE1b, 4_092_000.0, 25_000);
        for (sv, d) in &de {
            eprintln!("I/NAV anchor delta {sv}: {:+.4} s", d);
            assert!(
                (d - expect).abs() < 0.01,
                "{sv}: I/NAV anchor at {d:+.4} s (convention is {expect:+.3})"
            );
        }

        // The merge-critical invariant: identical conventions across signals.
        let mean = |v: &[(SV, f64)]| v.iter().map(|(_, d)| d).sum::<f64>() / v.len() as f64;
        let cross = mean(&dg) - mean(&de);
        eprintln!("LNAV-INAV convention difference: {cross:+.4} s");
        assert!(
            cross.abs() < 0.005,
            "anchor conventions diverge across constellations: {cross:+.4} s"
        );
    }

    // The epoch-sensitivity question, settled: sweep a common shift on every
    // tx anchor of a clean synthetic scene and solve each point with BOTH
    // solvers — gnss-rtk and the frame-free wls_fix. Findings this test locks:
    //  1. gnss-rtk is exonerated: it matches the independent solver to the
    //     decimetre at every shift, so there is no coordinate/frame
    //     integration issue (the EARTH_J2000 label is inert for SPP — ranges
    //     are computed from our raw coordinates; the only ANISE transform
    //     feeds el/az into bias models we disable).
    //  2. The sensitivity itself is plain orbital physics: ~700 m of fix
    //     error per second of common anchor-epoch offset (per-SV range-rate
    //     spread), identical in both solvers.
    //  3. The error minimum sits exactly at the anchor convention (shift 0) —
    //     the generator+decoder+solver system is self-consistent there. The
    //     tx-anchor truth instrument labels that point "true − 0.16 s"; that
    //     ±0.16 s is a bookkeeping question confined to the *instrument's*
    //     reference (trk_phase/code_off pairing semantics), not a production
    //     error: all fixes operate at the consistent point, and the
    //     cross-constellation alignment measures 0.0000 s on real SIS.
    #[test]
    #[ignore] // ~7 s — runs via `just test-all` / `cargo test -- --ignored`
    fn epoch_sensitivity_probe() {
        use gnss_rtk::prelude::Duration;
        let (fs, fi) = (2_046_000.0, 0.0);
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0];
        let cons = pick_geo_constellation(truth, 2348, 36_000, 6);
        let sats = cons
            .iter()
            .map(|e| e.sv.prn.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let feed = GeoFeed::new(&cons, truth, fs, fi, 30_000, 45.0, None);
        let cfg = ReceiverConfig {
            sats,
            fs,
            fi,
            ..Default::default()
        };
        let mut rx = Receiver::with_feed(
            Box::new(feed),
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        );
        rx.run_loop(0);
        let ephs: Vec<RxEphemeris> = rx
            .channels
            .values()
            .filter(|ch| ch.is_state_tracking() && ch.is_ephemeris_complete())
            .filter(|ch| ch.nav.eph.tx_anchored)
            .map(|ch| ch.nav.eph)
            .collect();
        assert!(ephs.len() >= 5);

        let err3 = |p: [f64; 3]| {
            ((p[0] - truth[0]).powi(2) + (p[1] - truth[1]).powi(2) + (p[2] - truth[2]).powi(2))
                .sqrt()
        };
        // Anchors sit at the LNAV convention; the tx-anchor instrument labels
        // that point "true − 0.160 s" (see finding 3 above).
        let mut results: Vec<(f64, f64, Option<f64>)> = Vec::new();
        for shift in [-0.16, 0.0, 0.08, 0.16, 0.24, 0.32] {
            let mut e2 = ephs.clone();
            for e in &mut e2 {
                e.tx_tow_gpst += Duration::from_seconds(shift);
            }
            let (wx, _, _, _) = wls_fix(&e2, truth);
            let mut solver = PositionSolver::new(Arc::new(Mutex::new(GnssState::new())));
            let rtk_err = if solver.compute_position(30.0, &e2) {
                let (la, lo, al) = solver.last_fix_geodetic().unwrap();
                let (x, y, z) = map_3d::geodetic2ecef(
                    la.to_radians(),
                    lo.to_radians(),
                    al,
                    map_3d::Ellipsoid::WGS84,
                );
                Some(err3([x, y, z]))
            } else {
                None
            };
            let rtk = match rtk_err {
                Some(e) => format!("{e:8.1} m"),
                None => "  no fix".to_string(),
            };
            eprintln!(
                "anchor shift {shift:+.2} s:  wls {:8.1} m   gnss-rtk {rtk}",
                err3(wx)
            );
            results.push((shift, err3(wx), rtk_err));
        }

        // Lock finding 1: the two solvers agree at every epoch.
        for (shift, wls_err, rtk_err) in &results {
            let rtk_err = rtk_err.expect("gnss-rtk must solve every shifted set");
            assert!(
                (wls_err - rtk_err).abs() < 1.0,
                "solvers diverge at shift {shift:+.2}: wls {wls_err:.1} vs rtk {rtk_err:.1} m"
            );
        }
        // Lock finding 3: the consistency point is the anchor convention.
        let min = results.iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
        assert_eq!(min.0, 0.0, "error minimum moved off the anchor convention");
        assert!(min.1 < 15.0, "consistency-point error {:.1} m", min.1);
    }

    /// Run one decode pass over the ION LimeSDR capture with the given signal
    /// and PRN list for `steps` code periods, and return the fix-ready
    /// ephemeris snapshots (tracking + complete + anchored — `compute_fix`'s
    /// filter). Both signals' passes end at the same file time, so their
    /// transmit-time snapshots share one receiver timeline and can be solved
    /// jointly.
    fn limesdr_pass(sig: Signal, sats: &str, steps: usize) -> Vec<RxEphemeris> {
        let cfg = ReceiverConfig {
            file: PathBuf::from("resources/ION_LimeSDR_Bands-L1.2xi16"),
            iq_file_type: IQFileType::TypePairInt16,
            fs: 10_000_000.0,
            fi: 420_000.0,
            sig,
            sats: sats.to_string(),
            ..Default::default()
        };
        let mut rx = Receiver::new(
            &cfg,
            Arc::new(AtomicBool::new(false)),
            Arc::new(Mutex::new(GnssState::new())),
        )
        .expect("open LimeSDR capture");
        rx.run_loop(steps);
        rx.channels
            .values()
            .filter(|ch| ch.is_state_tracking())
            .filter(|ch| ch.is_ephemeris_complete())
            .filter(|ch| ch.nav.eph.tx_anchored)
            .map(|ch| ch.nav.eph)
            .collect()
    }

    /// Solve a fix from `ephs` with a fresh solver; return (lat°, lon°, ECEF).
    fn solve(ephs: &[RxEphemeris]) -> (f64, f64, [f64; 3]) {
        let mut solver = PositionSolver::new(Arc::new(Mutex::new(GnssState::new())));
        assert!(
            solver.compute_position(60.0, ephs),
            "no fix from {} SVs",
            ephs.len()
        );
        let (lat, lon, alt) = solver.last_fix_geodetic().unwrap();
        let (x, y, z) = map_3d::geodetic2ecef(
            lat.to_radians(),
            lon.to_radians(),
            alt,
            map_3d::Ellipsoid::WGS84,
        );
        (lat, lon, [x, y, z])
    }

    /// Horizontal error (m) vs the ION antenna site (52.177, 4.488 — published
    /// to ~100 m precision, so gates against it are necessarily loose).
    fn ion_site_error_m(lat: f64, lon: f64) -> f64 {
        let (tlat, tlon) = (52.177f64, 4.488f64);
        ((lat - tlat) * 111_000.0).hypot((lon - tlon) * 111_000.0 * tlat.to_radians().cos())
    }

    /// Weighted Gauss-Newton fix over (x, y, z, c·dt, c·ISB_gal) — an
    /// independent cross-check of gnss-rtk, adding the two things it lacks for
    /// a mixed pool: per-measurement weights (its measurement sigma is
    /// literally `1.0 // TODO` in 0.8) and an inter-system bias state. Weights
    /// are self-calibrated: solve once equal-weighted, set each
    /// constellation's sigma to its residual RMS, solve again — no hand-tuned
    /// constants. Returns (fix_ecef, c·dt m, c·ISB m, per-constellation σ m).
    fn wls_fix(
        ephs: &[RxEphemeris],
        x0: [f64; 3],
    ) -> ([f64; 3], f64, f64, HashMap<Constellation, f64>) {
        use crate::constants::{EARTH_ROTATION_RATE, SPEED_OF_LIGHT};
        use gnss_rtk::prelude::Duration;

        // Fixed per-SV quantities: the measurement (pr + c·clk) and the SV
        // position rotated into the reception frame (functions of t_tx/now
        // only, independent of the position iterate).
        let tx: Vec<_> = ephs
            .iter()
            .map(|e| {
                let elapsed = (e.trk_phase - e.code_off_sec) - e.tow_trk_phase;
                e.tx_tow_gpst + Duration::from_seconds(elapsed)
            })
            .collect();
        let now = *tx.iter().max().unwrap() + Duration::from_seconds(0.070);
        let (mut meas, mut svp, mut gal) = (Vec::new(), Vec::new(), Vec::new());
        for (e, t_tx) in ephs.iter().zip(&tx) {
            let pr_sec = (now - *t_tx).to_seconds();
            let clk = crate::solver::get_sv_clock_correction(e, now) * SPEED_OF_LIGHT;
            let s = crate::solver::compute_sv_position_ecef(e, *t_tx);
            let w = EARTH_ROTATION_RATE * pr_sec;
            let (cw, sw) = (w.cos(), w.sin());
            meas.push(pr_sec * SPEED_OF_LIGHT + clk);
            svp.push([cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1, s.2]);
            gal.push(e.sv.constellation == Constellation::Galileo);
        }

        // The math itself is the production solver (promoted to solver.rs as
        // the live path); this helper only prepares raw uncorrected
        // measurements from the snapshots.
        let sol = crate::solver::wls_solve(&meas, &svp, &gal, x0).expect("wls_solve");
        (sol.pos, sol.cdt_m, sol.isb_m, sol.sigma)
    }

    /// Per-SV pseudorange residual (m) of `ephs`' snapshots against a receiver
    /// position: pr + c·clk − geometric range, the same t_tx / Sagnac math as
    /// compute_position's RESID diagnostic. Within one constellation the
    /// residual is ~constant (the common rx clock bias); spread = geometry
    /// error, constellation-mean difference = apparent inter-system bias.
    fn residuals(ephs: &[RxEphemeris], rx: [f64; 3]) -> Vec<(SV, f64)> {
        use crate::constants::{EARTH_ROTATION_RATE, SPEED_OF_LIGHT};
        use gnss_rtk::prelude::Duration;
        let tx: Vec<_> = ephs
            .iter()
            .map(|e| {
                let elapsed = (e.trk_phase - e.code_off_sec) - e.tow_trk_phase;
                e.tx_tow_gpst + Duration::from_seconds(elapsed)
            })
            .collect();
        let now = *tx.iter().max().unwrap() + Duration::from_seconds(0.070);
        ephs.iter()
            .zip(&tx)
            .map(|(e, t_tx)| {
                let pr_sec = (now - *t_tx).to_seconds();
                let pr = pr_sec * SPEED_OF_LIGHT;
                let clk = crate::solver::get_sv_clock_correction(e, now) * SPEED_OF_LIGHT;
                let s = crate::solver::compute_sv_position_ecef(e, *t_tx);
                let w = EARTH_ROTATION_RATE * pr_sec;
                let (cw, sw) = (w.cos(), w.sin());
                let (sx, sy) = (cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1);
                let geom =
                    ((sx - rx[0]).powi(2) + (sy - rx[1]).powi(2) + (s.2 - rx[2]).powi(2)).sqrt();
                (e.sv, pr + clk - geom)
            })
            .collect()
    }

    // The combined GPS + Galileo fix, without the multi-signal receiver: two
    // single-signal passes over the same capture (one file = one sample clock,
    // so the passes' tx anchors share a receiver timeline), merged into a
    // single gnss-rtk solve. De-risks the solver-side constellation mixing
    // before the receiver-side multi-signal stepping refactor.
    //
    // Findings this test locks in (ION LimeSDR, 2017):
    //  1. The LNAV 10-bit week (+2048 pin) and the 12-bit GST week disagree by
    //     exactly 1024 weeks on pre-2019 captures — fatal for a merge, invisible
    //     within one constellation. (The "GPS week rollover" roadmap item.)
    //  2. The two decoders' TOW->phase anchors disagreed by exactly 1.8400 s
    //     (2.0 s I/NAV page minus 8 LNAV bits — the two structural decode
    //     latencies, since measured directly against synthetic truth by
    //     tx_anchor_latency_measured_against_synthetic_truth and corrected in
    //     nav_anchor_tx). This test now asserts the corrected offset is in the
    //     GGTO class.
    //  3. With the weeks rebased, gnss-rtk solves the mixed 15-SV pool with no
    //     special configuration; the leftover inter-system bias is tens of
    //     metres (GGTO + inter-signal-delay class).
    #[test]
    #[ignore] // needs resources/ION_LimeSDR_Bands-L1.2xi16 (fetch.py ion-lime); ~1 min
    fn combined_gps_galileo_fix_from_two_passes() {
        if !Path::new("resources/ION_LimeSDR_Bands-L1.2xi16").exists() {
            eprintln!("skipping combined_gps_galileo_fix_from_two_passes: recording absent");
            return;
        }
        // Surface the solver's warn-level diagnostics (fix failures, RESID).
        let _ = env_logger::builder()
            .filter_level(log::LevelFilter::Warn)
            .try_init();

        // 60 s of file time in both passes (60000 1 ms L1CA steps, 15000 4 ms
        // E1B steps), so the snapshots refer to the same receive instant. The
        // GPS PRN list is the set the in-run fix accepts on this capture
        // (G19/G25 also complete an ephemeris but carry inconsistent
        // pseudoranges the in-run solver never used).
        let mut gps = limesdr_pass(Signal::L1ca, "2,5,6,7,9,13,16,23,29,30", 60_000);
        let gal = limesdr_pass(Signal::GalileoE1b, "1,4,9,11,19", 15_000);
        assert!(gps.len() >= 4, "GPS pass yielded only {} SVs", gps.len());
        assert!(
            gal.len() >= 4,
            "Galileo pass yielded only {} SVs",
            gal.len()
        );

        // The capture is from 2017 (GPS week 1971) — outside the [2048, 3071]
        // window the 10-bit LNAV week is pinned to (the "GPS week rollover"
        // roadmap item): the GPS pass decodes week 2995 while the 12-bit GST
        // week gives the true epoch, leaving the two passes exactly 1024 weeks
        // apart. A common week error cancels within one constellation (why
        // both single fixes work) but breaks the merge; rebase the GPS epochs
        // onto the Galileo timeline (GST week w == continuous GPS week
        // w + 1024).
        use gnss_rtk::prelude::Duration;
        let dw = gps[0].week as i64 - (gal[0].week as i64 + 1024);
        if dw != 0 {
            eprintln!("rebasing GPS week by {dw} weeks (LNAV 10-bit week rollover)");
            let shift = Duration::from_seconds(dw as f64 * 604_800.0);
            for e in &mut gps {
                e.week = (e.week as i64 - dw) as u32;
                e.tow_gpst -= shift;
                e.toe_gpst -= shift;
                e.toc_gpst -= shift;
                e.tx_tow_gpst -= shift;
            }
        }

        let (glat, glon, _) = solve(&gps);
        let (elat, elon, _) = solve(&gal);
        let mut merged = gps.clone();
        merged.extend(gal.iter().copied());
        let merged = reject_cross_correlations(merged);
        let n = merged.len();

        // Solver-independent consistency check first: per-SV residuals vs the
        // (coarse) site truth. Within one constellation the residual must be
        // ~constant (the rx clock bias); the constellation-mean difference is
        // the apparent inter-system timing offset.
        let site = {
            let (x, y, z) = map_3d::geodetic2ecef(
                52.177f64.to_radians(),
                4.488f64.to_radians(),
                30.0,
                map_3d::Ellipsoid::WGS84,
            );
            [x, y, z]
        };
        let rs = residuals(&merged, site);
        for (sv, r) in &rs {
            eprintln!("  resid {sv}: {:.3} km", r / 1000.0);
        }
        let cmean = |c: Constellation| {
            let v: Vec<f64> = rs
                .iter()
                .filter(|(sv, _)| sv.constellation == c)
                .map(|(_, r)| *r)
                .collect();
            v.iter().sum::<f64>() / v.len() as f64
        };
        let delta_sec = (cmean(Constellation::Galileo) - cmean(Constellation::GPS))
            / crate::constants::SPEED_OF_LIGHT;
        eprintln!("inter-constellation timing offset: {delta_sec:+.4} s");

        let (clat, clon, cfix) = solve(&merged);

        eprintln!(
            "GPS-only  ({:2} SV): {glat:.6},{glon:.6}  err {:.0} m",
            gps.len(),
            ion_site_error_m(glat, glon)
        );
        eprintln!(
            "GAL-only  ({:2} SV): {elat:.6},{elon:.6}  err {:.0} m",
            gal.len(),
            ion_site_error_m(elat, elon)
        );
        eprintln!(
            "combined  ({n:2} SV): {clat:.6},{clon:.6}  err {:.0} m",
            ion_site_error_m(clat, clon)
        );

        // Per-constellation residual stats vs the combined fix. The mean
        // difference is the leftover GPS-Galileo system bias the mixed solve
        // absorbed; the per-constellation SPREAD is the smoking gun for the
        // anchor correction being an *epoch* correction: if +Δ had only fixed
        // the pseudoranges while the orbit epochs stayed 1.84 s early, the
        // Galileo model errors would spread ±1.5 km here (range-rate × Δ).
        let mut stats = HashMap::<Constellation, Vec<f64>>::new();
        for (sv, r) in residuals(&merged, cfix) {
            stats.entry(sv.constellation).or_default().push(r);
        }
        let mean = |c: Constellation| stats[&c].iter().sum::<f64>() / stats[&c].len() as f64;
        let spread = |c: Constellation| {
            let v = &stats[&c];
            v.iter().cloned().fold(f64::MIN, f64::max) - v.iter().cloned().fold(f64::MAX, f64::min)
        };
        let bias = mean(Constellation::GPS) - mean(Constellation::Galileo);
        eprintln!(
            "residual spread vs combined fix: GPS {:.0} m, GAL {:.0} m",
            spread(Constellation::GPS),
            spread(Constellation::Galileo)
        );
        eprintln!(
            "apparent GPS-GAL system bias after correction: {bias:+.1} m ({:+.1} ns)",
            bias / crate::constants::SPEED_OF_LIGHT * 1e9
        );

        // The weighted + ISB solve: what gnss-rtk's equal-weight, single-clock
        // model cannot do with this pool. Per-constellation sigmas are
        // self-calibrated from residual RMS; the ISB state absorbs the common
        // Galileo bias instead of letting it leak into position.
        let to_geo = |p: [f64; 3]| {
            let (la, lo, _) = map_3d::ecef2geodetic(p[0], p[1], p[2], map_3d::Ellipsoid::WGS84);
            (la.to_degrees(), lo.to_degrees())
        };
        let (wg, _, _, _) = wls_fix(&gps, site);
        let (wlat_g, wlon_g) = to_geo(wg);
        let (wc, _, wisb, wsig) = wls_fix(&merged, site);
        let (wlat, wlon) = to_geo(wc);
        let sig = |c: Constellation| wsig.get(&c).copied().unwrap_or(0.0);
        eprintln!(
            "WLS GPS-only : {wlat_g:.6},{wlon_g:.6}  err {:.0} m",
            ion_site_error_m(wlat_g, wlon_g)
        );
        eprintln!(
            "WLS combined : {wlat:.6},{wlon:.6}  err {:.0} m  ISB {wisb:+.1} m ({:+.1} ns)  sigma GPS {:.0} m / GAL {:.0} m",
            ion_site_error_m(wlat, wlon),
            wisb / crate::constants::SPEED_OF_LIGHT * 1e9,
            sig(Constellation::GPS),
            sig(Constellation::Galileo)
        );
        // The ISB decomposition: the broadcast GST-GPST offset (word-10 GGTO)
        // is the system-time part of the measured inter-system bias; the
        // remainder is the receiver's own inter-signal (BPSK vs BOC path)
        // hardware delay. Under our t_tx convention (GST epochs mapped
        // nominally), a positive GGTO makes Galileo pseudoranges short by
        // c*GGTO, so hw = ISB + c*GGTO.
        match gal.iter().find(|e| e.ggto_valid) {
            Some(g) => {
                let ggto = crate::galileo_inav::ggto_at(g, g.week, g.tow as f64).unwrap();
                let isb_ns = wisb / crate::constants::SPEED_OF_LIGHT * 1e9;
                eprintln!(
                    "broadcast GGTO {:+.2} ns; measured ISB {isb_ns:+.2} ns -> receiver inter-signal delay {:+.2} ns",
                    ggto * 1e9,
                    isb_ns + ggto * 1e9
                );
            }
            None => eprintln!("no word-10 GGTO decoded in this pass"),
        }

        // Weighted + ISB, the combined solve must not be dragged below the
        // quality of its best constituent (the point of this experiment).
        let (wg_err, wc_err) = (
            ion_site_error_m(wlat_g, wlon_g),
            ion_site_error_m(wlat, wlon),
        );
        assert!(
            wc_err <= wg_err + 30.0,
            "weighted combined fix degraded: {wc_err:.0} m vs WLS GPS-only {wg_err:.0} m"
        );

        // Gates lock the experimental facts, not accuracy (the site truth is
        // only ~100 m precise): with the decoders' structural anchor latencies
        // corrected (nav_anchor_tx), the inter-constellation timing offset must
        // be in the GGTO/inter-signal class (sub-µs), not the 1.84 s the
        // uncorrected anchors carried; the merged solve must use both
        // constellations and land within the single-constellation error class.
        assert!(
            delta_sec.abs() < 1e-5,
            "inter-constellation timing offset {delta_sec:+.6} s — anchor latency regression?"
        );
        assert!(n > gps.len().max(gal.len()), "merge must add SVs");
        let c_err = ion_site_error_m(clat, clon);
        assert!(c_err < 1000.0, "combined fix off by {c_err:.0} m");
        assert!(
            bias.abs() < 30.0,
            "system bias {bias:.0} m — GGTO/hardware class is metres (was 145 m \
             when the old BOC DLL calibration double-compensated the anchor fix)"
        );
    }

    fn eph(prn: u8, m0: f64, omg0: f64, f0: f64, cn0: f64) -> RxEphemeris {
        let mut e = RxEphemeris::new(SV::new(Constellation::GPS, prn));
        e.m0 = m0;
        e.omg0 = omg0;
        e.f0 = f0;
        e.cn0 = cn0;
        e
    }

    #[test]
    fn cross_correlation_dropped_keeps_strongest() {
        // G15 is the true strong SV; G16 cross-correlated onto it and decoded the
        // same nav data (identical m0/omg0/f0) at a lower C/N0. G24 is a distinct SV.
        let strong = eph(15, 0.5, -1.2, 1.0e-4, 52.0);
        let xcorr = eph(16, 0.5, -1.2, 1.0e-4, 35.6);
        let other = eph(24, -0.3, 0.8, -4.7e-4, 55.0);

        let kept = reject_cross_correlations(vec![xcorr, strong, other]);

        assert_eq!(
            kept.len(),
            2,
            "the cross-correlation duplicate must be dropped"
        );
        assert!(
            kept.iter().any(|e| e.sv.prn == 15),
            "the strong (true) SV is kept"
        );
        assert!(
            kept.iter().any(|e| e.sv.prn == 24),
            "the distinct SV is kept"
        );
        assert!(
            !kept.iter().any(|e| e.sv.prn == 16),
            "the weaker cross-correlation lock is dropped"
        );
    }

    #[test]
    fn sat_list_tags_constellations_and_appends_blocks() {
        // Bare PRN list: constellation inferred from signal type.
        let l = get_sat_list("1,32,120,138,193,202", Signal::L1ca, false, false);
        assert_eq!(
            l.iter().map(|s| s.constellation).collect::<Vec<_>>(),
            vec![
                Constellation::GPS,
                Constellation::GPS,
                Constellation::SBAS,
                Constellation::SBAS,
                Constellation::QZSS,
                Constellation::QZSS,
            ]
        );

        // Prefixed format: constellation explicit, independent of --sig.
        let l = get_sat_list("G3,E11,G7", Signal::L1ca, false, false);
        assert_eq!(l[0], SV::new(Constellation::GPS, 3));
        assert_eq!(l[1], SV::new(Constellation::Galileo, 11));
        assert_eq!(l[2], SV::new(Constellation::GPS, 7));

        // Prefixed Galileo PRNs even when --sig is L1CA.
        let l = get_sat_list("E4,E9,E21", Signal::L1ca, false, false);
        assert!(l.iter().all(|s| s.constellation == Constellation::Galileo));
        assert_eq!(l.iter().map(|s| s.prn).collect::<Vec<_>>(), vec![4, 9, 21]);

        // --sbas appends the 120-138 block (19 PRNs) on the GPS default.
        let l = get_sat_list("", Signal::L1ca, true, false);
        assert_eq!(l.len(), 32 + 19);
        assert_eq!(
            l.iter()
                .filter(|s| s.constellation == Constellation::SBAS)
                .count(),
            19
        );

        // --qzss appends the 193-202 block (10 PRNs).
        let l = get_sat_list("", Signal::L1ca, false, true);
        assert_eq!(l.len(), 32 + 10);
        assert_eq!(
            l.iter()
                .filter(|s| s.constellation == Constellation::QZSS)
                .count(),
            10
        );

        // --sbas / --qzss append on top of an *explicit* --sats list too: this is
        // how validate_fix.py searches the GEOs (`--sats 1 --sbas`). The block
        // must follow the selection, not get dropped when --sats is non-empty.
        let l = get_sat_list("1", Signal::L1ca, true, true);
        assert_eq!(l.len(), 1 + 19 + 10);
        assert_eq!(l[0], SV::new(Constellation::GPS, 1));
        assert_eq!(
            l.iter()
                .filter(|s| s.constellation == Constellation::SBAS)
                .count(),
            19
        );
        assert_eq!(
            l.iter()
                .filter(|s| s.constellation == Constellation::QZSS)
                .count(),
            10
        );

        // Default Galileo block: 1..=36, all tagged Galileo.
        let l = get_sat_list("", Signal::GalileoE1b, false, false);
        assert_eq!(l.len(), 36);
        assert!(l.iter().all(|s| s.constellation == Constellation::Galileo));

        // Bare PRN with Galileo signal → Galileo constellation (backward compat).
        let l = get_sat_list("4,11", Signal::GalileoE1c, false, false);
        assert_eq!(l.len(), 2);
        assert_eq!(l[0], SV::new(Constellation::Galileo, 4));
        assert_eq!(l[1], SV::new(Constellation::Galileo, 11));

        // SBAS/QZSS are L1 C/A signals: an E1 session must drop them instead
        // of panicking in Channel::new ("no spreading code for E1B PRN 120" —
        // the tuni2025 UI crash, where the sticky --sbas flag followed a
        // signal switch to E1B). Both the appended blocks and explicit tokens.
        let l = get_sat_list("4", Signal::GalileoE1b, true, true);
        assert_eq!(l, vec![SV::new(Constellation::Galileo, 4)]);
        let l = get_sat_list("4,S120,J193", Signal::GalileoE1b, false, false);
        assert_eq!(l, vec![SV::new(Constellation::Galileo, 4)]);
    }

    #[test]
    fn distinct_svs_all_kept() {
        // Distinct satellites (different orbital/clock params) are never merged,
        // even if some fields coincide.
        let a = eph(5, 0.1, 0.2, 1.0e-4, 46.0);
        let b = eph(10, 0.1, 0.9, 2.0e-4, 49.0); // same m0, different omg0/f0
        let c = eph(12, 0.7, 0.2, 3.0e-4, 50.0); // same omg0, different m0/f0

        let kept = reject_cross_correlations(vec![a, b, c]);
        assert_eq!(kept.len(), 3, "distinct SVs must all be retained");
    }
}
