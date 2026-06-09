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
use crate::recording::IQFileType;
use crate::recording::IQRecording;
use crate::solver::PositionSolver;
use crate::state::GnssState;

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
    solver: PositionSolver,
    last_fix_sec: f64,
    exit_on_fix: bool,
    exit_req: Arc<AtomicBool>,
    stats: RunStats,
    json_out: Option<PathBuf>,
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
/// the tag drives the log/plot label (G/S/Q) and the solver. `sbas` appends the
/// legacy SBAS L1 block (PRN 120-138) and `qzss` the QZSS block (PRN 193-202) on
/// top of whatever was selected. (SBAS never completes a GPS ephemeris so the
/// solver ignores it; QZSS does and `gnss-rtk` solves it.)
fn get_sat_list(sats: &str, sig: Signal, sbas: bool, qzss: bool) -> Vec<SV> {
    // Galileo signals (E1B/E1C): the constellation follows the *signal*, not the
    // PRN number (E1 PRNs 1..=36 overlap GPS), so tag every selected PRN Galileo.
    if matches!(sig, Signal::GalileoE1b | Signal::GalileoE1c) {
        let prns: Vec<u8> = if sats.is_empty() {
            (1..=36).collect()
        } else {
            sats.split(',').map(|s| s.parse().unwrap()).collect()
        };
        return prns
            .into_iter()
            .map(|prn| SV::new(Constellation::Galileo, prn))
            .collect();
    }

    let sv_for_prn = |prn: u8| {
        let cons = if (193..=202).contains(&prn) {
            Constellation::QZSS
        } else if prn >= 120 {
            Constellation::SBAS
        } else {
            Constellation::GPS
        };
        SV::new(cons, prn)
    };

    let mut sat_vec = vec![];
    if !sats.is_empty() {
        for s in sats.split(',') {
            let prn = s.parse::<u8>().unwrap();
            sat_vec.push(sv_for_prn(prn));
        }
    } else {
        for prn in 1..=32_u8 {
            sat_vec.push(SV::new(Constellation::GPS, prn));
        }
    }
    if sbas {
        for prn in 120..=138_u8 {
            sat_vec.push(SV::new(Constellation::SBAS, prn));
        }
    }
    if qzss {
        for prn in 193..=202_u8 {
            sat_vec.push(SV::new(Constellation::QZSS, prn));
        }
    }
    sat_vec
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
        let mut channels = HashMap::<SV, Channel>::new();
        for sv in get_sat_list(&cfg.sats, cfg.sig, cfg.sbas, cfg.qzss) {
            channels.insert(
                sv,
                Channel::new(cfg.sig, sv, cfg.fs, cfg.fi, cfg.plots, state.clone()),
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
            solver: PositionSolver::new(state),
            last_fix_sec: 0.0,
            exit_on_fix: cfg.exit_on_fix,
            exit_req,
            stats: RunStats::default(),
            json_out: cfg.json.clone(),
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
        if ts_sec - self.last_fix_sec < 2.0 {
            return;
        }

        let ephs: Vec<_> = self
            .channels
            .values()
            .filter(|&ch| ch.is_state_tracking())
            .filter(|&ch| ch.is_ephemeris_complete())
            .filter(|&ch| ch.nav.eph.tx_anchored && ch.ts_sec - ch.nav.eph.tx_anchor_ts_sec > 3.0)
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

        Ok(())
    }

    pub fn run_loop(&mut self, num_msec: usize) {
        let mut n = 0;
        loop {
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
    use crate::synth::{SynthE1Sv, SynthSv, synth_e1, synth_l1ca};

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
        // Explicit list: PRN by number -> GPS / SBAS / QZSS.
        let l = get_sat_list("1,32,120,138,193,202", Signal::L1ca, false, false);
        let cons: Vec<_> = l.iter().map(|s| s.constellation).collect();
        assert_eq!(
            cons,
            vec![
                Constellation::GPS,
                Constellation::GPS,
                Constellation::SBAS,
                Constellation::SBAS,
                Constellation::QZSS,
                Constellation::QZSS,
            ]
        );

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

        // A Galileo signal tags every PRN Galileo (default block 1..=36).
        let l = get_sat_list("", Signal::GalileoE1b, false, false);
        assert_eq!(l.len(), 36);
        assert!(l.iter().all(|s| s.constellation == Constellation::Galileo));
        let l = get_sat_list("4,11", Signal::GalileoE1c, false, false);
        assert_eq!(l.len(), 2);
        assert_eq!(l[0], SV::new(Constellation::Galileo, 4));
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
