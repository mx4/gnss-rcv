use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::channel::Channel;
use crate::device::RtlSdrDevice;
use crate::ephemeris::Ephemeris as RxEphemeris;
use crate::network::RtlSdrTcp;
use crate::recording::IQFileType;
use crate::recording::IQRecording;
use crate::solver::PositionSolver;
use crate::state::GnssState;

const PERIOD_RCV: f64 = 0.001;

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
    pub sig: String,
    pub sats: String,
    pub sbas: bool,
    pub plots: bool,
    pub exit_on_fix: bool,
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
            sig: "L1CA".to_string(),
            sats: String::new(),
            sbas: false,
            plots: false,
            exit_on_fix: false,
        }
    }
}

pub struct Receiver {
    iq_feed: Box<dyn IQReader>,
    period_sp: usize, // samples per period
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<SV, Channel>,
    solver: PositionSolver,
    last_fix_sec: f64,
    exit_on_fix: bool,
    exit_req: Arc<AtomicBool>,
    stats: RunStats,
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

/// Build the channel list. PRNs >= 120 are SBAS (geostationary augmentation)
/// satellites; they share the L1 C/A code structure, so tag them as such — logs
/// and plots then read e.g. `S123`, and since they never complete a GPS
/// ephemeris the position solver leaves them out. `sbas` appends the legacy SBAS
/// L1 block (PRN 120-138) for a detection sweep on top of whatever was selected.
fn get_sat_list(sats: &str, sbas: bool) -> Vec<SV> {
    let sv_for_prn = |prn: u8| {
        let cons = if prn >= 120 {
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
    sig: &str,
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
            &cfg.sig,
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
        let period_sp = (PERIOD_RCV * cfg.fs) as usize;
        let mut channels = HashMap::<SV, Channel>::new();
        for sv in get_sat_list(&cfg.sats, cfg.sbas) {
            channels.insert(
                sv,
                Channel::new(&cfg.sig, sv, cfg.fs, cfg.fi, cfg.plots, state.clone()),
            );
        }

        Self {
            iq_feed,
            period_sp,
            off_samples: cfg.off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels,
            solver: PositionSolver::new(state),
            last_fix_sec: 0.0,
            exit_on_fix: cfg.exit_on_fix,
            exit_req,
            stats: RunStats::default(),
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
        self.cached_ts_sec_tail += num_samples as f64 / (1000.0 * self.period_sp as f64);

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
            self.cached_ts_sec_tail - 0.001,
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
        self.print_stats();
    }

    fn print_stats(&self) {
        let s = &self.stats;
        let data_sec = s.msec_processed as f64 / 1000.0;
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
        let tot_acq = sum(|c| c.stats.acq_attempts);
        let tot_acq_corr = sum(|c| c.stats.acq_corrs);
        let tot_trk = sum(|c| c.stats.trk_periods);
        let tot_sf = sum(|c| c.stats.subframes);
        let tot_par = sum(|c| c.stats.parity_errors);

        println!("\n===== run stats =====");
        println!("data {data_sec:.1}s   wall {wall:.1}s   real-time {rtf:.1}x");
        println!(
            "funnel: searched {} -> acquired {} -> tracked {} -> ephemeris {} -> used-in-fix {}",
            chans.len(),
            acquired,
            tracked,
            with_eph,
            used
        );
        println!(
            "fixes: {} attempts, {} ok, {} failed   xcorr-rejected {}",
            s.fix_attempts, s.fix_ok, s.fix_fail, s.xcorr_rejections
        );
        println!(
            "work: {tot_acq} acq-attempts, {tot_acq_corr} acq-correlations, \
             {tot_trk} tracking-periods, {tot_sf} subframes, {tot_par} parity-errors"
        );

        // Per-SV detail, only for PRNs that acquired at least once.
        println!("  SV    locks losses  trk(s) maxlk(s) ttfl(s)  cn0 subfr parity eph fix");
        for c in chans.iter().filter(|c| c.stats.locks > 0) {
            let st = &c.stats;
            let ttfl = if st.first_lock_ts > 0.0 {
                format!("{:.1}", st.first_lock_ts)
            } else {
                "-".to_string()
            };
            println!(
                "  {:<5} {:>5} {:>6} {:>7.1} {:>8.1} {:>7} {:>4.1} {:>5} {:>6} {:>3} {:>3}",
                c.sv.to_string(),
                st.locks,
                st.lock_losses,
                c.tracked_secs(),
                c.max_lock_secs(),
                ttfl,
                st.peak_cn0,
                st.subframes,
                st.parity_errors,
                if c.is_ephemeris_complete() { "y" } else { "-" },
                if st.used_in_fix { "y" } else { "-" },
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    // Synthesize `num_msec` of a clean (noiseless) L1CA signal for `prn`: the
    // upsampled PRN code (rotated by `code_phase_chips`) modulated onto a carrier
    // at the intermediate frequency, with the sign convention the receiver's
    // de-rotation cancels (iq * replica = bare code). Acquisition locks on the
    // first attempt. A non-zero code phase matches a real SV (never at exactly
    // phase 0, which would hit a first-tracking-step buffer edge case).
    fn synth_l1ca(
        prn: u8,
        fs: f64,
        fi: f64,
        code_phase_chips: usize,
        num_msec: usize,
    ) -> Vec<Complex64> {
        use crate::code::Code;
        const PI: f64 = std::f64::consts::PI;

        let code = Code::gen_code("L1CA", prn).unwrap();
        let code_len = code.len();
        let code_sp = (fs * 1e-3) as usize;
        let step = Complex64::from_polar(1.0, 2.0 * PI * fi / fs);

        let mut carrier = Complex64::new(1.0, 0.0);
        let mut sig = Vec::with_capacity(code_sp * num_msec);
        for n in 0..code_sp * num_msec {
            let chip = ((n % code_sp) * code_len / code_sp + code_phase_chips) % code_len;
            sig.push(carrier * code[chip] as f64);
            carrier *= step;
        }
        sig
    }

    // End-to-end DSP unit test with no recording or network: a synthetic clean
    // signal must drive a channel from Acquisition into Tracking. This is the
    // hermetic regression the file-based integration tests can't be in CI.
    #[test]
    fn synthetic_signal_acquires_and_tracks() {
        let (fs, fi) = (2_046_000.0, 0.0);
        let prn = 5u8;
        let sig = synth_l1ca(prn, fs, fi, 200, 60); // 60 ms: enough to acquire + track

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

    // Regression: a code phase of exactly 0 makes the first tracking step wrap
    // `code_off_sec` below zero while corr_p is still empty. That used to panic
    // (`corr_p.back().unwrap()` in get_code_and_carrier_phase); it must now track.
    #[test]
    fn tracks_at_code_phase_zero_without_panicking() {
        let (fs, fi) = (2_046_000.0, 0.0);
        let prn = 5u8;
        let sig = synth_l1ca(prn, fs, fi, 0, 60);
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
    fn sat_list_tags_sbas_and_appends_block() {
        // Explicit list: PRN >= 120 is tagged SBAS, the rest GPS.
        let l = get_sat_list("1,32,120,138", false);
        assert_eq!(l.len(), 4);
        assert_eq!(l[0].constellation, Constellation::GPS);
        assert_eq!(l[1].constellation, Constellation::GPS);
        assert_eq!(l[2].constellation, Constellation::SBAS);
        assert_eq!(l[3].constellation, Constellation::SBAS);

        // --sbas appends the 120-138 block (19 PRNs) on top of the GPS default.
        let l = get_sat_list("", true);
        let sbas = l.iter().filter(|s| s.constellation == Constellation::SBAS);
        assert_eq!(l.len(), 32 + 19);
        assert_eq!(sbas.count(), 19);
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
