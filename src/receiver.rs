use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::path::Path;
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

pub struct Receiver {
    iq_feed: Box<dyn IQReader>,
    period_sp: usize, // samples per period
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<SV, Channel>,
    solver: PositionSolver,
    last_fix_sec: f64,
    exit_req: Arc<AtomicBool>,
}

fn get_sat_list(sats: &str) -> Vec<SV> {
    let mut sat_vec = vec![];
    if !sats.is_empty() {
        for s in sats.split(',') {
            let prn = s.parse::<u8>().unwrap();
            sat_vec.push(SV::new(Constellation::GPS, prn));
        }
    } else {
        for prn in 1..=32_u8 {
            sat_vec.push(SV::new(Constellation::GPS, prn));
        }
        let use_sbas = false;
        if use_sbas {
            for prn in 120..=158_u8 {
                sat_vec.push(SV::new(Constellation::GPS, prn));
            }
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
) -> Option<Box<dyn IQReader>> {
    if use_device {
        let res = RtlSdrDevice::new(sig, fs);
        if res.is_err() {
            log::warn!("Failed to open rtl-sdr device.");
            return None;
        }
        let dev = res.unwrap();

        Some(Box::new(dev))
    } else if !hostname.is_empty() {
        let net = RtlSdrTcp::new(hostname, exit_req.clone(), sig, fs).unwrap();

        log::warn!("Using rtl_tcp backend: {}", hostname);
        Some(Box::new(net))
    } else {
        Some(Box::new(IQRecording::new(file, fs, iq_file_type)))
    }
}

impl Receiver {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        use_device: bool,
        hostname: &str,
        file: &Path,
        iq_file_type: &IQFileType,
        fs: f64,
        fi: f64,
        off_msec: usize,
        sig: &str,
        sats: &str,
        plots: bool,
        exit_req: Arc<AtomicBool>,
        state: Arc<Mutex<GnssState>>,
    ) -> Self {
        let period_sp = (PERIOD_RCV * fs) as usize;
        let mut channels = HashMap::<SV, Channel>::new();
        let sat_vec = get_sat_list(sats);

        for sv in sat_vec {
            let pub_state = state.clone();
            channels.insert(sv, Channel::new(sig, sv, fs, fi, plots, pub_state));
        }

        let iq_feed = get_iq_feed(
            use_device,
            hostname,
            sig,
            fs,
            file,
            iq_file_type,
            exit_req.clone(),
        )
        .unwrap();

        Self {
            iq_feed,
            period_sp,
            off_samples: off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels,
            solver: PositionSolver::new(state),
            last_fix_sec: 0.0,
            exit_req: exit_req.clone(),
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
            .filter(|&ch| {
                ch.nav.eph.tx_anchored && ch.ts_sec - ch.nav.eph.tx_anchor_ts_sec > 3.0
            })
            .map(|ch| ch.nav.eph)
            .collect();

        let ephs = reject_cross_correlations(ephs);

        if ephs.len() < 4 {
            return;
        }

        log::warn!(
            "t={ts_sec:.3} -- {}",
            format!("attempting fix with {} SVs", ephs.len()).red()
        );

        self.solver.compute_position(ts_sec, &ephs);
        self.last_fix_sec = ts_sec;
    }

    fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let (iq_vec, ts_sec) = self.fetch_samples_msec()?;

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
            n += 1;
            if num_msec != 0 && n >= num_msec {
                log::info!("{num_msec} msecs of iq-data processed");
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        assert_eq!(kept.len(), 2, "the cross-correlation duplicate must be dropped");
        assert!(kept.iter().any(|e| e.sv.prn == 15), "the strong (true) SV is kept");
        assert!(kept.iter().any(|e| e.sv.prn == 24), "the distinct SV is kept");
        assert!(
            !kept.iter().any(|e| e.sv.prn == 16),
            "the weaker cross-correlation lock is dropped"
        );
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
