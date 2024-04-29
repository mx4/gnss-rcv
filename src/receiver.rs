use colored::Colorize;
use rayon::prelude::*;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;
use std::ops::Mul;
use std::time::Instant;

use crate::gold_code::GoldCode;
use crate::recording::IQRecording;
use crate::satellite::GnssSatellite;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::calc_correlation;
use crate::util::get_2nd_max;
use crate::util::get_max_with_idx;
use crate::util::get_num_samples_per_msec;

const DOPPLER_SPREAD_HZ: u32 = 8 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const ACQUISITION_PERIOD_MSEC: usize = 10;
const SNR_THRESHOLD: f64 = 3.0;
const PI: f64 = std::f64::consts::PI;

pub struct GnssReceiver {
    gold_code: GoldCode,
    pub iq_recording: IQRecording,
    sat_vec: Vec<usize>,
    off_samples: usize,
    off_msec: usize,
    last_acq_off_msec: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_num_msec: usize,
    cached_off_msec_tail: usize,
    satellites: HashMap<usize, GnssSatellite>,
}

impl GnssReceiver {
    pub fn new(
        gold_code: GoldCode,
        iq_recording: IQRecording,
        off_msec: usize,
        sat_vec: Vec<usize>,
    ) -> Self {
        Self {
            gold_code,
            iq_recording,
            sat_vec,
            off_samples: off_msec * get_num_samples_per_msec(),
            off_msec,
            last_acq_off_msec: 0,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_num_msec: 0,
            cached_off_msec_tail: 0,
            satellites: HashMap::<usize, GnssSatellite>::new(),
        }
    }

    fn calc_cross_correlation(
        &self,
        fft_planner: &mut FftPlanner<f64>,
        iq_vec: &Vec<Complex64>,
        sat_id: usize,
        num_msec: usize,
        off_msec: usize,
        estimate_hz: i32,
        spread_hz: u32,
    ) -> GnssCorrelationParam {
        let sample_rate = self.iq_recording.sample_rate;
        let num_samples_per_msec = get_num_samples_per_msec();
        assert_eq!(iq_vec.len(), num_samples_per_msec * num_msec);

        let mut best_param = GnssCorrelationParam::default();
        let prn_code_fft = self.gold_code.get_fft_code(sat_id);

        let lo_hz = estimate_hz - spread_hz as i32;
        let hi_hz = estimate_hz + spread_hz as i32 + 1;

        for doppler_hz in (lo_hz..hi_hz).step_by((spread_hz / DOPPLER_SPREAD_BINS) as usize) {
            let imaginary = -2.0 * PI * doppler_hz as f64;
            let mut b_corr = vec![0f64; get_num_samples_per_msec()];

            for idx in 0..num_msec {
                let sample_index = idx * num_samples_per_msec + off_msec;
                let doppler_shift: Vec<_> = (0..num_samples_per_msec)
                    .map(|x| {
                        Complex64::from_polar(
                            1.0,
                            imaginary * (x + sample_index) as f64 / sample_rate as f64,
                        )
                    })
                    .collect();
                let range_lo = (idx + 0) * num_samples_per_msec;
                let range_hi = (idx + 1) * num_samples_per_msec;
                let mut iq_vec_sample = Vec::from(&iq_vec[range_lo..range_hi]);
                assert_eq!(iq_vec_sample.len(), doppler_shift.len());
                assert_eq!(iq_vec_sample.len(), prn_code_fft.len());

                for i in 0..iq_vec_sample.len() {
                    iq_vec_sample[i] = iq_vec_sample[i].mul(doppler_shift[i]);
                }

                let corr = calc_correlation(fft_planner, &iq_vec_sample, &prn_code_fft);
                for i in 0..corr.len() {
                    b_corr[i] += corr[i].norm();
                }
            }
            log::trace!(
                "  get_cross_correlation: {}-{} Hz -- trying {} Hz",
                estimate_hz - spread_hz as i32,
                estimate_hz + spread_hz as i32,
                doppler_hz,
            );

            let b_corr_norm = b_corr.iter().map(|&x| x * x).sum::<f64>();

            if b_corr_norm > best_param.corr_norm {
                let b_corr_second = get_2nd_max(&b_corr);
                let (idx, b_corr_peak) = get_max_with_idx(&b_corr);
                assert!(b_corr_peak != 0.0);
                assert!(b_corr_second != 0.0);
                // XXX: this results in faster acquisition and processing. Why?
                //let b_peak_to_second = 10.0 * (b_corr_peak / b_corr_second).log10();
                let b_peak_to_second =
                    10.0 * ((b_corr_peak - b_corr_second) / b_corr_second).log10();
                best_param.snr = b_peak_to_second;
                best_param.doppler_hz = doppler_hz as i32;
                best_param.phase_offset = idx;
                best_param.corr_norm = b_corr_norm;
                log::trace!(
                    "   best_doppler: {} Hz snr: {:+.1} idx={}",
                    doppler_hz,
                    b_peak_to_second,
                    idx / 2
                );
            }
        }
        best_param
    }

    fn try_acquisition_one_sat(
        &self,
        fft_planner: &mut FftPlanner<f64>,
        iq_vec: &Vec<Complex64>,
        sat_id: usize,
        off_msec: usize,
    ) -> Option<GnssCorrelationParam> {
        assert_eq!(
            iq_vec.len(),
            get_num_samples_per_msec() * ACQUISITION_PERIOD_MSEC
        );
        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_param = GnssCorrelationParam::default();

        while spread_hz > DOPPLER_SPREAD_BINS {
            //let (estimate_hz, snr, phase_idx) = self.calc_cross_correlation(
            let param = self.calc_cross_correlation(
                fft_planner,
                iq_vec,
                sat_id,
                ACQUISITION_PERIOD_MSEC,
                off_msec,
                best_param.doppler_hz,
                spread_hz,
            );
            if param.snr <= best_param.snr {
                break;
            }
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS;
            best_param = param;

            let s = format!(
                "BEST: sat_id={} -- estimate_hz={} spread_hz={:3} snr={:.3} phase_idx={}",
                sat_id, best_param.doppler_hz, spread_hz, best_param.snr, best_param.phase_offset
            );
            log::debug!("{}", s.green());
        }
        if best_param.snr >= SNR_THRESHOLD {
            log::info!(
                " sat_id: {} -- doppler_hz: {:5} phase_idx: {:4} snr: {}",
                format!("{:2}", sat_id).yellow(),
                best_param.doppler_hz,
                best_param.phase_offset / 2,
                format!("{:.2}", best_param.snr).green(),
            );
            Some(best_param)
        } else {
            None
        }
    }

    fn try_periodic_acquisition(&mut self) {
        if self.cached_num_msec < ACQUISITION_PERIOD_MSEC {
            return;
        }

        if self.cached_off_msec_tail < self.last_acq_off_msec + ACQUISITION_PERIOD_MSEC {
            return;
        }

        let ts = Instant::now();
        let cached_vec_len = self.cached_iq_vec.len();
        let num_samples = ACQUISITION_PERIOD_MSEC * get_num_samples_per_msec();

        log::warn!(
            "--- try_acq: cached_off_msec_tail={} cached_vec_len={}",
            format!("{}", self.cached_off_msec_tail).green(),
            cached_vec_len
        );

        let samples = &self.cached_iq_vec[cached_vec_len - num_samples..cached_vec_len].to_vec();
        let samples_off_msec = self.cached_off_msec_tail - ACQUISITION_PERIOD_MSEC;
        let mut new_sats = HashMap::<usize, GnssCorrelationParam>::new();

        // Perform acquisition on each possible satellite in parallel
        let new_sats_vec_res: Vec<_> = self
            .sat_vec
            .par_iter()
            .map(|&id| {
                let mut fft_planner: FftPlanner<f64> = FftPlanner::new();
                self.try_acquisition_one_sat(&mut fft_planner, &samples, id, samples_off_msec)
            })
            .collect();

        // So far we've only collected an array of options, let's insert them
        // in the map for easy look-up.
        for (i, opt) in new_sats_vec_res.iter().enumerate() {
            let id = self.sat_vec[i];
            if let Some(param) = opt {
                new_sats.insert(id, *param);
            }
        }

        for (id, param) in &new_sats {
            match self.satellites.get_mut(id) {
                Some(sat) => sat.update_param(param, samples_off_msec),
                None => {
                    self.satellites
                        .insert(*id, GnssSatellite::new(*id, *param, samples_off_msec));
                }
            }
        }
        let mut sat_removed = vec![];
        for id in self.satellites.keys() {
            if !new_sats.contains_key(&id) {
                log::warn!("{}", format!("sat {}: disappeared", id).red());
                sat_removed.push(id.clone());
            }
        }
        for id in sat_removed {
            self.satellites.remove(&id);
        }
        log::debug!("acquisition: {} msec", ts.elapsed().as_millis());
        self.last_acq_off_msec = self.cached_off_msec_tail;
    }

    fn fetch_samples_msec(
        &mut self,
        num_msec: usize,
    ) -> Result<IQSample, Box<dyn std::error::Error>> {
        let num_samples = num_msec * get_num_samples_per_msec();
        let mut sample = self
            .iq_recording
            .read_iq_file(self.off_samples, num_samples)?;

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut sample.iq_vec);
        self.cached_off_msec_tail += num_msec;
        self.cached_num_msec += num_msec;
        self.off_msec += num_msec;

        if self.cached_num_msec > ACQUISITION_PERIOD_MSEC {
            let num_msec_to_remove = self.cached_num_msec - ACQUISITION_PERIOD_MSEC;
            let num_samples = num_msec_to_remove * get_num_samples_per_msec();
            let _ = self.cached_iq_vec.drain(0..num_samples);
            self.cached_num_msec -= num_msec_to_remove;
        }
        let len = self.cached_iq_vec.len();
        Ok(IQSample {
            iq_vec: self.cached_iq_vec[len - num_samples..len].to_vec(),
            off_msec: self.cached_off_msec_tail - num_msec,
        })
    }

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        log::info!("process_step: off_msec={}", self.off_msec);

        let samples = self.fetch_samples_msec(1)?;
        self.try_periodic_acquisition();

        for (_id, sat) in &mut self.satellites {
            sat.process_samples(&samples);
        }

        Ok(())
    }
}
