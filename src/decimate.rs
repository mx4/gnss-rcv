//! Ingest decimation: wideband captures are FIR-filtered and integer-
//! downsampled to the lowest rate that still carries every enabled signal,
//! before anything else sees the samples.
//!
//! Everything in this receiver scales with `fs` — acquisition FFTs, carrier
//! tables, correlators, the acquisition grids — but the signals themselves
//! occupy only `|fi| ± 2.046 MHz` (the BOC(1,1) main lobes; C/A is narrower).
//! A 50 Msps capture therefore costs ~8× more than the information warrants.
//! [`DecimatingReader`] wraps the file reader and presents the stream at
//! `fs / k`: the receiver downstream is entirely unaware (it is simply
//! constructed with the lower rate).
//!
//! The anti-alias filter is a windowed-sinc low-pass; its constant group
//! delay ((taps−1)/2 input samples) shifts every satellite identically, so it
//! folds into the receiver clock bias like any common delay. Only file-backed
//! feeds are wrapped (streaming feeds have no random access for the overlap
//! reads); `GNSS_DECIM=off` disables for A/B runs.

use crate::receiver::IQReader;
use rustfft::num_complex::Complex32;

/// One-sided signal bandwidth that must survive: the E1 BOC(1,1) main lobes
/// (±1.023 MHz subcarrier ± 1.023 MHz code), plus front-end LO slack.
const SIGNAL_HALF_BW_HZ: f64 = 2.046e6 + 0.3e6;

/// Pick the decimation factor for a capture: the largest `k` that keeps the
/// output rate comfortably above the signal band (2.66× the one-sided width:
/// passband plus transition room for the FIR) while `fs/k` stays on the 1 ms
/// block grid (a whole number of samples per base block). 1 = no decimation.
pub fn decimation_factor(fs: f64, fi: f64) -> usize {
    let need = fi.abs() + SIGNAL_HALF_BW_HZ;
    let mut k = (fs / (2.66 * need)).floor() as usize;
    while k > 1 {
        let out = fs / k as f64;
        if (out * 1e-3).fract() == 0.0 {
            return k;
        }
        k -= 1;
    }
    1
}

pub struct DecimatingReader {
    inner: Box<dyn IQReader>,
    k: usize,
    taps: Vec<f32>,
}

impl DecimatingReader {
    pub fn new(inner: Box<dyn IQReader>, k: usize) -> Self {
        // Windowed-sinc low-pass at 0.41 of the *output* rate (0.82 of the
        // output Nyquist): passband covers the signal, the remaining 18% is
        // the transition band. 8k+1 taps put the first sidelobe well below
        // the 4..16-bit quantization floors of our captures.
        let t = 8 * k + 1;
        let fc = 0.41 / k as f64; // cycles per *input* sample
        let c = (t - 1) as f64 / 2.0;
        let taps: Vec<f32> = (0..t)
            .map(|j| {
                let x = j as f64 - c;
                let ideal = if x == 0.0 {
                    2.0 * fc
                } else {
                    (2.0 * std::f64::consts::PI * fc * x).sin() / (std::f64::consts::PI * x)
                };
                let hamming =
                    0.54 - 0.46 * (2.0 * std::f64::consts::PI * j as f64 / (t - 1) as f64).cos();
                (ideal * hamming) as f32
            })
            .collect();
        Self { inner, k, taps }
    }
}

impl IQReader for DecimatingReader {
    // Decimation lowers the sample *rate*, not the wall-clock length, so the
    // recording's total duration passes straight through to the progress bar.
    fn duration_sec(&self) -> Option<f64> {
        self.inner.duration_sec()
    }

    /// Output sample `n` is the filter evaluated at input sample `n·k`; each
    /// call re-reads the (taps−1)-sample overlap, which is noise next to the
    /// block itself. The start of the stream is zero-padded.
    fn get_iq_data(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        let t = self.taps.len();
        let in_first = (off_samples * self.k) as i64 - (t as i64 - 1);
        let in_start = in_first.max(0) as usize;
        let pad = (in_start as i64 - in_first) as usize;
        let in_len = num_samples * self.k + (t - 1) - pad;
        let input = self.inner.get_iq_data(in_start, in_len)?;

        let mut out = Vec::with_capacity(num_samples);
        for n in 0..num_samples {
            // Filter centred so that out[n] corresponds to input n·k, taps
            // applied to the preceding window (constant group delay).
            let end = n * self.k + (t - 1); // index into padded stream
            let mut acc = Complex32::new(0.0, 0.0);
            for (j, &h) in self.taps.iter().enumerate() {
                let idx = end as i64 - j as i64 - pad as i64;
                if idx >= 0 {
                    let s = input[idx as usize];
                    acc += Complex32::new(s.re * h, s.im * h);
                }
            }
            out.push(acc);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::receiver::MockIQReader;

    #[test]
    fn factor_respects_band_and_block_grid() {
        // tuni2025: 50 Msps, zero IF -> 8 (6.25 Msps, BOC lobes + margin).
        assert_eq!(decimation_factor(50e6, 0.0), 8);
        // Narrowband captures stay untouched.
        assert_eq!(decimation_factor(2.046e6, 0.0), 1);
        assert_eq!(decimation_factor(4e6, 0.0), 1);
        // A non-zero IF eats into the budget.
        assert_eq!(decimation_factor(10e6, 420e3), 1);
        assert_eq!(decimation_factor(26e6, 6.39e6), 1);
    }

    #[test]
    fn tone_survives_decimation() {
        // A 0.5 MHz complex tone at 16 Msps, decimated 2x: same frequency,
        // ~unit amplitude (passband), coherent phase after the group delay.
        let fs = 16.0e6;
        let f = 0.5e6;
        let n_in = 64_000;
        let samples: Vec<Complex32> = (0..n_in)
            .map(|n| {
                let ph = 2.0 * std::f64::consts::PI * f * n as f64 / fs;
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut d = DecimatingReader::new(Box::new(MockIQReader::new(samples)), 2);
        let out = d.get_iq_data(1000, 8000).unwrap();
        // Correlate against the expected tone at the output rate.
        let fs_out = fs / 2.0;
        let mut acc = (0.0f64, 0.0f64);
        for (n, s) in out.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * f * (1000 + n) as f64 / fs_out;
            let (c, sn) = (ph.cos(), ph.sin());
            acc.0 += s.re as f64 * c + s.im as f64 * sn;
            acc.1 += s.im as f64 * c - s.re as f64 * sn;
        }
        let amp = (acc.0.hypot(acc.1)) / out.len() as f64;
        assert!((amp - 1.0).abs() < 0.05, "passband amplitude {amp}");
    }

    #[test]
    fn out_of_band_tone_is_rejected() {
        // A tone past the output Nyquist (5 MHz at 16->8 Msps) must be
        // attenuated hard, not aliased in.
        let fs = 16.0e6;
        let f = 5.0e6;
        let samples: Vec<Complex32> = (0..64_000)
            .map(|n| {
                let ph = 2.0 * std::f64::consts::PI * f * n as f64 / fs;
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut d = DecimatingReader::new(Box::new(MockIQReader::new(samples)), 2);
        let out = d.get_iq_data(1000, 8000).unwrap();
        let pwr: f64 = out.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / out.len() as f64;
        assert!(pwr < 0.01, "stopband power {pwr}");
    }
}
