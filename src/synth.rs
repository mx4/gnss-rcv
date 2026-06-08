//! Synthetic GPS L1 C/A signal generator.
//!
//! Produces baseband / low-IF complex IQ that the *real* acquisition → tracking
//! → decode path can lock onto, with no recording or network. Two uses:
//!   * hermetic, deterministic unit tests (acquire/track regressions runnable in
//!     CI — see `receiver.rs`), and
//!   * a controlled DSP bench — e.g. emit a 45 dB-Hz SV with known 50 bps bit
//!     edges to reproduce/measure the slow bit-sync issue without downloading a
//!     multi-GB capture.
//!
//! The carrier sign convention matches the receiver's de-rotation (it mixes down
//! by `fi + doppler`), i.e. the modelled signal is
//! `code · nav · exp(+j·2π·(fi+fd)·t)`, so a correct replica cancels it to the
//! bare code.

use rustfft::num_complex::Complex64;
use std::f64::consts::TAU;

use crate::code::{Code, L1CA_CODE_LEN};

/// L1 C/A chip rate (1023 chips / 1 ms).
const CODE_RATE_HZ: f64 = 1_023_000.0;
/// L1 carrier, used to stretch the code rate by the carrier Doppler.
const L1_HZ: f64 = 1_575_420_000.0;
/// One nav data bit lasts 20 ms (50 bps) = 20 code periods.
const NAV_BIT_SEC: f64 = 0.020;

/// One synthetic satellite in a scene.
#[derive(Clone)]
pub struct SynthSv {
    pub prn: u8,
    /// Carrier Doppler (Hz). Also stretches the code rate by `fd / L1`.
    pub doppler_hz: f64,
    /// Initial code phase (chips, may be fractional). Real SVs are never at
    /// exactly 0 — which is itself a useful edge case (first tracking step).
    pub code_phase_chips: f64,
    /// Carrier-to-noise density (dB-Hz). Used only by the *noisy* generator to
    /// scale this SV's amplitude against unit-variance AWGN.
    pub cn0_dbhz: f64,
    /// Optional 50 bps nav data bits (±1). Empty = held at +1 (no transitions),
    /// which still acquires and tracks; supply bits to exercise bit/frame sync.
    pub nav_bits: Vec<i8>,
}

impl SynthSv {
    /// A satellite with no nav-data transitions (data held at +1).
    pub fn new(prn: u8, doppler_hz: f64, code_phase_chips: f64, cn0_dbhz: f64) -> Self {
        Self {
            prn,
            doppler_hz,
            code_phase_chips,
            cn0_dbhz,
            nav_bits: Vec::new(),
        }
    }

    /// Same, but BPSK-modulated by a 50 bps nav-bit pattern (±1) — for bit/frame
    /// sync benches and tests.
    pub fn with_nav_bits(mut self, bits: Vec<i8>) -> Self {
        self.nav_bits = bits;
        self
    }
}

/// Deterministic splitmix64 PRNG + Box–Muller Gaussian, so noisy scenes are
/// reproducible bit-for-bit without depending on the `rand` crate.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        // 53-bit mantissa uniform in [0, 1).
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// One N(0, 1) sample.
    fn gauss(&mut self) -> f64 {
        let u1 = self.unit().max(1e-300);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }
}

/// Render `num_msec` of complex IQ at sample rate `fs` and IF `fi`, summing every
/// satellite in `svs`.
///
/// * `seed = None` — a clean noiseless reference: each SV at unit amplitude,
///   `cn0_dbhz` ignored. Fully deterministic (no RNG).
/// * `seed = Some(s)` — adds complex AWGN (`E|n|² = 1` per sample) and scales each
///   SV's amplitude so it sits at its `cn0_dbhz` in that noise:
///   `A = sqrt(10^(cn0/10) / fs)`.
pub fn synth_l1ca(
    svs: &[SynthSv],
    fs: f64,
    fi: f64,
    num_msec: usize,
    seed: Option<u64>,
) -> Vec<Complex64> {
    let code_sp = (fs * 1e-3) as usize;
    let n_total = code_sp * num_msec;

    let codes: Vec<Vec<i8>> = svs
        .iter()
        .map(|s| Code::gen_code("L1CA", s.prn).expect("L1CA code"))
        .collect();
    let chip_rate: Vec<f64> = svs
        .iter()
        .map(|s| CODE_RATE_HZ * (1.0 + s.doppler_hz / L1_HZ))
        .collect();
    let amp: Vec<f64> = svs
        .iter()
        .map(|s| match seed {
            None => 1.0,
            Some(_) => (10f64.powf(s.cn0_dbhz / 10.0) / fs).sqrt(),
        })
        .collect();

    let mut rng = seed.map(Rng);
    let nstd = 0.5f64.sqrt(); // per-component std so E|n|² = 1 for the complex sample
    let inv_fs = 1.0 / fs;
    let mut out = Vec::with_capacity(n_total);
    for n in 0..n_total {
        let t = n as f64 * inv_fs;
        let mut x = Complex64::new(0.0, 0.0);
        for (k, s) in svs.iter().enumerate() {
            let chip = (s.code_phase_chips + chip_rate[k] * t).rem_euclid(L1CA_CODE_LEN as f64);
            let c = codes[k][chip as usize] as f64;
            let b = if s.nav_bits.is_empty() {
                1.0
            } else {
                let bi = (t / NAV_BIT_SEC) as usize % s.nav_bits.len();
                s.nav_bits[bi] as f64
            };
            let phase = TAU * (fi + s.doppler_hz) * t;
            x += Complex64::from_polar(amp[k] * c * b, phase);
        }
        if let Some(rng) = rng.as_mut() {
            x += Complex64::new(rng.gauss() * nstd, rng.gauss() * nstd);
        }
        out.push(x);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn output_length_and_unit_amplitude_when_noiseless() {
        let fs = 2_046_000.0;
        let sig = synth_l1ca(&[SynthSv::new(5, 0.0, 0.0, 0.0)], fs, 0.0, 3, None);
        assert_eq!(sig.len(), (fs * 1e-3) as usize * 3);
        // noiseless single SV: |code · carrier| == 1 everywhere.
        assert!(sig.iter().all(|c| (c.norm() - 1.0).abs() < 1e-9));
    }

    #[test]
    fn noisy_amplitude_tracks_requested_cn0() {
        // The despread carrier power should land near 10^(cn0/10)/fs · 1023
        // (1023 = coherent gain over one code). Just sanity-check the mean noise
        // power is ~1 and the run is deterministic for a fixed seed.
        let fs = 2_046_000.0;
        let a = synth_l1ca(&[SynthSv::new(5, 1000.0, 100.0, 45.0)], fs, 0.0, 2, Some(7));
        let b = synth_l1ca(&[SynthSv::new(5, 1000.0, 100.0, 45.0)], fs, 0.0, 2, Some(7));
        assert_eq!(a, b, "same seed must reproduce the same samples");
        let mean_pwr: f64 = a.iter().map(|c| c.norm_sqr()).sum::<f64>() / a.len() as f64;
        // noise power ~1 dominates the weak signal; should be close to 1.
        assert!((0.7..1.4).contains(&mean_pwr), "mean power {mean_pwr:.3}");
    }
}
