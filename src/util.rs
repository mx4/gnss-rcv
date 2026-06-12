//! Sample-domain DSP helpers: correlation (time-domain and FFT), carrier
//! generation/mixing by phase recurrence, and power utilities.

use rustfft::{
    FftPlanner,
    num_complex::{Complex32, Complex64},
};

const PI: f64 = std::f64::consts::PI;

pub fn norm_square(v: &[Complex32]) -> f64 {
    // f64 accumulator: summing ~50k f32 squares in f32 loses ~3 significant
    // digits; the cast is free next to the multiply.
    v.iter().map(|&x| x.norm_sqr() as f64).sum::<f64>()
}

pub fn norm(v: &[Complex32]) -> f64 {
    norm_square(v).sqrt()
}

pub fn get_max_with_idx(vec: &[f32]) -> (usize, f32) {
    let mut max = 0.0f32;
    let mut idx = 0;
    for (i, v) in vec.iter().enumerate() {
        if *v > max {
            max = *v;
            idx = i;
        }
    }

    (idx, max)
}

pub fn get_average(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / v.len() as f64
}

fn normalize_post_fft(data: &mut [Complex32]) {
    let len = data.len() as f32;
    data.iter_mut().for_each(|x| *x /= len);
}

/// Correlate two f32 sample vectors; the products are f32 (SIMD-friendly) but
/// the running sum is f64 — over a 50k-sample code period an f32 accumulator
/// would cost ~3 significant digits of the discriminator inputs.
pub fn correlate_vec(a: &[Complex32], b: &[Complex32]) -> Complex64 {
    let mut sum = Complex64 { re: 0.0, im: 0.0 };
    for (x, y) in a.iter().zip(b) {
        let p = x * y.conj();
        sum.re += p.re as f64;
        sum.im += p.im as f64;
    }
    sum
}

/// In-place FFT circular correlation: `buf` enters holding the (carrier-mixed)
/// samples and leaves holding the complex correlation against `prn_code_fft`.
/// Allocation-free — the acquisition hot path calls this per Doppler bin per
/// code period, where per-call buffers were ~120 MB/s of churn per searching
/// channel at 50 Msps.
pub fn calc_correlation_inplace(
    fft_planner: &mut FftPlanner<f32>,
    buf: &mut [Complex32],
    prn_code_fft: &[Complex32],
) {
    fft_planner.plan_fft_forward(buf.len()).process(buf);
    finish_correlation_inplace(fft_planner, buf, prn_code_fft);
}

/// The PRN-specific tail of the FFT correlation: `buf` enters holding the
/// *frequency-domain* (carrier-mixed, forward-FFT'd) samples — which are
/// PRN-independent and therefore shareable across every channel searching the
/// same block (see the scheduler's acquisition FFT cache) — and leaves
/// holding the complex correlation against `prn_code_fft`.
pub fn finish_correlation_inplace(
    fft_planner: &mut FftPlanner<f32>,
    buf: &mut [Complex32],
    prn_code_fft: &[Complex32],
) {
    assert_eq!(buf.len(), prn_code_fft.len());
    for (s, c) in buf.iter_mut().zip(prn_code_fft) {
        *s *= c.conj();
    }
    fft_planner.plan_fft_inverse(buf.len()).process(buf);
    normalize_post_fft(buf);
}

pub fn calc_correlation(
    fft_planner: &mut FftPlanner<f32>,
    iq_vec: &[Complex32],
    prn_code_fft: &[Complex32],
) -> Vec<Complex32> {
    let mut buf = iq_vec.to_owned();
    calc_correlation_inplace(fft_planner, &mut buf, prn_code_fft);
    buf
}

pub fn doppler_shifted_carrier(doppler_hz: f64, phi: f64, fs: f64, len: usize) -> Vec<Complex32> {
    // carrier[n] = exp(-j (2*pi*doppler*n/fs + 2*pi*phi))
    // Built by phase recurrence (carrier[n+1] = carrier[n] * step) so it costs
    // one complex multiply per sample instead of a sin/cos per sample. The
    // recurrence runs in f64 — over 50k samples an f32 recurrence drifts both
    // phase and amplitude — and only the stored sample is f32.
    let step = Complex64::from_polar(1.0, -2.0 * PI * doppler_hz / fs);
    let mut c = Complex64::from_polar(1.0, -2.0 * PI * phi);

    let mut carrier = Vec::with_capacity(len);
    for _ in 0..len {
        carrier.push(Complex32::new(c.re as f32, c.im as f32));
        c *= step;
    }
    carrier
}

pub fn doppler_shift(iq_vec: &mut [Complex32], doppler_hz: f64, phi: f64, fs: f64) {
    // Mix by phase recurrence (one complex multiply per sample) instead of
    // building a carrier vector with a sin/cos per sample. Equivalent to
    // multiplying sample n by exp(-j(2*pi*doppler*n/fs + 2*pi*phi)). The
    // recurrence stays f64 (see doppler_shifted_carrier); the sample multiply
    // is f32.
    //
    // A single serial recurrence `c *= step` is a loop-carried f64 dependency
    // and the most expensive part of the tracking step (~2.6x the correlation
    // on Apple M3) precisely because it cannot pipeline. Run LANES independent
    // phase accumulators instead — lane[k] = c*step^k, the whole block strided
    // by step^LANES — so LANES samples advance in parallel. Output is identical
    // to the serial form (still f64 recurrence, f32 store); ~2.2x faster.
    const LANES: usize = 8;
    let step = Complex64::from_polar(1.0, -2.0 * PI * doppler_hz / fs);
    let mut lane = [Complex64::default(); LANES];
    let mut c = Complex64::from_polar(1.0, -2.0 * PI * phi);
    let mut step_lanes = Complex64::new(1.0, 0.0);
    for l in lane.iter_mut() {
        *l = c;
        c *= step;
        step_lanes *= step; // == step^LANES after the loop
    }

    let mut chunks = iq_vec.chunks_exact_mut(LANES);
    for chunk in &mut chunks {
        for (s, l) in chunk.iter_mut().zip(lane.iter_mut()) {
            *s *= Complex32::new(l.re as f32, l.im as f32);
            *l *= step_lanes;
        }
    }
    // Tail (fewer than LANES samples left); lane[k] already holds c*step^k.
    for (s, l) in chunks.into_remainder().iter_mut().zip(lane.iter()) {
        *s *= Complex32::new(l.re as f32, l.im as f32);
    }
}
