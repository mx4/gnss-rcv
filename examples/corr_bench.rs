//! Isolated micro-benchmark for optimization #1: storing the tracking-loop
//! spreading-code replica as real `f32` instead of `Complex32` (im=0).
//!
//! Mirrors `Channel::tracking_compute_correlation`'s fused prompt/early/late/
//! neutral pass exactly, at nov3's parameters (fs=2.046 MHz, L1CA):
//!   code_sp = fs*1ms = 2046, sp_per_chip = 2.0, pos = 1, pos_neutral = 80.
//! Reports ns per code-period correlation for each variant and the speedup.
//! Run: cargo run --release --example corr_bench

use rustfft::num_complex::{Complex32, Complex64};
use std::time::Instant;

const CODE_SP: usize = 2046;
const POS: usize = 1; // SP_CORR(0.5) * sp_per_chip(2.0)
const POS_NEUTRAL: usize = 80; // NEUTRAL_CORR(40) * sp_per_chip(2.0)

#[inline]
fn acc(a: &mut Complex64, p: Complex32) {
    a.re += p.re as f64;
    a.im += p.im as f64;
}

/// Current code: replica is Complex32 with im=0; full complex*complex products.
#[inline(never)]
fn corr_complex(
    sig: &[Complex32],
    code: &[Complex32],
) -> (Complex64, Complex64, Complex64, Complex64) {
    let len = sig.len();
    let (pos, pos_neutral) = (POS, POS_NEUTRAL);
    let mut cp = Complex64::default();
    let mut ce = Complex64::default();
    let mut cl = Complex64::default();
    let mut cn = Complex64::default();
    for j in 0..len {
        let sj = sig[j];
        let cj = code[j];
        acc(&mut cp, sj * cj);
        if j + pos < len {
            acc(&mut ce, sj * code[j + pos]);
            acc(&mut cl, sig[j + pos] * cj);
        }
        if j + pos_neutral < len {
            acc(&mut cn, sj * code[j + pos_neutral]);
        }
    }
    (cp, ce, cl, cn)
}

/// Optimization #1 ALONE: real f32 replica, but the SAME branchy structure and
/// f64 accumulation as corr_complex -- isolates the halved-multiply effect from
/// the vectorization that the branch-split (corr_real) additionally unlocks.
#[inline(never)]
fn corr_real_only(
    sig: &[Complex32],
    code: &[f32],
) -> (Complex64, Complex64, Complex64, Complex64) {
    let len = sig.len();
    let (pos, pos_neutral) = (POS, POS_NEUTRAL);
    let mut cp = Complex64::default();
    let mut ce = Complex64::default();
    let mut cl = Complex64::default();
    let mut cn = Complex64::default();
    for j in 0..len {
        let sj = sig[j];
        let cj = code[j];
        acc(&mut cp, Complex32::new(sj.re * cj, sj.im * cj));
        if j + pos < len {
            let ec = code[j + pos];
            acc(&mut ce, Complex32::new(sj.re * ec, sj.im * ec));
            let sl = sig[j + pos];
            acc(&mut cl, Complex32::new(sl.re * cj, sl.im * cj));
        }
        if j + pos_neutral < len {
            let nc = code[j + pos_neutral];
            acc(&mut cn, Complex32::new(sj.re * nc, sj.im * nc));
        }
    }
    (cp, ce, cl, cn)
}

/// Optimization #1+#2: real f32 replica; products are real*complex (half the
/// multiplies), branch split out so the main region auto-vectorizes.
#[inline(never)]
fn corr_real(
    sig: &[Complex32],
    code: &[f32],
) -> (Complex64, Complex64, Complex64, Complex64) {
    let len = sig.len();
    let (pos, pos_neutral) = (POS, POS_NEUTRAL);
    let mut cp = Complex64::default();
    let mut ce = Complex64::default();
    let mut cl = Complex64::default();
    let mut cn = Complex64::default();
    // Branch-free main region: every tap (prompt, +/-pos, +pos_neutral) valid.
    let main = len - pos_neutral;
    // f32 lane partials, reduced into the f64 accumulators at the end (the
    // products stay f32 -- the SIMD-heavy part -- exactly as the comment in
    // tracking_compute_correlation describes).
    let (mut pr, mut pi) = (0.0f32, 0.0f32);
    let (mut er, mut ei) = (0.0f32, 0.0f32);
    let (mut lr, mut li) = (0.0f32, 0.0f32);
    let (mut nr, mut ni) = (0.0f32, 0.0f32);
    for j in 0..main {
        let sj = sig[j];
        let cj = code[j];
        pr += sj.re * cj;
        pi += sj.im * cj;
        let ce_c = code[j + pos];
        er += sj.re * ce_c;
        ei += sj.im * ce_c;
        let sl = sig[j + pos];
        lr += sl.re * cj;
        li += sl.im * cj;
        let cn_c = code[j + pos_neutral];
        nr += sj.re * cn_c;
        ni += sj.im * cn_c;
    }
    cp += Complex64::new(pr as f64, pi as f64);
    ce += Complex64::new(er as f64, ei as f64);
    cl += Complex64::new(lr as f64, li as f64);
    cn += Complex64::new(nr as f64, ni as f64);
    // Scalar tail (prompt + early/late only; neutral done).
    for j in main..len {
        let sj = sig[j];
        let cj = code[j];
        acc(&mut cp, Complex32::new(sj.re * cj, sj.im * cj));
        if j + pos < len {
            let ce_c = code[j + pos];
            acc(&mut ce, Complex32::new(sj.re * ce_c, sj.im * ce_c));
            let sl = sig[j + pos];
            acc(&mut cl, Complex32::new(sl.re * cj, sl.im * cj));
        }
    }
    (cp, ce, cl, cn)
}

/// Faithful copy of util::doppler_shift -- the carrier mix run once per tracking
/// step, over the same code_sp samples, before the correlation. Unchanged by #1,
/// so timing it gives the realistic per-step composition.
#[inline(never)]
fn doppler_shift(iq: &mut [Complex32], doppler_hz: f64, phi: f64, fs: f64) {
    const PI: f64 = std::f64::consts::PI;
    let step = Complex64::from_polar(1.0, -2.0 * PI * doppler_hz / fs);
    let mut c = Complex64::from_polar(1.0, -2.0 * PI * phi);
    for s in iq.iter_mut() {
        *s *= Complex32::new(c.re as f32, c.im as f32);
        c *= step;
    }
}

fn main() {
    // Deterministic pseudo-random signal and a +/-1 code (splitmix64).
    let mut s: u64 = 0x1234_5678_9abc_def0;
    let mut rng = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) as f64 / u64::MAX as f64) as f32
    };
    let sig: Vec<Complex32> = (0..CODE_SP)
        .map(|_| Complex32::new(rng() * 2.0 - 1.0, rng() * 2.0 - 1.0))
        .collect();
    let code_real: Vec<f32> = (0..CODE_SP)
        .map(|_| if rng() > 0.5 { 1.0 } else { -1.0 })
        .collect();
    let code_cplx: Vec<Complex32> = code_real.iter().map(|&c| Complex32::new(c, 0.0)).collect();

    const ITERS: usize = 400_000; // ~400 s of single-channel L1CA tracking
    const WARMUP: usize = 20_000;

    // Verify the two variants agree to f32 precision before timing.
    let a = corr_complex(&sig, &code_cplx);
    let b = corr_real(&sig, &code_real);
    let dp = ((a.0 - b.0).norm()).max((a.1 - b.1).norm());
    println!("agreement check: max prompt/early delta = {dp:.3e} (should be ~0)");

    let mut sink = Complex64::default();
    for _ in 0..WARMUP {
        sink += corr_complex(&sig, &code_cplx).0;
        sink += corr_real(&sig, &code_real).0;
    }

    let t0 = Instant::now();
    for _ in 0..ITERS {
        let r = corr_complex(&sig, &code_cplx);
        sink += r.0 + r.1 + r.2 + r.3;
    }
    let dur_complex = t0.elapsed();

    let t1 = Instant::now();
    for _ in 0..ITERS {
        let r = corr_real(&sig, &code_real);
        sink += r.0 + r.1 + r.2 + r.3;
    }
    let dur_real = t1.elapsed();

    let t2 = Instant::now();
    for _ in 0..ITERS {
        let r = corr_real_only(&sig, &code_real);
        sink += r.0 + r.1 + r.2 + r.3;
    }
    let dur_real_only = t2.elapsed();

    let mut dop_buf = sig.clone();
    let t3 = Instant::now();
    for _ in 0..ITERS {
        doppler_shift(&mut dop_buf, 1234.0, 0.1, 2_046_000.0);
    }
    let dur_dopp = t3.elapsed();
    std::hint::black_box(&dop_buf);

    let ns_complex = dur_complex.as_secs_f64() * 1e9 / ITERS as f64;
    let ns_real = dur_real.as_secs_f64() * 1e9 / ITERS as f64;
    let ns_real_only = dur_real_only.as_secs_f64() * 1e9 / ITERS as f64;
    let ns_dopp = dur_dopp.as_secs_f64() * 1e9 / ITERS as f64;
    println!("(sink guard: {:.3})", sink.norm().fract());
    println!("code_sp={CODE_SP} pos={POS} pos_neutral={POS_NEUTRAL} iters={ITERS}");
    println!("complex replica (current)      : {ns_complex:8.1} ns / code period");
    println!("real replica, same structure(#1): {ns_real_only:8.1} ns / code period");
    println!("real replica, vectorized (#1+#2): {ns_real:8.1} ns / code period");
    println!(
        "speedup #1 alone : {:.2}x  ({:.0}% faster)",
        ns_complex / ns_real_only,
        (ns_complex / ns_real_only - 1.0) * 100.0
    );
    println!(
        "speedup #1+#2    : {:.2}x  ({:.0}% faster)",
        ns_complex / ns_real,
        (ns_complex / ns_real - 1.0) * 100.0
    );
    println!("--- per-tracking-step composition (carrier mix is unchanged by #1) ---");
    println!("doppler_shift (carrier mix)    : {ns_dopp:8.1} ns / code period");
    // A tracking step's DSP cost ~= carrier mix + correlation (loop math is O(1)).
    let step_cur = ns_dopp + ns_complex;
    let step_1 = ns_dopp + ns_real_only;
    let step_12 = ns_dopp + ns_real;
    println!("step (mix+corr) current        : {step_cur:8.1} ns");
    println!("step (mix+corr) #1             : {step_1:8.1} ns  -> {:.2}x", step_cur / step_1);
    println!("step (mix+corr) #1+#2          : {step_12:8.1} ns  -> {:.2}x", step_cur / step_12);
}
