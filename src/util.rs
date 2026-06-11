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

pub fn get_max_with_idx(vec: &[f64]) -> (usize, f64) {
    let mut max = 0.0f64;
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
    assert_eq!(buf.len(), prn_code_fft.len());
    fft_planner.plan_fft_forward(buf.len()).process(buf);
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
    let step = Complex64::from_polar(1.0, -2.0 * PI * doppler_hz / fs);
    let mut c = Complex64::from_polar(1.0, -2.0 * PI * phi);
    for s in iq_vec.iter_mut() {
        *s *= Complex32::new(c.re as f32, c.im as f32);
        c *= step;
    }
}

pub fn getbitu(buf: &[u8], pos: usize, len: usize) -> u32 {
    assert!(len <= 32);
    let mut bits = 0;
    for i in pos..pos + len {
        bits = (bits << 1) | ((buf[i / 8] >> (7 - i % 8)) & 1) as u32;
    }
    bits
}

/// Read `len` bits at `pos` as a two's-complement signed value.
pub fn getbits(buf: &[u8], pos: usize, len: usize) -> i32 {
    let bits = getbitu(buf, pos, len);

    // Sign-extend the len-bit field: `mask` covers bit (len-1) — the field's
    // sign bit — and every bit above it. Negative values get all of them set
    // (the two's-complement extension); positive values get them cleared (a
    // no-op, since getbitu zero-fills above the field). The shift pair builds
    // the mask without overflowing at len == 32 (where `1 << len` would).
    let sign = (1 << (len - 1)) & bits;
    let mask = (0xffffffff >> (len - 1)) << (len - 1);
    let res = if sign != 0 { bits | mask } else { bits & !mask };
    res as i32
}

pub fn getbitu2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> u32 {
    assert!(l1 + l2 <= 32);
    let hi = getbitu(buf, p1, l1);
    let lo = getbitu(buf, p2, l2);
    (hi << l2) + lo
}

/// Read a signed value split across two bit ranges (nav messages split several
/// fields across word boundaries): the high part carries the sign — extend it,
/// shift it up, and append the unsigned low part.
pub fn getbits2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> i32 {
    assert!(l1 + l2 <= 32);
    if getbitu(buf, p1, 1) != 0 {
        (getbits(buf, p1, l1) << l2) + getbitu(buf, p2, l2) as i32
    } else {
        getbitu2(buf, p1, l1, p2, l2) as i32
    }
}

pub fn hex_str(data: &[u8]) -> String {
    let num_bits = data.len();
    let mut s = String::new();
    let num = num_bits.div_ceil(8);
    for v in data.iter().take(num) {
        let n = format!("{:02x}", *v);
        s.push_str(&n);
    }
    s
}

pub fn xor_bits(v: u32) -> u8 {
    const XOR_8B: [u8; 256] = [
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    ];

    let bytes = v.to_le_bytes().map(|v| v as usize);
    XOR_8B[bytes[0]] ^ XOR_8B[bytes[1]] ^ XOR_8B[bytes[2]] ^ XOR_8B[bytes[3]]
}

pub fn bits_opposed(bits0: &[u8], bits1: &[u8]) -> bool {
    let bits1_rev: Vec<_> = bits1.iter().map(|v| 1 - v).collect();
    bits_equal(bits0, bits1_rev.as_slice())
}

pub fn bits_equal(bits0: &[u8], bits1: &[u8]) -> bool {
    assert_eq!(bits0.len(), bits1.len());
    bits0 == bits1
}

pub fn setbitu(buf: &mut [u8], pos: usize, len: usize, data: u32) {
    let mut mask = 1u32 << (len - 1);
    if len > 32 {
        return;
    }
    for i in pos..pos + len {
        let bit = 1u8 << (7 - i % 8);
        if data & mask != 0 {
            buf[i / 8] |= bit;
        } else {
            buf[i / 8] &= !bit;
        }
        mask >>= 1;
    }
}
