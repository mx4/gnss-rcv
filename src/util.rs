use rustfft::{num_complex::Complex64, FftPlanner};
use std::ops::Mul;

use crate::constants::PRN_CODE_LEN;

const PI: f64 = std::f64::consts::PI;

pub fn norm_square(v: &Vec<Complex64>) -> f64 {
    v.iter().map(|&x| x.norm_sqr()).sum::<f64>()
}

pub fn norm(v: &Vec<Complex64>) -> f64 {
    norm_square(v).sqrt()
}

pub fn get_normalized_correlation_strength(corr: &Vec<f64>) -> f64 {
    let (peak_idx, peak_val) = get_max_with_idx(corr);
    let mut all_but_peak = 0.0;
    for i in 0..corr.len() {
        if i != peak_idx {
            all_but_peak += corr[i];
        }
    }
    peak_val / (all_but_peak / (corr.len() - 1) as f64)
}

pub fn get_max_with_idx(v: &Vec<f64>) -> (usize, f64) {
    let mut max = 0.0f64;
    let mut idx = 0;
    for i in 0..v.len() {
        if v[i] > max {
            max = v[i];
            idx = i;
        }
    }
    (idx, max)
}

pub fn get_num_samples_per_msec() -> usize {
    PRN_CODE_LEN * 2
}

pub fn get_2nd_max(v: &Vec<f64>) -> f64 {
    let (i_max, max) = get_max_with_idx(v);

    let mut second = 0.0;
    let delta = 50;
    for i in 0..v.len() {
        if v[i] > second && v[i] < max && (i > i_max + delta || i < i_max - delta) {
            second = v[i];
        }
    }
    second
}

fn normalize_post_fft(data: &mut Vec<Complex64>) {
    let len = data.len() as f64;
    data.iter_mut().for_each(|x| *x /= len);
}

pub fn correlate_vec(a: &Vec<Complex64>, b: &Vec<Complex64>) -> Complex64 {
    let mut sum = Complex64 { re: 0.0, im: 0.0 };
    for i in 0..a.len() {
        sum += a[i].mul(b[i].conj());
    }
    sum
}

pub fn calc_correlation(
    fft_planner: &mut FftPlanner<f64>,
    v_antenna: &Vec<Complex64>,
    prn_code_fft: &Vec<Complex64>,
) -> Vec<Complex64> {
    let num_samples = v_antenna.len();
    assert_eq!(v_antenna.len(), prn_code_fft.len());
    let fft_fw = fft_planner.plan_fft_forward(num_samples);

    let mut iq_samples_fft = v_antenna.clone();

    fft_fw.process(&mut iq_samples_fft);

    let mut v_res: Vec<_> = (0..num_samples)
        .map(|i| iq_samples_fft[i] * prn_code_fft[i].conj())
        .collect();

    let fft_bw = fft_planner.plan_fft_inverse(num_samples);
    fft_bw.process(&mut v_res);
    normalize_post_fft(&mut v_res);
    v_res
}

fn doppler_shifted_carrier(
    doppler_hz: f64,
    off_sec: f64,
    carrier_phase_shift: f64,
    sample_rate: usize,
    len: usize,
) -> Vec<Complex64> {
    let imaginary = -2.0 * PI * doppler_hz;
    let sample_rate_f64 = sample_rate as f64;
    let carrier: Vec<Complex64> = (0..len)
        .map(|x| x as f64)
        .map(|y| {
            Complex64::from_polar(
                1.0,
                imaginary * (y / sample_rate_f64 + off_sec) + carrier_phase_shift,
            )
        })
        .collect();
    carrier
}

pub fn doppler_shift(
    doppler_hz: f64,
    off_sec: f64,
    iq_vec: &mut Vec<Complex64>,
    carrier_phase_shift: f64,
    sample_rate: usize,
) {
    let carrier = doppler_shifted_carrier(
        doppler_hz,
        off_sec,
        carrier_phase_shift,
        sample_rate,
        iq_vec.len(),
    );

    assert_eq!(iq_vec.len(), carrier.len());

    for i in 0..iq_vec.len() {
        iq_vec[i] = iq_vec[i].mul(carrier[i]);
    }
}

pub fn vector_mean(vec: &[f64]) -> f64 {
    let n = vec.len() as f64;
    vec.iter().fold(0.0, |acc, v| acc + *v) / n
}

pub fn vector_mean_complex(vec: &[Complex64]) -> Complex64 {
    let n = vec.len() as f64;
    vec.iter()
        .fold(Complex64 { re: 0.0, im: 0.0 }, |acc, v| acc + *v)
        / n
}

pub fn vector_variance(vec: &[f64]) -> f64 {
    let n = vec.len() as f64;
    let mean = vector_mean(vec);
    vec.iter().fold(0.0, |acc, v| acc + (*v - mean).powi(2)) / n
}

/*
//extern crate "nalgebra" as na;
use nalgebra::DMatrix;

pub fn get_circularity(vec: &[Complex64]) -> f64 {
    let len = vec.len();
    let mut corr_re_im: Vec<f64>;

    for v in vec {
        corr_re_im.push(v.re);
    }
    for v in vec {
        corr_re_im.push(v.im);
    }
    let mat = DMatrix::from_col_vec(len, 2, corr_re_im);

    let cov = nalgebra::Cov(&mat);
    0.0
}
*/
