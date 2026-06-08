use glob::glob;
use gnss_rs::sv::SV;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;

use crate::channel::History;

const PLOT_FONT_SIZE: u32 = 15;
const PLOT_SIZE_X: u32 = 200;
const PLOT_SIZE_Y: u32 = 200;
const PLOT_FOLDER: &str = "plots";

pub fn plot_remove_old_graph() {
    let pattern = format!("{}/*.png", PLOT_FOLDER);

    for path in glob(&pattern).unwrap() {
        match path {
            Ok(path) => {
                log::info!("Removing chart: {:?}", path.display());
                std::fs::remove_file(path).unwrap();
            }
            Err(e) => println!("{:?}", e),
        }
    }
}

pub fn plot_remove(sv: SV) {
    let pattern = format!("{}/sat-{}-*.png", PLOT_FOLDER, sv.prn);

    for path in glob(&pattern).unwrap() {
        match path {
            Ok(path) => {
                log::info!("Removing chart: {:?}", path.display());
                std::fs::remove_file(path).unwrap();
            }
            Err(e) => println!("{:?}", e),
        }
    }
}

pub fn plot_time_graph(sv: SV, name: &str, time_series: &[f64], y_delta: f64, color: &RGBColor) {
    plot_time_graph_with_sz(
        sv,
        name,
        time_series,
        y_delta,
        color,
        PLOT_SIZE_X,
        PLOT_SIZE_Y,
    );
}

pub fn plot_time_graph_with_sz(
    sv: SV,
    name: &str,
    time_series: &[f64],
    y_delta: f64,
    color: &RGBColor,
    size_x: u32,
    size_y: u32,
) {
    let file_name = format!("{}/sat-{}-{}.png", PLOT_FOLDER, sv.prn, name);
    let root_area = BitMapBackend::new(&file_name, (size_x, size_y)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();

    if time_series.len() < 10 {
        return;
    }

    let x_max = time_series.len() as f64 * 0.001;

    let mut y_max = time_series
        .iter()
        .fold(f64::MIN, |acc, v| if *v > acc { *v } else { acc });
    y_max += y_delta;
    let mut y_min = time_series
        .iter()
        .fold(f64::MAX, |acc, v| if *v < acc { *v } else { acc });
    y_min -= y_delta;

    let mut ctx = ChartBuilder::on(&root_area)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .caption(
            format!("sat {}: {}", sv.prn, name),
            ("sans-serif", PLOT_FONT_SIZE),
        )
        .build_cartesian_2d(0.0..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        time_series
            .iter()
            .enumerate()
            .map(|(idx, v)| Circle::new((idx as f64 * 0.001, *v), 1, color)),
    )
    .unwrap();
}

/// Render all per-channel diagnostic plots for `sv` from its rolling History
/// (the receiver calls this on a throttle when `--plots` is enabled).
pub fn plot_channel(sv: SV, hist: &History) {
    // nav-msg: real part of the prompt correlator over time.
    let nav: Vec<f64> = hist.corr_p.iter().map(|c| c.re).collect();
    plot_time_graph_with_sz(sv, "nav-msg", &nav, 0.001, &BLACK, 400, 200);

    let code_phase: Vec<f64> = hist.code_phase_offset.iter().copied().collect();
    plot_time_graph(sv, "code-phase-offset", &code_phase, 50.0, &BLUE);

    let phi_err: Vec<f64> = hist.phi_error.iter().copied().collect();
    plot_time_graph(sv, "phi-error", &phi_err, 0.5, &BLACK);

    let doppler: Vec<f64> = hist.doppler_hz.iter().copied().collect();
    plot_time_graph(sv, "doppler-hz", &doppler, 10.0, &BLACK);

    // iq-scatter: the most recent (<=2000) prompt correlator samples.
    let len = hist.corr_p.len();
    let n = usize::min(len, 2000);
    let iq: Vec<Complex64> = hist.corr_p.iter().skip(len - n).copied().collect();
    plot_iq_scatter(sv, &iq);
}

pub fn plot_iq_scatter(sv: SV, series: &[Complex64]) {
    let file_name = format!("{}/sat-{}-iq-scatter.png", PLOT_FOLDER, sv.prn);
    let root_area = BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();

    if series.len() < 10 {
        return;
    }

    let delta = 1.4;
    let factor = 1000.0;
    let mut y_max = f64::MIN;
    let mut y_min = f64::MAX;
    let mut x_max = f64::MIN;
    let mut x_min = f64::MAX;

    for c in series {
        if c.im > y_max {
            y_max = c.im;
        }
        if c.im < y_min {
            y_min = c.im;
        }
        if c.re > x_max {
            x_max = c.re;
        }
        if c.re < x_min {
            x_min = c.re;
        }
    }
    y_max = (y_max * delta * factor).round();
    y_min = (y_min * delta * factor).round();
    x_max = (x_max * delta * factor).round();
    x_min = (x_min * delta * factor).round();
    x_max = f64::max(x_max, y_max);
    y_max = x_max;
    x_min = f64::min(x_min, y_min);
    y_min = x_min;

    let mut ctx = ChartBuilder::on(&root_area)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .caption(
            format!("sat {}: iq-scatter", sv.prn),
            ("sans-serif", PLOT_FONT_SIZE),
        )
        .build_cartesian_2d(x_min..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        series
            .iter()
            .map(|c| Circle::new((c.re * factor, c.im * factor), 1, RED)),
    )
    .unwrap();
}
