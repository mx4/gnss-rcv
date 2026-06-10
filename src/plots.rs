use glob::glob;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;

use crate::channel::History;

const PLOT_FONT_SIZE: u32 = 12;
const PLOT_W: u32 = 300;
const PLOT_H: u32 = 150;
const PLOT_W_NAV: u32 = 400;
const PLOT_H_NAV: u32 = 150;
const PLOT_W_SCATTER: u32 = 180;
const PLOT_H_SCATTER: u32 = 180;
const PLOT_FOLDER: &str = "plots";

// Filesystem-safe label: uses the gnss_rs SV Display ("G01", "E01", "S120" …)
// which is already alphanumeric, but replace any unexpected punctuation just
// in case future gnss_rs versions change the format.
fn sv_slug(sv: SV) -> String {
    format!("{sv}").replace([':', ' ', '/'], "-")
}

pub fn plot_remove_old_graph() {
    let pattern = format!("{}/*.png", PLOT_FOLDER);
    for path in glob(&pattern).unwrap().flatten() {
        log::info!("Removing chart: {:?}", path.display());
        let _ = std::fs::remove_file(path);
    }
}

pub fn plot_remove(sv: SV) {
    let pattern = format!("{}/{}-*.png", PLOT_FOLDER, sv_slug(sv));
    for path in glob(&pattern).unwrap().flatten() {
        log::info!("Removing chart: {:?}", path.display());
        let _ = std::fs::remove_file(path);
    }
}

/// Generate (or overwrite) `plots/index.html` covering every possible SV so
/// that the static file works for GPS, Galileo and SBAS runs without manual
/// editing.  Missing PNGs are silently removed by the `onerror` handler.
pub fn plot_generate_html() {
    let abs =
        std::fs::canonicalize(std::env::current_dir().unwrap_or_default()).unwrap_or_default();
    log::warn!("plots: writing to {}/{}", abs.display(), PLOT_FOLDER);
    let plots = [
        "cn0",
        "iq-scatter",
        "doppler-hz",
        "code-phase-offset",
        "phi-error",
        "amplitude",
        "phase-angle",
        "nav-msg",
    ];

    let mut svs: Vec<SV> = Vec::new();
    for prn in 1u8..=32 {
        svs.push(SV::new(Constellation::GPS, prn));
    }
    for prn in 1u8..=36 {
        svs.push(SV::new(Constellation::Galileo, prn));
    }
    for prn in 120u8..=138 {
        svs.push(SV::new(Constellation::SBAS, prn));
    }

    let mut html = String::from(
        "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n\
         <meta charset=\"utf-8\">\n\
         <meta http-equiv=\"refresh\" content=\"4\">\n\
         <title>GNSS diagnostic</title>\n\
         <style>body{font-family:sans-serif;background:#111;color:#ccc}\
         .row{display:flex;flex-wrap:wrap;align-items:flex-start;margin-bottom:6px}\
         .label{width:40px;font-size:11px;color:#888;padding-top:8px}\
         img{margin:2px;background:#222}</style>\n\
         </head>\n<body>\n",
    );

    for sv in &svs {
        let slug = sv_slug(*sv);
        html.push_str(&format!(
            "<div class=\"row\"><span class=\"label\">{sv}</span>\n"
        ));
        for p in &plots {
            html.push_str(&format!(
                "  <img onerror=\"this.style.display='none'\" title=\"{sv} {p}\" src=\"{slug}-{p}.png\" />\n"
            ));
        }
        html.push_str("</div>\n");
    }

    html.push_str("</body>\n</html>\n");

    let _ = std::fs::create_dir_all(PLOT_FOLDER);
    std::fs::write(format!("{PLOT_FOLDER}/index.html"), html)
        .unwrap_or_else(|e| log::error!("plot HTML write: {e}"));
}

fn plot_time_graph(
    sv: SV,
    name: &str,
    series: &[f64],
    y_delta: f64,
    color: &RGBColor,
    x_step: f64,
) {
    plot_time_graph_with_sz(PlotOpts {
        sv,
        name,
        series,
        y_delta,
        color,
        x_step,
        size_x: PLOT_W,
        size_y: PLOT_H,
    });
}

struct PlotOpts<'a> {
    sv: SV,
    name: &'a str,
    series: &'a [f64],
    y_delta: f64,
    color: &'a RGBColor,
    x_step: f64,
    size_x: u32,
    size_y: u32,
}

fn plot_time_graph_with_sz(opts: PlotOpts<'_>) {
    let PlotOpts {
        sv,
        name,
        series,
        y_delta,
        color,
        x_step,
        size_x,
        size_y,
    } = opts;
    if series.len() < 10 {
        return;
    }

    let file_name = format!("{}/{}-{}.png", PLOT_FOLDER, sv_slug(sv), name);
    let root = BitMapBackend::new(&file_name, (size_x, size_y)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let x_max = series.len() as f64 * x_step;
    let y_max = series.iter().cloned().fold(f64::MIN, f64::max) + y_delta;
    let y_min = series.iter().cloned().fold(f64::MAX, f64::min) - y_delta;

    let mut ctx = ChartBuilder::on(&root)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 20)
        .caption(format!("{sv}: {name}"), ("sans-serif", PLOT_FONT_SIZE))
        .build_cartesian_2d(0.0..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        series
            .iter()
            .enumerate()
            .map(|(i, v)| Circle::new((i as f64 * x_step, *v), 1, color)),
    )
    .unwrap();
}

/// Render all per-channel diagnostic plots for `sv` from its rolling History.
/// `code_sec` is the code period (1 ms for L1 C/A, 4 ms for E1-B) and sets
/// the true time axis so Galileo plots are not 4× compressed.
pub fn plot_channel(sv: SV, hist: &History, code_sec: f64) {
    // C/N₀ over time.
    let cn0: Vec<f64> = hist.cn0.iter().copied().collect();
    plot_time_graph(sv, "cn0", &cn0, 1.0, &RED, code_sec);

    // Prompt correlator magnitude — signal-level variation and multipath.
    let amp: Vec<f64> = hist.corr_p.iter().map(|c| c.norm()).collect();
    plot_time_graph(sv, "amplitude", &amp, 0.0, &BLUE, code_sec);

    // Prompt phase angle — clustering reveals lock quality and false locks.
    let phase: Vec<f64> = hist.corr_p.iter().map(|c| c.arg().to_degrees()).collect();
    plot_time_graph(sv, "phase-angle", &phase, 1.0, &BLACK, code_sec);

    // Code phase offset (samples).
    let code_phase: Vec<f64> = hist.code_phase_offset.iter().copied().collect();
    plot_time_graph(sv, "code-phase-offset", &code_phase, 50.0, &BLUE, code_sec);

    // PLL phase error (radians).
    let phi_err: Vec<f64> = hist.phi_error.iter().copied().collect();
    plot_time_graph(sv, "phi-error", &phi_err, 0.5, &BLACK, code_sec);

    // Doppler (Hz).
    let doppler: Vec<f64> = hist.doppler_hz.iter().copied().collect();
    plot_time_graph(sv, "doppler-hz", &doppler, 10.0, &BLACK, code_sec);

    // Nav-message: prompt I over time — wideband to show the bit transitions.
    let nav: Vec<f64> = hist.corr_p.iter().map(|c| c.re).collect();
    plot_time_graph_with_sz(PlotOpts {
        sv,
        name: "nav-msg",
        series: &nav,
        y_delta: 0.001,
        color: &BLACK,
        x_step: code_sec,
        size_x: PLOT_W_NAV,
        size_y: PLOT_H_NAV,
    });

    // IQ scatter: last 2000 prompt samples.
    let n = usize::min(hist.corr_p.len(), 2000);
    let iq: Vec<Complex64> = hist.corr_p.iter().rev().take(n).copied().collect();
    plot_iq_scatter(sv, &iq);
}

pub fn plot_iq_scatter(sv: SV, series: &[Complex64]) {
    if series.len() < 10 {
        return;
    }

    let file_name = format!("{}/{}-iq-scatter.png", PLOT_FOLDER, sv_slug(sv));
    let root = BitMapBackend::new(&file_name, (PLOT_W_SCATTER, PLOT_H_SCATTER)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let factor = 1000.0;
    let x_max = series.iter().map(|c| c.re.abs()).fold(0.0_f64, f64::max) * factor * 1.4;
    let y_max = series.iter().map(|c| c.im.abs()).fold(0.0_f64, f64::max) * factor * 1.4;
    let lim = f64::max(x_max, y_max).max(1.0);

    let mut ctx = ChartBuilder::on(&root)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 20)
        .caption(format!("{sv}: iq-scatter"), ("sans-serif", PLOT_FONT_SIZE))
        .build_cartesian_2d(-lim..lim, -lim..lim)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        series
            .iter()
            .map(|c| Circle::new((c.re * factor, c.im * factor), 1, RED)),
    )
    .unwrap();
}
