use glob::glob;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;

use crate::channel::History;

// ── Dimensions ────────────────────────────────────────────────────────────────
const FONT: u32 = 14;
const PLOT_W: u32 = 360;
const PLOT_H: u32 = 200; // unified height — all charts the same
const PLOT_W_SCATTER: u32 = 200; // square at PLOT_H
const PLOT_W_NAV: u32 = 480;
const PLOT_FOLDER: &str = "plots";

// ── Dark-theme palette ────────────────────────────────────────────────────────
const BG: RGBColor = RGBColor(15, 17, 26);
const GRID_BOLD: RGBColor = RGBColor(38, 45, 68);
const GRID_LIGHT: RGBColor = RGBColor(24, 29, 46);
const AXIS_COL: RGBColor = RGBColor(65, 80, 120);
const CAPTION_COL: RGBColor = RGBColor(155, 170, 200);

// ── Series colors ─────────────────────────────────────────────────────────────
const COL_CN0: RGBColor = RGBColor(70, 195, 90);
const COL_AMP: RGBColor = RGBColor(55, 175, 200);
const COL_PHASE: RGBColor = RGBColor(205, 175, 45);
const COL_CODE: RGBColor = RGBColor(95, 140, 255);
const COL_PHI: RGBColor = RGBColor(220, 115, 45);
const COL_DOPP: RGBColor = RGBColor(175, 95, 230);
const COL_NAV: RGBColor = RGBColor(150, 165, 190);
const COL_IQ: RGBColor = RGBColor(70, 195, 90);

// ── Helpers ───────────────────────────────────────────────────────────────────

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

// ── HTML generation ───────────────────────────────────────────────────────────

pub fn plot_generate_html() {
    let abs =
        std::fs::canonicalize(std::env::current_dir().unwrap_or_default()).unwrap_or_default();
    log::warn!("plots: writing to {}/{}", abs.display(), PLOT_FOLDER);

    let plot_names = [
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

    let mut html = String::from(concat!(
        "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n",
        "<meta charset=\"utf-8\">\n",
        "<meta http-equiv=\"refresh\" content=\"4\">\n",
        "<title>gnss-rcv diagnostics</title>\n",
        "<style>\n",
        "  *, *::before, *::after { box-sizing: border-box; }\n",
        "  body { margin: 0; padding: 16px 20px;\n",
        "         font-family: system-ui, -apple-system, sans-serif;\n",
        "         background: #080a12; color: #b0bcd4; }\n",
        "  h1 { font-size: 11px; font-weight: 400; color: #3a4460;\n",
        "       letter-spacing: .14em; text-transform: uppercase; margin: 0 0 14px; }\n",
        "  .sv { display: flex; align-items: flex-start; gap: 10px;\n",
        "        background: #0e1020; border-radius: 6px;\n",
        "        padding: 10px 12px 10px 10px; margin-bottom: 6px;\n",
        "        border-left: 3px solid #2a3460; }\n",
        "  .sv-label { min-width: 32px; font-size: 12px; font-weight: 700;\n",
        "              color: #6878b8; padding-top: 4px; flex-shrink: 0;\n",
        "              letter-spacing: .04em; }\n",
        "  .charts { display: flex; flex-wrap: wrap; gap: 3px; }\n",
        "  img { display: block; border-radius: 2px; }\n",
        "</style>\n",
        "</head>\n<body>\n",
        "<h1>gnss-rcv &mdash; live diagnostics</h1>\n",
    ));

    for sv in &svs {
        let slug = sv_slug(*sv);
        html.push_str(&format!("<div class=\"sv\" id=\"sv-{slug}\">\n"));
        html.push_str(&format!("  <span class=\"sv-label\">{sv}</span>\n"));
        html.push_str("  <div class=\"charts\">\n");
        for p in &plot_names {
            html.push_str(&format!(
                "    <img src=\"{slug}-{p}.png\" title=\"{sv} {p}\" \
                 onerror=\"this.style.display='none';checkSv('{slug}')\">\n"
            ));
        }
        html.push_str("  </div>\n</div>\n");
    }

    // Hide SV rows where every chart failed to load.
    html.push_str(concat!(
        "<script>\n",
        "function checkSv(slug) {\n",
        "  var b = document.getElementById('sv-' + slug);\n",
        "  if (!b) return;\n",
        "  var imgs = b.querySelectorAll('img');\n",
        "  if ([...imgs].every(i => i.style.display === 'none')) b.style.display = 'none';\n",
        "}\n",
        "</script>\n",
        "</body>\n</html>\n",
    ));

    let _ = std::fs::create_dir_all(PLOT_FOLDER);
    std::fs::write(format!("{PLOT_FOLDER}/index.html"), html)
        .unwrap_or_else(|e| log::error!("plot HTML write: {e}"));
}

// ── Chart rendering ───────────────────────────────────────────────────────────

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
    root.fill(&BG).unwrap();

    let x_max = series.len() as f64 * x_step;
    let y_max = series.iter().cloned().fold(f64::MIN, f64::max) + y_delta;
    let y_min = series.iter().cloned().fold(f64::MAX, f64::min) - y_delta;

    let mut ctx = ChartBuilder::on(&root)
        .set_label_area_size(LabelAreaPosition::Left, 44)
        .set_label_area_size(LabelAreaPosition::Bottom, 22)
        .margin(6)
        .caption(format!("{sv}: {name}"), ("sans-serif", FONT, &CAPTION_COL))
        .build_cartesian_2d(0.0..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh()
        .bold_line_style(GRID_BOLD)
        .light_line_style(GRID_LIGHT)
        .axis_style(AXIS_COL)
        .draw()
        .unwrap();

    ctx.draw_series(
        series
            .iter()
            .enumerate()
            .map(|(i, v)| Circle::new((i as f64 * x_step, *v), 1, color)),
    )
    .unwrap();
}

// ── Public entry point ────────────────────────────────────────────────────────

pub fn plot_channel(sv: SV, hist: &History, code_sec: f64) {
    let cn0: Vec<f64> = hist.cn0.iter().copied().collect();
    plot_time_graph(sv, "cn0", &cn0, 1.0, &COL_CN0, code_sec);

    let amp: Vec<f64> = hist.corr_p.iter().map(|c| c.norm()).collect();
    plot_time_graph(sv, "amplitude", &amp, 0.0, &COL_AMP, code_sec);

    let phase: Vec<f64> = hist.corr_p.iter().map(|c| c.arg().to_degrees()).collect();
    plot_time_graph(sv, "phase-angle", &phase, 1.0, &COL_PHASE, code_sec);

    let code_phase: Vec<f64> = hist.code_phase_offset.iter().copied().collect();
    plot_time_graph(
        sv,
        "code-phase-offset",
        &code_phase,
        50.0,
        &COL_CODE,
        code_sec,
    );

    let phi_err: Vec<f64> = hist.phi_error.iter().copied().collect();
    plot_time_graph(sv, "phi-error", &phi_err, 0.5, &COL_PHI, code_sec);

    let doppler: Vec<f64> = hist.doppler_hz.iter().copied().collect();
    plot_time_graph(sv, "doppler-hz", &doppler, 10.0, &COL_DOPP, code_sec);

    let nav: Vec<f64> = hist.corr_p.iter().map(|c| c.re).collect();
    plot_time_graph_with_sz(PlotOpts {
        sv,
        name: "nav-msg",
        series: &nav,
        y_delta: 0.001,
        color: &COL_NAV,
        x_step: code_sec,
        size_x: PLOT_W_NAV,
        size_y: PLOT_H,
    });

    let n = usize::min(hist.corr_p.len(), 2000);
    let iq: Vec<Complex64> = hist.corr_p.iter().rev().take(n).copied().collect();
    plot_iq_scatter(sv, &iq);
}

pub fn plot_iq_scatter(sv: SV, series: &[Complex64]) {
    if series.len() < 10 {
        return;
    }

    let file_name = format!("{}/{}-iq-scatter.png", PLOT_FOLDER, sv_slug(sv));
    let root = BitMapBackend::new(&file_name, (PLOT_W_SCATTER, PLOT_H)).into_drawing_area();
    root.fill(&BG).unwrap();

    let factor = 1000.0;
    let x_max = series.iter().map(|c| c.re.abs()).fold(0.0_f64, f64::max) * factor * 1.4;
    let y_max = series.iter().map(|c| c.im.abs()).fold(0.0_f64, f64::max) * factor * 1.4;
    let lim = f64::max(x_max, y_max).max(1.0);

    let mut ctx = ChartBuilder::on(&root)
        .set_label_area_size(LabelAreaPosition::Left, 44)
        .set_label_area_size(LabelAreaPosition::Bottom, 22)
        .margin(6)
        .caption(
            format!("{sv}: iq-scatter"),
            ("sans-serif", FONT, &CAPTION_COL),
        )
        .build_cartesian_2d(-lim..lim, -lim..lim)
        .unwrap();

    ctx.configure_mesh()
        .bold_line_style(GRID_BOLD)
        .light_line_style(GRID_LIGHT)
        .axis_style(AXIS_COL)
        .draw()
        .unwrap();

    ctx.draw_series(
        series
            .iter()
            .map(|c| Circle::new((c.re * factor, c.im * factor), 1, COL_IQ)),
    )
    .unwrap();
}
