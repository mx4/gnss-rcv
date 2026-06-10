use egui_extras::{Column, TableBuilder};
use egui_extras::{Size, StripBuilder};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::thread;

use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;

use crate::channel::{History, State};
use crate::code::Signal;
use crate::receiver::{Receiver, ReceiverConfig};
use crate::recording::IQFileType;
use crate::recordings::{self, Recording};
use crate::state::GnssState;

const PI: f64 = std::f64::consts::PI;

const WIDTH: usize = 900;
const HEIGHT: usize = 700;

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Dashboard,
    Diagnostics,
}

pub struct GnssRcvApp {
    recordings: Vec<Recording>,
    selected: usize,
    iq_file: String,
    iq_file_type: IQFileType,
    fs: f64,
    fi: f64,
    sig: Signal,
    osnma: bool,
    plots: bool,
    needs_stop: Arc<AtomicBool>,
    active: Arc<AtomicBool>,
    pub_state: Arc<Mutex<GnssState>>,
    tab: Tab,
    diag_sv: Option<SV>,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        let mut app = Self {
            recordings: recordings::load_provisioned(),
            selected: 0,
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            iq_file_type: IQFileType::TypePairFloat32,
            fs: 2_046_000.0,
            fi: 0.0,
            sig: Signal::L1ca,
            osnma: true,
            plots: false,
            active: Arc::new(AtomicBool::new(false)),
            needs_stop: Arc::new(AtomicBool::new(false)),
            pub_state: Arc::new(Mutex::new(GnssState::new())),
            tab: Tab::Dashboard,
            diag_sv: None,
        };
        let default_idx = app
            .recordings
            .iter()
            .position(|r| r.name == "nov3")
            .unwrap_or(0);
        if !app.recordings.is_empty() {
            app.apply_recording(default_idx);
        }
        app
    }
}

fn async_receive(
    active: Arc<AtomicBool>,
    needs_stop: Arc<AtomicBool>,
    config: ReceiverConfig,
    pub_state: Arc<Mutex<GnssState>>,
) {
    log::info!("start_receiving");

    active.store(true, Ordering::SeqCst);

    match Receiver::new(&config, needs_stop.clone(), pub_state) {
        Ok(mut receiver) => {
            log::info!("run_loop");
            receiver.run_loop(0);
        }
        Err(e) => log::error!("cannot start receiver for {}: {e}", config.file.display()),
    }

    active.store(false, Ordering::SeqCst);
    log::info!("start_receiving: done");
}

fn sig_label(sig: Signal) -> &'static str {
    match sig {
        Signal::L1ca => "L1CA",
        Signal::GalileoE1b => "E1B",
        Signal::GalileoE1c => "E1C",
    }
}

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>, plots: bool) -> Self {
        Self {
            plots,
            ..Default::default()
        }
    }

    fn apply_recording(&mut self, i: usize) {
        self.selected = i;
        self.iq_file = self.recordings[i].path.to_string_lossy().into_owned();
        self.iq_file_type = self.recordings[i].iq_file_type.clone();
        self.fs = self.recordings[i].fs;
        self.fi = self.recordings[i].fi;
        self.sig = self.recordings[i].sig;
    }

    fn stop_async(&mut self) {
        self.needs_stop.store(true, Ordering::SeqCst);
        log::info!("stop_async");
    }

    fn start_async(&mut self, ctx: &egui::Context) {
        log::info!("start_async");
        self.needs_stop.store(false, Ordering::SeqCst);

        let active = self.active.clone();
        let needs_stop = self.needs_stop.clone();

        self.pub_state = Arc::new(Mutex::new(GnssState::new()));
        let pub_state = self.pub_state.clone();
        let ctx_clone = ctx.clone();

        let update_func = move || {
            ctx_clone.request_repaint_after_secs(0.05);
        };
        self.pub_state
            .lock()
            .unwrap()
            .set_update_func(Box::new(update_func.clone()));

        let config = ReceiverConfig {
            file: PathBuf::from(&self.iq_file),
            iq_file_type: self.iq_file_type.clone(),
            fs: self.fs,
            fi: self.fi,
            sig: self.sig,
            osnma: self.osnma,
            plots: self.plots,
            ..Default::default()
        };

        thread::spawn(move || {
            log::info!("thread_start");
            async_receive(active, needs_stop, config, pub_state);
            log::info!("thread_stop");
        });
    }
}

pub fn egui_main(plots: bool) {
    log::warn!("egui_main");
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([WIDTH as f32, HEIGHT as f32]),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "gnss-rcv",
        native_options,
        Box::new(move |cc| Ok(Box::new(GnssRcvApp::new(cc, plots)))),
    )
    .unwrap();
}

impl eframe::App for GnssRcvApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.update_top(ctx);
        self.update_central(ctx);
    }
}

impl GnssRcvApp {
    fn update_file_picker(&mut self, ui: &mut egui::Ui) {
        let selected_text = self
            .recordings
            .get(self.selected)
            .map(|r| r.name.as_str())
            .unwrap_or("(no provisioned recordings)")
            .to_owned();
        let mut clicked = None;
        egui::ComboBox::from_id_salt("file_picker")
            .width(230.0)
            .selected_text(selected_text)
            .show_ui(ui, |ui| {
                for (i, r) in self.recordings.iter().enumerate() {
                    let mut resp = ui.selectable_label(self.selected == i, r.name.as_str());
                    if !r.note.is_empty() {
                        resp = resp.on_hover_text(r.note.as_str());
                    }
                    if resp.clicked() {
                        clicked = Some(i);
                    }
                }
            });
        if let Some(i) = clicked {
            self.apply_recording(i);
        }
    }

    fn update_iq_type(&mut self, ui: &mut egui::Ui) {
        egui::ComboBox::from_label("iq-format")
            .selected_text(self.iq_file_type.to_string())
            .show_ui(ui, |ui| {
                for t in [
                    IQFileType::TypePairFloat32,
                    IQFileType::TypePairInt16,
                    IQFileType::TypePairInt8,
                    IQFileType::TypeOneInt8,
                    IQFileType::TypeOne4Bit,
                    IQFileType::TypeOneBit,
                    IQFileType::TypeRtlSdrFile,
                ] {
                    let label = t.to_string();
                    ui.selectable_value(&mut self.iq_file_type, t, label);
                }
            });
    }

    fn update_sig_type(&mut self, ui: &mut egui::Ui) {
        egui::ComboBox::from_label("signal")
            .selected_text(sig_label(self.sig))
            .show_ui(ui, |ui| {
                for s in [Signal::L1ca, Signal::GalileoE1b, Signal::GalileoE1c] {
                    ui.selectable_value(&mut self.sig, s, sig_label(s));
                }
            });
    }

    fn update_freqs(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("fs");
            ui.add(
                egui::DragValue::new(&mut self.fs)
                    .speed(1000.0)
                    .suffix(" Hz"),
            );
            ui.label("fi");
            ui.add(
                egui::DragValue::new(&mut self.fi)
                    .speed(1000.0)
                    .suffix(" Hz"),
            );
        });
    }

    fn update_start_stop(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let button_text = if self.active.load(Ordering::SeqCst) {
            "stop"
        } else {
            "start"
        };
        let btn_color = ui.visuals().selection.bg_fill;
        let h = ui.spacing().interact_size.y;
        if ui
            .add_sized(
                [ui.available_width(), h],
                egui::Button::new(button_text).fill(btn_color),
            )
            .clicked()
        {
            if self.active.load(Ordering::SeqCst) {
                self.stop_async();
            } else {
                self.start_async(ctx);
            }
        }
    }

    fn update_top(&mut self, ctx: &egui::Context) {
        let (sv_elaz, tow_text, almanac_n, has_ion, has_utc, pos_text, pos_url) = {
            let st = self.pub_state.lock().unwrap();
            let mut sv_elaz: Vec<(SV, f64, f64)> = st
                .channels
                .iter()
                .filter(|(_, cs)| cs.state == State::Tracking && cs.elevation_deg != 0.0)
                .map(|(sv, cs)| (*sv, cs.elevation_deg, cs.azimuth_deg))
                .collect();
            sv_elaz.sort_by_key(|t| t.0);
            let tow_text = format!("{:?}", st.tow_gpst);
            let almanac_n = st.almanac.iter().filter(|a| a.sat != 0).count();
            let (pos_text, pos_url) = if st.longitude != 0.0 {
                (
                    format!(
                        "lat={:.4}  lon={:.4}  alt={:.1} m",
                        st.latitude, st.longitude, st.height
                    ),
                    Some(format!(
                        "https://maps.google.com/?ll={},{}",
                        st.latitude, st.longitude
                    )),
                )
            } else {
                ("no position fix".to_string(), None)
            };
            (
                sv_elaz, tow_text, almanac_n, st.ion_adj, st.utc_adj, pos_text, pos_url,
            )
        };

        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // Left column: all controls + status box.
                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            self.update_file_picker(ui);
                            let h = ui.spacing().interact_size.y;
                            ui.add_sized(
                                [ui.available_width(), h],
                                egui::TextEdit::singleline(&mut self.iq_file),
                            );
                        });
                        ui.horizontal(|ui| {
                            self.update_iq_type(ui);
                            self.update_sig_type(ui);
                            if self.sig.is_boc11() {
                                ui.checkbox(&mut self.osnma, "OSNMA");
                            }
                        });
                        self.update_freqs(ui);
                        egui::Frame::group(ui.style()).show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.monospace(&tow_text);
                                ui.separator();
                                ui.monospace(format!("almanac: {almanac_n}"));
                                if has_ion {
                                    ui.separator();
                                    ui.monospace("ion: 1");
                                }
                                if has_utc {
                                    ui.separator();
                                    ui.monospace("utc: 1");
                                }
                            });
                            if let Some(url) = &pos_url {
                                ui.hyperlink_to(&pos_text, url);
                            } else {
                                ui.monospace(&pos_text);
                            }
                        });
                    });
                    // Right column: sky plot, pinned to the right edge.
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::TOP), |ui| {
                        draw_sky_plot(ui, &sv_elaz);
                    });
                });
            });
    }

    fn update_central(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            self.update_start_stop(ui, ctx);
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.tab, Tab::Dashboard, "Dashboard");
                ui.selectable_value(&mut self.tab, Tab::Diagnostics, "Diagnostics");
            });
            ui.separator();
            match self.tab {
                Tab::Dashboard => {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        StripBuilder::new(ui)
                            .size(Size::remainder().at_least(100.0))
                            .vertical(|mut strip| {
                                strip.cell(|ui| {
                                    egui::ScrollArea::horizontal().show(ui, |ui| {
                                        self.table_ui(ui);
                                    });
                                });
                            });
                    });
                }
                Tab::Diagnostics => {
                    self.update_diagnostics(ui);
                }
            }
        });
    }

    fn update_diagnostics(&mut self, ui: &mut egui::Ui) {
        let mut tracked_svs: Vec<(SV, f64)> = {
            let st = self.pub_state.lock().unwrap();
            st.channels
                .iter()
                .filter(|(_, cs)| cs.state == State::Tracking)
                .map(|(sv, cs)| (*sv, cs.cn0))
                .collect()
        };
        tracked_svs.sort_by_key(|(sv, _)| *sv);

        egui::SidePanel::left("diag_sv_list")
            .resizable(false)
            .exact_width(58.0)
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for (sv, cn0) in &tracked_svs {
                        let color = if *cn0 >= 40.0 {
                            egui::Color32::from_rgb(80, 200, 100)
                        } else if *cn0 >= 35.0 {
                            egui::Color32::from_rgb(220, 190, 50)
                        } else {
                            egui::Color32::from_rgb(220, 80, 60)
                        };
                        let label = egui::RichText::new(format!("{sv}")).color(color);
                        if ui
                            .selectable_label(self.diag_sv == Some(*sv), label)
                            .clicked()
                        {
                            self.diag_sv = Some(*sv);
                        }
                    }
                });
            });

        egui::CentralPanel::default().show_inside(ui, |ui| {
            let selected = self.diag_sv;
            if let Some(sv) = selected {
                let hist = {
                    let st = self.pub_state.lock().unwrap();
                    st.histories.get(&sv).cloned()
                };
                if let Some(hist) = hist {
                    draw_diagnostics_charts(ui, sv, &hist);
                } else {
                    ui.label("No data yet — waiting for first history snapshot (2 s).");
                }
            } else {
                ui.label("Select a satellite from the list.");
            }
        });
    }

    fn table_ui(&mut self, ui: &mut egui::Ui) {
        let available_height = ui.available_height();
        let is_galileo = self.sig.is_boc11();

        // Make the alternating stripe visible on a dark background.
        ui.visuals_mut().faint_bg_color = egui::Color32::from_rgba_premultiplied(30, 36, 58, 220);

        let tb = TableBuilder::new(ui)
            .resizable(true)
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::auto())
            .column(Column::auto().at_least(30.0).resizable(true))
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::auto()) // ephemeris
            .min_scrolled_height(0.0)
            .max_scroll_height(available_height);
        let tb = if is_galileo {
            tb.column(Column::remainder())
        } else {
            tb
        };

        let (constellation, max_prn) = if is_galileo {
            (Constellation::Galileo, 36)
        } else {
            (Constellation::GPS, 32)
        };

        tb.header(20.0, |mut header| {
            header.col(|ui| {
                ui.strong("SV");
            });
            header.col(|ui| {
                ui.strong("dB-Hz");
            });
            header.col(|ui| {
                ui.strong("doppler");
            });
            header.col(|ui| {
                ui.strong("code_idx");
            });
            header.col(|ui| {
                ui.strong("phi");
            });
            header.col(|ui| {
                ui.strong("ephemeris");
            });
            if is_galileo {
                header.col(|ui| {
                    ui.strong("osnma");
                });
            }
        })
        .body(|mut body| {
            for row_index in 1..=max_prn {
                let row_height = 20.0;
                let sv = SV::new(constellation, row_index);
                let pub_state = self.pub_state.lock().unwrap();
                let channel = pub_state.channels.get(&sv);

                if channel.is_none() {
                    continue;
                }
                let state = channel.unwrap().state.clone();
                if state != State::Tracking {
                    continue;
                }
                let cn0 = channel.unwrap().cn0;
                let phi = (channel.unwrap().phi % 1.0) * 2.0 * PI;
                let doppler_hz = channel.unwrap().doppler_hz;
                let code_idx = channel.unwrap().code_idx;
                let has_eph = channel.unwrap().has_eph;
                let osnma_verified = channel.unwrap().osnma_verified;

                body.row(row_height, |mut row| {
                    row.col(|ui| {
                        ui.label(format!("{}", sv));
                    });
                    row.col(|ui| {
                        let color = if cn0 >= 40.0 {
                            egui::Color32::from_rgb(80, 200, 100)
                        } else if cn0 >= 35.0 {
                            egui::Color32::from_rgb(220, 190, 50)
                        } else {
                            egui::Color32::from_rgb(220, 80, 60)
                        };
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.colored_label(color, format!("{:.1}", cn0));
                        });
                    });
                    row.col(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(format!("{:.0}", doppler_hz));
                        });
                    });
                    row.col(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(format!("{:.0}", code_idx));
                        });
                    });
                    row.col(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(format!("{:.2}", phi));
                        });
                    });
                    row.col(|ui| {
                        if has_eph {
                            ui.label("✓");
                        }
                    });
                    if is_galileo {
                        row.col(|ui| {
                            if osnma_verified {
                                ui.colored_label(egui::Color32::GREEN, "✓ verified");
                            }
                        });
                    }
                });
            }
        });
    }
}

/// Polar sky plot: azimuth around the circle, elevation as radial distance from
/// centre (90° elev = centre, 0° = horizon ring).
fn draw_sky_plot(ui: &mut egui::Ui, sv_elaz: &[(SV, f64, f64)]) {
    let size = 155.0_f32;
    let (response, painter) = ui.allocate_painter(egui::vec2(size, size), egui::Sense::hover());
    let rect = response.rect;
    let c = rect.center();
    let r = size * 0.42;

    // Background
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(15, 15, 30));

    // Concentric elevation rings (horizon, 30°, 60°)
    for (elev, stroke_w) in [(0.0_f64, 1.5_f32), (30.0, 0.6), (60.0, 0.6)] {
        let ring_r = r * (1.0 - elev as f32 / 90.0);
        painter.circle_stroke(
            c,
            ring_r,
            egui::Stroke::new(stroke_w, egui::Color32::from_rgb(60, 70, 90)),
        );
    }

    // Cardinal labels — CENTER_CENTER keeps them within the allocated rect.
    let lo = r + 9.0;
    let font = egui::FontId::proportional(10.0);
    let grey = egui::Color32::from_rgb(130, 140, 160);
    painter.text(
        egui::pos2(c.x, c.y - lo),
        egui::Align2::CENTER_CENTER,
        "N",
        font.clone(),
        grey,
    );
    painter.text(
        egui::pos2(c.x, c.y + lo),
        egui::Align2::CENTER_CENTER,
        "S",
        font.clone(),
        grey,
    );
    painter.text(
        egui::pos2(c.x + lo, c.y),
        egui::Align2::CENTER_CENTER,
        "E",
        font.clone(),
        grey,
    );
    painter.text(
        egui::pos2(c.x - lo, c.y),
        egui::Align2::CENTER_CENTER,
        "W",
        font.clone(),
        grey,
    );

    // SV dots
    for (sv, elev_deg, azim_deg) in sv_elaz {
        let azim_rad = azim_deg.to_radians() as f32;
        let dist = r * (1.0 - *elev_deg as f32 / 90.0);
        let sx = c.x + dist * azim_rad.sin();
        let sy = c.y - dist * azim_rad.cos();
        let pos = egui::pos2(sx, sy);
        painter.circle_filled(pos, 5.0, egui::Color32::from_rgb(80, 200, 100));
        painter.text(
            pos + egui::vec2(6.0, -6.0),
            egui::Align2::LEFT_TOP,
            format!("{sv}"),
            egui::FontId::proportional(8.0),
            egui::Color32::WHITE,
        );
    }
}

/// Live diagnostics charts for one SV's rolling History, drawn with egui Painter.
fn draw_diagnostics_charts(ui: &mut egui::Ui, sv: SV, hist: &History) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        let cn0: Vec<f64> = hist.cn0.iter().copied().collect();
        let doppler: Vec<f64> = hist.doppler_hz.iter().copied().collect();
        let phi: Vec<f64> = hist.phi_error.iter().copied().collect();
        let cpo: Vec<f64> = hist.code_phase_offset.iter().copied().collect();
        let n_iq = usize::min(hist.corr_p.len(), 2000);
        let iq: Vec<(f64, f64)> = hist
            .corr_p
            .iter()
            .rev()
            .take(n_iq)
            .map(|c| (c.re, c.im))
            .collect();

        mini_line_chart(
            ui,
            &format!("{sv} C/N0 dB-Hz"),
            &cn0,
            egui::Color32::from_rgb(80, 200, 100),
            10.0,
        );
        mini_line_chart(
            ui,
            &format!("{sv} Doppler Hz"),
            &doppler,
            egui::Color32::from_rgb(80, 140, 255),
            4000.0,
        );
        mini_line_chart(
            ui,
            &format!("{sv} Phase error rad"),
            &phi,
            egui::Color32::YELLOW,
            0.5,
        );
        mini_line_chart(
            ui,
            &format!("{sv} Code phase offset"),
            &cpo,
            egui::Color32::from_rgb(200, 120, 50),
            200.0,
        );
        iq_scatter_chart(ui, &format!("{sv} IQ scatter"), &iq);
    });
}

/// Line chart with min/max envelope rendering for dense data, label above, grid lines.
/// `min_range` prevents the y-axis from zooming into noise on stable signals.
fn mini_line_chart(
    ui: &mut egui::Ui,
    label: &str,
    data: &[f64],
    color: egui::Color32,
    min_range: f64,
) {
    if data.len() < 2 {
        return;
    }

    ui.add_space(3.0);
    ui.label(
        egui::RichText::new(label)
            .size(10.0)
            .color(egui::Color32::from_rgb(150, 165, 185)),
    );

    let desired = egui::vec2(ui.available_width(), 90.0);
    let (response, painter) = ui.allocate_painter(desired, egui::Sense::hover());
    let rect = response.rect;
    let w = rect.width();
    let h = rect.height();

    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(15, 17, 26));

    // Y range: enforce a minimum span so stable signals don't autoscale into noise.
    let raw_min = data.iter().copied().fold(f64::MAX, f64::min);
    let raw_max = data.iter().copied().fold(f64::MIN, f64::max);
    let span = (raw_max - raw_min).max(min_range);
    let mid = (raw_max + raw_min) / 2.0;
    let y_lo = mid - span / 2.0;
    let y_hi = mid + span / 2.0;

    let to_y = |v: f64| -> f32 {
        (rect.max.y - ((v - y_lo) / (y_hi - y_lo)) as f32 * h).clamp(rect.min.y, rect.max.y)
    };

    // Subtle horizontal grid lines at 25 / 50 / 75 %.
    let grid = egui::Stroke::new(0.5, egui::Color32::from_rgb(32, 37, 52));
    for frac in [0.25_f32, 0.5, 0.75] {
        let y = rect.min.y + h * frac;
        painter.line_segment([egui::pos2(rect.min.x, y), egui::pos2(rect.max.x, y)], grid);
    }

    // Draw: connected polyline when data fits in pixels, min/max envelope when dense.
    let n = data.len();
    let px = w as usize;
    if n <= px {
        let pts: Vec<egui::Pos2> = data
            .iter()
            .enumerate()
            .map(|(i, &v)| egui::pos2(rect.min.x + (i as f32 / (n - 1) as f32) * w, to_y(v)))
            .collect();
        painter.add(egui::Shape::Path(egui::epaint::PathShape::line(
            pts,
            egui::Stroke::new(1.0, color),
        )));
    } else {
        for col in 0..px {
            let s = col * n / px;
            let e = ((col + 1) * n / px).min(n);
            let bucket = &data[s..e];
            let lo = bucket.iter().copied().fold(f64::MAX, f64::min);
            let hi = bucket.iter().copied().fold(f64::MIN, f64::max);
            let x = rect.min.x + col as f32 + 0.5;
            painter.line_segment(
                [egui::pos2(x, to_y(hi)), egui::pos2(x, to_y(lo))],
                egui::Stroke::new(1.0, color),
            );
        }
    }

    // Y-axis min / max in the corners.
    let dim = egui::Color32::from_rgb(85, 95, 115);
    let fnt = egui::FontId::proportional(8.0);
    painter.text(
        egui::pos2(rect.max.x - 2.0, rect.min.y + 2.0),
        egui::Align2::RIGHT_TOP,
        format!("{y_hi:.1}"),
        fnt.clone(),
        dim,
    );
    painter.text(
        egui::pos2(rect.max.x - 2.0, rect.max.y - 2.0),
        egui::Align2::RIGHT_BOTTOM,
        format!("{y_lo:.1}"),
        fnt.clone(),
        dim,
    );
}

/// IQ scatter chart drawn with Painter, fixed 180×180, label above.
fn iq_scatter_chart(ui: &mut egui::Ui, label: &str, data: &[(f64, f64)]) {
    if data.len() < 4 {
        return;
    }

    ui.add_space(3.0);
    ui.label(
        egui::RichText::new(label)
            .size(10.0)
            .color(egui::Color32::from_rgb(150, 165, 185)),
    );

    let sz = 180.0_f32;
    let (response, painter) = ui.allocate_painter(egui::vec2(sz, sz), egui::Sense::hover());
    let rect = response.rect;

    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(15, 17, 26));

    let amp: f64 = data
        .iter()
        .flat_map(|(re, im)| [re.abs(), im.abs()])
        .fold(0.0_f64, f64::max)
        * 1.4;
    let amp = amp.max(1e-12);
    let cx = rect.center().x;
    let cy = rect.center().y;
    let scale = (sz as f64 * 0.45 / amp) as f32;

    let ax = egui::Stroke::new(0.5, egui::Color32::from_rgb(40, 46, 62));
    painter.line_segment([egui::pos2(rect.min.x, cy), egui::pos2(rect.max.x, cy)], ax);
    painter.line_segment([egui::pos2(cx, rect.min.y), egui::pos2(cx, rect.max.y)], ax);

    for (re, im) in data {
        let x = cx + (*re as f32) * scale;
        let y = cy - (*im as f32) * scale;
        painter.circle_filled(egui::pos2(x, y), 1.0, egui::Color32::from_rgb(80, 200, 100));
    }
}
