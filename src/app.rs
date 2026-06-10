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

use crate::channel::State;
use crate::code::Signal;
use crate::receiver::{Receiver, ReceiverConfig};
use crate::recording::IQFileType;
use crate::recordings::{self, Recording};
use crate::state::GnssState;

const PI: f64 = std::f64::consts::PI;

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

pub struct GnssRcvApp {
    /// Recordings present on disk (from `resources/manifest.json`), in pick order.
    recordings: Vec<Recording>,
    selected: usize, // index into `recordings` (meaningful only when non-empty)
    // Run parameters, auto-filled from the selected recording but editable.
    iq_file: String,
    iq_file_type: IQFileType,
    fs: f64,
    fi: f64,
    sig: Signal,
    osnma: bool,
    needs_stop: Arc<AtomicBool>,
    active: Arc<AtomicBool>,
    pub_state: Arc<Mutex<GnssState>>,
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
            osnma: true, // OSNMA on by default (it only acts on an E1B run)
            active: Arc::new(AtomicBool::new(false)),
            needs_stop: Arc::new(AtomicBool::new(false)),
            pub_state: Arc::new(Mutex::new(GnssState::new())),
        };
        if !app.recordings.is_empty() {
            app.apply_recording(0);
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
        // A bad path or unavailable source must not crash the UI; log and stop.
        Err(e) => log::error!("cannot start receiver for {}: {e}", config.file.display()),
    }

    active.store(false, Ordering::SeqCst);
    log::info!("start_receiving: done");
}

/// Short label for the signal dropdown (Galileo `Signal` has no `Display`).
fn sig_label(sig: Signal) -> &'static str {
    match sig {
        Signal::L1ca => "L1CA",
        Signal::GalileoE1b => "E1B",
        Signal::GalileoE1c => "E1C",
    }
}

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
    }

    /// Copy a provisioned recording's path + run parameters into the editable
    /// fields, so picking a file auto-selects its format / fs / fi / signal.
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
            ..Default::default()
        };

        thread::spawn(move || {
            log::info!("thread_start");
            async_receive(active, needs_stop, config, pub_state);
            log::info!("thread_stop");
        });
    }
}

pub fn egui_main() {
    log::warn!("egui_main");
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([WIDTH as f32, HEIGHT as f32]),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "gnss-rcv",
        native_options,
        Box::new(|cc| Ok(Box::new(GnssRcvApp::new(cc)))),
    )
    .unwrap();
}

impl eframe::App for GnssRcvApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.update_top(ctx);
        self.update_mid(ctx);
        self.update_table(ctx);
    }
}

impl GnssRcvApp {
    /// The IQ-file picker: every provisioned recording, picking one auto-fills its
    /// run parameters. Empty when nothing is downloaded — type a path instead.
    fn update_file_picker(&mut self, ui: &mut egui::Ui) {
        let selected_text = self
            .recordings
            .get(self.selected)
            .map(|r| r.name.as_str())
            .unwrap_or("(no provisioned recordings)")
            .to_owned();
        let mut clicked = None;
        egui::ComboBox::from_label("recording")
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
            ui.add(egui::DragValue::new(&mut self.fs).speed(1000.0).suffix(" Hz"));
            ui.label("fi");
            ui.add(egui::DragValue::new(&mut self.fi).speed(1000.0).suffix(" Hz"));
        });
    }

    fn update_start_stop(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let button_text = if self.active.load(Ordering::SeqCst) {
            "stop"
        } else {
            "start"
        };
        if ui
            .add_sized(ui.available_size(), egui::Button::new(button_text.to_owned()))
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
        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .min_height(25.0)
            .show(ctx, |ui| {
                egui::Grid::new("TopGrid").num_columns(3).show(ui, |ui| {
                    self.update_file_picker(ui);
                    ui.add(
                        egui::TextEdit::singleline(&mut self.iq_file)
                            .desired_width(f32::INFINITY)
                            .clip_text(false),
                    );
                    ui.end_row();

                    self.update_iq_type(ui);
                    self.update_sig_type(ui);
                    ui.checkbox(&mut self.osnma, "OSNMA");
                    ui.end_row();

                    self.update_freqs(ui);
                    ui.end_row();
                });
                self.update_start_stop(ui, ctx);
            });
    }

    fn update_mid(&mut self, ctx: &egui::Context) {
        let pub_state = self.pub_state.lock().unwrap();
        egui::TopBottomPanel::top("mid_panel")
            .resizable(true)
            .min_height(50.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new("MidGrid0").show(ui, |ui| {
                        ui.monospace(format!("{:?}", pub_state.tow_gpst).to_string());
                        ui.add(egui::Separator::default().vertical());
                        ui.horizontal(|ui| {
                            let n = pub_state.almanac.iter().filter(|&alm| alm.sat != 0).count();
                            ui.monospace(format!("almanac: {n}").to_string());
                        });

                        if pub_state.ion_adj {
                            ui.horizontal(|ui| {
                                ui.monospace("ion: 1".to_string());
                                ui.add(egui::Separator::default().vertical());
                            });
                        }
                        if pub_state.utc_adj {
                            ui.horizontal(|ui| {
                                ui.monospace("utc: 1".to_string());
                                ui.add(egui::Separator::default().vertical());
                            });
                        }
                        ui.end_row();
                    });
                    egui::Grid::new("MidGrid1").show(ui, |ui| {
                        if pub_state.longitude != 0.0 {
                            let s = format!(
                                "lat={:.3} long={:.3} height={:.1}",
                                pub_state.latitude, pub_state.longitude, pub_state.height
                            );
                            let url = format!(
                                "https://maps.google.com/?ll={},{}",
                                pub_state.latitude, pub_state.longitude
                            );
                            ui.hyperlink_to(s, url.to_string());
                        } else {
                            let s = "no position fix".to_string();
                            ui.monospace(s);
                        };
                    });
                });
            });
    }

    fn update_table(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                StripBuilder::new(ui)
                    .size(Size::remainder().at_least(100.0)) // for the table
                    .vertical(|mut strip| {
                        strip.cell(|ui| {
                            egui::ScrollArea::horizontal().show(ui, |ui| {
                                self.table_ui(ui);
                            });
                        });
                    });
            });
        });
    }
    fn table_ui(&mut self, ui: &mut egui::Ui) {
        let available_height = ui.available_height();
        let table = TableBuilder::new(ui)
            .resizable(true)
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::auto())
            .column(Column::auto().at_least(30.0).resizable(true))
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::remainder())
            .min_scrolled_height(0.0)
            .max_scroll_height(available_height);

        // Show the constellation that matches the selected signal: Galileo (E1B/E1C)
        // tracks PRNs 1..=36, GPS L1 C/A 1..=32.
        let (constellation, max_prn) = if self.sig.is_boc11() {
            (Constellation::Galileo, 36)
        } else {
            (Constellation::GPS, 32)
        };

        table
            .header(20.0, |mut header| {
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
                header.col(|ui| {
                    ui.strong("osnma");
                });
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
                            ui.label(format!("{}", sv).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", cn0).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", doppler_hz).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("{:4.0}", code_idx).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", phi).to_string());
                        });
                        row.col(|ui| {
                            let s = if has_eph { "1" } else { "-" };
                            ui.label(s.to_string());
                        });
                        row.col(|ui| {
                            // OSNMA: green ✓ once this SV's nav data is authenticated.
                            if osnma_verified {
                                ui.colored_label(egui::Color32::GREEN, "✓ verified");
                            }
                        });
                    });
                }
            });
    }
}
