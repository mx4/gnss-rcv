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
use crate::state::GnssState;

const PI: f64 = std::f64::consts::PI;

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

pub struct GnssRcvApp {
    iq_file: String,
    iq_file_choice: usize,
    iq_type_choice: usize,
    sig_choice: usize,
    needs_stop: Arc<AtomicBool>,
    active: Arc<AtomicBool>,
    pub_state: Arc<Mutex<GnssState>>,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        Self {
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            iq_file_choice: 0,
            iq_type_choice: 0,
            sig_choice: 0,
            active: Arc::new(AtomicBool::new(false)),
            needs_stop: Arc::new(AtomicBool::new(false)),
            pub_state: Arc::new(Mutex::new(GnssState::new())),
        }
    }
}

fn async_receive(
    active: Arc<AtomicBool>,
    needs_stop: Arc<AtomicBool>,
    file: PathBuf,
    iq_file_type: IQFileType,
    sig: Signal,
    pub_state: Arc<Mutex<GnssState>>,
) {
    log::info!("start_receiving");

    active.store(true, Ordering::SeqCst);

    let config = ReceiverConfig {
        file,
        iq_file_type,
        sig,
        ..Default::default()
    };
    let receiver = Receiver::new(&config, needs_stop.clone(), pub_state);

    match receiver {
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

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
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
        let iq_file = self.iq_file.clone();

        self.pub_state = Arc::new(Mutex::new(GnssState::new()));
        let pub_state = self.pub_state.clone();
        let sig = Signal::L1ca;
        let ctx_clone = ctx.clone();
        let iq_file_type = if self.iq_file_choice == 0 {
            IQFileType::TypePairFloat32
        } else {
            IQFileType::TypePairInt16
        };

        let update_func = move || {
            ctx_clone.request_repaint_after_secs(0.05);
        };

        self.pub_state
            .lock()
            .unwrap()
            .set_update_func(Box::new(update_func.clone()));

        thread::spawn(move || {
            log::info!("thread_start");
            async_receive(
                active,
                needs_stop,
                iq_file.into(),
                iq_file_type,
                sig,
                pub_state,
            );
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
    fn update_iq_type(&mut self, ui: &mut egui::Ui) {
        let type_str = ["2xf32", "2xi16"];
        egui::ComboBox::from_label("iq-format")
            .width(30.0)
            .selected_text(type_str[self.iq_type_choice])
            .show_ui(ui, |ui| {
                for (i, s) in type_str.iter().enumerate() {
                    let value = ui.selectable_value(&mut self.iq_type_choice, i, s.to_string());
                    if value.clicked() {
                        self.iq_type_choice = i;
                    }
                }
            });
    }
    fn update_sig_type(&mut self, ui: &mut egui::Ui) {
        let sig_str = ["L1CA"];

        egui::ComboBox::from_label("signal")
            .width(30.0)
            .selected_text(sig_str[self.sig_choice])
            .show_ui(ui, |ui| {
                for (i, s) in sig_str.iter().enumerate() {
                    let value = ui.selectable_value(&mut self.iq_type_choice, i, s.to_string());
                    if value.clicked() {
                        self.sig_choice = i;
                    }
                }
            });
    }
    fn update_start_stop(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let button_text = if self.active.load(Ordering::SeqCst) {
            "stop"
        } else {
            "start"
        };
        if ui
            //  .add_sized([150.0, 25.], egui::Button::new(button_text.to_owned()))
            .add_sized(
                ui.available_size(),
                egui::Button::new(button_text.to_owned()),
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
        let vec_str = [
            "nov_3_time_18_48_st_ives",
            "gpssim_2xi16",
            "L1_20211202_084700_4MHz_IQ.bin",
            "GPS-L1-2022-03-27.sigmf-data",
        ];

        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .min_height(25.0)
            .show(ctx, |ui| {
                egui::Grid::new("TopGrid").show(ui, |ui| {
                    egui::ComboBox::from_label("Pick file")
                        .width(230.0)
                        .selected_text(vec_str[self.iq_file_choice])
                        .show_ui(ui, |ui| {
                            for (i, s) in vec_str.iter().enumerate() {
                                let value =
                                    ui.selectable_value(&mut self.iq_file_choice, i, s.to_string());
                                if value.clicked() {
                                    self.iq_file_choice = i;
                                    self.iq_file = format!("resources/{}", vec_str[i]);
                                }
                            }
                        });
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut self.iq_file)
                                .desired_width(f32::INFINITY)
                                .clip_text(false),
                        );
                    });
                    ui.horizontal(|ui| {
                        self.update_iq_type(ui);
                    });
                    ui.horizontal(|ui| {
                        self.update_sig_type(ui);
                    });
                    ui.end_row();
                    self.update_start_stop(ui, ctx);
                });
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
                    ui.strong("other");
                });
            })
            .body(|mut body| {
                for row_index in 1..=32 {
                    let row_height = 20.0;
                    let sv = SV::new(Constellation::GPS, row_index);
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
                            ui.label("".to_string());
                        });
                    });
                }
            });
    }
}
