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

const WIDTH: usize = 650;
const HEIGHT: usize = 700;

/// Common width for the format/signal dropdowns and the fs/fi fields, so the two
/// columns of the controls grid line up.
const FIELD_W: f32 = 115.0;

/// Width of the controls-grid label column ("recording" / "iq-format" / "signal").
const LABEL_W: f32 = 72.0;

/// Width of the controls-grid value column (file path / fs / fi) — wider than the
/// dropdown column so the recording path stays legible.
const VALUE_W: f32 = 175.0;

/// Horizontal gap between the controls-grid columns (matches `Grid::spacing`).
const GRID_GAP_X: f32 = 10.0;

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
    /// Enabled signal families (the M6 model: a set, bandwidth-gated). `gps`
    /// is the L1 C/A family (GPS + the SBAS/QZSS blocks); `gal` is Galileo
    /// E1, selectable only when fs carries BOC(1,1) (>= 4.092 MHz).
    gps: bool,
    gal: bool,
    plots: bool,
    /// Also search the SBAS L1 block (PRN 120-138, EGNOS/WAAS GEOs) — same
    /// C/A machinery as GPS; decoded messages show in the log, the GEOs in
    /// the SV table. Seeded from the CLI --sbas flag, toggleable here.
    sbas: bool,
    /// Also search the QZSS L1 block (PRN 193-202).
    qzss: bool,
    needs_stop: Arc<AtomicBool>,
    active: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
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
            gps: true,
            gal: false,
            plots: false,
            sbas: false,
            qzss: false,
            active: Arc::new(AtomicBool::new(false)),
            needs_stop: Arc::new(AtomicBool::new(false)),
            paused: Arc::new(AtomicBool::new(false)),
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
    paused: Arc<AtomicBool>,
    config: ReceiverConfig,
    pub_state: Arc<Mutex<GnssState>>,
) {
    log::info!("start_receiving");

    active.store(true, Ordering::SeqCst);

    match Receiver::new(&config, needs_stop.clone(), pub_state) {
        Ok(mut receiver) => {
            receiver.set_pause_flag(paused);
            log::info!("run_loop");
            receiver.run_loop(0);
        }
        Err(e) => log::error!("cannot start receiver for {}: {e}", config.file.display()),
    }

    active.store(false, Ordering::SeqCst);
    log::info!("start_receiving: done");
}

/// Colour for the SV label in the tracking table, by constellation — the
/// quickest visual cue that a row is a GPS, Galileo, SBAS or QZSS satellite.
fn constellation_color(c: Constellation) -> egui::Color32 {
    match c {
        Constellation::GPS => egui::Color32::from_rgb(170, 200, 255), // pale blue
        Constellation::Galileo => egui::Color32::from_rgb(110, 220, 210), // teal
        Constellation::SBAS => egui::Color32::from_rgb(255, 170, 70), // orange
        Constellation::QZSS => egui::Color32::from_rgb(215, 140, 230), // violet
        _ => egui::Color32::LIGHT_GRAY,
    }
}

/// Hover text identifying an SV — most useful for SBAS PRNs, whose "S1xx"
/// labels say nothing about which augmentation system or GEO bird they are.
/// PRN→GEO assignments come from the SBAS service providers (the well-known,
/// long-lived ones only; unlisted PRNs fall back to the system name).
fn sv_hover_text(sv: SV) -> String {
    let prn = sv.prn;
    match sv.constellation {
        Constellation::SBAS => {
            let system = match prn {
                120 | 121 | 123 | 124 | 126 | 136 => "EGNOS (Europe)",
                131 | 133 | 135 | 138 => "WAAS (North America)",
                129 | 137 => "MSAS (Japan)",
                127 | 128 | 132 => "GAGAN (India)",
                125 | 140 | 141 => "SDCM (Russia)",
                130 | 143 | 144 => "BDSBAS (China)",
                134 => "KASS (Korea)",
                122 => "SouthPAN (Australia/NZ)",
                _ => "SBAS",
            };
            let geo = match prn {
                120 => Some("Inmarsat 3-F2 / AOR-E"),
                121 => Some("Eutelsat 5WB"),
                123 => Some("ASTRA 5B"),
                124 => Some("Artemis"),
                126 => Some("Inmarsat 4-F2 / EMEA"),
                131 => Some("Eutelsat 117WB"),
                133 => Some("SES-15"),
                135 => Some("Galaxy 30"),
                138 => Some("ANIK F1R"),
                _ => None,
            };
            match geo {
                Some(g) => format!("{system} — {g}"),
                None => system.to_string(),
            }
        }
        Constellation::QZSS => "QZSS (Japan) — Michibiki".to_string(),
        c => format!("{c}"),
    }
}

/// Short SBAS system name for a GEO PRN, so the corrections flag names the
/// actual augmentation system (the source could be WAAS, MSAS, … not just the
/// European EGNOS). Mirrors the PRN→system map in `sv_hover_text`; unmapped PRNs
/// fall back to the generic "SBAS".
fn sbas_system_short(sv: SV) -> &'static str {
    match sv.prn {
        120 | 121 | 123 | 124 | 126 | 136 => "EGNOS",
        131 | 133 | 135 | 138 => "WAAS",
        129 | 137 => "MSAS",
        127 | 128 | 132 => "GAGAN",
        125 | 140 | 141 => "SDCM",
        130 | 143 | 144 => "BDSBAS",
        134 => "KASS",
        122 => "SouthPAN",
        _ => "SBAS",
    }
}

/// Colour a dilution-of-precision value by the usual GNSS quality bands (lower is
/// better): ≤2 excellent, ≤5 good, ≤10 moderate, else poor.
fn dop_color(dop: f64) -> egui::Color32 {
    if dop <= 2.0 {
        egui::Color32::from_rgb(80, 200, 100) // green
    } else if dop <= 5.0 {
        egui::Color32::from_rgb(220, 190, 50) // yellow
    } else if dop <= 10.0 {
        egui::Color32::from_rgb(230, 140, 50) // orange
    } else {
        egui::Color32::from_rgb(220, 80, 60) // red
    }
}

/// C/N0 quality band: 0 = green (strong, ≥40 dB-Hz), 1 = yellow (≥35), 2 = red.
/// Drives both the dB-Hz cell colour and the table's strong-first row order, so
/// the two never disagree.
fn cn0_band(cn0: f64) -> u8 {
    if cn0 >= 40.0 {
        0
    } else if cn0 >= 35.0 {
        1
    } else {
        2
    }
}

fn cn0_color(cn0: f64) -> egui::Color32 {
    match cn0_band(cn0) {
        0 => egui::Color32::from_rgb(80, 200, 100), // green
        1 => egui::Color32::from_rgb(220, 190, 50), // yellow
        _ => egui::Color32::from_rgb(220, 80, 60),  // red
    }
}

/// A fixed-width, left-aligned label for a status row, so the values after it line
/// up down the column (the position row, which has none, sits flush left instead).
fn status_label(ui: &mut egui::Ui, text: &str, h: f32) {
    ui.allocate_ui_with_layout(
        egui::vec2(LABEL_W, h),
        egui::Layout::left_to_right(egui::Align::Center),
        |ui| {
            ui.label(text);
        },
    );
}

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>, plots: bool, sbas: bool, qzss: bool) -> Self {
        Self {
            plots,
            sbas,
            qzss,
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
        // Seed the family set: C/A always; Galileo when the recording is E1
        // or the bandwidth admits it (the all-signals default).
        self.gps = true;
        self.gal = self.sig.is_boc11() || self.fs >= 4_092_000.0;
    }

    fn stop_async(&mut self) {
        self.needs_stop.store(true, Ordering::SeqCst);
        log::info!("stop_async");
    }

    fn start_async(&mut self, ctx: &egui::Context) {
        log::info!("start_async");
        self.needs_stop.store(false, Ordering::SeqCst);
        self.paused.store(false, Ordering::SeqCst);

        let active = self.active.clone();
        let needs_stop = self.needs_stop.clone();
        let paused = self.paused.clone();

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

        let mut families = Vec::new();
        if self.gps {
            families.push(Signal::L1ca);
        }
        if self.gal {
            families.push(Signal::GalileoE1b);
        }
        let config = ReceiverConfig {
            file: PathBuf::from(&self.iq_file),
            iq_file_type: self.iq_file_type.clone(),
            fs: self.fs,
            fi: self.fi,
            sig: families[0],
            families,
            plots: self.plots,
            // The egui diagnostics tab reads the published History snapshots, so
            // keep the full depth even without --plots.
            diagnostics: true,
            sbas: self.sbas,
            qzss: self.qzss,
            ..Default::default()
        };

        thread::spawn(move || {
            log::info!("thread_start");
            async_receive(active, needs_stop, paused, config, pub_state);
            log::info!("thread_stop");
        });
    }
}

pub fn egui_main(plots: bool, sbas: bool, qzss: bool) {
    log::warn!("egui_main");
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([WIDTH as f32, HEIGHT as f32]),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "gnss-rcv",
        native_options,
        Box::new(move |cc| Ok(Box::new(GnssRcvApp::new(cc, plots, sbas, qzss)))),
    )
    .unwrap();
}

impl eframe::App for GnssRcvApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_shortcuts(ctx);
        self.update_top(ctx);
        self.update_central(ctx);
    }
}

impl GnssRcvApp {
    /// Global keyboard shortcuts: Space = start / pause / resume, Esc = stop.
    /// Suppressed while a field (file path, fs/fi) has focus so typing still works.
    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.memory(|m| m.focused().is_some()) {
            return;
        }
        let (space, esc) = ctx.input(|i| {
            (
                i.key_pressed(egui::Key::Space),
                i.key_pressed(egui::Key::Escape),
            )
        });
        if space {
            if self.active.load(Ordering::SeqCst) {
                let paused = self.paused.load(Ordering::SeqCst);
                self.paused.store(!paused, Ordering::SeqCst);
            } else {
                self.start_async(ctx);
            }
        }
        if esc && self.active.load(Ordering::SeqCst) {
            self.stop_async();
        }
    }

    fn update_file_picker(&mut self, ui: &mut egui::Ui) {
        let selected_text = self
            .recordings
            .get(self.selected)
            .map(|r| r.name.as_str())
            .unwrap_or("(no provisioned recordings)")
            .to_owned();
        let mut clicked = None;
        egui::ComboBox::from_id_salt("file_picker")
            .width(FIELD_W)
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
        egui::ComboBox::from_id_salt("iq_format")
            .width(FIELD_W)
            .selected_text(self.iq_file_type.to_string())
            .show_ui(ui, |ui| {
                for t in [
                    IQFileType::TypePairFloat32,
                    IQFileType::TypePairInt16,
                    IQFileType::TypePairInt8,
                    IQFileType::TypeOneInt8,
                    IQFileType::TypeOne4Bit,
                    IQFileType::TypeOne2Bit,
                    IQFileType::TypeOneBit,
                    IQFileType::TypeRtlSdrFile,
                ] {
                    let label = t.to_string();
                    ui.selectable_value(&mut self.iq_file_type, t, label);
                }
            });
    }

    /// Signal-family selection. Families are a *set* (the receiver runs every
    /// checked one on its own scheduler grid), so checkboxes — not a
    /// dropdown: L1CA and E1 combine into a mixed GPS+Galileo session. The
    /// E1 box is greyed out when the sample rate cannot carry BOC(1,1); SBAS
    /// and QZSS ride the C/A family, so they grey out without it.
    fn update_sig_type(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.gps, "L1CA")
                    .on_hover_text("GPS L1 C/A family (also carries the SBAS/QZSS blocks)");
                let e1_ok = self.fs >= 4_092_000.0;
                if !e1_ok {
                    self.gal = false;
                }
                ui.add_enabled(e1_ok, egui::Checkbox::new(&mut self.gal, "E1"))
                    .on_hover_text("Galileo E1-B, on its own 4 ms grid")
                    .on_disabled_hover_text(
                        "Galileo E1 BOC(1,1) needs its ±2.046 MHz main lobes: fs ≥ 4.092 MHz",
                    );
                // A session needs at least one family.
                if !self.gps && !self.gal {
                    self.gps = true;
                }
            });
            ui.horizontal(|ui| {
                ui.add_enabled(self.gps, egui::Checkbox::new(&mut self.sbas, "SBAS"))
                    .on_hover_text(
                        "EGNOS/WAAS GEOs (PRN 120-138): iono grid + fast/long-term                          corrections applied to the fix",
                    );
                ui.add_enabled(self.gps, egui::Checkbox::new(&mut self.qzss, "QZSS"))
                    .on_hover_text("QZSS L1 block (PRN 193-202)");
            });
        });
    }

    /// The recording / format / signal settings as one `label | dropdown | label |
    /// value` table: recording + file path, then iq-format + fs, then signal + fi.
    fn update_controls(&mut self, ui: &mut egui::Ui) {
        let h = ui.spacing().interact_size.y;
        egui::Grid::new("rx_controls")
            .num_columns(4)
            .spacing([GRID_GAP_X, 6.0])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.set_min_width(LABEL_W);
                    ui.label("recording");
                });
                self.update_file_picker(ui);
                ui.label("file");
                ui.add_sized([VALUE_W, h], egui::TextEdit::singleline(&mut self.iq_file));
                ui.end_row();

                ui.horizontal(|ui| {
                    ui.set_min_width(LABEL_W);
                    ui.label("iq-format");
                });
                self.update_iq_type(ui);
                ui.label("fs");
                ui.add_sized(
                    [VALUE_W, h],
                    egui::DragValue::new(&mut self.fs)
                        .speed(1000.0)
                        .suffix(" Hz"),
                );
                ui.end_row();

                ui.horizontal(|ui| {
                    ui.set_min_width(LABEL_W);
                    ui.label("signal");
                });
                self.update_sig_type(ui);
                ui.label("fi");
                ui.add_sized(
                    [VALUE_W, h],
                    egui::DragValue::new(&mut self.fi)
                        .speed(1000.0)
                        .suffix(" Hz"),
                );
                ui.end_row();
            });
    }

    /// The run controls — start/pause/resume + stop — side by side in an `area_w`
    /// wide strip `box_h` tall (it fills the height of the status table beside it).
    fn update_start_stop(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        box_h: f32,
        area_w: f32,
    ) {
        let active = self.active.load(Ordering::SeqCst);
        let paused = self.paused.load(Ordering::SeqCst);
        let accent = ui.visuals().selection.bg_fill;
        let red = egui::Color32::from_rgb(0xC0, 0x39, 0x2B);
        let gap = ui.spacing().item_spacing.x;
        // Running: start/pause shares the strip with the red stop button but stays
        // twice its width. Idle: the single start button spans the whole strip.
        let (main_w, stop_w) = if active {
            let stop = ((area_w - gap) / 3.0).max(40.0);
            ((area_w - gap - stop).max(60.0), stop)
        } else {
            (area_w, 0.0)
        };
        let label = if !active {
            "start"
        } else if paused {
            "resume"
        } else {
            "pause"
        };
        ui.horizontal(|ui| {
            // start (idle) -> pause (running) -> resume (paused).
            if ui
                .add_sized(
                    [main_w, box_h],
                    egui::Button::new(egui::RichText::new(label).size(15.0)).fill(accent),
                )
                .clicked()
            {
                if !active {
                    self.start_async(ctx);
                } else {
                    self.paused.store(!paused, Ordering::SeqCst);
                }
            }
            // Stop (red), only while running — tears the run down.
            if active
                && ui
                    .add_sized(
                        [stop_w, box_h],
                        egui::Button::new(egui::RichText::new("stop").size(14.0)).fill(red),
                    )
                    .clicked()
            {
                self.stop_async();
            }
        });
    }

    fn update_top(&mut self, ctx: &egui::Context) {
        let (sv_elaz, tow_text, has_ion, has_utc, pos_text, pos_url, osnma_kroot, dop, sbas, run) = {
            let st = self.pub_state.lock().unwrap();
            let mut sv_elaz: Vec<(SV, f64, f64, f64)> = st
                .channels
                .iter()
                .filter(|(_, cs)| cs.state == State::Tracking && cs.elevation_deg != 0.0)
                .map(|(sv, cs)| (*sv, cs.elevation_deg, cs.azimuth_deg, cs.cn0))
                .collect();
            sv_elaz.sort_by_key(|t| t.0);
            let tow_text = format!("{:?}", st.tow_gpst);
            let (pos_text, pos_url) = if st.longitude != 0.0 {
                (
                    format!("{:.4} {:.4} {:.0}m", st.latitude, st.longitude, st.height),
                    Some(format!(
                        "https://maps.google.com/?ll={},{}",
                        st.latitude, st.longitude
                    )),
                )
            } else {
                ("no position fix".to_string(), None)
            };
            // Fix precision, alongside the position (so it shares the fix gate).
            let dop = if st.longitude != 0.0 {
                Some((st.hdop, st.vdop, st.fix_sv_count))
            } else {
                None
            };
            (
                sv_elaz,
                tow_text,
                st.ion_adj,
                st.utc_adj,
                pos_text,
                pos_url,
                st.osnma_kroot,
                dop,
                (
                    st.sbas_iono.len(),
                    st.sbas_corr.fast_len(),
                    st.sbas_corr.long_len(),
                    st.sbas_source.map(sbas_system_short).unwrap_or("SBAS"),
                ),
                (st.run_progress, st.realtime_x),
            )
        };

        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .show(ctx, |ui| {
                // One frame box: controls + status on the left, sky plot column on the right.
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    StripBuilder::new(ui)
                        .size(Size::remainder()) // controls + status
                        .size(Size::exact(1.0)) // separator
                        .size(Size::exact(180.0)) // sky plot
                        .horizontal(|mut strip| {
                            strip.cell(|ui| {
                                self.update_controls(ui);
                                ui.separator();
                                ui.horizontal_top(|ui| {
                                    // Narrower so the status text column + the correction-flag
                                    // column both fit to its left.
                                    let btn_area_w = 130.0_f32;
                                    let status_w =
                                        (ui.available_width() - btn_area_w - GRID_GAP_X).max(240.0);
                                    let status = ui.scope(|ui| {
                                        ui.set_width(status_w);
                                        let h = ui.spacing().interact_size.y;
                                        // The scope inherits the outer row's horizontal layout;
                                        // a text column (GPST / position / precision / KROOT)
                                        // with the correction-flag column beside it.
                                        ui.horizontal_top(|ui| {
                                            ui.vertical(|ui| {
                                                // GPST + time.
                                                ui.horizontal(|ui| {
                                                    status_label(ui, "GPST", h);
                                                    ui.monospace(&tow_text);
                                                });
                                                // Position — bare (no label, no prefixes).
                                                if let Some(url) = &pos_url {
                                                    ui.hyperlink_to(&pos_text, url);
                                                } else {
                                                    ui.monospace(&pos_text);
                                                }
                                                // Precision (HDOP/VDOP), no label. Always keep the
                                                // row (blank until the first fix) so the box
                                                // doesn't grow when a fix appears.
                                                let prec = ui.horizontal(|ui| {
                                                    if let Some((hdop, vdop, nsv)) = dop {
                                                        ui.colored_label(
                                                            dop_color(hdop),
                                                            format!("HDOP {hdop:.1}"),
                                                        );
                                                        ui.weak(format!(
                                                            "· VDOP {vdop:.1} · {nsv} SV"
                                                        ));
                                                    } else {
                                                        ui.allocate_space(egui::vec2(0.0, h));
                                                    }
                                                });
                                                if dop.is_some() {
                                                    prec.response.on_hover_text(
                                                        "Dilution of precision (lower is better — \
                                                     <2 excellent, 2-5 good). VDOP is vertical, \
                                                     then the SV count used in the fix.",
                                                    );
                                                }
                                                // OSNMA DSM-KROOT assembly progress.
                                                if let Some((got, total)) = osnma_kroot {
                                                    ui.horizontal(|ui| {
                                                        status_label(ui, "KROOT", h);
                                                        let fill = if got >= total {
                                                            egui::Color32::from_rgb(80, 200, 100)
                                                        } else {
                                                            egui::Color32::from_rgb(70, 110, 180)
                                                        };
                                                        ui.add(
                                                            egui::ProgressBar::new(
                                                                got as f32 / total as f32,
                                                            )
                                                            .desired_width(VALUE_W)
                                                            .text(format!("{got}/{total} blocks"))
                                                            .fill(fill),
                                                        )
                                                        .on_hover_text(
                                                            "OSNMA TESLA root key (DSM-KROOT): \
                                                         blocks assembled / needed before nav \
                                                         data is authenticated",
                                                        );
                                                    });
                                                }
                                            });
                                            // Correction-flag column, beside all the text lines
                                            // and left of the run controls.
                                            ui.vertical(|ui| {
                                                if has_ion {
                                                    ui.monospace("ion");
                                                }
                                                if has_utc {
                                                    ui.monospace("utc");
                                                }
                                                let (n_igp, n_fast, n_long, sbas_sys) = sbas;
                                                if n_igp + n_fast + n_long > 0 {
                                                    ui.colored_label(
                                                        constellation_color(Constellation::SBAS),
                                                        sbas_sys,
                                                    )
                                                    .on_hover_text(format!(
                                                        "SBAS corrections applied to the fix: iono \
                                                     grid {n_igp} IGPs · {n_fast} fast + \
                                                     {n_long} long-term corrections"
                                                    ));
                                                }
                                            });
                                        });
                                    });
                                    let box_h = status.response.rect.height();
                                    self.update_start_stop(ui, ctx, box_h, btn_area_w);
                                });
                                // Run progress — fraction of the recording processed,
                                // plus the real-time factor; only while a run is active.
                                let (progress, realtime_x) = run;
                                if self.active.load(Ordering::SeqCst) {
                                    // `animate` keeps the bar visibly shimmering even when
                                    // the fraction barely moves (a long capture at sub-real-
                                    // time), and the "starting…" indeterminate bar covers
                                    // the gap before the first progress publish — so Start
                                    // gives instant feedback instead of a dead box during the
                                    // seconds of setup + cold-start acquisition.
                                    let bar = match progress {
                                        Some(p) => egui::ProgressBar::new(p).text(format!(
                                            "{:.0}%  ·  {:.1}× real-time",
                                            p * 100.0,
                                            realtime_x
                                        )),
                                        None => egui::ProgressBar::new(0.0).text("starting…"),
                                    };
                                    ui.add(
                                        bar.fill(egui::Color32::from_rgb(70, 110, 180))
                                            .animate(true),
                                    );
                                }
                            }); // strip.cell (left column)
                            strip.cell(|ui| {
                                let rect = ui.max_rect();
                                ui.painter().line_segment(
                                    [rect.center_top(), rect.center_bottom()],
                                    ui.visuals().widgets.noninteractive.bg_stroke,
                                );
                            });
                            strip.cell(|ui| {
                                ui.centered_and_justified(|ui| {
                                    draw_sky_plot(ui, &sv_elaz);
                                });
                            });
                        }); // StripBuilder
                }); // Frame::group
            });
    }

    fn update_central(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
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
        // Same strong-first stable ordering as the dashboard table: a deterministic
        // PRN base order, then a stable sort by C/N0 colour band.
        tracked_svs.sort_by_key(|(sv, _)| *sv);
        tracked_svs.sort_by_key(|(_, cn0)| cn0_band(*cn0));

        egui::SidePanel::left("diag_sv_list")
            .resizable(false)
            .exact_width(58.0)
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for (sv, cn0) in &tracked_svs {
                        let label = egui::RichText::new(format!("{sv}")).color(cn0_color(*cn0));
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
        let is_galileo = self.gal;

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
            .column(Column::auto()) // ephemeris
            .min_scrolled_height(0.0)
            .max_scroll_height(available_height);
        let tb = if is_galileo {
            tb.column(Column::auto())
        } else {
            tb
        };

        // Rows come from whatever is actually tracking — covering GPS,
        // Galileo, and the SBAS/QZSS blocks alike (a fixed PRN range used to
        // hide S/J satellites even while they tracked).
        let tracked: Vec<SV> = {
            let st = self.pub_state.lock().unwrap();
            let mut v: Vec<(SV, f64)> = st
                .channels
                .iter()
                .filter(|(_, cs)| cs.state == State::Tracking)
                .map(|(sv, cs)| (*sv, cs.cn0))
                .collect();
            // Strong-first, but stable: a deterministic PRN base order (HashMap
            // iteration is unordered), then a *stable* sort by C/N0 colour band
            // (green ≥40, yellow ≥35, red below). Rows keep their place within a
            // band and only move when an SV actually changes band — so a wobbling
            // C/N0 doesn't reshuffle the table.
            v.sort_by_key(|(sv, _)| *sv);
            v.sort_by_key(|(_, cn0)| cn0_band(*cn0));
            v.into_iter().map(|(sv, _)| sv).collect()
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
                // "decode", not "ephemeris": ranging SVs show ephemeris pages,
                // but SBAS GEOs show decoded correction message types.
                ui.strong("decode").on_hover_text(
                    "decode progress: ephemeris pages (GPS/Galileo/QZSS) or \
                     correction message types (SBAS GEOs)",
                );
            });
            if is_galileo {
                header.col(|ui| {
                    ui.strong("osnma");
                });
            }
        })
        .body(|mut body| {
            for sv in tracked {
                let row_height = 20.0;
                let pub_state = self.pub_state.lock().unwrap();
                let channel = pub_state.channels.get(&sv);

                if channel.is_none() {
                    continue;
                }
                let cn0 = channel.unwrap().cn0;
                let doppler_hz = channel.unwrap().doppler_hz;
                let code_idx = channel.unwrap().code_idx;
                let eph_mask = channel.unwrap().eph_mask;
                let osnma_verified = channel.unwrap().osnma_verified;
                let sbas_msgs = channel.unwrap().sbas_msgs;
                let sbas_msg_mask = channel.unwrap().sbas_msg_mask;
                let tx_anchored = channel.unwrap().tx_anchored;
                let iode = channel.unwrap().iode;
                let eph_age_sec = (pub_state.tow_gpst - channel.unwrap().toe_gpst).to_seconds();
                let used_in_fix = pub_state.fix_svs.contains(&sv);
                let is_sbas_source = pub_state.sbas_source == Some(sv);

                body.row(row_height, |mut row| {
                    row.col(|ui| {
                        ui.colored_label(constellation_color(sv.constellation), format!("{}", sv))
                            .on_hover_text(sv_hover_text(sv));
                    });
                    row.col(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.colored_label(cn0_color(cn0), format!("{:.1}", cn0));
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
                        if sv.constellation == Constellation::SBAS {
                            // SBAS GEOs broadcast corrections, not ephemerides:
                            // show which correction types are flowing + the source.
                            draw_sbas_cell(ui, sbas_msgs, sbas_msg_mask, is_sbas_source);
                            return;
                        }
                        if used_in_fix {
                            // Contributing to the fix: collapse the pips to a
                            // freshness-coloured check (detail moves to the hover).
                            draw_eph_used(ui, iode, eph_age_sec);
                        } else {
                            draw_eph_pages(ui, sv, eph_mask, tx_anchored);
                        }
                    });
                    if is_galileo {
                        row.col(|ui| {
                            if osnma_verified {
                                // A lock (not a plain check) — this column means
                                // "cryptographically authenticated", distinct from
                                // the ephemeris ✓ next to it.
                                ui.colored_label(egui::Color32::from_rgb(80, 200, 100), "🔒")
                                    .on_hover_text("OSNMA: navigation data authenticated");
                            }
                        });
                    }
                });
            }
        });
    }
}

/// Per-page ephemeris decode status: one pip per orbit/clock message — GPS LNAV
/// subframes 1-3, Galileo I/NAV words 1-5 — filled as each arrives, so a stuck
/// decode shows *which* message is missing rather than just a count. Painter-drawn
/// circles (no glyph-font dependency). Filled pips are green once the full set is
/// decoded *and* the transmit anchor is pinned (the SV is solver-usable), amber
/// when fully decoded but not yet anchored, constellation-tinted while partial.
fn draw_eph_pages(ui: &mut egui::Ui, sv: SV, eph_mask: u8, tx_anchored: bool) {
    // Page bits are 1-indexed (bit 0 unused): GPS subframes 1..=3, Galileo word
    // types 1..=5. The names drive the hover so the pips are self-documenting.
    let labels: &[&str] = if sv.constellation == Constellation::Galileo {
        &[
            "W1 ephemeris 1/4",
            "W2 ephemeris 2/4",
            "W3 ephemeris 3/4 + SISA",
            "W4 ephemeris 4/4 + clock",
            "W5 iono + BGD + GST",
        ]
    } else {
        &[
            "SF1 clock / health",
            "SF2 ephemeris (orbit shape)",
            "SF3 ephemeris (orientation)",
        ]
    };
    let n = labels.len();
    let got = |i: usize| (eph_mask >> (i + 1)) & 1 == 1;
    let count = (0..n).filter(|&i| got(i)).count();
    let complete = count == n;

    let fill = if complete && tx_anchored {
        egui::Color32::from_rgb(80, 200, 100) // green: decoded and solver-usable
    } else if complete {
        egui::Color32::from_rgb(220, 190, 50) // amber: decoded, pinning the anchor
    } else {
        constellation_color(sv.constellation) // partial
    };
    let dim = egui::Color32::from_gray(80);

    let r = 3.5_f32;
    let gap = 3.0_f32;
    let h = ui.spacing().interact_size.y;
    let w = n as f32 * (2.0 * r) + (n as f32 - 1.0) * gap;
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::hover());
    let painter = ui.painter();
    let cy = rect.center().y;
    for i in 0..n {
        let cx = rect.left() + r + i as f32 * (2.0 * r + gap);
        let c = egui::pos2(cx, cy);
        if got(i) {
            painter.circle_filled(c, r, fill);
        } else {
            painter.circle_stroke(c, r, egui::Stroke::new(1.0, dim));
        }
    }

    let mut tip = String::new();
    for (i, label) in labels.iter().enumerate() {
        tip.push_str(if got(i) { "✔ " } else { "· " });
        tip.push_str(label);
        tip.push('\n');
    }
    tip.push_str(&format!("{count}/{n} decoded"));
    if complete && !tx_anchored {
        tip.push_str(" — pinning transmit anchor");
    } else if complete {
        tip.push_str(" — usable");
    }
    resp.on_hover_text(tip);
}

/// Compact ephemeris age for the hover.
fn fmt_age(sec: f64) -> String {
    if sec < 90.0 {
        format!("{sec:.0}s")
    } else if sec < 5400.0 {
        format!("{:.0}m", sec / 60.0)
    } else {
        format!("{:.1}h", sec / 3600.0)
    }
}

/// Collapsed ephemeris state once the SV is contributing to the fix: a single
/// check coloured by ephemeris freshness — green fresh → amber aging → red past
/// the ~2 h fit interval — with IODE + age in the hover. (On file replay the
/// ephemeris is contemporaneous so it reads green; the colour earns its keep on
/// long live runs, and a changed IODE flags a fresh upload.)
fn draw_eph_used(ui: &mut egui::Ui, iode: u32, age_sec: f64) {
    let color = if age_sec < 7200.0 {
        egui::Color32::from_rgb(80, 200, 100)
    } else if age_sec < 14400.0 {
        egui::Color32::from_rgb(220, 190, 50)
    } else {
        egui::Color32::from_rgb(220, 80, 60)
    };
    ui.colored_label(color, "✔").on_hover_text(format!(
        "used in fix · IODE {iode} · age {}",
        fmt_age(age_sec)
    ));
}

/// SBAS GEO status: a "src" mark for the GEO actually feeding the fix's
/// corrections (vs. a redundant or other-system bird), plus pips for which
/// correction message *types* are flowing — the SBAS analog of the ephemeris
/// page row, replacing the bare message count.
fn draw_sbas_cell(ui: &mut egui::Ui, sbas_msgs: u64, msg_mask: u64, is_source: bool) {
    // The correction groups that matter, each lit if any of its message types
    // has been decoded.
    let groups: &[(&str, &[u8])] = &[
        ("PRN mask (MT1)", &[1]),
        ("fast corrections (MT2-5)", &[2, 3, 4, 5]),
        ("iono grid mask (MT18)", &[18]),
        ("iono delays (MT26)", &[26]),
        ("long-term (MT24/25)", &[24, 25]),
    ];
    let got = |mts: &[u8]| mts.iter().any(|&t| msg_mask & (1u64 << t) != 0);
    let on = constellation_color(Constellation::SBAS);

    // Every correction type in: collapse the pips to a check (like the
    // ephemeris column). The active source is marked "✔ src"; a complete
    // non-source GEO gets a plain ✔ — fully provisioned, standing by to take
    // over if the source goes dark.
    if groups.iter().all(|(_, mts)| got(mts)) {
        let (label, tip) = if is_source {
            (
                "✔ src",
                format!(
                    "active correction source — all types received ({sbas_msgs} CRC-valid messages)"
                ),
            )
        } else {
            (
                "✔",
                format!(
                    "all correction types received — standby backup ({sbas_msgs} CRC-valid messages)"
                ),
            )
        };
        ui.colored_label(on, label).on_hover_text(tip);
        return;
    }

    ui.horizontal(|ui| {
        if is_source {
            ui.colored_label(on, "src")
                .on_hover_text("active correction source for the fix");
        }
        let n = groups.len();
        let r = 3.5_f32;
        let gap = 3.0_f32;
        let h = ui.spacing().interact_size.y;
        let w = n as f32 * (2.0 * r) + (n as f32 - 1.0) * gap;
        let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, h), egui::Sense::hover());
        let painter = ui.painter();
        let cy = rect.center().y;
        let dim = egui::Color32::from_gray(80);
        for (i, (_, mts)) in groups.iter().enumerate() {
            let cx = rect.left() + r + i as f32 * (2.0 * r + gap);
            let c = egui::pos2(cx, cy);
            if got(mts) {
                painter.circle_filled(c, r, on);
            } else {
                painter.circle_stroke(c, r, egui::Stroke::new(1.0, dim));
            }
        }
        let mut tip = String::new();
        for (label, mts) in groups {
            tip.push_str(if got(mts) { "✔ " } else { "· " });
            tip.push_str(label);
            tip.push('\n');
        }
        tip.push_str(&format!("{sbas_msgs} CRC-valid messages"));
        resp.on_hover_text(tip);
    });
}

/// Polar sky plot: azimuth around the circle, elevation as radial distance from
/// centre (90° elev = centre, 0° = horizon ring).
fn draw_sky_plot(ui: &mut egui::Ui, sv_elaz: &[(SV, f64, f64, f64)]) {
    let size = 168.0_f32;
    let (response, painter) = ui.allocate_painter(egui::vec2(size, size), egui::Sense::hover());
    let rect = response.rect;
    let c = rect.center();
    // Leave room outside the ring for the N/E/S/W labels so they aren't clipped at
    // the box edges: each label sits 9 px past the ring, is ~5 px tall, plus a few
    // px of margin (= 18). Otherwise S spills off the bottom.
    let r = size * 0.5 - 18.0;

    // No background fill: let the panel show through so the plot blends into the
    // UI rather than sitting in its own dark box.

    let grid_stroke = egui::Stroke::new(0.6, egui::Color32::from_rgb(60, 70, 90));

    // Cardinal spokes (N-S and E-W diameters) for an azimuth reference.
    painter.line_segment(
        [egui::pos2(c.x - r, c.y), egui::pos2(c.x + r, c.y)],
        grid_stroke,
    );
    painter.line_segment(
        [egui::pos2(c.x, c.y - r), egui::pos2(c.x, c.y + r)],
        grid_stroke,
    );

    // Concentric elevation rings (horizon, 30°, 60°) — the horizon a touch heavier.
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

    // SV dots, tinted by constellation — same palette as the SV table, so an
    // orange dot on the plot is the same SBAS GEO as the orange row. The radius
    // tracks C/N0 (stronger = bigger), with a dark outline so dots read against
    // the rings.
    let outline = egui::Stroke::new(1.0, egui::Color32::from_rgb(20, 24, 34));
    for (sv, elev_deg, azim_deg, cn0) in sv_elaz {
        let azim_rad = azim_deg.to_radians() as f32;
        let dist = r * (1.0 - *elev_deg as f32 / 90.0);
        let sx = c.x + dist * azim_rad.sin();
        let sy = c.y - dist * azim_rad.cos();
        let pos = egui::pos2(sx, sy);
        let radius = match cn0_band(*cn0) {
            0 => 4.0, // green / strong
            1 => 3.5, // yellow
            _ => 3.0, // red / weak
        };
        painter.circle(pos, radius, constellation_color(sv.constellation), outline);
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
