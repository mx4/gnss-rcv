use crate::{almanac::Almanac, channel::{History, State}};
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;
use std::collections::HashMap;

pub struct UpdateFunc {
    pub func: Box<dyn Fn() + Send + Sync>,
}

pub struct ChannelState {
    pub state: State,
    pub cn0: f64,
    pub doppler_hz: f64,
    pub code_idx: f64,
    pub phi: f64,
    pub has_eph: bool,
    /// Galileo OSNMA: this SV's navigation data has been cryptographically
    /// authenticated (set by the receiver-level verifier; always false off OSNMA).
    pub osnma_verified: bool,
    pub elevation_deg: f64,
    pub azimuth_deg: f64,
}
impl Default for ChannelState {
    fn default() -> Self {
        Self {
            state: State::Acquisition,
            cn0: 0.0,
            doppler_hz: 0.0,
            code_idx: 0.0,
            phi: 0.0,
            has_eph: false,
            osnma_verified: false,
            elevation_deg: 0.0,
            azimuth_deg: 0.0,
        }
    }
}

pub struct GnssState {
    pub tow_gpst: Epoch,
    pub almanac: Vec<Almanac>,
    pub utc_adj: bool,
    pub ion_adj: bool,
    /// Klobuchar ionosphere model coefficients (decoded from subframe 4, page 18).
    /// Valid once `ion_adj` is true.
    pub iono_alpha: [f64; 4],
    pub iono_beta: [f64; 4],
    pub latitude: f64,
    pub longitude: f64,
    pub height: f64,

    pub channels: HashMap<SV, ChannelState>,
    pub histories: HashMap<SV, History>,
    pub update_func: UpdateFunc,
}

impl Default for GnssState {
    fn default() -> Self {
        Self::new()
    }
}

impl GnssState {
    pub fn new() -> Self {
        Self {
            tow_gpst: Epoch::default(),
            almanac: vec![Almanac::default(); 32],
            utc_adj: false,
            ion_adj: false,
            iono_alpha: [0.0; 4],
            iono_beta: [0.0; 4],
            latitude: 0.0,
            longitude: 0.0,
            height: 0.0,
            channels: HashMap::<SV, ChannelState>::new(),
            histories: HashMap::new(),
            update_func: UpdateFunc {
                func: Box::new(|| {}),
            },
        }
    }
    pub fn set_update_func(&mut self, func: Box<dyn Fn() + Send + Sync>) {
        self.update_func.func = func;
    }
}
