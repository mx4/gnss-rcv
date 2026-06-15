use crate::{
    almanac::Almanac,
    channel::{History, State},
    sbas_corr::SbasCorrections,
    sbas_iono::SbasIonoGrid,
};
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
    /// Transmit-time anchor pinned — the SV is actually usable by the solver
    /// (a complete ephemeris alone is not; see `nav_anchor_tx`).
    pub tx_anchored: bool,
    /// Ephemeris decode progress as a per-message bitmask: bit *n* set once
    /// message *n* is decoded — GPS LNAV subframes 1-3 (bits 1-3) or Galileo
    /// I/NAV word types 1-5 (bits 1-5). The UI renders one pip per message so a
    /// stuck decode shows *which* is missing; a full set = `has_eph`.
    pub eph_mask: u8,
    /// Galileo OSNMA: this SV's navigation data has been cryptographically
    /// authenticated (set by the receiver-level verifier; always false off OSNMA).
    pub osnma_verified: bool,
    pub elevation_deg: f64,
    pub azimuth_deg: f64,
    /// SBAS channels: CRC-valid 250-bit messages decoded so far (SBAS GEOs
    /// broadcast corrections, not ephemerides, so this replaces `eph_mask`
    /// as the decode-progress indicator in the UI).
    pub sbas_msgs: u64,
    /// SBAS message types decoded so far, as a bitmask (bit *t* = message type
    /// *t*, e.g. MT1 PRN mask, MT2-5 fast, MT18 iono mask, MT26 iono delays,
    /// MT24/25 long-term). The UI renders which correction types are flowing.
    pub sbas_msg_mask: u64,
    /// Ephemeris issue-of-data and reference epoch, for the "used in fix"
    /// freshness indicator: `age = tow_gpst − toe_gpst`, and a changed `iode`
    /// flags a fresh upload. Valid once `has_eph`.
    pub iode: u32,
    pub toe_gpst: Epoch,
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
            tx_anchored: false,
            eph_mask: 0,
            osnma_verified: false,
            elevation_deg: 0.0,
            azimuth_deg: 0.0,
            sbas_msgs: 0,
            sbas_msg_mask: 0,
            iode: 0,
            toe_gpst: Epoch::default(),
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
    /// Fix altitude above the WGS-84 ellipsoid, in metres.
    pub height: f64,
    /// Galileo NeQuick-G effective-ionisation coefficients [ai0, ai1, ai2]
    /// (I/NAV word 5), once decoded. Inputs for a future NeQuick-G model;
    /// mixed runs meanwhile apply the GPS Klobuchar to all SVs.
    pub gal_iono_az: Option<[f64; 3]>,
    /// SBAS ionospheric grid (EGNOS/WAAS MT18 + MT26), assembled live by the
    /// SBAS channels. When non-empty the solver prefers it over Klobuchar —
    /// it is the only iono source that arrives within seconds (Klobuchar's
    /// page 18 needs up to 12.5 min of GPS nav data).
    pub sbas_iono: SbasIonoGrid,
    /// SBAS fast + long-term corrections (MT1-5/24/25), assembled live by the
    /// SBAS channels and applied per GPS SV at the pseudorange level.
    pub sbas_corr: SbasCorrections,
    /// The GEO whose stream feeds sbas_iono/sbas_corr — first come, first
    /// serve. GEOs of one system broadcast the same data *nominally*, but a
    /// test-mode bird (tuni2025's S121 vs S123) can carry a different
    /// IODP/IODI generation, and interleaving two generations wipes the
    /// shared state on every MT1/MT18 (observed: corrections reset to 0/0,
    /// iono grid stuck at 0 IGPs).
    pub sbas_source: Option<SV>,
    /// Galileo OSNMA DSM-KROOT assembly progress as `(blocks_received, total)`,
    /// or `None` until the first block (which carries the total) arrives. Drives
    /// the UI's KROOT progress bar.
    pub osnma_kroot: Option<(u8, u8)>,
    /// Fix precision from the most recent solve: horizontal and vertical dilution
    /// of precision (unitless geometry factors, lower is better) and the number
    /// of satellites that contributed. All zero before the first fix.
    pub hdop: f64,
    pub vdop: f64,
    pub fix_sv_count: usize,
    /// SVs that contributed to the most recent successful fix. The per-SV table
    /// collapses a contributing SV's ephemeris pips to a freshness check.
    pub fix_svs: Vec<SV>,
    /// Run progress (fraction of the recording processed, 0..1) and the real-time
    /// processing factor (data seconds per wall second). `None` = no known total
    /// (idle, or a live device); drives the UI progress bar.
    pub run_progress: Option<f32>,
    pub realtime_x: f32,

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
            gal_iono_az: None,
            sbas_iono: SbasIonoGrid::default(),
            sbas_corr: SbasCorrections::default(),
            sbas_source: None,
            osnma_kroot: None,
            hdop: 0.0,
            vdop: 0.0,
            fix_sv_count: 0,
            fix_svs: Vec::new(),
            run_progress: None,
            realtime_x: 0.0,
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
