use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

#[derive(Default, Clone, Copy)]
pub struct Ephemeris {
    pub sv: SV,
    pub tow: u32,
    pub cn0: f64,
    pub code_off_sec: f64,
    pub ts_sec: f64, // receiver time for 1st subframe
    // Integer transmit-time in seconds since tracking start: num_trk_samples *
    // code_sec. Because num_trk_samples counts *transmitted* code periods
    // (carrier-aided), it advances at the SV clock rate, unlike the receiver
    // wall clock ts_sec. The absolute sub-ms code phase is kept in code_off_sec.
    pub trk_phase: f64,        // current integer transmit-time
    pub tow_trk_phase: f64,    // integer-ms phase at tx_tow_gpst (set once)
    pub tx_tow_gpst: Epoch,    // GPS time pinned at first ephemeris lock
    pub tx_anchor_ts_sec: f64, // receiver time when tx was anchored
    pub tx_anchored: bool,
    pub tow_gpst: Epoch,
    pub toe_gpst: Epoch, // cf toe
    pub toc_gpst: Epoch,
    pub tlm: u32,

    pub iode: u32,    // Issue of Data, Ephemeris
    pub iodc: u32,    // Issue of Data, Clock
    pub sva: u32,     // SV accuracy (URA index)
    pub svh: u32,     // SV health (0:ok)
    pub week: u32,    // GPS/QZS: gps week, GAL: galileo week
    pub code: u32,    // GPS/QZS: code on L2, GAL/CMP: data sources
    pub flag: u32,    // GPS/QZS: L2 P data flag, CMP: nav type
    pub tgd: f64,     // GPS: Estimated Group Delay Differential
    pub f0: f64,      // SV Clock Bias Correction Coefficient
    pub f1: f64,      // SV Clock Drift Correction Coefficient
    pub f2: f64,      // Drift Rate Correction Coefficient
    pub omg: f64,     // Argument of Perigee
    pub omg0: f64,    // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    pub omg_dot: f64, // Rate of Right Ascension
    pub cic: f64, // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    pub cis: f64, // Amplitude of the Sine   Harmonic Correction Term to the Angle of Inclination
    pub crc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    pub crs: f64, // Amplitude of the Sine   Harmonic Correction Term to the Orbit Radius
    pub cuc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    pub cus: f64, // Amplitude of the Sine   Harmonic Correction Term to the Argument of Latitude
    pub i_dot: f64, // Rate of Inclination Angle
    pub i0: f64,  // Inclination Angle at Reference Time
    pub m0: f64,  // Mean Anomaly at Reference Time
    pub a: f64,   // semi major axis
    pub ecc: f64, // Eccentricity
    pub deln: f64, // Mean Motion Difference From Computed Value
    pub toc: u32, // Time of Clock
    pub toe: u32, // Reference Time Ephemeris
    pub fit: u32, // fit interval (h)
}

impl Ephemeris {
    pub fn new(sv: SV) -> Self {
        Self {
            sv,
            ..Default::default()
        }
    }
    /// True once subframes 1-3 have decoded into a physically plausible
    /// ephemeris. The position solver should only consume valid ones; this
    /// checks at least one field set by each subframe (so a half-decoded
    /// ephemeris can't slip through) plus an eccentricity sanity bound to reject
    /// a corrupt subframe-2. Constellation-agnostic on orbit size/inclination so
    /// it stays correct if QZSS/GEO ephemerides are added later.
    pub fn is_valid(&self) -> bool {
        self.ts_sec != 0.0           // first subframe was timestamped
            && self.week != 0        // subframe 1
            && self.toc != 0         // subframe 1
            && self.toe != 0         // subframe 2
            && self.a >= 20_000_000.0 // subframe 2 (sqrt_a decoded)
            && self.ecc < 0.5        // subframe 2 sanity (real orbits: ecc << 1)
            && self.i0 != 0.0        // subframe 3
            && self.omg_dot != 0.0 // subframe 3
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gnss_rs::constellation::Constellation;

    // A fully-decoded GPS ephemeris (representative values: a ~ 26560 km,
    // i0 ~ 55 deg, small eccentricity).
    fn valid_eph() -> Ephemeris {
        let mut e = Ephemeris::new(SV::new(Constellation::GPS, 1));
        e.ts_sec = 1.0;
        e.week = 2300;
        e.toc = 100 * 16;
        e.toe = 450 * 16;
        e.a = 26_560_000.0;
        e.ecc = 0.012;
        e.i0 = 0.96;
        e.omg_dot = -8.0e-9;
        e
    }

    #[test]
    fn fully_decoded_ephemeris_is_valid() {
        assert!(valid_eph().is_valid());
    }

    #[test]
    fn default_ephemeris_is_invalid() {
        assert!(!Ephemeris::default().is_valid());
    }

    #[test]
    fn missing_any_subframe_field_is_invalid() {
        // Zeroing any single field a subframe sets must invalidate the ephemeris.
        let check = |zero: fn(&mut Ephemeris), name: &str| {
            let mut e = valid_eph();
            zero(&mut e);
            assert!(!e.is_valid(), "{name} missing should be invalid");
        };
        check(|e| e.ts_sec = 0.0, "untimestamped");
        check(|e| e.week = 0, "sf1 week");
        check(|e| e.toc = 0, "sf1 toc");
        check(|e| e.toe = 0, "sf2 toe");
        check(|e| e.a = 0.0, "sf2 a");
        check(|e| e.i0 = 0.0, "sf3 i0");
        check(|e| e.omg_dot = 0.0, "sf3 omg_dot");
    }

    #[test]
    fn corrupt_eccentricity_is_invalid() {
        let mut e = valid_eph();
        e.ecc = 0.9; // a garbled subframe-2 can decode to ecc ~ 1
        assert!(!e.is_valid());
    }
}
