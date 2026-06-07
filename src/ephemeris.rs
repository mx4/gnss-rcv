use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

use crate::{
    constants::{P2_5, P2_19, P2_29, P2_31, P2_33, P2_43, P2_55, SC2RAD},
    util::{getbits, getbits2, getbitu, getbitu2},
};

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

    pub fn nav_decode_lnav_subframe1(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        // GPS Time started on Jan 6, 1980
        // 1st GPS Time Epoch ended on 21 August 1999
        // 2nd GPS Time Epoch ended on 06 April 2019
        self.week = getbitu(buf, 60, 10) + 2048;
        // 00 = Invalid,
        // 01 = P-code ON,
        // 10 = C/A-code ON,
        // 11= Invalid
        self.code = getbitu(buf, 70, 2);
        self.sva = getbitu(buf, 72, 4);
        // The MSB shall indicate a summary of the health of the LNAV data, where
        // 0 = all LNAV data are OK,
        // 1 = some or all LNAV data are bad.
        self.svh = getbitu(buf, 76, 6);

        self.iodc = getbitu2(buf, 82, 2, 210, 8);
        self.flag = getbitu(buf, 90, 1);
        self.tgd = getbits(buf, 196, 8) as f64 * P2_31;
        self.toc = getbitu(buf, 218, 16) * 16;
        self.f2 = getbits(buf, 240, 8) as f64 * P2_55;
        self.f1 = getbits(buf, 248, 16) as f64 * P2_43;
        self.f0 = getbits(buf, 270, 22) as f64 * P2_31;

        log::warn!(
            "{sv}: {} tow={} week={} code={} sva={} svh={} iodc={} tgd={:+.3e} toc={} f0={:+.3e} f1={:+.3e} f2={:+.3e}",
            "subframe-1".blue(),
            self.tow,
            self.week,
            self.code,
            self.sva,
            self.svh,
            self.iodc,
            self.tgd,
            self.toc,
            self.f0,
            self.f1,
            self.f2
        );
    }

    pub fn nav_decode_lnav_subframe2(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        self.iode = getbitu(buf, 60, 8);
        self.crs = getbits(buf, 68, 16) as f64 * P2_5;
        self.deln = getbits(buf, 90, 16) as f64 * P2_43 * SC2RAD;
        self.m0 = getbits2(buf, 106, 8, 120, 24) as f64 * P2_31 * SC2RAD;
        self.ecc = getbitu2(buf, 166, 8, 180, 24) as f64 * P2_33;
        self.cuc = getbits(buf, 150, 16) as f64 * P2_29;
        self.cus = getbits(buf, 210, 16) as f64 * P2_29;
        let sqrt_a = getbitu2(buf, 226, 8, 240, 24) as f64 * P2_19;
        self.a = sqrt_a * sqrt_a;
        self.toe = getbitu(buf, 270, 16) * 16;
        self.fit = getbitu(buf, 286, 1);

        log::warn!(
            "{sv}: {} tow={} a={:.2} iode={} crs={} crc={} cuc={:+.3e} cus={:+.3e} ecc={:+.3e} m0={:+.4e} toe={}",
            "subframe-2".blue(),
            self.tow,
            self.a,
            self.iode,
            self.crs,
            self.crc,
            self.cuc,
            self.cus,
            self.ecc,
            self.m0,
            self.toe,
        );
    }

    pub fn nav_decode_lnav_subframe3(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        self.cic = getbits(buf, 60, 16) as f64 * P2_29;
        self.cis = getbits(buf, 120, 16) as f64 * P2_29;
        self.omg0 = getbits2(buf, 76, 8, 90, 24) as f64 * P2_31 * SC2RAD;
        self.i0 = getbits2(buf, 136, 8, 150, 24) as f64 * P2_31 * SC2RAD;
        self.crc = getbits(buf, 180, 16) as f64 * P2_5;
        self.omg = getbits2(buf, 196, 8, 210, 24) as f64 * P2_31 * SC2RAD;
        self.omg_dot = getbits(buf, 240, 24) as f64 * P2_43 * SC2RAD;
        self.iode = getbitu(buf, 270, 8);
        self.i_dot = getbits(buf, 278, 14) as f64 * P2_43 * SC2RAD;

        log::warn!(
            "{sv}: {} tow={} cic={:+e} cis={:+e} omg={:.3} omg0={:.3} omgd={:+.3e} i0={:+.3e} idot={:+.3e}",
            "subframe-3".blue(),
            self.tow,
            self.cic,
            self.cis,
            self.omg,
            self.omg0,
            self.omg_dot,
            self.i0,
            self.i_dot
        );
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

    fn hex_to_bytes(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    // Real parity-stripped LNAV subframes captured from G01 in the gpssim_2xi16
    // fixture (the `LNAV: id=N -- <hex>` log lines). Decoding them must recover a
    // physically plausible GPS ephemeris. This locks the bit-field offsets in
    // nav_decode_lnav_subframe{1,2,3}; a regression there would fail the ranges.
    const G01_SF1: &str =
        "8b00000130bc1805c10020000000000000000000000000000ec0174e808000ffb20096b22400";
    const G01_SF2: &str =
        "8b00000130b42405df43d00cc12000adc63303d76c000e2306f0048c28400cef4100e8080c00";
    const G01_SF3: &str =
        "8b00000130b63c0fff05603c788700fff727000a987401a420701c6e3280ffa6fb0177be6400";

    #[test]
    fn decodes_real_lnav_subframes_to_a_valid_ephemeris() {
        let pi = std::f64::consts::PI;
        let sv = SV::new(Constellation::GPS, 1);
        let mut e = Ephemeris::new(sv);
        // The receiver sets ts_sec when it timestamps the first subframe; mimic.
        e.ts_sec = 1.0;

        e.nav_decode_lnav_subframe1(&hex_to_bytes(G01_SF1), sv);
        e.nav_decode_lnav_subframe2(&hex_to_bytes(G01_SF2), sv);
        e.nav_decode_lnav_subframe3(&hex_to_bytes(G01_SF3), sv);

        // Subframe 1 (clock / week).
        assert!(
            (2048..3000).contains(&e.week),
            "GPS week {} implausible",
            e.week
        );
        assert_ne!(e.toc, 0);
        assert!(e.f0.abs() < 1.0e-2, "clock bias {} out of range", e.f0);

        // Subframe 2 (orbit shape). GPS: a ~ 26560 km, near-circular.
        assert!((2.6e7..2.66e7).contains(&e.a), "semi-major axis {} m", e.a);
        assert!((0.0..0.03).contains(&e.ecc), "eccentricity {}", e.ecc);
        assert_ne!(e.toe, 0);
        assert!((-pi..=pi).contains(&e.m0), "mean anomaly {}", e.m0);

        // Subframe 3 (orientation). GPS inclination ~ 55 deg = 0.96 rad.
        assert!((0.9..1.1).contains(&e.i0), "inclination {} rad", e.i0);
        assert!((-pi..=pi).contains(&e.omg0), "ascending node {}", e.omg0);
        assert!(
            e.omg_dot < 0.0 && e.omg_dot > -1.0e-8,
            "omg_dot {}",
            e.omg_dot
        );

        // All three subframes decoded -> the ephemeris passes the solver gate.
        assert!(e.is_valid());
    }
}
