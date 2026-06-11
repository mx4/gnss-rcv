//! SBAS ionospheric grid (DO-229 MT18 + MT26): the first ionosphere
//! correction usable on short captures.
//!
//! SBAS broadcasts vertical ionospheric delays on a fixed grid of Ionospheric
//! Grid Points (IGPs). **MT18** carries, per band (0-10), a 201-bit mask of
//! which IGPs are monitored; **MT26** carries the delays themselves, 15 IGPs
//! per block, in *mask order*. [`SbasIonoGrid::feed`] assembles both into a
//! `(lat, lon) → vertical delay` table, and [`SbasIonoGrid::delay_m`]
//! evaluates the slant correction for one satellite: pierce point at 350 km →
//! 4-point bilinear interpolation over the 5°×5° cell → obliquity factor
//! (DO-229 A.4.4.10). Unlike Klobuchar (whose coefficients arrive once per
//! 12.5 min GPS master frame), EGNOS cycles the active bands within tens of
//! seconds, so even a 40 s capture yields a usable regional grid.
//!
//! The IGP band geometry (which mask bit is which lat/lon) follows DO-229
//! Annex A. Cells are 5°×5° below |lat| 55° and 10°×10° between 55° and 75°
//! (needed as far south as Tampere — high-latitude pierce points land there);
//! polewards of 75° `delay_m` returns `None` and the caller falls back to
//! Klobuchar/none.

use crate::sbas_l1::SbasMessage;
use std::collections::HashMap;

/// Mean Earth radius and ionospheric shell height of the DO-229 model (km).
const RE_KM: f64 = 6378.1363;
const HI_KM: f64 = 350.0;

/// The four latitude ladders of DO-229 Annex A (degrees).
const X1: [i16; 28] = [
    -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35,
    40, 45, 50, 55, 65, 75, 85,
];
const X2: [i16; 23] = [
    -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
    55,
];
const X3: [i16; 27] = [
    -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35,
    40, 45, 50, 55, 65, 75,
];
const X4: [i16; 28] = [
    -85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 65, 75,
];

/// One longitude column of a band: `(lon, lats, first_mask_bit)` — mask bits
/// `first..first+lats.len()` (1-based, per DO-229) are this column, south to
/// north.
type Column = (i16, &'static [i16], usize);

/// Bands 0-8 (the vertical bands): 8 columns each (DO-229 Annex A).
#[rustfmt::skip]
fn band_columns(band: u8) -> &'static [Column] {
    match band {
        0 => &[(-180, &X1, 1), (-175, &X2, 29), (-170, &X3, 52), (-165, &X2, 79),
               (-160, &X3, 102), (-155, &X2, 129), (-150, &X3, 152), (-145, &X2, 179)],
        1 => &[(-140, &X4, 1), (-135, &X2, 29), (-130, &X3, 52), (-125, &X2, 79),
               (-120, &X3, 102), (-115, &X2, 129), (-110, &X3, 152), (-105, &X2, 179)],
        2 => &[(-100, &X3, 1), (-95, &X2, 28), (-90, &X1, 51), (-85, &X2, 79),
               (-80, &X3, 102), (-75, &X2, 129), (-70, &X3, 152), (-65, &X2, 179)],
        3 => &[(-60, &X3, 1), (-55, &X2, 28), (-50, &X4, 51), (-45, &X2, 79),
               (-40, &X3, 102), (-35, &X2, 129), (-30, &X3, 152), (-25, &X2, 179)],
        4 => &[(-20, &X3, 1), (-15, &X2, 28), (-10, &X3, 51), (-5, &X2, 78),
               (0, &X1, 101), (5, &X2, 129), (10, &X3, 152), (15, &X2, 179)],
        5 => &[(20, &X3, 1), (25, &X2, 28), (30, &X3, 51), (35, &X2, 78),
               (40, &X4, 101), (45, &X2, 129), (50, &X3, 152), (55, &X2, 179)],
        6 => &[(60, &X3, 1), (65, &X2, 28), (70, &X3, 51), (75, &X2, 78),
               (80, &X3, 101), (85, &X2, 128), (90, &X1, 151), (95, &X2, 179)],
        7 => &[(100, &X3, 1), (105, &X2, 28), (110, &X3, 51), (115, &X2, 78),
               (120, &X3, 101), (125, &X2, 128), (130, &X4, 151), (135, &X2, 179)],
        8 => &[(140, &X3, 1), (145, &X2, 28), (150, &X3, 51), (155, &X2, 78),
               (160, &X3, 101), (165, &X2, 128), (170, &X3, 151), (175, &X2, 178)],
        _ => &[],
    }
}

/// Bands 9-10 (the horizontal, high-latitude bands): rows of fixed latitude,
/// `(lat, lon_start, lon_step, count, first_mask_bit)` (DO-229 Annex A).
#[rustfmt::skip]
fn band_rows(band: u8) -> &'static [(i16, i16, i16, usize, usize)] {
    match band {
        9 => &[(60, -180, 5, 72, 1), (65, -180, 10, 36, 73), (70, -180, 10, 36, 109),
               (75, -180, 10, 36, 145), (85, -180, 30, 12, 181)],
        10 => &[(-60, -180, 5, 72, 1), (-65, -180, 10, 36, 73), (-70, -180, 10, 36, 109),
                (-75, -180, 10, 36, 145), (-85, -170, 30, 12, 181)],
        _ => &[],
    }
}

/// (lat, lon) of 1-based mask bit `n` of `band`, if defined.
fn igp_coords(band: u8, n: usize) -> Option<(i16, i16)> {
    for &(lon, lats, first) in band_columns(band) {
        if n >= first && n < first + lats.len() {
            return Some((lats[n - first], lon));
        }
    }
    for &(lat, lon0, step, count, first) in band_rows(band) {
        if n >= first && n < first + count {
            return Some((lat, lon0 + step * (n - first) as i16));
        }
    }
    None
}

/// Read `len` bits MSB-first at `pos` from a one-bit-per-byte message.
fn bits(m: &[u8], pos: usize, len: usize) -> u32 {
    m[pos..pos + len]
        .iter()
        .fold(0, |a, &b| (a << 1) | (b & 1) as u32)
}

/// DO-229 GIVEI 0-14 → variance bound (m²) on the residual *vertical* iono
/// error at an IGP after applying its broadcast delay (15 = not monitored,
/// handled as "clear the IGP").
const GIVE_VAR_M2: [f64; 15] = [
    0.0084, 0.0333, 0.0749, 0.1331, 0.2079, 0.2994, 0.4075, 0.5322, 0.6735, 0.8315, 1.1974, 1.8709,
    3.326, 20.787, 187.0826,
];

/// The assembled ionospheric grid.
#[derive(Default, Clone)]
pub struct SbasIonoGrid {
    /// Monitored-IGP lists per band (from MT18), in mask order.
    masks: HashMap<u8, (u8, Vec<(i16, i16)>)>, // band -> (iodi, igps)
    /// Per IGP (from MT26): vertical delay (m) and its GIVE variance (m²).
    delays: HashMap<(i16, i16), (f32, f32)>,
    /// MT26 messages that arrived before their band's MT18 mask (MT26 repeats
    /// every few seconds, MT18 only every ~300 s — on a short capture the
    /// delays usually come first). Replayed when the mask lands.
    pending: HashMap<u8, Vec<Vec<u8>>>, // band -> raw message bits
}

impl SbasIonoGrid {
    /// Number of IGPs with a usable delay.
    pub fn len(&self) -> usize {
        self.delays.len()
    }

    pub fn is_empty(&self) -> bool {
        self.delays.is_empty()
    }

    /// Feed one CRC-valid SBAS message; MT18/MT26 update the grid (true if
    /// the message was one of them).
    pub fn feed(&mut self, msg: &SbasMessage) -> bool {
        match msg.mtype {
            18 => self.feed_mt18(&msg.bits),
            26 => self.feed_mt26(&msg.bits),
            _ => return false,
        }
        true
    }

    /// MT18: 4-bit band count, 4-bit band, 2-bit IODI, 201-bit IGP mask.
    fn feed_mt18(&mut self, m: &[u8]) {
        let band = bits(m, 18, 4) as u8;
        let iodi = bits(m, 22, 2) as u8;
        let mut igps = Vec::new();
        for n in 1..=201 {
            if m[24 + n - 1] == 1
                && let Some(c) = igp_coords(band, n)
            {
                igps.push(c);
            }
        }
        self.masks.insert(band, (iodi, igps));
        // The mask is in: place any delays that were waiting for it.
        if let Some(pend) = self.pending.remove(&band) {
            for m in pend {
                self.feed_mt26(&m);
            }
        }
    }

    /// MT26: 4-bit band, 4-bit block, 15 × (9-bit delay ×0.125 m + 4-bit
    /// GIVEI), 2-bit IODI. Block `b` carries the delays of the band's
    /// mask-order IGPs `15b .. 15b+14`.
    fn feed_mt26(&mut self, m: &[u8]) {
        let band = bits(m, 14, 4) as u8;
        let block = bits(m, 18, 4) as usize;
        let iodi = bits(m, 217, 2) as u8;
        let placeable = matches!(self.masks.get(&band), Some((mi, _)) if *mi == iodi);
        if !placeable {
            // No mask (or a different issue-of-data) yet: park the message
            // until the matching MT18 arrives. Bounded — one band cycles only
            // a handful of distinct blocks.
            let pend = self.pending.entry(band).or_default();
            if pend.len() < 64 {
                pend.push(m.to_vec());
            }
            return;
        }
        let (_, igps) = &self.masks[&band];
        for k in 0..15 {
            let Some(&igp) = igps.get(block * 15 + k) else {
                break;
            };
            let raw = bits(m, 22 + 13 * k, 9);
            let givei = bits(m, 22 + 13 * k + 9, 4);
            // 511 = don't use; GIVEI 15 = not monitored.
            if raw != 511 && givei != 15 {
                let var = GIVE_VAR_M2[givei as usize] as f32;
                self.delays.insert(igp, (raw as f32 * 0.125, var));
            } else {
                self.delays.remove(&igp);
            }
        }
    }

    /// Vertical delay (m) at geographic `(lat, lon)` degrees, by 4-point
    /// bilinear interpolation over the surrounding grid cell (DO-229
    /// A.4.4.10.3): a 5°×5° cell below |lat| 55°, a 10°×10° cell anchored on
    /// the 55°/65°/75° rows between 55° and 75°. `None` polewards of 75°,
    /// or when a cell corner with
    /// non-zero weight lacks a delay (the 3-point fallback is not
    /// implemented); zero-weight corners — the point sits exactly on the
    /// opposite cell edge — may be absent.
    pub fn vertical_delay_m(&self, lat: f64, lon: f64) -> Option<f64> {
        self.vertical_delay_var_m(lat, lon).map(|(d, _)| d)
    }

    /// [`vertical_delay_m`](Self::vertical_delay_m) plus the GIVE variance
    /// (m²), bilinearly interpolated over the same cell — the broadcast bound
    /// on the *residual* vertical iono error after applying the delay.
    pub fn vertical_delay_var_m(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        if lat.abs() >= 75.0 {
            return None;
        }
        let lon = (lon + 180.0).rem_euclid(360.0) - 180.0;
        let (lat0, lon0, cell) = if lat.abs() < 55.0 {
            ((lat / 5.0).floor() * 5.0, (lon / 5.0).floor() * 5.0, 5.0)
        } else {
            // 10° cells on the ...55, 65, 75 latitude rows (both hemispheres).
            (
                ((lat - 5.0) / 10.0).floor() * 10.0 + 5.0,
                (lon / 10.0).floor() * 10.0,
                10.0,
            )
        };
        let x = (lon - lon0) / cell;
        let y = (lat - lat0) / cell;
        let at = |la: f64, lo: f64| -> Option<(f64, f64)> {
            let lo = if lo >= 180.0 { lo - 360.0 } else { lo };
            self.delays
                .get(&(la as i16, lo as i16))
                .map(|&(d, v)| (d as f64, v as f64))
        };
        let corners = [
            ((1.0 - x) * (1.0 - y), at(lat0, lon0)),
            (x * (1.0 - y), at(lat0, lon0 + cell)),
            ((1.0 - x) * y, at(lat0 + cell, lon0)),
            (x * y, at(lat0 + cell, lon0 + cell)),
        ];
        let (mut delay, mut var) = (0.0, 0.0);
        for (w, dv) in corners {
            match dv {
                Some((d, v)) => {
                    delay += w * d;
                    var += w * v;
                }
                None if w == 0.0 => {}
                None => return None,
            }
        }
        Some((delay, var))
    }

    /// Slant ionospheric delay (m) for a satellite at `elev`/`azim` (radians)
    /// seen from `lat`/`lon` (radians): pierce point at 350 km, vertical delay
    /// interpolated from the grid, slant-mapped by the DO-229 obliquity.
    pub fn delay_m(&self, lat: f64, lon: f64, elev: f64, azim: f64) -> Option<f64> {
        self.delay_var_m(lat, lon, elev, azim).map(|(d, _)| d)
    }

    /// [`delay_m`](Self::delay_m) plus the slant residual variance (m²): the
    /// interpolated GIVE variance scaled by obliquity² (vertical → slant).
    pub fn delay_var_m(&self, lat: f64, lon: f64, elev: f64, azim: f64) -> Option<(f64, f64)> {
        let (pp_lat, pp_lon) = pierce_point(lat, lon, elev, azim);
        let (vert, var) = self.vertical_delay_var_m(pp_lat.to_degrees(), pp_lon.to_degrees())?;
        let f = obliquity(elev);
        Some((vert * f, var * f * f))
    }
}

/// Ionospheric pierce point (radians) of the ray at `elev`/`azim` from
/// `lat`/`lon` (radians), for the 350 km thin shell (DO-229 A.4.4.10.1).
pub fn pierce_point(lat: f64, lon: f64, elev: f64, azim: f64) -> (f64, f64) {
    let psi = std::f64::consts::FRAC_PI_2 - elev - (RE_KM / (RE_KM + HI_KM) * elev.cos()).asin();
    let pp_lat = (lat.sin() * psi.cos() + lat.cos() * psi.sin() * azim.cos()).asin();
    let pp_lon = lon + (psi.sin() * azim.sin() / pp_lat.cos()).asin();
    (pp_lat, pp_lon)
}

/// DO-229 obliquity factor mapping vertical to slant delay at `elev` (radians).
pub fn obliquity(elev: f64) -> f64 {
    let c = RE_KM * elev.cos() / (RE_KM + HI_KM);
    1.0 / (1.0 - c * c).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put(m: &mut [u8], pos: usize, len: usize, val: u32) {
        for i in 0..len {
            m[pos + i] = ((val >> (len - 1 - i)) & 1) as u8;
        }
    }

    /// MT18 for `band` with the given IGPs set (must be in that band).
    fn mt18(band: u8, igps: &[(i16, i16)]) -> SbasMessage {
        let mut m = [0u8; 250];
        put(&mut m, 8, 6, 18);
        put(&mut m, 14, 4, 1); // number of bands
        put(&mut m, 18, 4, band as u32);
        put(&mut m, 22, 2, 1); // IODI
        for n in 1..=201 {
            if let Some(c) = igp_coords(band, n)
                && igps.contains(&c)
            {
                m[24 + n - 1] = 1;
            }
        }
        SbasMessage { mtype: 18, bits: m }
    }

    /// MT26 block 0 for `band` with delays (m) for the first mask-order IGPs.
    fn mt26(band: u8, block: usize, delays_m: &[f64]) -> SbasMessage {
        let mut m = [0u8; 250];
        put(&mut m, 8, 6, 26);
        put(&mut m, 14, 4, band as u32);
        put(&mut m, 18, 4, block as u32);
        for k in 0..15 {
            let raw = delays_m
                .get(k)
                .map(|d| (d / 0.125).round() as u32)
                .unwrap_or(511); // beyond the provided list: don't use
            put(&mut m, 22 + 13 * k, 9, raw);
            put(&mut m, 22 + 13 * k + 9, 4, if raw == 511 { 15 } else { 5 });
        }
        put(&mut m, 217, 2, 1); // IODI matching the MT18 above
        SbasMessage { mtype: 26, bits: m }
    }

    #[test]
    fn grid_assembles_and_interpolates() {
        // A 2×2 cell over Catalonia: IGPs (40,0) (40,5) (45,0) (45,5) — all in
        // band 4 (columns 0° = X1 and 5° = X2). Mask order is column-major
        // (lon 0 before lon 5), south to north.
        let igps = [(40, 0), (45, 0), (40, 5), (45, 5)];
        let mut g = SbasIonoGrid::default();
        assert!(g.feed(&mt18(4, &igps)));
        // Mask order: (40,0), (45,0) come first (lon 0 column), then (40,5), (45,5).
        assert!(g.feed(&mt26(4, 0, &[2.0, 4.0, 6.0, 8.0])));
        assert_eq!(g.len(), 4);

        // Exact corners.
        assert_eq!(g.vertical_delay_m(40.0, 0.0), Some(2.0));
        assert_eq!(g.vertical_delay_m(45.0, 5.0), Some(8.0));
        // Cell centre = mean of the corners.
        let c = g.vertical_delay_m(42.5, 2.5).unwrap();
        assert!((c - 5.0).abs() < 1e-9, "centre {c}");
        // Outside the populated cell: no extrapolation.
        assert_eq!(g.vertical_delay_m(48.0, 2.5), None);
        // Polar region unsupported.
        assert_eq!(g.vertical_delay_m(80.0, 2.5), None);
    }

    #[test]
    fn high_latitude_uses_ten_degree_cells() {
        // A Tampere-like pierce point (61.5°N, 24°E) sits in the 10°×10° cell
        // on the 55/65 rows — corners straight from band 5's lat ladders.
        let igps = [(55, 20), (55, 30), (65, 20), (65, 30)];
        let mut g = SbasIonoGrid::default();
        g.feed(&mt18(5, &igps));
        // Mask order: column-major by lon (20, 25, 30...), south to north —
        // (55,20), (65,20) first (lon 20 = X3), then (55,30), (65,30).
        g.feed(&mt26(5, 0, &[2.0, 4.0, 6.0, 8.0]));
        assert_eq!(g.len(), 4);
        assert_eq!(g.vertical_delay_m(55.0, 20.0), Some(2.0));
        let c = g.vertical_delay_m(60.0, 25.0).unwrap();
        assert!((c - 5.0).abs() < 1e-9, "centre {c}");
        assert_eq!(g.vertical_delay_m(61.5, 35.0), None); // unpopulated cell
    }

    #[test]
    fn band9_geometry_spot_checks() {
        // Band 9 rows (60..85°N): bit 1 = (60, -180); the 65° row starts at
        // bit 73 with 10° spacing; the 85° row is 30°-spaced.
        assert_eq!(igp_coords(9, 1), Some((60, -180)));
        assert_eq!(igp_coords(9, 72), Some((60, 175)));
        assert_eq!(igp_coords(9, 74), Some((65, -170)));
        assert_eq!(igp_coords(9, 181), Some((85, -180)));
        assert_eq!(igp_coords(9, 192), Some((85, 150)));
        // Band 10 (south): the 85°S row is offset to -170.
        assert_eq!(igp_coords(10, 181), Some((-85, -170)));
        assert_eq!(igp_coords(10, 193), None);
    }

    #[test]
    fn mt26_before_mt18_is_replayed() {
        // The real-capture order: delays stream every few seconds, the mask
        // only every ~300 s. The buffered MT26 must apply once MT18 lands.
        let igps = [(40, 0), (45, 0)];
        let mut g = SbasIonoGrid::default();
        g.feed(&mt26(4, 0, &[2.0, 4.0]));
        assert!(g.is_empty());
        g.feed(&mt18(4, &igps));
        assert_eq!(g.len(), 2);
        assert_eq!(g.vertical_delay_m(40.0, 0.0), Some(2.0));
    }

    #[test]
    fn mt26_respects_iodi_and_dont_use() {
        let igps = [(40, 0), (45, 0)];
        let mut g = SbasIonoGrid::default();
        g.feed(&mt18(4, &igps));
        // Wrong IODI: rejected.
        let mut bad = mt26(4, 0, &[2.0, 4.0]);
        put(&mut bad.bits, 217, 2, 3);
        g.feed(&bad);
        assert!(g.is_empty());
        // Good IODI: applied; the 511/GIVEI-15 entries stay absent.
        g.feed(&mt26(4, 0, &[2.0]));
        assert_eq!(g.len(), 1);
    }

    #[test]
    fn pierce_and_obliquity_are_sane() {
        // Zenith: pierce point = receiver, obliquity = 1.
        let (la, lo) = pierce_point(0.73, 0.03, std::f64::consts::FRAC_PI_2, 1.0);
        assert!((la - 0.73).abs() < 1e-9 && (lo - 0.03).abs() < 1e-9);
        assert!((obliquity(std::f64::consts::FRAC_PI_2) - 1.0).abs() < 1e-12);
        // 30° elevation: obliquity within the DO-229 plausible band, pierce
        // point a few degrees down-range.
        let f = obliquity(30f64.to_radians());
        assert!((1.5..2.2).contains(&f), "obliquity {f}");
        let (la, _) = pierce_point(0.72, 0.035, 30f64.to_radians(), 0.0); // due north
        assert!(la > 0.72 && la < 0.72 + 0.1);
    }

    #[test]
    fn igp_geometry_matches_do229_spot_checks() {
        // Band 4, bit 101 = first entry of the 0° column (X1 ladder) = -75°.
        assert_eq!(igp_coords(4, 101), Some((-75, 0)));
        // Band 4 last bit: 179 + 23 - 1 = 201 = (55, 15).
        assert_eq!(igp_coords(4, 201), Some((55, 15)));
        // Band 8's mask only reaches bit 200.
        assert_eq!(igp_coords(8, 201), None);
        // Every band 0-8 defines a contiguous 1..=N mask with no holes.
        for band in 0..=8u8 {
            let max = if band == 8 { 200 } else { 201 };
            for n in 1..=max {
                assert!(igp_coords(band, n).is_some(), "band {band} bit {n}");
            }
        }
    }
}
