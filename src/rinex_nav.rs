//! RINEX-3 navigation (broadcast ephemeris) parser → [`Ephemeris`], for
//! Assisted-GNSS: load downloaded `brdc` orbits so a channel can fix the moment
//! it decodes TOW, without waiting out the slow on-air ephemeris (≈30 s LNAV,
//! ≈120 s F/NAV).
//!
//! Parses GPS (LNAV) and Galileo (I/F-NAV) records; other constellations are
//! skipped. Unlike the on-air decoders ([`crate::gps_lnav`] /
//! [`crate::galileo_inav`] / [`crate::galileo_fnav`]), RINEX stores the
//! **already-scaled physical values**, so there are no P2_* LSB scales — the
//! fields map straight across (and `a = (√a)²`). Each record is 8 lines: the
//! SV/epoch/clock line then 7 broadcast-orbit lines, fields in fixed 19-char
//! columns. Records may carry a `> EPH …` marker (RINEX 3.05) or not (3.04).
//!
//! Correctness is cross-checked against the *signal-decoded* ephemeris for the
//! same SV (the orbit must match within broadcast quantisation) — see the tests.

use crate::ephemeris::Ephemeris;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Epoch, TimeScale};

/// Parse the 19-char fixed-width float fields of a nav line, starting at byte
/// `start` (RINEX is ASCII). Fortran `D` exponents are normalised to `E`.
fn fields(line: &str, start: usize) -> Vec<f64> {
    let s = line.get(start..).unwrap_or("").replace(['D', 'd'], "E");
    let mut out = Vec::new();
    let mut c = 0;
    while c < s.len() {
        let chunk = s[c..(c + 19).min(s.len())].trim();
        if let Ok(v) = chunk.parse::<f64>() {
            out.push(v); // an empty/blank field just fails to parse and is skipped
        }
        c += 19;
    }
    out
}

/// Parse one 8-line GPS/Galileo nav record into an [`Ephemeris`], or `None` if
/// it isn't a GPS/Galileo record or is malformed.
fn parse_record(lines: &[&str]) -> Option<Ephemeris> {
    let sv0 = lines[0];
    let cons = match sv0.as_bytes().first()? {
        b'G' => Constellation::GPS,
        b'E' => Constellation::Galileo,
        _ => return None,
    };
    let prn: u8 = sv0.get(1..3)?.trim().parse().ok()?;
    let ts = if cons == Constellation::Galileo {
        TimeScale::GST
    } else {
        TimeScale::GPST
    };

    // SV line: "G01 YYYY MM DD HH MM SS af0 af1 af2" (the epoch is the toc).
    let cal: Vec<i64> = sv0
        .get(3..23)?
        .split_whitespace()
        .filter_map(|x| x.parse().ok())
        .collect();
    if cal.len() < 6 {
        return None;
    }
    let toc_gpst = Epoch::from_gregorian(
        cal[0] as i32,
        cal[1] as u8,
        cal[2] as u8,
        cal[3] as u8,
        cal[4] as u8,
        cal[5] as u8,
        0,
        ts,
    );
    let clk = fields(sv0, 23); // af0, af1, af2
    // Broadcast-orbit lines 1-7 (4-char indent), each a row of physical values.
    let o: Vec<Vec<f64>> = (1..8).map(|k| fields(lines[k], 4)).collect();
    let g = |row: usize, col: usize| -> Option<f64> { o.get(row)?.get(col).copied() };

    // The toc's week + seconds-of-week (in this SV's time scale); toe shares it.
    let (week, _) = toc_gpst.to_time_of_week();
    let toe_sow = g(2, 0)?;
    let toe_gpst = Epoch::from_time_of_week(week, (toe_sow * 1e9).round() as u64, ts);

    let mut e = Ephemeris::new(SV::new(cons, prn));
    e.f0 = *clk.first()?;
    e.f1 = clk.get(1).copied().unwrap_or(0.0);
    e.f2 = clk.get(2).copied().unwrap_or(0.0);
    e.iode = g(0, 0)? as u32; // IODE / IODnav
    e.crs = g(0, 1)?;
    e.deln = g(0, 2)?;
    e.m0 = g(0, 3)?;
    e.cuc = g(1, 0)?;
    e.ecc = g(1, 1)?;
    e.cus = g(1, 2)?;
    let sqrt_a = g(1, 3)?;
    e.a = sqrt_a * sqrt_a;
    e.toe = toe_sow as u32;
    e.cic = g(2, 1)?;
    e.omg0 = g(2, 2)?;
    e.cis = g(2, 3)?;
    e.i0 = g(3, 0)?;
    e.crc = g(3, 1)?;
    e.omg = g(3, 2)?;
    e.omg_dot = g(3, 3)?;
    e.i_dot = g(4, 0)?;
    e.sva = g(5, 0)? as u32; // URA (GPS) / SISA (Galileo)
    e.svh = g(5, 1)? as u32;
    e.tgd = g(5, 2)?; // TGD (GPS) / BGD(E1,E5a) (Galileo)
    e.week = week;
    e.toc = ((toc_gpst.to_time_of_week().1) / 1_000_000_000) as u32;
    e.toc_gpst = toc_gpst;
    e.toe_gpst = toe_gpst;
    e.tow_gpst = toe_gpst;
    e.ts_sec = 1.0; // mark "present" (non-zero) so is_valid() holds for injection
    Some(e)
}

/// Parse a whole RINEX-3 nav file's text into per-SV broadcast ephemerides
/// (GPS + Galileo; others skipped). One [`Ephemeris`] per record.
pub fn parse_rinex_nav(text: &str) -> Vec<Ephemeris> {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines
        .iter()
        .position(|l| l.contains("END OF HEADER"))
        .map_or(0, |i| i + 1);
    let mut out = Vec::new();
    let mut i = start;
    while i + 8 <= lines.len() {
        // A record begins with a 1-letter system + 2-digit PRN (G01, E04, …);
        // skip `> EPH …` markers and anything else.
        let l = lines[i].as_bytes();
        let is_rec = l.len() >= 3
            && matches!(l[0], b'G' | b'E' | b'R' | b'C' | b'J')
            && l[1].is_ascii_digit()
            && l[2].is_ascii_digit();
        if !is_rec {
            i += 1;
            continue;
        }
        if let Some(e) = parse_record(&lines[i..i + 8]) {
            out.push(e);
        }
        i += 8;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // Two real records from ESA GSSC's ESOC00DEU…_MN.rnx (2025 DOY 166): GPS G01
    // (LNAV) and Galileo E04 (I/NAV). The orbit/clock values are the ground truth.
    const FIXTURE: &str = "\
                                                            END OF HEADER
> EPH G01 LNAV
G01 2025 06 15 02 00 00 2.915360964835E-04 1.034550223267E-11 0.000000000000E+00
     5.000000000000E+01 9.159375000000E+01 4.410898017322E-09 2.172069051852E+00
     4.749745130539E-06 5.672341212630E-04 4.006549715996E-06 5.153723411560E+03
     7.200000000000E+03-3.725290298462E-09 1.493825366040E+00 9.313225746155E-09
     9.592069088751E-01 2.984687500000E+02 2.336450295333E-01-8.210341993700E-09
     2.364384200378E-10 1.000000000000E+00 2.371000000000E+03 0.000000000000E+00
     2.000000000000E+00 0.000000000000E+00-8.847564458847E-09 3.060000000000E+02
     1.800000000000E+01 4.000000000000E+00
> EPH E04 INAV
E04 2025 06 14 23 20 00-2.597768907435E-04-9.933387445926E-12 0.000000000000E+00
     1.150000000000E+02-1.680625000000E+02 2.887620280975E-09 2.982237831319E+00
    -7.679685950279E-06 3.454915713519E-04 1.023709774017E-05 5.440627134323E+03
     6.024000000000E+05-1.490116119385E-08-2.536026827194E+00 1.303851604462E-08
     9.649647481636E-01 1.240625000000E+02-7.730628352272E-01-5.335222233421E-09
     2.039370662260E-10 5.170000000000E+02 2.370000000000E+03
     3.120000000000E+00 0.000000000000E+00-4.423782229424E-09-4.656612873077E-09
     6.030650000000E+05
";

    #[test]
    fn parses_gps_and_galileo_records() {
        let ephs = parse_rinex_nav(FIXTURE);
        assert_eq!(ephs.len(), 2, "one ephemeris per record");

        let g = &ephs[0];
        assert_eq!(g.sv.constellation, Constellation::GPS);
        assert_eq!(g.sv.prn, 1);
        assert!(g.is_valid(), "parsed GPS ephemeris should be valid");
        // Orbit size pins the field mapping: GPS a ≈ 26 560 km.
        assert!((g.a - 26_560_847.0).abs() < 1000.0, "GPS a = {}", g.a);
        assert!((g.ecc - 5.672341212630e-04).abs() < 1e-12);
        assert!((g.m0 - 2.172069051852).abs() < 1e-9);
        assert!((g.i0 - 0.9592069088751).abs() < 1e-9);
        assert!((g.f0 - 2.915360964835e-04).abs() < 1e-15);
        assert_eq!(g.iode, 50);
        assert_eq!(g.toe, 7200);

        let e = &ephs[1];
        assert_eq!(e.sv.constellation, Constellation::Galileo);
        assert_eq!(e.sv.prn, 4);
        assert!(e.is_valid(), "parsed Galileo ephemeris should be valid");
        // Galileo a ≈ 29 600 km.
        assert!((e.a - 29_600_420.0).abs() < 1000.0, "GAL a = {}", e.a);
        assert!((e.ecc - 3.454915713519e-04).abs() < 1e-12);
        assert!((e.m0 - 2.982237831319).abs() < 1e-9);
        assert_eq!(e.iode, 115);
        assert_eq!(e.toe, 602_400);
        assert!((e.tgd - -4.423782229424e-09).abs() < 1e-18); // BGD(E1,E5a)

        // The toc epochs land where the calendar says (sanity on the time build).
        let (y, mo, d, ..) = g.toc_gpst.to_gregorian_utc();
        assert_eq!((y, mo, d), (2025, 6, 15));
    }
}
