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
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;

/// GSSC daily stations to try, in order. Any IGS station's `brdc` carries the
/// same broadcast ephemeris, but no single station is archived *every* day
/// (ESOC, e.g., is on GSSC for 2025 but absent for 2022-03-27), so we fall
/// through the list. All are ESA/IGS core sites with long, well-archived records.
const BRDC_STATIONS: [&str; 5] = [
    "ESOC00DEU", // ESA operations centre, Germany
    "KIRU00SWE", // ESA Kiruna, Sweden
    "CEBR00ESP", // ESA Cebreros, Spain
    "KOUR00GUF", // ESA Kourou, French Guiana
    "MGUE00ARG", // ESA Malargüe, Argentina
];

/// RINEX-3 mixed-navigation filename (GPS + Galileo + …) for `station` and
/// `year` / `doy` (day-of-year). The downloaded `.gz` decompresses to this name.
fn mn_name(station: &str, year: u32, doy: u32) -> String {
    format!("{station}_R_{year}{doy:03}0000_01D_MN.rnx")
}

/// The default station's mixed-nav filename — public for callers / tests.
pub fn brdc_mn_name(year: u32, doy: u32) -> String {
    mn_name(BRDC_STATIONS[0], year, doy)
}

/// ESA GSSC (no-auth) URL of `station`'s compressed daily mixed-nav.
fn mn_url(station: &str, year: u32, doy: u32) -> String {
    format!(
        "ftp://gssc.esa.int/gnss/data/daily/{year}/{doy:03}/{}.gz",
        mn_name(station, year, doy)
    )
}

/// RINEX-2 combined GPS broadcast-nav filename (the IGS `brdc….n`, archived on
/// GSSC back to the 1990s — pre-dates RINEX-3 mixed-nav). GPS-only.
fn brdc2_name(year: u32, doy: u32) -> String {
    format!("brdc{doy:03}0.{:02}n", year % 100)
}

/// Download `url` to `path` with curl (FTP, no auth); true on success.
fn curl_to(path: &Path, url: &str) -> bool {
    Command::new("curl")
        .args(["-sS", "--fail", "--connect-timeout", "8", "-m", "30", "-o"])
        .arg(path)
        .arg(url)
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// `curl` `url` then `gunzip` (handles both `.gz` and `.Z`) into `local`.
fn fetch_compressed(local: &Path, archive: &Path, url: &str) -> std::io::Result<PathBuf> {
    if !curl_to(archive, url) {
        let _ = std::fs::remove_file(archive);
        return Err(std::io::Error::other("download failed"));
    }
    let unzipped = Command::new("gunzip")
        .arg("-f")
        .arg(archive)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    if !unzipped || !local.exists() {
        return Err(std::io::Error::other("gunzip failed"));
    }
    Ok(local.to_path_buf())
}

/// Trivial read-only ephemeris cache: if a `brdc` for the day is already in
/// `dir`, use it; otherwise download one from ESA GSSC (no auth) and decompress.
/// Tries RINEX-3 mixed-nav (GPS+Galileo) across a list of stations first, then
/// falls back to the older RINEX-2 combined GPS brdc (GPS-only) for dates the
/// RINEX-3 archive doesn't reach. Returns the local RINEX path. Shells out to
/// `curl` + `gunzip` — the tools `fetch.py` / `gen_gpssim.py` use — so no runtime
/// crate dependency; the cached file is never modified (read-only).
pub fn ensure_brdc(dir: &Path, year: u32, doy: u32) -> std::io::Result<PathBuf> {
    // Cache hit: any candidate RINEX-3 station file, or the RINEX-2 brdc.
    for station in BRDC_STATIONS {
        let local = dir.join(mn_name(station, year, doy));
        if local.exists() {
            return Ok(local);
        }
    }
    let r2 = dir.join(brdc2_name(year, doy));
    if r2.exists() {
        return Ok(r2);
    }
    // Download: RINEX-3 mixed-nav (GPS+Galileo) first…
    for station in BRDC_STATIONS {
        log::warn!(
            "A-GNSS: ephemeris not cached, trying {}",
            mn_url(station, year, doy)
        );
        let local = dir.join(mn_name(station, year, doy));
        let gz = dir.join(format!("{}.gz", mn_name(station, year, doy)));
        if let Ok(p) = fetch_compressed(&local, &gz, &mn_url(station, year, doy)) {
            return Ok(p);
        }
    }
    // …else the older RINEX-2 combined GPS brdc (GPS-only).
    let url2 = format!(
        "ftp://gssc.esa.int/gnss/data/daily/{year}/{doy:03}/{}.Z",
        brdc2_name(year, doy)
    );
    log::warn!("A-GNSS: no RINEX-3 mixed-nav for {year}/{doy:03}; trying RINEX-2 GPS brdc {url2}");
    let z = dir.join(format!("{}.Z", brdc2_name(year, doy)));
    if let Ok(p) = fetch_compressed(&r2, &z, &url2) {
        return Ok(p);
    }
    Err(std::io::Error::other(
        "brdc download failed (offline, or no file for that day yet)",
    ))
}

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

/// Map a broadcast-orbit field table — the clock triple plus orbit lines 1-7,
/// whose layout is identical across RINEX-2/3 and GPS/Galileo — into an
/// [`Ephemeris`]. `toc_gpst` is the record's clock-reference epoch, `ts` its
/// time scale. Shared by the RINEX-3 and RINEX-2 record parsers.
fn eph_from_fields(
    sv: SV,
    ts: TimeScale,
    toc_gpst: Epoch,
    clk: &[f64],
    o: &[Vec<f64>],
) -> Option<Ephemeris> {
    let g = |row: usize, col: usize| -> Option<f64> { o.get(row)?.get(col).copied() };
    // The toc's week + seconds-of-week (in this SV's time scale); toe shares it.
    let (week, _) = toc_gpst.to_time_of_week();
    let toe_sow = g(2, 0)?;
    let toe_gpst = Epoch::from_time_of_week(week, (toe_sow * 1e9).round() as u64, ts);

    let mut e = Ephemeris::new(sv);
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

/// Parse one 8-line RINEX-3 GPS/Galileo nav record, or `None` if it isn't a
/// GPS/Galileo record or is malformed.
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
    // Broadcast-orbit lines 1-7 at the RINEX-3 4-char indent.
    let o: Vec<Vec<f64>> = (1..8).map(|k| fields(lines[k], 4)).collect();
    eph_from_fields(SV::new(cons, prn), ts, toc_gpst, &clk, &o)
}

/// Parse one 8-line RINEX-2 GPS nav record: no system letter (PRN in cols 1-2),
/// 2-digit year (pivot at 80), and a 3-char orbit indent vs RINEX-3's 4. The
/// older `.n` files are GPS-only.
fn parse_rinex2_record(lines: &[&str]) -> Option<Ephemeris> {
    let sv0 = lines[0];
    let prn: u8 = sv0.get(0..2)?.trim().parse().ok()?;
    // Epoch "YY MM DD HH MM SS.S"; take the integer part of each field.
    let t: Vec<i64> = sv0
        .get(2..22)?
        .split_whitespace()
        .filter_map(|x| x.split('.').next()?.parse().ok())
        .collect();
    if t.len() < 6 {
        return None;
    }
    let year = if t[0] >= 80 { 1900 + t[0] } else { 2000 + t[0] };
    let toc_gpst = Epoch::from_gregorian(
        year as i32,
        t[1] as u8,
        t[2] as u8,
        t[3] as u8,
        t[4] as u8,
        t[5] as u8,
        0,
        TimeScale::GPST,
    );
    let clk = fields(sv0, 22); // af0, af1, af2
    // Broadcast-orbit lines 1-7 at the RINEX-2 3-char indent.
    let o: Vec<Vec<f64>> = (1..8).map(|k| fields(lines[k], 3)).collect();
    eph_from_fields(
        SV::new(Constellation::GPS, prn),
        TimeScale::GPST,
        toc_gpst,
        &clk,
        &o,
    )
}

/// Parse a RINEX nav file into per-SV broadcast ephemerides, auto-detecting the
/// version from the header: RINEX-3 carries GPS + Galileo (others skipped); the
/// older RINEX-2 `.n` is GPS-only. One [`Ephemeris`] per record.
pub fn parse_rinex_nav(text: &str) -> Vec<Ephemeris> {
    let version: f64 = text
        .lines()
        .next()
        .and_then(|l| l.get(0..9))
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(3.0);
    if version < 3.0 {
        parse_rinex2_nav(text)
    } else {
        parse_rinex3_nav(text)
    }
}

/// Walk a RINEX-3 nav file: each record begins with a 1-letter system + 2-digit
/// PRN (G01, E04, …); `> EPH …` markers and anything else are skipped.
fn parse_rinex3_nav(text: &str) -> Vec<Ephemeris> {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines
        .iter()
        .position(|l| l.contains("END OF HEADER"))
        .map_or(0, |i| i + 1);
    let mut out = Vec::new();
    let mut i = start;
    while i + 8 <= lines.len() {
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

/// Walk a RINEX-2 GPS nav file: each record line begins with a (space-padded)
/// PRN in cols 1-2.
fn parse_rinex2_nav(text: &str) -> Vec<Ephemeris> {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines
        .iter()
        .position(|l| l.contains("END OF HEADER"))
        .map_or(0, |i| i + 1);
    let mut out = Vec::new();
    let mut i = start;
    while i + 8 <= lines.len() {
        let is_rec = lines[i]
            .get(0..2)
            .is_some_and(|s| s.trim().parse::<u8>().is_ok());
        if !is_rec {
            i += 1;
            continue;
        }
        if let Some(e) = parse_rinex2_record(&lines[i..i + 8]) {
            out.push(e);
        }
        i += 8;
    }
    out
}

/// Parse "YYYY-MM-DD" or "YYYY-MM-DDThh:mm:ss" into a GPST [`Epoch`]. A date with
/// no time defaults to 12:00 (mid-day) — a daily brdc is centred there, so it is
/// the least-bad reference when only the day is known.
pub fn parse_ref_epoch(s: &str) -> Option<Epoch> {
    let (date, time) = s.split_once('T').unwrap_or((s, "12:00:00"));
    let mut d = date.split('-');
    let y: i32 = d.next()?.trim().parse().ok()?;
    let mo: u8 = d.next()?.parse().ok()?;
    let da: u8 = d.next()?.parse().ok()?;
    let mut t = time.split(':');
    let h: u8 = t.next()?.parse().ok()?;
    let mi: u8 = t.next().and_then(|x| x.parse().ok()).unwrap_or(0);
    let se: u8 = t.next().and_then(|x| x.parse().ok()).unwrap_or(0);
    Some(Epoch::from_gregorian(
        y,
        mo,
        da,
        h,
        mi,
        se,
        0,
        TimeScale::GPST,
    ))
}

/// Year + day-of-year (1-based) for `e`, the address of the GSSC daily brdc.
fn year_doy(e: Epoch) -> (u32, u32) {
    let (y, ..) = e.to_gregorian_utc();
    let jan1 = Epoch::from_gregorian(y, 1, 1, 0, 0, 0, 0, TimeScale::GPST);
    let doy = ((e - jan1).to_seconds() / 86_400.0).floor() as u32 + 1;
    (y as u32, doy)
}

/// Resolve, parse, and select the broadcast ephemeris to inject for Assisted-GNSS.
/// `eph` is a RINEX-3 nav file path, or `"auto"` to download the day's brdc from
/// ESA GSSC (caching in `cache_dir`; needs `date`). `date` (optional) is the
/// reference epoch used to pick, per SV, the record whose `toe` is nearest — and,
/// for `"auto"`, the day to fetch. Without `date`, the file's median `toe` is the
/// reference. Returns SV → chosen ephemeris.
pub fn load_assist_ephemerides(
    eph: &str,
    date: Option<&str>,
    cache_dir: &Path,
) -> std::io::Result<HashMap<SV, Ephemeris>> {
    let ref_epoch = date.and_then(parse_ref_epoch);
    let path = if eph == "auto" {
        let r = ref_epoch.ok_or_else(|| {
            std::io::Error::other("--eph auto needs --eph-date YYYY-MM-DD[Thh:mm:ss]")
        })?;
        let (y, doy) = year_doy(r);
        ensure_brdc(cache_dir, y, doy)?
    } else {
        PathBuf::from(eph)
    };
    let all = parse_rinex_nav(&std::fs::read_to_string(&path)?);
    if all.is_empty() {
        return Err(std::io::Error::other(format!(
            "no GPS/Galileo ephemerides parsed from {}",
            path.display()
        )));
    }
    // Reference for the per-SV pick: the given epoch, else the file's median toe.
    let reference = ref_epoch.unwrap_or_else(|| {
        let mut toes: Vec<Epoch> = all.iter().map(|e| e.toe_gpst).collect();
        toes.sort();
        toes[toes.len() / 2]
    });
    let mut by_sv: HashMap<SV, Ephemeris> = HashMap::new();
    for e in all {
        if !e.is_valid() {
            continue;
        }
        let closer = by_sv
            .get(&e.sv)
            .is_none_or(|cur| (e.toe_gpst - reference).abs() < (cur.toe_gpst - reference).abs());
        if closer {
            by_sv.insert(e.sv, e);
        }
    }
    Ok(by_sv)
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

    // A real RINEX-2 GPS record from GSSC's brdc0940.13n (2013 DOY 094), PRN 1.
    // Note the format vs RINEX-3: no system letter, 2-digit year, 3-char indent.
    const FIXTURE_R2: &str = "\
     2              NAVIGATION DATA                         RINEX VERSION / TYPE
                                                            END OF HEADER
 1 13  4  4  0  0  0.0 0.130245462060D-04 0.329691829393D-11 0.000000000000D+00
    0.660000000000D+02 0.110312500000D+02 0.445589989183D-08-0.129411133012D+01
    0.603497028351D-06 0.170189398341D-02 0.977143645287D-05 0.515370147133D+04
    0.345600000000D+06-0.931322574616D-08-0.462984461481D+00-0.931322574615D-07
    0.960438103656D+00 0.193500000000D+03 0.315473100675D+00-0.795818863336D-08
    0.174292974288D-09 0.100000000000D+01 0.173400000000D+04 0.000000000000D+00
    0.200000000000D+01 0.000000000000D+00 0.838190317154D-08 0.660000000000D+02
    0.338418000000D+06 0.400000000000D+01 0.000000000000D+00 0.000000000000D+00";

    #[test]
    fn parses_rinex2_gps_record() {
        let ephs = parse_rinex_nav(FIXTURE_R2); // version auto-detected as 2
        assert_eq!(ephs.len(), 1);
        let g = &ephs[0];
        assert_eq!(g.sv.constellation, Constellation::GPS);
        assert_eq!(g.sv.prn, 1);
        assert!(g.is_valid());
        assert_eq!(g.week, 1734);
        assert_eq!(g.toe, 345_600);
        assert_eq!(g.iode, 66);
        assert!((g.a - 5153.70147133_f64.powi(2)).abs() < 1.0, "a = {}", g.a);
        assert!((g.ecc - 0.001_701_893_983_41).abs() < 1e-12);
        assert!((g.i0 - 0.960_438_103_656).abs() < 1e-12);
        assert!((g.tgd - 0.838_190_317_154e-8).abs() < 1e-18);
        // 2-digit "13" → 2013; epoch is GPST (00:00 GPST is 2013-04-03 in UTC).
        assert_eq!(
            g.toc_gpst,
            Epoch::from_gregorian(2013, 4, 4, 0, 0, 0, 0, TimeScale::GPST)
        );
    }

    #[test]
    fn brdc_filename_and_url() {
        assert_eq!(
            brdc_mn_name(2025, 166),
            "ESOC00DEU_R_20251660000_01D_MN.rnx"
        );
        // day-of-year is zero-padded to 3 digits.
        assert_eq!(brdc_mn_name(2025, 9), "ESOC00DEU_R_20250090000_01D_MN.rnx");
        let url = mn_url(BRDC_STATIONS[0], 2025, 166);
        assert!(url.starts_with("ftp://gssc.esa.int/"));
        assert!(url.ends_with("ESOC00DEU_R_20251660000_01D_MN.rnx.gz"));
    }

    #[test]
    fn cache_hit_uses_existing_file_without_downloading() {
        // An already-cached file is returned as-is (no curl invocation).
        let dir = std::env::temp_dir();
        let p = dir.join(brdc_mn_name(1999, 1));
        std::fs::write(&p, b"cached").unwrap();
        let got = ensure_brdc(&dir, 1999, 1).expect("cache hit");
        assert_eq!(got, p);
        std::fs::remove_file(&p).ok();
    }

    #[test]
    fn parse_ref_epoch_date_and_datetime() {
        // date-only defaults to mid-day (a daily brdc is centred there).
        assert_eq!(
            parse_ref_epoch("2025-06-15"),
            Some(Epoch::from_gregorian(
                2025,
                6,
                15,
                12,
                0,
                0,
                0,
                TimeScale::GPST
            ))
        );
        assert_eq!(
            parse_ref_epoch("2025-06-15T06:30:00"),
            Some(Epoch::from_gregorian(
                2025,
                6,
                15,
                6,
                30,
                0,
                0,
                TimeScale::GPST
            ))
        );
        assert!(parse_ref_epoch("nonsense").is_none());
    }

    #[test]
    fn year_doy_maps_calendar_to_day_of_year() {
        assert_eq!(
            year_doy(parse_ref_epoch("2025-06-15").unwrap()),
            (2025, 166)
        );
        assert_eq!(year_doy(parse_ref_epoch("2025-01-01").unwrap()), (2025, 1));
        assert_eq!(
            year_doy(parse_ref_epoch("2024-12-31").unwrap()),
            (2024, 366)
        );
    }

    #[test]
    fn load_assist_selects_per_sv_from_a_file() {
        let dir = std::env::temp_dir();
        let p = dir.join("gnss_assist_fixture.rnx");
        std::fs::write(&p, FIXTURE).unwrap();
        let by_sv = load_assist_ephemerides(p.to_str().unwrap(), Some("2025-06-15"), &dir).unwrap();
        // FIXTURE carries G01 (GPS) and E04 (Galileo), one record each.
        assert_eq!(by_sv.len(), 2);
        assert!(
            by_sv
                .keys()
                .any(|sv| sv.constellation == Constellation::GPS)
        );
        assert!(
            by_sv
                .keys()
                .any(|sv| sv.constellation == Constellation::Galileo)
        );
        assert!(by_sv.values().all(|e| e.is_valid()));
        std::fs::remove_file(&p).ok();
    }

    // The whole A-GNSS fetch path end-to-end: download a real brdc from ESA GSSC,
    // parse it, and confirm the cache hits on the second call. Needs network, so
    // it skips cleanly when offline.
    #[test]
    #[ignore = "network: downloads a real brdc from ESA GSSC"]
    fn downloads_caches_and_parses_real_brdc() {
        let dir = std::env::temp_dir().join("gnss_agnss_test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = match ensure_brdc(&dir, 2025, 166) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("skipping: {e}");
                return;
            }
        };
        let ephs = parse_rinex_nav(&std::fs::read_to_string(&path).unwrap());
        let gps = ephs
            .iter()
            .filter(|e| e.sv.constellation == Constellation::GPS)
            .count();
        let gal = ephs
            .iter()
            .filter(|e| e.sv.constellation == Constellation::Galileo)
            .count();
        let valid = ephs.iter().filter(|e| e.is_valid()).count();
        eprintln!(
            "parsed {} ephemerides ({gps} GPS, {gal} Galileo), {valid} valid",
            ephs.len()
        );
        assert!(
            gps > 20 && gal > 10,
            "expected many GPS + Galileo ephemerides"
        );
        // Essentially all are valid — including the Sunday-00:00 records whose
        // toe/toc == 0 (the week boundary), now that is_valid() gates on orbit
        // values rather than a 0-sentinel on toe/toc (see Phase 2 boundary fix).
        assert!(
            valid > ephs.len() * 99 / 100,
            "nearly all ephemerides should be valid, got {valid}/{}",
            ephs.len()
        );
        // Second call is a cache hit (same path, no re-download).
        assert_eq!(ensure_brdc(&dir, 2025, 166).unwrap(), path);
        std::fs::remove_dir_all(&dir).ok();
    }

    // The RINEX-2 fallback end-to-end: 2013 predates GSSC's RINEX-3 mixed-nav, so
    // ensure_brdc must fall through to the combined GPS brdc….n.Z (the cttc case).
    #[test]
    #[ignore = "network: downloads a real RINEX-2 brdc from ESA GSSC"]
    fn downloads_and_parses_rinex2_brdc_2013() {
        let dir = std::env::temp_dir().join("gnss_agnss_r2");
        std::fs::create_dir_all(&dir).unwrap();
        let path = match ensure_brdc(&dir, 2013, 94) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("skipping: {e}");
                return;
            }
        };
        assert!(path.to_string_lossy().ends_with(".13n")); // RINEX-2 GPS brdc
        let ephs = parse_rinex_nav(&std::fs::read_to_string(&path).unwrap());
        let gps = ephs
            .iter()
            .filter(|e| e.sv.constellation == Constellation::GPS && e.is_valid())
            .count();
        eprintln!(
            "RINEX-2 2013/094: {gps} valid GPS eph from {}",
            path.display()
        );
        assert!(gps > 20, "expected many GPS ephemerides, got {gps}");
        std::fs::remove_dir_all(&dir).ok();
    }
}
