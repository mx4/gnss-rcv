//! Provisioned IQ recordings, read from `resources/manifest.json`.
//!
//! The manifest is the single source of truth shared between the Python fetch
//! tool ([`resources/fetch.py`](../resources/fetch.py), which writes it) and the
//! desktop UI (which reads it). So the UI's file picker lists exactly the
//! recordings `fetch.py` knows about *and* that are present on disk, and
//! auto-fills each one's `-t`/`--fs`/`--fi`/`--sig` run parameters from the same
//! flags `fetch.py` would use — no second copy of that list to keep in sync.

use crate::code::Signal;
use crate::recording::IQFileType;
use std::path::PathBuf;
use std::str::FromStr;

/// A recording present on disk, with its run parameters parsed from the flags
/// `fetch.py` records for it.
pub struct Recording {
    pub name: String,
    pub path: PathBuf,
    pub iq_file_type: IQFileType,
    pub fs: f64,
    pub fi: f64,
    pub sig: Signal,
    pub note: String,
}

#[derive(serde::Deserialize)]
struct ManifestEntry {
    name: String,
    dest: String,
    flags: String,
    #[serde(default)]
    note: String,
}

/// The recordings from `resources/manifest.json` that are actually present under
/// `resources/`, with their flags parsed into typed run parameters.
///
/// A missing or malformed manifest, an entry whose file isn't downloaded, or
/// flags that don't parse simply yield fewer (or no) entries — the UI then falls
/// back to manual entry. Never errors.
pub fn load_provisioned() -> Vec<Recording> {
    let Ok(text) = std::fs::read_to_string("resources/manifest.json") else {
        return Vec::new();
    };
    let Ok(entries) = serde_json::from_str::<Vec<ManifestEntry>>(&text) else {
        return Vec::new();
    };
    entries
        .into_iter()
        .filter_map(|e| {
            let path = PathBuf::from("resources").join(&e.dest);
            if !path.exists() {
                return None;
            }
            let (iq_file_type, fs, fi, sig) = parse_flags(&e.flags)?;
            Some(Recording {
                name: e.name,
                path,
                iq_file_type,
                fs,
                fi,
                sig,
                note: e.note,
            })
        })
        .collect()
}

/// Pull the `-t`/`--fs`/`--fi`/`--sig` values out of a flags string such as
/// `"-t i8 --fs 26000000 --fi 6390000 --sig E1B"`. `-t` and `--fs` are required;
/// `--fi` defaults to 0 and `--sig` to L1CA, matching the CLI defaults. Returns
/// `None` if a required flag is missing or unparseable.
fn parse_flags(flags: &str) -> Option<(IQFileType, f64, f64, Signal)> {
    let toks: Vec<&str> = flags.split_whitespace().collect();
    let val = |key: &str| {
        toks.iter()
            .position(|&t| t == key)
            .and_then(|i| toks.get(i + 1))
            .copied()
    };
    let iq_file_type = IQFileType::from_str(val("-t")?).ok()?;
    let fs = parse_freq(val("--fs")?)?;
    let fi = val("--fi").and_then(parse_freq).unwrap_or(0.0);
    let sig = val("--sig")
        .and_then(|s| Signal::from_str(s).ok())
        .unwrap_or(Signal::L1ca);
    Some((iq_file_type, fs, fi, sig))
}

/// Parse a frequency that may carry a K/M/G suffix ("10M", "420K") or be plain
/// ("26000000") — the forms `fetch.py` and the CLI both accept.
fn parse_freq(s: &str) -> Option<f64> {
    let s = s.trim();
    let (num, mult) = match s.chars().last() {
        Some('k' | 'K') => (&s[..s.len() - 1], 1e3),
        Some('m' | 'M') => (&s[..s.len() - 1], 1e6),
        Some('g' | 'G') => (&s[..s.len() - 1], 1e9),
        _ => (s, 1.0),
    };
    num.trim().parse::<f64>().ok().map(|v| v * mult)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_full_flag_string() {
        let (t, fs, fi, sig) =
            parse_flags("-t i8 --fs 26000000 --fi 6390000 --sig E1B").unwrap();
        assert_eq!(t, IQFileType::TypeOneInt8);
        assert_eq!(fs, 26_000_000.0);
        assert_eq!(fi, 6_390_000.0);
        assert_eq!(sig, Signal::GalileoE1b);
    }

    #[test]
    fn defaults_fi_and_sig_when_absent_and_honors_suffixes() {
        let (t, fs, fi, sig) = parse_flags("-t 2xi16 --fs 4M").unwrap();
        assert_eq!(t, IQFileType::TypePairInt16);
        assert_eq!(fs, 4_000_000.0);
        assert_eq!(fi, 0.0); // no --fi
        assert_eq!(sig, Signal::L1ca); // no --sig
    }

    #[test]
    fn rejects_a_flag_string_missing_required_fields() {
        assert!(parse_flags("--fs 4000000").is_none()); // no -t
        assert!(parse_flags("-t 2xi16").is_none()); // no --fs
    }
}
