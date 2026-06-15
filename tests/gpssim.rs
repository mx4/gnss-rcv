//! Integration regression tests against gps-sdr-sim recordings.
//!
//! These drive the full receiver pipeline (acquisition -> tracking -> nav
//! decode -> position fix). Two of them run on a pre-existing recording
//! `resources/gpssim_2xi16` (see resources/README.md); a third generates a
//! fresh recording end-to-end (download ephemeris -> gps-sdr-sim -> solve).
//!
//! All recordings are large and gitignored (absent in CI), and the generator
//! needs external tools + network, so every test skips cleanly (prints
//! `skipping...` and passes) when its inputs aren't available.

use std::path::Path;
use std::process::Command;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use gnss_rcv::channel::State;
use gnss_rcv::receiver::{Receiver, ReceiverConfig};
use gnss_rcv::recording::IQFileType;
use gnss_rcv::state::GnssState;

const GPSSIM: &str = "resources/gpssim_2xi16";

// The static location gps-sdr-sim was told to simulate: Geneva (Jet d'Eau),
// 2026/04/28 17:00 -- the default scenario of resources/gen_gpssim.py.
const TRUE_LAT: f64 = 46.2075;
const TRUE_LON: f64 = 6.1557;

// PRNs present in that scenario (see resources/gpssim_gen.txt). Restricting the
// search to these keeps the tests fast and avoids cross-correlation noise.
const SIM_SATS: &str = "1,2,3,4,6,9,17,19,28,31";

// End-to-end generated recording + the meta the generator script writes.
const GEN_SCRIPT: &str = "resources/gen_gpssim.py";
const GEN_GPSSIM: &str = "resources/gpssim_gen_2xi16";
const GEN_META: &str = "resources/gpssim_gen.meta";

/// Run the receiver over `num_msec` of `recording` (0 = until EOF/fix),
/// restricting the search to `sats`. Returns the shared state, or `None`
/// (skip) when the recording isn't present.
fn run(
    recording: &str,
    sats: &str,
    num_msec: usize,
    exit_on_fix: bool,
) -> Option<Arc<Mutex<GnssState>>> {
    if !Path::new(recording).exists() {
        eprintln!("skipping: {recording} not present (see resources/README.md)");
        return None;
    }

    let state = Arc::new(Mutex::new(GnssState::new()));
    let exit_req = Arc::new(AtomicBool::new(false));
    let cfg = ReceiverConfig {
        file: Path::new(recording).to_path_buf(),
        iq_file_type: IQFileType::TypePairInt16,
        sats: sats.to_string(),
        exit_on_fix,
        ..Default::default()
    };
    let mut rx = Receiver::new(&cfg, exit_req, state.clone()).expect("receiver");
    rx.run_loop(num_msec);
    Some(state)
}

/// Assert the receiver's computed fix is within 0.02 deg of `(lat, lon)`.
fn assert_fix_near(state: &Arc<Mutex<GnssState>>, lat: f64, lon: f64) {
    let st = state.lock().unwrap();
    let (got_lat, got_lon) = (st.latitude, st.longitude);
    assert!(
        (got_lat - lat).abs() < 0.02 && (got_lon - lon).abs() < 0.02,
        "fix {got_lat:.5},{got_lon:.5} not within 0.02 deg of {lat},{lon}"
    );
}

/// Fast signal: acquisition + tracking lock onto the simulated satellites.
#[test]
fn acquires_and_tracks_gpssim() {
    let Some(state) = run(GPSSIM, SIM_SATS, 5_000, false) else {
        return;
    };

    let tracking = state
        .lock()
        .unwrap()
        .channels
        .values()
        .filter(|c| c.state == State::Tracking)
        .count();

    assert!(
        tracking >= 4,
        "expected >=4 tracking SVs after 5s, got {tracking}"
    );
}

/// Full pipeline through a position fix on the pre-existing recording. Needs
/// ~40s of signal (three subframes to decode an ephemeris, then >=4 SVs to
/// solve); runs until the first fix (exit_on_fix) or EOF, so it is `#[ignore]`'d;
/// run it explicitly with `cargo test -- --ignored`.
#[test]
#[ignore = "slow: runs the pipeline until the first position fix"]
fn computes_position_fix_gpssim() {
    let Some(state) = run(GPSSIM, SIM_SATS, 0, true) else {
        return;
    };
    assert_fix_near(&state, TRUE_LAT, TRUE_LON);
}

/// End-to-end: generate a fresh recording (pick a date+location, download the
/// matching broadcast ephemeris, run gps-sdr-sim) and verify the receiver
/// recovers that location. Needs gps-sdr-sim + network, so it is `#[ignore]`'d
/// and skips cleanly when the generator reports its tools/network are missing.
#[test]
#[ignore = "needs gps-sdr-sim + network: generates a fresh recording end-to-end"]
fn generates_and_solves_gpssim() {
    // Steps 1-4: select date/location, download ephemeris, run gps-sdr-sim.
    match Command::new(GEN_SCRIPT).status() {
        Ok(s) if s.success() => {}
        Ok(s) => {
            eprintln!("skipping: {GEN_SCRIPT} exited {s} (missing gps-sdr-sim/network?)");
            return;
        }
        Err(e) => {
            eprintln!("skipping: cannot run {GEN_SCRIPT}: {e}");
            return;
        }
    }

    // Read truth location + visible PRNs the generator recorded.
    let meta = std::fs::read_to_string(GEN_META).expect("generator wrote a meta file");
    let get = |key: &str| -> String {
        meta.lines()
            .find_map(|l| l.strip_prefix(&format!("{key}=")))
            .unwrap_or_else(|| panic!("meta missing '{key}'"))
            .to_string()
    };
    let lat: f64 = get("lat").parse().expect("meta lat");
    let lon: f64 = get("lon").parse().expect("meta lon");
    let sats = get("sats");

    // Step 5: run the receiver against the generated recording and check the fix.
    let state = run(GEN_GPSSIM, &sats, 0, true).expect("generated recording present");
    assert_fix_near(&state, lat, lon);
}
