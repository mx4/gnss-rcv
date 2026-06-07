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
use gnss_rcv::receiver::Receiver;
use gnss_rcv::recording::IQFileType;
use gnss_rcv::state::GnssState;

const GPSSIM: &str = "resources/gpssim_2xi16";

// The static location gps-sdr-sim was told to simulate.
const TRUE_LAT: f64 = 35.681298;
const TRUE_LON: f64 = 139.766247;

// PRNs actually present in the sim (see resources/gpssim.txt). Restricting the
// search to these keeps the tests fast and avoids cross-correlation noise.
const SIM_SATS: &str = "5,10,12,13,15,18,23,24,25,28,32";

// End-to-end generated recording + the meta the generator script writes.
const GEN_SCRIPT: &str = "resources/gen_gpssim.sh";
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
    let mut rx = Receiver::new(
        false, // use_device
        "",    // hostname (rtl_tcp)
        Path::new(recording),
        &IQFileType::TypePairInt16,
        2_046_000.0, // fs
        0.0,         // fi
        0,           // off_msec
        "L1CA",      // sig
        sats,
        false, // plots
        exit_on_fix,
        exit_req,
        state.clone(),
    );
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
/// solve) but stops at the first fix via exit_on_fix, so it is `#[ignore]`'d;
/// run it explicitly with `cargo test -- --ignored`.
#[test]
#[ignore = "slow: runs the pipeline until the first position fix"]
fn computes_position_fix_gpssim() {
    let Some(state) = run(GPSSIM, SIM_SATS, 40_000, true) else {
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
