//! Integration regression test against a known gps-sdr-sim recording.
//!
//! Drives the full receiver pipeline (acquisition -> tracking -> nav decode ->
//! position fix) on `resources/gpssim_2xi16`, the recording documented in
//! resources/README.md, generated with:
//!
//!   gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 \
//!               -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
//!
//! That recording is large and gitignored (absent in CI), so both tests skip
//! cleanly when it isn't present; generate it locally to exercise them.

use std::path::Path;
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

/// Run the receiver over `num_msec` of the gpssim recording and return the
/// resulting shared state, or `None` (skip) when the recording isn't present.
fn run(num_msec: usize) -> Option<Arc<Mutex<GnssState>>> {
    if !Path::new(GPSSIM).exists() {
        eprintln!("skipping: {GPSSIM} not present (see resources/README.md)");
        return None;
    }
    // update_all_plots() writes PNGs into plots/; ensure it exists for headless runs.
    let _ = std::fs::create_dir_all("plots");

    let state = Arc::new(Mutex::new(GnssState::new()));
    let exit_req = Arc::new(AtomicBool::new(false));
    let mut rx = Receiver::new(
        false, // use_device
        "",    // hostname (rtl_tcp)
        Path::new(GPSSIM),
        &IQFileType::TypePairInt16,
        2_046_000.0, // fs
        0.0,         // fi
        0,           // off_msec
        "L1CA",      // sig
        SIM_SATS,    // sats
        false,       // plots
        exit_req,
        state.clone(),
    );
    rx.run_loop(num_msec);
    Some(state)
}

/// Fast signal: acquisition + tracking lock onto the simulated satellites.
#[test]
fn acquires_and_tracks_gpssim() {
    let Some(state) = run(5_000) else { return };

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

/// Full pipeline through a position fix. Needs ~40s of signal (three subframes
/// to decode an ephemeris, then >=4 SVs to solve), so it is marked `#[ignore]`;
/// run it explicitly with `cargo test -- --ignored`.
#[test]
#[ignore = "slow: processes ~40s of IQ to reach a position fix"]
fn computes_position_fix_gpssim() {
    let Some(state) = run(40_000) else { return };

    let st = state.lock().unwrap();
    let (lat, lon) = (st.latitude, st.longitude);
    assert!(
        (lat - TRUE_LAT).abs() < 0.02 && (lon - TRUE_LON).abs() < 0.02,
        "fix {lat:.5},{lon:.5} not within 0.02 deg of {TRUE_LAT},{TRUE_LON}"
    );
}
