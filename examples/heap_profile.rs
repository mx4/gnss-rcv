//! Heap profile of a SJTU E1B (25 MHz Galileo) receiver run, to attribute the
//! ~10 MiB/channel that RSS accounting couldn't pin down. dhat hooks the global
//! allocator and writes dhat-heap.json (open in https://nnethercote.github.io/
//! dh_view/dh_view.html) plus a peak summary to stderr on exit.
//!
//!   cargo run --release --example heap_profile -- 1000
//!
//! Arg: msec of data (default 1000 — the peak is the cold-start acquisition,
//! reached early). dhat is slow (a backtrace per allocation), so keep it short.

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

use gnss_rcv::code::Signal;
use gnss_rcv::receiver::{Receiver, ReceiverConfig};
use gnss_rcv::recording::IQFileType;
use gnss_rcv::state::GnssState;
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

fn main() {
    let _profiler = dhat::Profiler::new_heap();
    let msec: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);

    let cfg = ReceiverConfig {
        file: PathBuf::from("resources/SJTU_Bands-L1E1.dat"),
        iq_file_type: IQFileType::TypeOne4Bit,
        fs: 25_000_000.0,
        fi: 6_250_000.0,
        sig: Signal::GalileoE1b,
        families: vec![Signal::GalileoE1b],
        sbas: false,
        ..Default::default()
    };
    let exit = Arc::new(AtomicBool::new(false));
    let state = Arc::new(Mutex::new(GnssState::new()));
    let mut rx = Receiver::new(&cfg, exit, state).expect("receiver init");
    rx.run_loop(msec);
    // _profiler drops here: writes dhat-heap.json and prints the peak summary.
}
