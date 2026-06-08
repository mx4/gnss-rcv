//! Bit-sync bench: drive the receiver with a *synthetic* SV (no recording) at a
//! chosen C/N0 and a toggling 50 bps nav pattern (a bit edge every 20 ms — the
//! easiest case for bit sync), then watch how fast and how reliably nav bit-sync
//! is achieved and *held*. Isolates the bit-sync logic from any one recording's
//! carrier quality.
//!
//!   RUST_LOG=info cargo run --release --example bitsync_bench -- 45
//!
//! Look for `SYNC:` (achieved) vs `SYNC LOST` lines: on a clean strong signal it
//! should sync once and hold.

use gnss_rcv::receiver::{MockIQReader, Receiver, ReceiverConfig};
use gnss_rcv::state::GnssState;
use gnss_rcv::synth::{SynthSv, synth_l1ca};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .format_target(false)
        .init();

    let cn0: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(45.0);
    let secs: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(6);
    // optional fs/fi (Hz) to probe a specific regime, e.g. SJTU's 25e6 6.25e6.
    let fs: f64 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(2_046_000.0);
    let fi: f64 = std::env::args()
        .nth(4)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let prn = 5u8;

    // Toggling 50 bps data: a clean bit edge every 20 ms (max transition density).
    let bits: Vec<i8> = (0..50).map(|i| if i % 2 == 0 { 1 } else { -1 }).collect();
    let sv = SynthSv::new(prn, 600.0, 137.0, cn0).with_nav_bits(bits);
    let sig = synth_l1ca(&[sv], fs, fi, secs * 1000, Some(1));

    eprintln!("bitsync_bench: PRN {prn}, C/N0 {cn0} dB-Hz, {secs}s, toggling 50bps");
    let cfg = ReceiverConfig {
        sats: prn.to_string(),
        fs,
        fi,
        ..Default::default()
    };
    let mut rx = Receiver::with_feed(
        Box::new(MockIQReader::new(sig)),
        &cfg,
        Arc::new(AtomicBool::new(false)),
        Arc::new(Mutex::new(GnssState::new())),
    );
    rx.run_loop(0);
}
