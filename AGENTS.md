# gnss-rcv — notes for agents

A GPS L1 C/A software receiver in Rust: reads an SDR IQ recording (or rtl-sdr
device), then does acquisition → tracking → ephemeris decode → position fix.

## Testing

CI (Linux + macOS) gates every push/PR on three checks; keep all green locally
before pushing:
- **`cargo fmt --all -- --check`**
- **`cargo clippy --release --all-targets -- -D warnings`**
- **`cargo test --release`**

- **`cargo test --release`** — unit tests (incl. `synthetic_signal_acquires_and_tracks`,
  a hermetic acquisition→tracking test that synthesizes its own signal and needs
  no recording) + the fast integration test `acquires_and_tracks_gpssim` (~0.6 s).
  Run this after any change.
- **`cargo test --release -- --ignored`** — also runs the heavy
  `computes_position_fix_gpssim` (~3 s; pipeline until the first fix, via
  `-x`/`--exit-on-fix`) and `generates_and_solves_gpssim` (see below).
- Unit-testing the receiver without a recording: `MockIQReader`
  ([receiver.rs](src/receiver.rs)) feeds a `Vec<Complex64>`; build a `Receiver`
  with `Receiver::with_feed(...)`.

The integration tests live in [tests/gpssim.rs](tests/gpssim.rs) and drive the
**full pipeline** against a gps-sdr-sim recording at a known location:
- `acquires_and_tracks_gpssim` (fast) — ≥4 SVs reach tracking.
- `computes_position_fix_gpssim` (ignored) — fix within 0.02° of the simulated
  location, on the pre-existing `resources/gpssim_2xi16`.
- `generates_and_solves_gpssim` (ignored) — **end-to-end**: runs
  [resources/gen_gpssim.sh](resources/gen_gpssim.sh) to pick a date+location,
  download the matching broadcast ephemeris (ESA GSSC, auth-free FTP), and run
  gps-sdr-sim, then verifies the receiver recovers that location. Needs
  `gps-sdr-sim` (`$GPS_SDR_SIM`, or `~/git/gps-sdr-sim/gps-sdr-sim`, or PATH)
  plus network; **skips cleanly when those are missing**. The script caches by
  scenario, so reruns skip regeneration.

> The recordings are large, gitignored, and not in CI. Every test **skips
> cleanly (prints `skipping…` and passes)** when its input recording (or, for
> the generated test, gps-sdr-sim/network) is absent, so `cargo test` is always
> safe to run. `gpssim_2xi16` is generated with gps-sdr-sim (see
> [resources/README.md](resources/README.md)), not downloaded.

**When you change anything in the DSP/receiver path** (`channel.rs`,
`navigation.rs`, `ephemeris.rs`, `solver.rs`, `receiver.rs`, `recording.rs`),
run the gpssim integration tests — they are the regression signal that the end-
to-end receiver still acquires, tracks, decodes, and solves.

## Build & manual runs

- Build: `cargo build --release`
- Run: `RUST_LOG=info cargo run --release -- -f <file> -t <type> [--fs <hz> --fi <hz>]`
- IQ formats (`-t`): `2xf32` (default), `2xi16`, `i8`, `rtlsdr-file`, `1bit`.
  Sample rate / IF are set with `--fs` / `--fi` (defaults 2.046 MHz / 0 Hz).

### Faster iteration
- `--sats 5,10,12,...` restricts the satellite search (~2× faster; absent PRNs
  otherwise run an FFT search every cycle). gpssim's PRNs are in
  `resources/gpssim.txt`.
- `--num-msec N` bounds the run; `RUST_LOG=warn` cuts log noise.
- `-p` / `--plots` enables per-SV PNG diagnostics in `plots/` (off by default — skipping saves I/O during headless runs).
- `-x` / `--exit-on-fix` stops the run as soon as the first fix is computed (useful with long files; the fix test uses this).
- A position fix needs ~3 subframes decoded (~20–40 s of IQ). Use `-x` to stop
  at the first fix, or `--num-msec` to bound a run that may never get one.

## Sample data

`./resources/fetch.sh` downloads the downloadable IQ recordings (run it with no
args to list them). `gpssim_2xi16` is *generated* by gps-sdr-sim, not downloaded.

## Improvement backlog

Evidence-ranked candidates for making the code more maintainable and easier to
evolve. Verified against the source; framing reflects actual measured cost, not
guesses. Tackle roughly top-to-bottom.

**Done**
- ~~`History` ring buffers → `VecDeque`~~: the four diagnostics buffers
  (`HISTORY_NUM = 20000`) were `Vec`s trimmed with `rotate_left(1)` + `pop()` on
  every code period (~1 kHz/SV) — a full-buffer memmove per tick (`corr_p` is
  `Complex64`, ~320 KB). Now O(1) `pop_front`.
- ~~`-x`/`--exit-on-fix`~~, ~~`-p`/`--plots` (opt-in plotting)~~.
- ~~**CI gate**~~: Linux/macOS CI now runs fmt + clippy (`-D warnings`) + test,
  not just `cargo build`.
- ~~**Mock `IQReader`** + hermetic DSP test~~: `MockIQReader` + a synthetic-signal
  helper drive a channel acquisition→tracking with no recording
  (`synthetic_signal_acquires_and_tracks`).
- ~~**`ReceiverConfig` struct**~~: `Receiver::new(&ReceiverConfig, ...)` (+
  `with_feed` for injected sources); callers build a struct with
  `..Default::default()` instead of a dozen positional args.
- ~~SBAS PRNs in `get_sat_list`~~: constellation-aware tagging (PRN ≥ 120 → SBAS)
  + a `--sbas` sweep, replacing the dead `use_sbas` flag. (Detection only — no
  SBAS *decode* or ranging; not seen above the noise floor in any recording.)

**High value, low risk**
- **Nav-decode unit tests**: the mock reader + synthetic signal now cover
  Acquisition→Tracking; still untested is LNAV subframe decoding — feed known-good
  subframes through [navigation.rs](src/navigation.rs) (no IQ needed) and assert
  the parsed ephemeris fields.
- **`is_ephemeris_complete` is under-specified** ([channel.rs](src/channel.rs)):
  checks 5 fields; missing keplerian/clock terms (`ecc`, `f0`, `omg_dot`) can let
  a half-decoded ephemeris reach the solver. Move to `RxEphemeris::is_valid()`
  and table-test it.
- **`get_code_and_carrier_phase` unwraps an empty `corr_p`**
  ([channel.rs](src/channel.rs)): it runs at the *start* of `tracking_process`,
  before the first `push_back`, and calls `corr_p.back().unwrap()` / `pop_back()`
  when `code_off_sec` crosses a period boundary. A code phase landing within one
  carrier-aiding step of exactly 0 panics on the first tracking step (found via
  the synthetic test, which sidesteps it with a non-zero code phase). Guard it.

**Real but lower priority**
- **Per-correlation allocations in `calc_correlation`** ([util.rs](src/util.rs)):
  `iq_vec.to_owned()` + a `.map().collect()` allocate two `Vec`s per acquisition
  correlation. The dominant CPU cost is the FFTs themselves, not malloc, so this
  is churn-reduction, not a bottleneck. (FFT *plans* are already cached by
  `rustfft` — not a concern.)
- **`GnssState` lock contention**: channels run under `par_iter_mut`
  ([receiver.rs](src/receiver.rs)), so the shared `Mutex<GnssState>` is
  cross-thread. Most locks fire on state *transitions* (not per tick), but
  batching per-channel updates into one per-ms apply would remove the contention
  and the double-lock at the cn0 update.
- **`nav.bits` uses `rotate_left(1)`** on an 18 KB buffer per nav symbol
  ([navigation.rs](src/navigation.rs)): same anti-pattern as the old `History`,
  but at ~50 Hz it's negligible. Fix for consistency, not speed.

**Multi-constellation**
- **QZSS L1 C/A — cheap first step**: same 1023 Gold codes (PRN 193-202 live in
  the same extended table SBAS uses), same 50 bps LNAV and ephemeris format, and
  `gnss-rtk` supports QZSS — so it decodes *and solves* through the existing GPS
  path. Add the PRN range to `get_sat_list` (tag `Constellation::QZSS`) and
  confirm the codes with the Gold-code test. This validates the multi-
  constellation seams cheaply before the hard signals.

The rest are correct refactors but mostly scaffolding until there's a second
*solving* constellation; adding them earlier is speculative generality:
- Replace the string signal id (`"L1CA"`, matched in `code.rs`/`channel.rs`) with
  a `SignalType` enum carrying frequency/code params.
- Extract navigation decoding (~440 lines of `impl Channel` in
  [navigation.rs](src/navigation.rs)) behind a `NavigationDecoder` trait.
- Galileo E1 needs a BOC(1,1)-aware correlator, embedded 4092-chip memory codes,
  and an I/NAV decoder (FEC + interleaving); GLONASS L1 is FDMA (per-SV carrier),
  which breaks the single-IF assumption end-to-end. Both rank well below QZSS.
- Trait-based ionosphere model (Klobuchar / NeQuick / none) in
  [solver.rs](src/solver.rs).
