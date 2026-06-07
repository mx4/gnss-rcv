# gnss-rcv — notes for agents

A GPS L1 C/A software receiver in Rust: reads an SDR IQ recording (or rtl-sdr
device), then does acquisition → tracking → ephemeris decode → position fix.

## Testing

- **`cargo test --release`** — unit tests + the fast integration test
  `acquires_and_tracks_gpssim` (~0.6 s). Run this after any change.
- **`cargo test --release -- --ignored`** — also runs the heavy
  `computes_position_fix_gpssim` test (~3 s; runs the pipeline until the first
  position fix, then stops via `-x`/`--exit-on-fix`).
- **`cargo clippy --release --tests`** — must be clean; the project keeps it so.

The integration tests live in [tests/gpssim.rs](tests/gpssim.rs) and drive the
**full pipeline** against `resources/gpssim_2xi16` (a gps-sdr-sim recording at a
known location). The fast test asserts ≥4 SVs reach tracking; the ignored test
asserts the computed fix is within 0.02° of the simulated location.

> The test recording is large, gitignored, and not in CI. The tests **skip
> cleanly (print `skipping…` and pass) when `resources/gpssim_2xi16` is absent**,
> so `cargo test` is always safe to run. To actually exercise them the file must
> be present — it is generated with gps-sdr-sim (see
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

**High value, low risk**
- **Mock `IQReader`**: the `IQReader` trait (`receiver.rs`) has no test impl, so
  `Channel`/`Receiver` logic can't be unit-tested without a real recording. A
  `MockIQReader { samples: Vec<Complex64> }` unblocks everything below.
- **Unit tests for the state machine and nav decode**: with a mock reader, cover
  Acquisition→Tracking→Idle transitions and LNAV subframe decoding (use
  known-good subframes; no IQ needed). Today only cross-correlation rejection is
  unit-tested.
- **`ReceiverConfig` struct**: `Receiver::new` takes 13 positional args; callers
  pass runs of bare `false, false` ([app.rs](src/app.rs)). A config struct makes
  callsites self-documenting and stops breaking them on every new option.
- **`is_ephemeris_complete` is under-specified** ([channel.rs](src/channel.rs)):
  checks 5 fields; missing keplerian/clock terms (`ecc`, `f0`, `omg_dot`) can let
  a half-decoded ephemeris reach the solver. Move to `RxEphemeris::is_valid()`
  and table-test it.

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

**Only if multi-constellation (Galileo/GLONASS) becomes a real goal**
These are correct refactors but pure scaffolding until there's a second
constellation — adding them now is speculative generality.
- Replace the string signal id (`"L1CA"`, matched in `code.rs`/`channel.rs`) with
  a `SignalType` enum carrying frequency/code params.
- Extract navigation decoding (~440 lines of `impl Channel` in
  [navigation.rs](src/navigation.rs)) behind a `NavigationDecoder` trait.
- Make `get_sat_list` ([receiver.rs](src/receiver.rs)) constellation-aware
  instead of hardcoding GPS PRN 1–32 (and the dead `use_sbas = false`).
- Trait-based ionosphere model (Klobuchar / NeQuick / none) in
  [solver.rs](src/solver.rs).
