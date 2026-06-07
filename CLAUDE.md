# gnss-rcv — notes for agents

A GPS L1 C/A software receiver in Rust: reads an SDR IQ recording (or rtl-sdr
device), then does acquisition → tracking → ephemeris decode → position fix.

## Testing

- **`cargo test --release`** — unit tests + the fast integration test
  `acquires_and_tracks_gpssim` (~0.6 s). Run this after any change.
- **`cargo test --release -- --ignored`** — also runs the heavy
  `computes_position_fix_gpssim` test (~5 s; processes ~40 s of IQ all the way
  to a position fix).
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
- A position fix needs ~3 subframes decoded (~20–40 s of IQ). There is no
  exit-on-fix flag yet, so bound long files with `--num-msec`.

## Sample data

`./resources/fetch.sh` downloads the downloadable IQ recordings (run it with no
args to list them). `gpssim_2xi16` is *generated* by gps-sdr-sim, not downloaded.
