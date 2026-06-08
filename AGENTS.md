# gnss-rcv — notes for agents

A GPS L1 C/A software receiver in Rust: reads an SDR IQ recording (or rtl-sdr
device), then does acquisition → tracking → ephemeris decode → position fix.

## Testing

CI (Linux + macOS) gates every push/PR on three checks; keep all green locally
before pushing. The full local gate (the three CI checks, plus the heavy
`#[ignore]`'d end-to-end tests) is, in order — stop at the first failure, fix,
re-run from the top:

```sh
cargo fmt --all -- --check
cargo clippy --release --all-targets -- -D warnings
cargo test --release
cargo test --release -- --ignored
```

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

### Validating a positioning change

The `gpssim_2xi16` fixture has known ground truth (Geneva, Jet d'Eau: lat
46.2075, lon 6.1557; antenna ECEF `4396463.3, 474169.7, 4581510.0` — from the
gps-sdr-sim run, see [resources/README.md](resources/README.md)). Use it to
check pseudorange / transmit-time / solver changes:

- Set `GNSS_TRUTH_ECEF="4396463.3,474169.7,4581510.0"` to turn on the per-SV
  `RESID` diagnostic in [solver.rs](src/solver.rs). Each SV's `resid` should be
  ≈ a *common constant* (the receiver clock bias); the spread across SVs is the
  geometry error (sub-km when timing is right, 100s of km when it isn't).
- The end-of-run stats funnel
  (`searched → acquired → tracked → ephemeris → used-in-fix`) shows where SVs
  drop out.

[`scripts/validate_fix.py`](scripts/validate_fix.py) wraps this into one
command: it builds release, runs to the first fix with `GNSS_TRUTH_ECEF` on,
and prints the residual spread + the fix error vs truth. It **skips cleanly**
(exit 0) when the fixture is absent. Read the result as: `resid` per SV ≈ a
common constant (the rx clock bias); the **spread** is the geometry error
(sub-km good, 100s of km means transmit-time/pseudorange is wrong); `fix error
vs truth` should be well under the test's 0.02° (~2 km) gate.

```sh
./scripts/validate_fix.py
# equivalent one-liner:
GNSS_TRUTH_ECEF="4396463.3,474169.7,4581510.0" RUST_LOG=warn \
  cargo run --release -- -f resources/gpssim_2xi16 -t 2xi16 \
  --sats 1,2,3,4,6,9,17,19,28,31 -x 2>&1 | rg "RESID|position fix"
```

## Build & manual runs

- Build: `cargo build --release`
- Run: `RUST_LOG=info cargo run --release -- -f <file> -t <type> [--fs <hz> --fi <hz>]`
- IQ formats (`-t`): `2xf32` (default), `2xi16`, `2xi8`, `i8`, `rtlsdr-file`, `1bit`.
  Sample rate / IF are set with `--fs` / `--fi` (defaults 2.046 MHz / 0 Hz).

### Faster iteration
- `--sats 5,10,12,...` restricts the satellite search (~2× faster; absent PRNs
  otherwise run an FFT search every cycle). The `gpssim_2xi16` fixture's PRNs are
  `1,2,3,4,6,9,17,19,28,31` (truth + PRN list in `resources/gpssim_gen.meta`).
- `--num-msec N` bounds the run; `RUST_LOG=warn` cuts log noise.
- `-p` / `--plots` enables per-SV PNG diagnostics in `plots/` (off by default — skipping saves I/O during headless runs).
- `-x` / `--exit-on-fix` stops the run as soon as the first fix is computed (useful with long files; the fix test uses this).
- A position fix needs ~3 subframes decoded (~20–40 s of IQ). Use `-x` to stop
  at the first fix, or `--num-msec` to bound a run that may never get one.

## Sample data

`./resources/fetch.sh` is the IQ-recording provisioner: no args lists them;
`fetch.sh <name>` downloads one (resuming/skipping if present) **and prints the
exact command to run it**. `gpssim_2xi16` is *generated* by gps-sdr-sim
(`./resources/gen_gpssim.sh`, needs gps-sdr-sim + network), not downloaded.

**Master validation list** — run these to check receiver stability across rates,
formats and IFs. Each needs the right `-t` (and sometimes `--fs`/`--fi`); append
`-x` to stop at the first fix. Per-recording truth/notes in
[resources/README.md](resources/README.md):

| `fetch.sh` name | flags | expected |
|---|---|---|
| `nov3` | `-t 2xf32` | ✅ fix 52.334, −0.081 (St Ives, Cambs) |
| `cttc` | `-t 2xi16 --fs 4000000` | ✅ fix 41.274, 1.986 (Castelldefels) |
| `ion-rtlsdr` | `-t rtlsdr-file --fs 2048000` | ✅ fix 52.177, 4.489 (Netherlands) |
| `ion-bladerf` | `-t 2xi16 --fs 10000000` | tracks 13 SVs; ~13 s — too short for a fix |
| `ion-hackrf` | `-t 2xi8 --fs 10000000 --fi 420000` | tracks; partial nav decode, no full fix (open) |
| `zenodo-sigmf` | `-t 2xi16 --fs 4000000` | tracks; ~15 s — too short for a fix |
| `jks-1bit` | `-t 1bit --fs 5456000 --fi 1364000` | tracks marginal ~30 dB-Hz; no fix |
| (generated) `gpssim_2xi16` | `-t 2xi16` | ✅ fix 46.207, 6.156 (Geneva); the integration test |
| TEXBAT `cleanStatic` (manual; 44 GB, ~70 s prefix is enough) | `-t 2xi16 --fs 25000000` | ✅ fix 30.287, −97.736 (Austin TX) |

For the generated `gpssim_2xi16` positioning regression see
[`scripts/validate_fix.py`](scripts/validate_fix.py) / "Validating a positioning change".

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
- ~~**Nav-decode unit tests**~~: `decodes_real_lnav_subframes_to_a_valid_ephemeris`
  ([ephemeris.rs](src/ephemeris.rs)) feeds real captured LNAV subframes 1-3
  through the decoders and range-checks the result (a≈26 560 km, ecc<0.03,
  i0≈0.96 rad…), locking the bit-field offsets.
- ~~**`is_ephemeris_complete` → `Ephemeris::is_valid()`**~~
  ([ephemeris.rs](src/ephemeris.rs)): moved + broadened (now also checks `toc`,
  an eccentricity sanity bound, and `omg_dot`); constellation-agnostic on orbit
  size/inclination (QZSS-safe). Table-tested.
- ~~**`get_code_and_carrier_phase` empty-`corr_p` panic**~~
  ([channel.rs](src/channel.rs)): `num_trk_samples` now moves with the buffer
  pop/push (it tracks buffer alignment) while `num_tx_codes`/`code_off_sec`
  always wrap, so a first tracking step at code phase ~0 no longer panics.
  Regression: `tracks_at_code_phase_zero_without_panicking`.
- ~~**`channel.rs` low-risk extractions**~~: the four `update_state_*` methods
  collapsed into one `publish()` helper; the five `plot_*` methods moved to
  `plots::plot_channel(sv, &History)`. `channel.rs` 835 → 743 lines, no longer
  depends on `plotters`. (The deeper acquisition/tracking split is deferred — see
  Architecture.)

**Architecture**
- **Full `channel.rs` acquisition/tracking split — deferred on purpose.** The
  cheap, orthogonal extractions above are done. A true split into independent
  `Acquisition`/`Tracking` types is harder than it looks: the tracking methods
  read/write most of `Channel`'s fields (`num_trk_samples`, `num_tx_codes`,
  `hist`, `trk`, `fc/fi/fs`, `code_sp`, `pub_state`), so it needs a real
  interface, not a mechanical move — and it's pure cleanup with no functional
  payoff. Do it when QZSS / the `SignalType` refactor (Multi-constellation,
  below) forces the seams, not speculatively. The synthetic-signal +
  position-fix tests make it safe to attempt whenever it happens.

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
