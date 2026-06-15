# Testing and validation

How this receiver is tested, what each tier proves, and how to validate a
positioning change against ground truth. The operational gates and the pinned
baselines live in [AGENTS.md](../AGENTS.md); this is the architecture and the
methodology.

## The tiers

```sh
cargo fmt --all -- --check
cargo clippy --release --all-targets -- -D warnings
cargo test --release                  # unit + fast integration
cargo test --release -- --ignored     # heavy/hermetic end-to-end tier
```

(`just check` wraps the first three; `just test-all` adds the ignored tier.)

- **`cargo test --release`** — unit tests, including the hermetic
  synthetic-signal acquire/track tests
  (`synthetic_signal_acquires_and_tracks`,
  `synthetic_noisy_multi_sv_acquires_and_tracks`), which synthesize their own
  IQ via [`synth.rs`](../src/synth.rs) and need no recording, plus the fast
  integration test `acquires_and_tracks_gpssim` (~0.6 s).
- **`cargo test --release -- --ignored`** — the heavy tier: the hermetic
  positioning regressions (below), `computes_position_fix_gpssim` (pipeline
  until the first fix, via `-x`), and `generates_and_solves_gpssim`. CI runs
  this tier too: recording-based tests skip there, the hermetic ones run.
- Unit-testing the receiver without a recording: `MockIQReader`
  ([receiver.rs](../src/receiver.rs)) feeds a `Vec<Complex64>`; build a
  `Receiver` with `Receiver::with_feed(...)`.

Recordings are large, gitignored, and not in CI. Every test **skips cleanly**
(prints `skipping…` and passes) when its input recording — or, for the
generated test, gps-sdr-sim/network — is absent, so `cargo test` is always
safe to run anywhere.

## The hermetic positioning regressions

The backbone is `synthetic_geometry_solves_to_truth`
([receiver.rs](../src/receiver.rs)): `synth::GeoFeed` renders a multi-SV L1CA
scene whose per-SV code phase, code rate, Doppler and LNAV bit timing all
derive from the true ranges between a truth position and orbits the scene
also *broadcasts* (full LNAV streams via
`gps_lnav::encode_lnav_subframe_source`; the generator flies the
LSB-quantized ephemeris the receiver will decode — `quantize_via_lnav`). The
full real pipeline — acquisition → tracking → bit/frame sync → ephemeris
decode → tx anchor → solve — must produce a fix within 15 m of the truth
(measured ~2.5 m). No recording, no gps-sdr-sim, no network; **this is the
positioning regression CI runs on every push.**

Around it:

- **Noisy twin** `synthetic_geometry_solves_to_truth_in_noise`: the same
  scene in seeded AWGN at a realistic 44 dB-Hz (gate 30 m, measured ~10 m).
  The clean test pins the systematic error; the noisy one locks noise
  robustness — a tracking-loop regression that only hurts in noise shows up
  there.
- **Galileo twin** `synthetic_e1_geometry_solves_to_truth`
  (`GeoFeed::new_e1`): BOC(1,1) codes + full I/NAV pages with ICD-convention
  word-5 TOWs, GST-built epochs; measured ~4 m.
- **Anchor instrument** `tx_anchor_latency_measured_against_synthetic_truth`:
  measures both decoders' TOW→phase anchor conventions *directly* against
  generator truth, per SV, solver-free — locking the cross-constellation
  anchor alignment (LNAV = I/NAV convention, difference < 5 ms) that mixed
  GPS+Galileo solves require (see
  [gps-galileo-timing.md](gps-galileo-timing.md)).
- **SBAS corrections bench**
  `sbas_fast_corrections_recover_broadcast_clock_errors`
  (`GeoFeed::new_diverged`): broadcasts ±10 m per-SV clock errors the signal
  doesn't have; synthetic MT1+MT2 through the production path must recover
  < 3.5 m against exact truth (see [sbas.md](sbas.md)). This bench also
  prices tracking-loop noise — it sized the DLL integrator gain
  ([dll-pi-loop.md](dll-pi-loop.md)).
- Constellation geometry comes from `synth::pick_geo_constellation` (one
  near-zenith SV + a ~30° ring; all-ring geometry was measured at 28 m error,
  27.9 m of it vertical).

## The gps-sdr-sim integration tests

[tests/gpssim.rs](../tests/gpssim.rs) drives the full pipeline against a
gps-sdr-sim recording at a known location:

- `acquires_and_tracks_gpssim` (fast) — ≥4 SVs reach tracking.
- `computes_position_fix_gpssim` (ignored) — fix within 0.02° of the
  simulated location, on the pre-existing `resources/gpssim_2xi16`.
- `generates_and_solves_gpssim` (ignored) — **end-to-end**: runs
  [resources/gen_gpssim.py](../resources/gen_gpssim.py) to pick a
  date+location, download the matching broadcast ephemeris (ESA GSSC,
  auth-free FTP), run gps-sdr-sim, and verify the receiver recovers that
  location. Needs `gps-sdr-sim` (`$GPS_SDR_SIM`, or
  `~/git/gps-sdr-sim/gps-sdr-sim`, or PATH) plus network; skips cleanly when
  missing. The script caches by scenario, so reruns skip regeneration.

**When you change anything in the DSP/receiver path** (`channel.rs`,
`navigation.rs`/`gps_lnav.rs`/`galileo_inav.rs`, `ephemeris.rs`, `solver.rs`,
`receiver.rs`, `recording.rs`), run the gpssim integration tests — they are
the regression signal that the end-to-end receiver still acquires, tracks,
decodes, and solves.

## Validating a positioning change: the truth residual

The `gpssim_2xi16` fixture has known ground truth (Geneva, Jet d'Eau: lat
46.2075, lon 6.1557; antenna ECEF `4396463.3, 474169.7, 4581510.0` — from
the gps-sdr-sim run, see [resources/README.md](../resources/README.md)).

Set `GNSS_TRUTH_ECEF="<x>,<y>,<z>"` to turn on the per-SV `RESID` diagnostic
in [solver.rs](../src/solver.rs):

```
RESID = pr_m + clk_m − geom
```

where `pr_m` is the pseudorange, `clk_m` the SV clock correction, and `geom`
the true geometric range (SV-at-transmit to the known antenna,
Sagnac-corrected). If the transmit-time / pseudorange model is correct,
**every SV's residual equals the same common-mode value** `c·dT_rx` (the
receiver clock bias). Read it as:

- flat residual across SVs → good; the common value is the clock bias;
- a **spread** across SVs is the part the position solve must absorb as
  geometry error (sub-km when timing is right, 100s of km when it isn't);
- a per-SV deviation that correlates with Doppler, C/N0, or lock age points
  at the corresponding subsystem (this instrument found the Doppler-
  proportional bias of [dll-group-delay.md](dll-group-delay.md) and the
  re-locking-channel biases of [dll-pi-loop.md](dll-pi-loop.md)).

The end-of-run stats funnel
(`searched → acquired → tracked → ephemeris → used-in-fix`) shows where SVs
drop out; `--json` emits the machine-readable twin of the stats block —
prefer asserting on it over grepping logs.

## `validate_fix.py`

[`scripts/validate_fix.py`](../scripts/validate_fix.py) wraps the
recording-based acceptance checks into one command. It builds release and
runs `--json`-driven checks, each skipping cleanly (exit 0) when its
recording is absent, failing (non-zero) only if a *present* recording's
check fails:

- **GPS fix** (gpssim fixture): runs to the first fix with
  `GNSS_TRUTH_ECEF` on, prints the residual spread + the fix error vs truth
  with a PASS/FAIL verdict.
- **Galileo E1-B I/NAV decode + fix** (every present recording): asserts
  Galileo SVs track and decode CRC-valid I/NAV words; on a recording long
  enough to complete ≥4 ephemerides (ION LimeSDR, 60 s) it also asserts a
  real Galileo-only position fix. Short recordings assert only the decode
  chain.
- **SBAS L1 decode** (a capture with an SBAS GEO overhead — CTTC Spain →
  EGNOS): asserts a floor of CRC-valid SBAS messages from ≥1 GEO.
- **Mixed GPS+Galileo fix** (tuni2025, flag-less): ≥12 SVs, ≥3 Galileo used,
  within 0.01° — the live multi-signal-stepping gate
  ([multi-signal-stepping.md](multi-signal-stepping.md)).

Current pinned numbers (fix coordinates, σ, perf baselines) live in
[AGENTS.md](../AGENTS.md) — they move with the code, and re-pins are
documented there with their cause.
