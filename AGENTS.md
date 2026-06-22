# gnss-rcv — notes for agents

A GPS L1 C/A software receiver in Rust: reads an SDR IQ recording (or rtl-sdr
device), then does acquisition → tracking → ephemeris decode → position fix.

## Testing

CI (Linux + macOS) gates every push/PR on three checks; keep all green locally
before pushing. The whole workflow is wrapped in a [`justfile`](justfile) — run
`just` to list recipes. The full local gate is **`just check`** (fmt-check +
clippy `-D warnings` + test); add the heavy `#[ignore]`'d end-to-end tests with
`just test-all`. Other recipes: `just validate` (the GPS-fix + Galileo + SBAS
checks), `just run <file> "<flags>"`, `just galileo`, `just fetch`, `just bench`.

The raw commands behind `just check` (stop at the first failure, fix, re-run):

```sh
cargo fmt --all -- --check
cargo clippy --release --all-targets -- -D warnings
cargo test --release
cargo test --release -- --ignored
```

- **`cargo test --release`** — unit tests (incl. the hermetic synthetic-signal
  acquire/track tests, which synthesize their own IQ via [`synth.rs`](src/synth.rs)
  and need no recording) + the fast integration test `acquires_and_tracks_gpssim`
  (~0.6 s). Run this after any change.
- **`cargo test --release -- --ignored`** — adds the heavy tier: the **hermetic
  exact-truth positioning regressions** (synthetic GeoFeed scenes — clean, noisy,
  Galileo E1, anchor-latency, SBAS-corrections; the positioning gate CI runs on
  every push), `computes_position_fix_gpssim`, and `generates_and_solves_gpssim`
  (end-to-end via gps-sdr-sim + network; skips cleanly when those are missing).
- Every test **skips cleanly** (prints `skipping…` and passes) when its input
  recording is absent, so `cargo test` is always safe to run. Recordings are
  large, gitignored, and not in CI — the hermetic tests are what CI runs.
- Unit-testing the receiver without a recording: `MockIQReader`
  ([receiver.rs](src/receiver.rs)) + `Receiver::with_feed(...)`.

What each tier proves — the GeoFeed exact-truth architecture, the gpssim
integration tests, the generator — is documented in
[docs/testing.md](docs/testing.md).

**When you change anything in the DSP/receiver path** (`channel.rs`,
`navigation.rs`/`gps_lnav.rs`/`galileo_inav.rs`, `ephemeris.rs`, `solver.rs`,
`receiver.rs`, `recording.rs`),
run the gpssim integration tests — they are the regression signal that the end-
to-end receiver still acquires, tracks, decodes, and solves.

### Validating a positioning change

The `gpssim_2xi16` fixture has known ground truth (Geneva, Jet d'Eau: lat
46.2075, lon 6.1557; antenna ECEF `4396463.3, 474169.7, 4581510.0`). Set
`GNSS_TRUTH_ECEF="4396463.3,474169.7,4581510.0"` to turn on the per-SV
`RESID` diagnostic in [solver.rs](src/solver.rs): each SV's `resid` should be
≈ a *common constant* (the receiver clock bias); the **spread** across SVs is
the geometry error (sub-km when timing is right, 100s of km when it isn't).
The end-of-run stats funnel
(`searched → acquired → tracked → ephemeris → used-in-fix`) shows where SVs
drop out. Methodology and how to read the residuals:
[docs/testing.md](docs/testing.md).

[`scripts/validate_fix.py`](scripts/validate_fix.py) wraps the GPS / Galileo
/ SBAS / Mixed-fix acceptance checks into one command, each **skipping
cleanly** (exit 0) when its recording is absent, failing (non-zero) only if
a *present* recording's check fails.

Baseline (2026-06-12, pre multi-signal-stepping; after the WLS+ISB live
solver, SBAS corrections/weights, retroactive anchor, f32 DSP and lazy
acquisition grids): gpssim fix error ~0 km (5 SVs); ION LimeSDR Galileo-only
~0.1 km (5 SVs); CTTC SBAS 74 msgs/40 s (S120+S126); tuni2025 15 s bench
~1.5 CPU-s per data-second / 646 MB peak RSS mixed (post ingest decimation:
50 -> 6.25 Msps auto, GNSS_DECIM=off to disable; was 17.2 CPU-s & 3.59 GB);
C/A-only 15 s slice 24.1 s wall / 130.5 s CPU (post shared-FFT M5; CTTC
first fix re-pinned 41.274836, 1.987583 σ 1.0 m — moved ~2 m when the t_tx
anchor-latency fix corrected every SV's orbit epoch by 0.16 s, landing
slightly closer to the documented antenna; the prior pin 41.274820,
1.987562 dated from the DLL PI-loop change. SJTU σgps 37 m → 1–3 m on the
same fix. Exact-truth suite unchanged (gpssim fix ~2 m, σ 2.4–2.9 m). All
gates PASS.

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
- For iteration you can **drop `--release`**: the dev profile optimizes deps
  (`opt-level=3`) and our crate (`opt-level=1`), so a debug build compiles in
  ~1.5 s (vs ~7.4 s release) and still runs the DSP fast enough (it solves the
  gpssim fixture in ~3.6 s, same fix). Use `--release` for the final check.
- IQ formats (`-t`): `2xf32` (default), `2xi16`, `2xi16-be`, `2xi8`, `i8`, `rtlsdr-file`, `1bit`, `4bit`.
  Sample rate / IF are set with `--fs` / `--fi` (defaults 2.046 MHz / 0 Hz).

### Faster iteration
- `--sats 5,10,12,...` restricts the satellite search (~2× faster; absent PRNs
  otherwise run an FFT search every cycle). The `gpssim_2xi16` fixture's PRNs are
  `1,2,3,4,6,9,17,19,28,31` (truth + PRN list in `resources/gpssim_gen.meta`).
- `--num-msec N` bounds the run; `RUST_LOG=warn` cuts log noise.
- `-p` / `--plots` enables per-SV PNG diagnostics in `plots/` (off by default — skipping saves I/O during headless runs).
- `-x` / `--exit-on-fix` stops the run as soon as the first fix is computed (useful with long files; the fix test uses this).
- `--json <path>` writes a machine-readable end-of-run summary (`fix`, `funnel`,
  `stats`, per-SV `sats`) — the serialized twin of the `===== run stats =====`
  block, built from the same data. `-` = stdout (pipe to `jq`; the human stats
  and the `file: …` banner go to stderr in that mode), a file path keeps the
  human stats on stdout. Prefer asserting on this over grepping logs.
- A position fix needs ~3 subframes decoded (~20–40 s of IQ). Use `-x` to stop
  at the first fix, or `--num-msec` to bound a run that may never get one.
- `examples/bitsync_bench.rs` drives the receiver with a *synthetic* SV (chosen
  C/N0, toggling 50 bps, optional fs/fi) and no recording, to study nav bit-sync
  in isolation: `RUST_LOG=info cargo run --release --example bitsync_bench -- 45`.

## Sample data

`./resources/fetch.py` is the IQ-recording provisioner (Python 3, stdlib only;
shells out to curl/unzip/tar): no args lists them with constellation tags;
`fetch.py <name>` downloads one (resuming/skipping if present) **and prints the
exact command to run it**. A positional arg can be a recording name, a tag
(`gps`/`qzss`/`galileo`), or `all`; `--tag <t>` filters the listing.
`gpssim_2xi16` is *generated* by gps-sdr-sim (`./resources/gen_gpssim.py`, needs
gps-sdr-sim + network), not downloaded.

**Master validation list** — run these to check receiver stability across rates,
formats and IFs. Each needs the right `-t` (and sometimes `--fs`/`--fi`); append
`-x` to stop at the first fix. Per-recording truth/notes in
[resources/README.md](resources/README.md):

| `fetch.py` name | flags | expected |
|---|---|---|
| `nov3` | `-t 2xf32` | ✅ fix 52.334, −0.081 (St Ives, Cambs) |
| `cttc` | `-t 2xi16 --fs 4000000` | ✅ fix 41.274, 1.986 (Castelldefels) |
| `ion-rtlsdr` | `-t rtlsdr-file --fs 2048000` | ✅ fix 52.177, 4.489 (Netherlands) |
| `ion-bladerf` | `-t 2xi16 --fs 10000000` | tracks 13 SVs; ~13 s — too short for a fix |
| `ion-lime` | `-t 2xi16 --fs 10000000 --fi 420000` | ✅ fix 52.177, 4.488 (NL); non-zero-IF check |
| `ion-hackrf` | `-t 2xi8 --fs 10000000 --fi 420000` | tracks; partial nav decode, no full fix (open) |
| `zenodo-sigmf` | `-t 2xi16 --fs 4000000` | tracks; ~15 s — too short for a fix |
| `jks-1bit` | `-t 1bit --fs 5456000 --fi 1364000` | tracks marginal ~30 dB-Hz; no fix |
| `sjtu` | `-t 4bit --fs 25000000 --fi 6250000` | ✅ fix 31.0251, 121.4394 (Shanghai, SJTU Minhang), **σgps 1–3 m**; 6 ephemerides, 60 s unbroken locks since the DLL rate-trim integrator (the front end's ~850 ns/s clock skew used to walk every MEO off the peak in ~23 s). σ was tens of m until the t_tx anchor-latency fix removed the orbit-epoch bias (see the per-SV-bias row) |
| `pocketsdr` | `-t i8 --fs 12000000 --fi 3000000 --qzss` | Tokyo; tracks 26 GPS + QZSS J194/195/199, decodes SF2–5; ~30 s, ~6 s short of an ephemeris |
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
- ~~**Mock `IQReader`** + synthetic-signal generator~~: [`synth.rs`](src/synth.rs)
  renders multi-SV L1CA IQ (Doppler, fractional code phase, C/N0-scaled AWGN,
  optional 50 bps nav bits) into `MockIQReader`, driving the real acquisition→
  tracking path with no recording. Covered by `synthetic_signal_acquires_and_tracks`
  (clean) and `synthetic_noisy_multi_sv_acquires_and_tracks` (3 SVs in one AWGN
  realization); also a controlled bench for the slow-bit-sync issue.
- ~~**`ReceiverConfig` struct**~~: `Receiver::new(&ReceiverConfig, ...)` (+
  `with_feed` for injected sources); callers build a struct with
  `..Default::default()` instead of a dozen positional args.
- ~~SBAS PRNs in `get_sat_list`~~: constellation-aware tagging (PRN ≥ 120 → SBAS)
  + a `--sbas` sweep, replacing the dead `use_sbas` flag.
- ~~**SBAS L1 message decode**~~, ~~**iono grid (MT18/26)**~~ and
  ~~**fast + long-term corrections (MT1-5/24/25)**~~: decoded
  ([`sbas_l1.rs`](src/sbas_l1.rs)) and applied live in the solver
  ([`sbas_corr.rs`](src/sbas_corr.rs), [`sbas_iono.rs`](src/sbas_iono.rs));
  certified against exact truth by the hermetic
  `sbas_fast_corrections_recover_broadcast_clock_errors` bench (bit layouts
  cross-checked against RTKLIB sbas.c during bring-up). The wrap bookkeeping,
  EGNOS test-mode conventions, buffering and the per-capture yield numbers:
  [docs/sbas.md](docs/sbas.md).
- **CTTC height drift (open observation)**: every CTTC run slides h ~45 → 17 m
  and ~10 m east over 95 s, SBAS on or off. Truth-residual analysis
  (GNSS_TRUTH_ECEF + RESID trends): the common-mode residual ramps 3.3 m/s
  (≈11 ppb front-end TCXO, absorbed by the clock state, harmless); the slide
  comes from slowly evolving *differential* residuals (−4..+1 m per SV over
  the run, G32/G23/G20 dominating) on top of standing per-SV biases up to
  ±12 m (G17 +12 m, G23 +10 m — rooftop multipath suspects; 2013 solar-max
  morning iono ramp is the other candidate). VDOP ~4 amplifies the trend
  into the height. Not attributable to receiver code so far; the per-SV
  code-carrier divergence instrument is the right next probe.
- ~~**Nav-decode unit tests**~~: `decodes_real_lnav_subframes_to_a_valid_ephemeris`
  ([gps_lnav.rs](src/gps_lnav.rs)) feeds real captured LNAV subframes 1-3 through
  the `decode_lnav_subframe{1,2,3}` parsers and range-checks the result (a≈26 560
  km, ecc<0.03, i0≈0.96 rad…), locking the bit-field offsets. (`Ephemeris` itself
  is now a constellation-agnostic data struct; the LNAV parsing lives in gps_lnav.)
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
- ~~`LnavState.bits` 18 KB `rotate_left(1)` per nav bit~~: the buffer is now
  `NAV_BIT_WINDOW` (308 bits — one subframe + the next preamble, all the decoder
  ever reads), so the per-bit rotate moves 308 bytes at 50 Hz — noise. The old
  18000-entry sizing was inherited, 58× larger than ever read.
- ~~**Geometry-consistent synth→fix test**~~: `synth::GeoFeed` +
  `gps_lnav::encode_lnav_subframe_source`/`quantize_via_lnav` +
  `synth::pick_geo_constellation` give a hermetic IQ→position regression
  (`synthetic_geometry_solves_to_truth`, ~6 s, fix ~2.5 m from truth) that CI
  runs on every push — the first end-to-end positioning check that needs no
  recording. Also the seed of a pure-Rust gps-sdr-sim replacement.

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

**Multi-constellation**
- **QZSS L1 C/A — cheap first step**: same 1023 Gold codes (PRN 193-202 live in
  the same extended table SBAS uses), same 50 bps LNAV and ephemeris format, and
  `gnss-rtk` supports QZSS — so it decodes *and solves* through the existing GPS
  path. Add the PRN range to `get_sat_list` (tag `Constellation::QZSS`) and
  confirm the codes with the Gold-code test. This validates the multi-
  constellation seams cheaply before the hard signals.

The rest are correct refactors but mostly scaffolding until there's a second
*solving* constellation; adding them earlier is speculative generality:
- ~~Replace the string signal id with a `SignalType` enum~~ — **done**:
  `code::Signal` (in [code.rs](src/code.rs)) carries the carrier/code params and
  the spreading code, threaded through channel/device/network/receiver/main. It
  is the seam the Galileo E1 work builds on.
- ~~Extract navigation decoding behind a generic dispatch~~ — **done**:
  `nav_decode` ([navigation.rs](src/navigation.rs)) dispatches by constellation;
  GPS/QZSS LNAV lives in [gps_lnav.rs](src/gps_lnav.rs), Galileo I/NAV in
  [galileo_inav.rs](src/galileo_inav.rs). (A `NavigationDecoder` trait could
  formalize it further, but the module split already removes the GPS-centricity.)
- Galileo E1 needs a BOC(1,1)-aware correlator, embedded 4092-chip memory codes,
  and an I/NAV decoder (FEC + interleaving); GLONASS L1 is FDMA (per-SV carrier),
  which breaks the single-IF assumption end-to-end. Both rank well below QZSS.
- ~~**Hatch filter (carrier smoothing)**~~: implemented and measured — then
  reverted as redundant. A per-SV Hatch filter on the solver's pseudoranges
  (predict by −λ·Δadr, blend the code at 1/N) produced **bit-identical fixes**
  on CTTC with smoothing on vs off: the tracking loop is *carrier-aided*
  (`code_off -= doppler/fc` every period, DLL nudging at τ≈0.16 s), so the
  pseudoranges are already carrier-smoothed at the loop level and an explicit
  Hatch stage has nothing left to remove (σ_gps ≈ 1.6 m raw single-frequency
  residual RMS on CTTC corroborates). Don't re-attempt without first widening
  the DLL bandwidth or de-aiding the code loop — the smoothing already lives
  there.
- Trait-based ionosphere model (Klobuchar / NeQuick / none) in
  [solver.rs](src/solver.rs).

## Feature roadmap

Gap analysis (2026-06) — missing pieces for developer-cornerstone status.
The "Improvement backlog" above covers code quality; this covers capability.
Items are roughly ordered by leverage.

### A. Standard output interfaces — highest-leverage gap

Without these gnss-rcv cannot integrate with any external tool:

| Item | Impact |
|---|---|
| **NMEA output** (`$GPGGA`, `$GPRMC`) | Universal interface — chart plotters, GIS, autopilots, loggers. Add `--nmea <port/file/->`. |
| **RINEX observation file** | Post-processing with rtklib/BERN/PPP services; needs raw pseudoranges+Doppler per epoch. |
| **Structured observation log** | Line-per-epoch JSON/CSV `(tow, sv, pseudorange, cn0, doppler)` for Python/MATLAB algorithm work. |
| **RTCM3** | Interop with RTK base stations and NTRIP correction streams. |

### B. Accuracy — open issues

| Item | Status |
|---|---|
| **Troposphere model** (Saastamoinen) | ✅ Done — standard-atmosphere ZHD+ZWD, slant-mapped, applied per SV in `solver.rs`. |
| **Per-SV pseudorange bias** | ✅ **Closed at the root (2026-06-12)** — the Doppler-proportional bias (−0.03 m/Hz, ~170 m on gpssim) was the **orbit-epoch error of a t_tx anchored 0.160 s early**: the LNAV anchor omitted its 8-preamble-bit decode latency "by convention", and while the time part folds into the receiver clock bias, the solver also evaluates each SV *orbit* at t_tx — 0.16 s early = v_sv·Δt along-track = λ·doppler·0.16 on the LOS. Pinned by an fs sweep (raw slope −0.0292 ± 0.0002 m/Hz, *identical* from 2.046 to 12.276 Msps — an epoch error's signature, not the sampling grid's) and by 0.157 ≈ 0.160 s. Both decoders now anchor at their full structural latency (LNAV 0.160 s, I/NAV 2.000 s; t_tx absolutely correct, anchor-instrument delta 0.000 s) and the `doppler/fc·τ` measurement term + `GNSS_DLL_LAG` are **removed** (`DLL_DISC_GAIN_*` remain as loop tuning only). gpssim raw slope +0.001 m/Hz, σ 4 m, fix ~2 m, no correction at all. Full trail: [docs/dll-group-delay.md](docs/dll-group-delay.md). |
| **Combined GPS + Galileo fix** | ✅ **Live** — a flag-less run solves the mixed GPS+Galileo(+EGNOS) pool through the production WLS with per-constellation weighting and an inter-system-bias state (`wls_solve`; `GNSS_SOLVER=rtk` selects the gnss-rtk fallback); the receiver-side single-pass architecture is [docs/multi-signal-stepping.md](docs/multi-signal-stepping.md) and the mixed gate is pinned in validate_fix.py (tuni2025). The full de-risking trail — week rebase, the 1.840 s LNAV/I-NAV anchor alignment, the epoch-sensitivity exoneration, weighting+ISB, GGTO decode and the ISB decomposition (+2.9 ns GGTO / −7.5 ns hardware) — is [docs/gps-galileo-timing.md](docs/gps-galileo-timing.md). Open there: NeQuick-G for Galileo-only iono. (The "anchor-instrument 0.16 s bookkeeping" turned out not to be cosmetic — it was the per-SV pseudorange bias; both anchors now pin at their full structural latency, see that row.) |
| **GPS week rollover** | ✅ **Closed (2026-06-22)** — `gps_lnav::resolve_gps_week(raw10, anchor)` reconstructs the full week by anchoring to the receiver's known time (system clock via `current_gps_week`, rebased to GPST; overridable by `--eph-date`) and stepping 1024-week eras back until the epoch is not in the future — correct for any capture within ~19.6 yr of the anchor. ion-rtlsdr/sjtu now decode 2017 (were 2037); this also unblocked A-GNSS auto-date-recovery for pre-2019 GPS captures. (Mixed pools were already fixed by the solver's live week-rebase; Galileo's 12-bit GST week has no near-term rollover.) |
| **Sustained nav bit-sync on marginal recordings** | ✅ Improved (sync), open (deep). The brittle LNAV recovery is fixed (`gps_lnav.rs`): bit sync rides out brief weak-bit runs (`WEAK_BIT_LIMIT`, keeping the 300-bit alignment) and a frame-sync slip keeps bit sync. On the **FGI 2023 recording** GPS went from 3 ephemerides / no fix → a **fix** (54 subframes; matches that capture's Galileo fix to ~30 m); gpssim unchanged. **SJTU resolved (2026-06-12)**: the ~23 s lock sawtooth was the front end's ~850 ns/s sample-clock vs LO skew outrunning the first-order DLL's authority — fixed by making the code loop PI (rate-trim integrator + persistence-gated pull-in gear; divergence-guard baseline delayed past the windup) → 60 s unbroken locks, 6 ephemerides, **first fix ever** (17/17, SJTU Minhang). Full analysis: [docs/dll-pi-loop.md](docs/dll-pi-loop.md). SJTU σ since dropped to 1–3 m with the tx-anchor-latency fix (per-SV-bias row). Opens: deeply marginal captures (HackRF). |

### C. Constellation & frequency gaps

| Item | Notes |
|---|---|
| **Galileo E1** | ✅ Done — acquires, tracks, decodes, fixes. See [docs/galileo-e1.md](docs/galileo-e1.md). |
| **Galileo OSNMA** | ✅ **done — full nav-data authentication, live.** Wired automatically on any E1B run: I/NAV → 40-bit field (`page[132..172]`) → one shared verifier over the [`galileo-osnma`](https://github.com/daniestevez/galileo-osnma) crate ([`osnma.rs`](src/osnma.rs)), GST per page from the word-5 anchor; 2023/2024/2025 trust anchors built in, auto-selected by decoded GST week. On **`tuni2025`** (clean 2025, 8 E1B SVs) the whole chain closes: DSM-KROOT (NB=8) reassembled → ECDSA-verified against the built-in 2024 PKID-1 key → TESLA chain validated → **7 SVs authenticated** (E02/10/11/25/30/34/36). On the FGI 2023 captures it reaches the DSM-PKR (clean, decode byte-perfect) / a KROOT-minus-one-block (jammertest) but not full auth — those windows lack a complete KROOT. Full write-up: [docs/osnma.md](docs/osnma.md). |
| **GPS L2C / L5** | Dual-frequency → ionosphere-free combination; opens the door to PPP. |
| **GLONASS L1** | FDMA (per-SV carrier); breaks single-IF assumption end-to-end. Hard. |
| **BeiDou B1** | Completes global coverage. |

#### Galileo E1 — implementation notes

✅ **Complete** — E1-B acquires, tracks, decodes I/NAV, and fixes
(Galileo-only and mixed). The implementation story — embedded memory codes,
BOC(1,1) correlator, the I/NAV FEC/decoder and per-word ephemeris
extraction, GST/BGD solver wiring, and the half-symbol-rate Costas false
lock (root cause + two-layer fix) — is in
[docs/galileo-e1.md](docs/galileo-e1.md). (I/NAV bit layouts were
cross-checked against gnss-sdr's Galileo_INAV.h during bring-up.)

### D. Developer experience

| Item | Notes |
|---|---|
| **SigMF `.meta` auto-config** | Parse the JSON sidecar to set `--fs`/`--fi`/center_freq automatically instead of requiring manual flags. |
| **Config file (TOML)** | Save per-recording profiles; avoids long CLI commands. |
| **Interactive time-series in UI** | Live scrolling CN0/Doppler/code-phase in the egui window; more useful than `--plots` PNGs. |
| **NTRIP client** | Fetch DGNSS/RTK corrections from a public caster (IGS, EUREF) for differential positioning. |
| **SoapySDR abstraction** | Cover USRP, HackRF, LimeSDR, BladeRF live; currently only RTL-SDR. |
| **Apply SBAS corrections** | ✅ Done — MT2-5/24/25 + MT18/26 decoded and applied live in the solver ([docs/sbas.md](docs/sbas.md)). |

### E. Library & API surface

`src/lib.rs` is ~20 lines; no stable embeddable API. For downstream use:
- Clean `pub` API with docstrings on `Channel`, `Receiver`, `Ephemeris`.
- A `Measurement` type exposing `(sv, t_tx, pseudorange, carrier_phase, cn0, doppler)` per epoch — the atom every GNSS algorithm consumes.
- `#[no_std]` compatibility for embedded targets (long-term).

### F. Desktop UI (egui) — IQ-file picker & OSNMA

The egui app ([`app.rs`](src/app.rs)) is now **metadata-driven off the recordings
we track in [`fetch.py`](resources/fetch.py)**: `fetch.py` emits
[`resources/manifest.json`](resources/manifest.json) on every run (name/dest/flags),
and [`recordings.rs`](src/recordings.rs) reads it — one source of truth, no
duplicated list. All items below are done.

| Item | Status |
|---|---|
| **File list synced with `fetch.py`** | ✅ The picker lists the manifest's recordings ([`recordings.rs`](src/recordings.rs)); add a recording in `fetch.py` → it shows up. |
| **Only show provisioned files** | ✅ Filtered to recordings whose `dest` exists under `resources/`. |
| **Auto-select `-t` format on pick** | ✅ Set from the recording's parsed `flags`. |
| **Auto-select `--fi` / `--fs` on pick** | ✅ Filled from the parsed `flags` (editable `DragValue`s). |
| **Signal dropdown: `L1CA` / `E1B` / `E1C`** | ✅ Drives `config.sig` (was mis-wired, L1CA-only). The SV table also follows the signal's constellation, so E1B runs populate it. |
| **OSNMA on by default** | ✅ Checkbox, default on; wired into `ReceiverConfig.osnma`. |
| **Per-SV OSNMA-verified badge** | ✅ The SV table's "osnma" column shows a green ✓ once a satellite authenticates, from `ChannelState.osnma_verified` (set by the receiver's `feed_osnma` via its `pub_state` handle). Lights up on a KROOT-bearing stream. |
