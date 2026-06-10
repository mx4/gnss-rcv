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
  acquire/track tests `synthetic_signal_acquires_and_tracks` and
  `synthetic_noisy_multi_sv_acquires_and_tracks`, which synthesize their own IQ
  via [`synth.rs`](src/synth.rs) and need no recording) + the fast integration
  test `acquires_and_tracks_gpssim` (~0.6 s). Run this after any change.
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
`navigation.rs`/`gps_lnav.rs`/`galileo_inav.rs`, `ephemeris.rs`, `solver.rs`,
`receiver.rs`, `recording.rs`),
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

[`scripts/validate_fix.py`](scripts/validate_fix.py) wraps this into one command.
It builds release and runs three `--json`-driven checks, each **skipping cleanly**
(exit 0) when its recording is absent, failing (non-zero) only if a *present*
recording's check fails:

- **GPS fix** (gpssim fixture): runs to the first fix with `GNSS_TRUTH_ECEF` on,
  and prints the residual spread (from the `RESID` stderr diagnostic) + the fix
  error vs truth with a **PASS/FAIL** verdict (gate ~2 km). Read it as: `resid`
  per SV ≈ a common constant (the rx clock bias); the **spread** is the geometry
  error (sub-km good, 100s of km means transmit-time/pseudorange is wrong).
- **Galileo E1-B I/NAV decode + fix** (every present recording): asserts Galileo
  SVs track and decode CRC-valid I/NAV words; on a recording long enough to
  complete ≥4 ephemerides (ION LimeSDR, 60 s) it also asserts a real Galileo-only
  position fix (~110 m from the 52.177, 4.488 site truth). Short recordings
  (PocketSDR ~30 s) assert only the decode chain.
- **SBAS L1 decode** (a capture with an SBAS GEO overhead — CTTC Spain → EGNOS):
  asserts a floor of CRC-valid SBAS messages from ≥1 GEO. On CTTC, S120 + S126
  decode ~32 messages (types 0/1/2/3/4/24/25/26/27).

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
- IQ formats (`-t`): `2xf32` (default), `2xi16`, `2xi8`, `i8`, `rtlsdr-file`, `1bit`, `4bit`.
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
`gpssim_2xi16` is *generated* by gps-sdr-sim (`./resources/gen_gpssim.sh`, needs
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
| `sjtu` | `-t 4bit --fs 25000000 --fi 6250000` | Shanghai; tracks 15–29 SVs 45–50 dB-Hz, decodes subframes; slow bit-sync, no full fix in 60 s |
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
- ~~**SBAS L1 message decode**~~: [`sbas_l1.rs`](src/sbas_l1.rs) — 2 ms symbols
  (2 C/A periods) → continuous K=7 Viterbi → 250-bit messages framed on the
  0x53/9A/C6 preamble → CRC-24Q. Shares the FEC code + CRC with Galileo via
  [`fec.rs`](src/fec.rs). On CTTC (Spain) EGNOS S120/S126 decode MT 0/1/2/3/4/24/25/26/27.
  (Decode only — *applying* the corrections to the fix is still open, see roadmap.)
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
- **`LnavState.bits` uses `rotate_left(1)`** on an 18 KB buffer per nav symbol
  ([gps_lnav.rs](src/gps_lnav.rs)): same anti-pattern as the old `History`,
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
| **Per-SV pseudorange bias** | ✅ Fixed — it was the **DLL code-loop group delay** (the tracked code phase lags the true one by code-Doppler × τ), so the residual was *linear in Doppler* (−0.03 m/Hz, ~170 m, instantaneous). Compensated by `code_off += doppler/fc·τ` in `channel.rs`, τ = `0.25/(B_DLL·DLL_DISC_GAIN)` per signal (gpssim 165 m→4 m; CTTC real GPS ~20 m). E1's BOC peak needs a much smaller discriminator gain (τ≈1.95 s) → Galileo 3.4 km→~110 m. Full write-up: [docs/dll-group-delay.md](docs/dll-group-delay.md). |
| **Combined GPS + Galileo fix** | Open. Today a run tracks one signal (`--sig`); decoding GPS L1 C/A *and* Galileo E1 together (they share the L1 band) would give more SVs, better geometry, and a per-constellation cross-check. Needs the receiver to run two signals at once (the single-signal-per-run limit) and the solver to mix GPST + GST candidates (gnss-rtk already handles per-constellation timescales). Would also let a thin-Galileo capture lean on GPS. |
| **GPS week rollover** | `week = getbitu(…,10) + 2048` only covers to ~2038; needs a date-anchored resolver (known real-world recordings already show "2032 GPST"). |
| **Sustained nav bit-sync on marginal recordings** | ✅ Improved (sync), open (deep). The brittle LNAV recovery is fixed (`gps_lnav.rs`): bit sync now rides out brief weak-bit runs (`WEAK_BIT_LIMIT`, keeping the 300-bit alignment) and a frame-sync slip keeps bit sync — so the 3 consecutive clean subframes an ephemeris needs line up. On the **FGI 2023 recording** GPS went from 3 ephemerides / no fix → a **fix** (54 subframes, 1 lock loss; matches that capture's Galileo fix to ~30 m). gpssim unchanged. Still open for **deeply marginal** captures (SJTU 4-bit/25 MHz, HackRF): there the bottleneck is now *tracking* lock loss (~23 losses in 60 s → only 3 subframes), i.e. carrier/code-loop robustness, **not** nav sync — a separate item. |

### C. Constellation & frequency gaps

| Item | Notes |
|---|---|
| **Galileo E1** | BOC(1,1) correlator, 4092-chip memory codes, I/NAV FEC decoder. Highest ROI after GPS. See detailed plan below. |
| **Galileo OSNMA** | ✅ wired (`--osnma`): I/NAV → 40-bit field (`page[132..172]`) → one shared verifier over the [`galileo-osnma`](https://github.com/daniestevez/galileo-osnma) crate ([`osnma.rs`](src/osnma.rs)), GST per page from the word-5 anchor. On the FGI 2023 recording the **DSM-PKR public key verifies against the GSC Merkle root** (decode byte-perfect); full nav-data auth pends a complete DSM-KROOT (absent from that PKR-dominated capture). 2023 trust anchor built in; mind the 2024-01-15 tree renewal. Full write-up: [docs/osnma.md](docs/osnma.md). |
| **GPS L2C / L5** | Dual-frequency → ionosphere-free combination; opens the door to PPP. |
| **GLONASS L1** | FDMA (per-SV carrier); breaks single-IF assumption end-to-end. Hard. |
| **BeiDou B1** | Completes global coverage. |

#### Galileo E1 — implementation plan

**Progress — E1-B acquires, tracks, decodes, and *fixes*.** `code::Signal`
replaced the `"L1CA"` string; the E1-B/E1-C primary memory codes are embedded
([`galileo_e1_codes.rs`](src/galileo_e1_codes.rs)) and BOC(1,1)-modulated
(`code::boc11()`); the receiver steps the signal's 4 ms code period; and
`get_sat_list` tags Galileo PRNs from the signal. On the ION LimeSDR capture
`--sig E1B` locks **E01/E04/E09/E11/E19** and holds a **60 s lock at 37–49 dB-Hz**.
The **I/NAV page decoder** ([`galileo_inav.rs`](src/galileo_inav.rs)) is in and
**decodes real, CRC-valid words**: per-4 ms symbols → preamble sync (with polarity
*and the (−1)ⁿ half-rate ambiguity*) → de-interleave → Viterbi → even/odd page
assembly → CRC-24Q → 128-bit word. The **ephemeris extraction** from word types
1–5 (`galileo_inav::decode_ephemeris_word`, Steps 4–5) and the **time + solver
wiring** (Step 6 — GST week+TOW → absolute epochs, constellation-aware µ, shared
transmit-time anchor) are both in. **All 5 SVs now complete a physically valid
orbit** (GST week 947 ≈ Oct 2017, a≈29 600 km) and feed gnss-rtk to produce a
**Galileo-only fix ~110 m from the 52.177, 4.488 site truth** (after the signal-
aware DLL group-delay compensation — E1's BOC peak needs a much larger τ than
L1CA, see the per-SV-bias backlog entry).

The earlier "only 2 SVs (E09/E11) complete an ephemeris" was **not** a data
limitation — it was a decode bug. E01/E04/E19 held a strong continuous lock
(E01 the *strongest* at 47.8 dB-Hz) yet decoded zero CRC-valid words because
their carrier had settled into a **Costas-loop false lock at half the symbol
rate** (±125 Hz = 250 sym/s ÷ 2): a π/symbol rotation that the `atan(Q/I)`
prompt discriminator and the `atan(cross/dot)` FLL both fold to zero error, so
tracking is happy while every symbol is multiplied by (−1)ⁿ — deterministic per
SV, independent of C/N0. (GPS L1 C/A is immune: its PLL updates 20× per data bit,
so its discriminator is not data-ambiguous at the bit rate. The hazard is
specific to E1-B's one-symbol-per-code-period layout.)

It is fixed at **two layers**: (1) the decoder resolves the ambiguity at frame
sync (`InavDecoder::match_preamble` tries the de-alternated stream alongside the
polarity hypotheses), and (2) the **root cause** in tracking —
`Channel::correct_half_rate_false_lock` (channel.rs) compares the PLL Doppler to
the *code-implied* Doppler from the transmit-phase slope (`d(t_tx)/d(t_rx) =
1 + dopp/fc`; the code/DLL loop is immune to the carrier aliasing) and snaps the
carrier onto the nearest half-rate step, pulling it onto the true lock within
~5 s. Layer (1) covers the seconds before (2) engages and any transient slip;
(2) makes the carrier Doppler itself correct (needed for velocity / carrier-phase,
not just for the pseudorange-only fix). Verified on LimeSDR: E01/E04/E19 each
correct exactly once (+125 Hz) with no lock loss, and the PLL vs code-implied
Doppler gap drops from ~−125 Hz to ~0 (`GAL_DOPP_CHECK=1` logs the comparison).

Key differences from GPS L1 C/A that drive every change:

| Property | GPS L1 C/A | Galileo E1-B |
|---|---|---|
| Modulation | BPSK | BOC(1,1) — code × square-wave subcarrier at 1.023 MHz |
| Code length | 1023 chips | 4092 chips (memory codes, not Gold codes) |
| Code period | 1 ms | 4 ms |
| Carrier | 1575.42 MHz | 1575.42 MHz (same — no RF changes) |
| Nav data rate | 50 bps | 250 bps (before FEC) |
| FEC | None (Hamming parity per word) | Rate-1/2 conv (K=7, G1=171₈, G2=133₈) + 8×30 block interleaver |
| Frame check | 6-bit Hamming parity | CRC-24Q per page |
| Full ephemeris | Subframes 1–3 (~60 s) | Word types 1–4 (~8 pages ≈ 16 s) |

**Implementation order** (each step is independently testable):

**Step 1 — `src/code.rs`: E1-B/E1-C memory codes** ✅ DONE
- The 4092-chip E1-B/E1-C primary memory codes are embedded as hex in
  [`galileo_e1_codes.rs`](src/galileo_e1_codes.rs) (OS SIS ICD Annex C, 50 PRNs);
  `e1_primary_code` decodes them to ±1, `spreading_code()` wraps them in `boc11()`.
- Tested: `e1_primary_codes_are_valid` (bipolar, balanced, strong autocorr peak).

**Step 2 — `src/channel.rs`: BOC(1,1) correlator** ✅ acquires + tracks
- `code::boc11()` produces the `[+code, −code]` per-chip replica, returned by
  `spreading_code()`, so `Channel::new`'s resampling already consumes it — E1-B
  acquires *and* tracks as-is (the GPS-only DLL assertion `n == 10` was removed;
  the loops are otherwise period-generic — all 5 LimeSDR SVs hold a 60 s lock).
- Left: `nav_decode()` routing — branch on `Constellation::Galileo` before the
  LNAV path, feeding the I/NAV symbol (sign of prompt-I, one per 4 ms code
  period) to a `nav_decode_inav()` built on [`galileo_inav.rs`](src/galileo_inav.rs).

**Step 3 — receiver wiring** ✅ DONE
- No separate `--galileo` flag: selecting the signal (`--sig E1B`) drives it.
  `get_sat_list` tags every selected PRN `Constellation::Galileo` when the signal
  is Galileo (E1 PRNs 1–36 overlap GPS, so the *signal*, not the number, decides).
  The receiver steps the signal's 4 ms code period. Stats / xcorr rejection are
  already generic.

**Step 4 — `src/ephemeris.rs`: Galileo fields** ✅ DONE
- `Ephemeris` is constellation-agnostic (the GPS LNAV parsers moved to
  `gps_lnav.rs`); the Keplerian/clock fields are shared as-is. BGD(E1,E5a) is
  stored in the existing `tgd` field (the E1 group delay for E1-only reception);
  separate `bgd_*` fields can come later if E5 is added.
- `is_valid()` is constellation-agnostic — it gates on `week != 0` (no GPS week
  range) plus an orbit-size bound that passes both GPS and Galileo MEO.

**Step 5 — `src/galileo_inav.rs`: I/NAV ephemeris extraction** ✅ DONE
The page decoder (sync → FEC → CRC → CRC-valid 128-bit word) and the ephemeris
extraction are both in. `decode_ephemeris_word(&mut Ephemeris, &InavWord)` fills
the orbit/clock per word type, `ubits`/`sbits` reading the ICD bit layout
(offsets + scales cross-checked vs gnss-sdr's `Galileo_INAV.h`):
- **Word 1**: IODnav, t0e (60 s LSB), M0, e, √a
- **Word 2**: Ω0, i0, ω, i_dot
- **Word 3**: Ω_dot, Δn, C_uc/C_us/C_rc/C_rs
- **Word 4**: C_ic/C_is, t0c (60 s LSB), a_f0/a_f1/a_f2 (2^-34/-46/-59)
- **Word 5**: BGD(E1,E5a)→`tgd`, **GST week** (the only page carrying it), GST TOW

Tested by `decodes_inav_words_into_a_valid_ephemeris` (hermetic — locks every
offset, scale, the 60 s LSBs, and signedness) and confirmed on the LimeSDR
capture (all 5 SVs' orbits complete, is_valid). The Viterbi decoder (rate-1/2 K=7)
was the hardest single piece. Reference: Galileo OS SIS ICD (ESA GSC), Tables 4
and 24–29.

**Step 6 — `src/solver.rs` + `src/constants.rs`: GST and BGD** ✅ DONE — fix lands
- `EARTH_MU_GAL = 3.986004418e14` added; `solver::earth_mu(sv)` picks GTRF vs WGS-84
  µ in the Kepler solve and the relativistic clock term (~1 mm difference).
- BGD: `group_delay` already returns `eph.tgd`, which the I/NAV decoder fills with
  BGD(E1,E5a) — the E1-only group delay — so no Galileo-specific branch is needed.
- Time: absolute toe/toc/tow epochs are built from the GST week+TOW via
  `Epoch::from_time_of_week(week, .., TimeScale::GST)` (hifitime carries the GST↔GPST
  offset, so the solver's absolute-duration math is correct without a manual GGTO).
  The transmit anchor is shared with GPS (`Channel::nav_anchor_tx`), pinned on a
  word-type-5 page so the TOW and code-period count are captured together.
- Carrier::L1 already covers Galileo E1 (1575.42 MHz) in gnss-rtk — no change needed.
- **A real Galileo-only fix lands.** Once the half-rate false-lock decode bug was
  fixed (see the progress note above), the 61 s LimeSDR capture completes **all 5
  Galileo ephemerides** (E01/E04/E09/E11/E19) and `validate_fix.py` asserts a fix
  **~110 m** from the 52.177, 4.488 site truth. Also covered by solver unit tests
  (`earth_mu_is_constellation_specific`, `galileo_ephemeris_computes_a_meo_position`).
  The per-SV DLL-group-delay bias is compensated per signal — E1's BOC correlation
  peak needs a much smaller discriminator gain (τ≈1.95 s vs L1CA's 0.157 s), which
  took the Galileo fix 3.4 km → ~110 m (see the per-SV-bias backlog entry).

### D. Developer experience

| Item | Notes |
|---|---|
| **SigMF `.meta` auto-config** | Parse the JSON sidecar to set `--fs`/`--fi`/center_freq automatically instead of requiring manual flags. |
| **Config file (TOML)** | Save per-recording profiles; avoids long CLI commands. |
| **Interactive time-series in UI** | Live scrolling CN0/Doppler/code-phase in the egui window; more useful than `--plots` PNGs. |
| **NTRIP client** | Fetch DGNSS/RTK corrections from a public caster (IGS, EUREF) for differential positioning. |
| **SoapySDR abstraction** | Cover USRP, HackRF, LimeSDR, BladeRF live; currently only RTL-SDR. |
| **Apply SBAS corrections** | `--sbas` now *decodes* the L1 messages ([`sbas_l1.rs`](src/sbas_l1.rs)); parsing MT2–5/24/25 (fast + long-term) and MT18/26 (iono grid) and applying them in the solver would bring single-frequency accuracy to ~1 m. The SBAS GEO can also serve as an extra ranging source (MT9 ephemeris). |

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
duplicated list. Items 1–6 below are done; the OSNMA-verified badge remains.

| Item | Status |
|---|---|
| **File list synced with `fetch.py`** | ✅ The picker lists the manifest's recordings ([`recordings.rs`](src/recordings.rs)); add a recording in `fetch.py` → it shows up. |
| **Only show provisioned files** | ✅ Filtered to recordings whose `dest` exists under `resources/`. |
| **Auto-select `-t` format on pick** | ✅ Set from the recording's parsed `flags`. |
| **Auto-select `--fi` / `--fs` on pick** | ✅ Filled from the parsed `flags` (editable `DragValue`s). |
| **Signal dropdown: `L1CA` / `E1B` / `E1C`** | ✅ Drives `config.sig` (was mis-wired, L1CA-only). The SV table also follows the signal's constellation, so E1B runs populate it. |
| **OSNMA on by default** | ✅ Checkbox, default on; wired into `ReceiverConfig.osnma`. |
| **Per-SV OSNMA-verified badge** | Open. Show a ✓/lock next to authenticated SVs in the channel table, fed by `OsnmaVerifier::is_authenticated`. Needs the verified-SV set surfaced from the receiver into the shared UI state ([`state.rs`](src/state.rs)) — the receiver currently keeps it in `Receiver.osnma_authenticated`. |
