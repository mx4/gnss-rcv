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
reads the `--json` summary (fix + funnel) from stdout plus the `RESID` stderr
diagnostic, and prints the residual spread + the fix error vs truth with a
**PASS/FAIL** verdict (non-zero exit when the fix is missing or worse than the
~2 km gate). It **skips cleanly** (exit 0) when the fixture is absent. Read the
result as: `resid` per SV ≈ a
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
- ~~Replace the string signal id with a `SignalType` enum~~ — **done**:
  `code::Signal` (in [code.rs](src/code.rs)) carries the carrier/code params and
  the spreading code, threaded through channel/device/network/receiver/main. It
  is the seam the Galileo E1 work builds on.
- Extract navigation decoding (~440 lines of `impl Channel` in
  [navigation.rs](src/navigation.rs)) behind a `NavigationDecoder` trait.
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
| **Per-SV ~0.5 ms bias** | Open; residual visible in `validate_fix.py` spread. |
| **GPS week rollover** | `week = getbitu(…,10) + 2048` only covers to ~2038; needs a date-anchored resolver (known real-world recordings already show "2032 GPST"). |
| **Sustained nav bit-sync on marginal recordings** | Open. The bit-sync *logic* is sound — a clean synthetic SV holds sync with zero loss at 40–50 dB-Hz in both the 2.046 MHz and SJTU 25 MHz/fi=fs/4 regimes (`examples/bitsync_bench.rs`). But SJTU/HackRF lose sync after ~1 subframe: degraded real prompt-I (the 20 ms coherent nav integration `nav_mean_ip` collapses below `THRESHOLD_LOST`) **plus** a brittle recovery path — a single failed frame-sync check hard-resets both `bit_sync` and `nav_sync` (navigation.rs ~471), forcing a full ~7 s re-sync, so 3 *consecutive* clean subframes rarely line up. Fix: gentler sync recovery (hysteresis / don't hard-reset on one miss) and/or tighter carrier tracking; validate that the recordings that already fix still do. |

### C. Constellation & frequency gaps

| Item | Notes |
|---|---|
| **Galileo E1** | BOC(1,1) correlator, 4092-chip memory codes, I/NAV FEC decoder. Highest ROI after GPS. See detailed plan below. |
| **GPS L2C / L5** | Dual-frequency → ionosphere-free combination; opens the door to PPP. |
| **GLONASS L1** | FDMA (per-SV carrier); breaks single-IF assumption end-to-end. Hard. |
| **BeiDou B1** | Completes global coverage. |

#### Galileo E1 — implementation plan

**Progress — E1-B acquires *and tracks*.** `code::Signal` replaced the `"L1CA"`
string; the E1-B/E1-C primary memory codes are embedded
([`galileo_e1_codes.rs`](src/galileo_e1_codes.rs)) and BOC(1,1)-modulated
(`code::boc11()`); the receiver steps the signal's 4 ms code period; and
`get_sat_list` tags Galileo PRNs from the signal. On the ION LimeSDR capture
`--sig E1B` locks **E01/E04/E11** and holds a **40 s lock at 43–48 dB-Hz**. The
I/NAV FEC layer — rate-1/2 Viterbi, 30×8 interleaver, CRC-24Q — is in
([`galileo_inav.rs`](src/galileo_inav.rs)). **Remaining for an E1 *fix*:** I/NAV
page-part sync + even/odd assembly, the word-type ephemeris fields (Step 4), and
routing the per-4 ms-period tracking symbols into the decoder.

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
  the loops are otherwise period-generic — E01/E04/E11 hold a 40 s lock).
- Left: `nav_decode()` routing — branch on `Constellation::Galileo` before the
  LNAV path, feeding the I/NAV symbol (sign of prompt-I, one per 4 ms code
  period) to a `nav_decode_inav()` built on [`galileo_inav.rs`](src/galileo_inav.rs).

**Step 3 — receiver wiring** ✅ DONE
- No separate `--galileo` flag: selecting the signal (`--sig E1B`) drives it.
  `get_sat_list` tags every selected PRN `Constellation::Galileo` when the signal
  is Galileo (E1 PRNs 1–36 overlap GPS, so the *signal*, not the number, decides).
  The receiver steps the signal's 4 ms code period. Stats / xcorr rejection are
  already generic.

**Step 4 — `src/ephemeris.rs`: Galileo fields**
- Add `bgd_e5a_e1: f64` and `bgd_e5b_e1: f64` (replaces `tgd` for Galileo; use `bgd_e5a_e1`
  as the L1 group-delay correction for E1-only reception).
- `is_valid()`: the GPS week range check `(2048..3000).contains(&self.week)` rejects Galileo
  weeks; make it constellation-aware.
- The orbital math (a, e, i0, M0, ω, Ω0, perturbations) is identical to GPS — no new fields.

**Step 5 — `src/navigation.rs`: I/NAV decoder**
New functions, all independently unit-testable with known I/NAV frames from the ICD:
```
nav_decode_inav()             — top-level: sync → FEC → CRC → word dispatch
nav_sync_e1b_symbol()         — 1 symbol per 4 ms code period (vs 20 ms/bit for GPS)
nav_viterbi_decode(symbols)   — rate-1/2 K=7 Viterbi, 500 channel symbols → 250 bits
nav_deinterleave_8x30(bits)   — undo the 8×30 block interleaver before Viterbi
nav_crc24q(data) -> bool      — CRC-24Q verification per page
nav_decode_inav_word1..4()    — Keplerian elements from ICD Tables 25–29
```
Word type content:
- **Word 1**: IODnav, t_oe, M0, e, √a
- **Word 2**: IODnav, Ω0, i0, ω, i_dot
- **Word 3**: IODnav, Ω_dot, Δn, C_uc/C_us/C_rc/C_rs, SISA
- **Word 4**: IODnav, C_ic/C_is, t_oc, a_f0, a_f1, a_f2, BGD(E1,E5a), BGD(E1,E5b)

The Viterbi decoder (~150 lines, rate-1/2 K=7) is the hardest single piece.
Reference: Galileo OS SIS ICD (freely downloadable from ESA GSC), Tables 4 and 24–29.

**Step 6 — `src/solver.rs` + `src/constants.rs`: GST and BGD**
- Add `EARTH_MU_GAL = 3.986004418e14` (Galileo ICD value; GPS is `3.9860058e14` — differs
  ~0.01 ppm → ~1 mm on SV position; use the right constant based on constellation).
- `ReceiverSpacebornBias::group_delay`: return `eph.bgd_e5a_e1` for Galileo SVs.
- GST → GPST: Galileo System Time differs from GPST by a constant offset carried in I/NAV
  word type 6. For a first implementation, treat GST ≈ GPST (offset is sub-µs historically)
  and refine once word-type-6 decoding is in place.

### D. Developer experience

| Item | Notes |
|---|---|
| **SigMF `.meta` auto-config** | Parse the JSON sidecar to set `--fs`/`--fi`/center_freq automatically instead of requiring manual flags. |
| **Config file (TOML)** | Save per-recording profiles; avoids long CLI commands. |
| **Interactive time-series in UI** | Live scrolling CN0/Doppler/code-phase in the egui window; more useful than `--plots` PNGs. |
| **NTRIP client** | Fetch DGNSS/RTK corrections from a public caster (IGS, EUREF) for differential positioning. |
| **SoapySDR abstraction** | Cover USRP, HackRF, LimeSDR, BladeRF live; currently only RTL-SDR. |
| **SBAS correction decoding** | The `--sbas` flag acquires SBAS SVs but MT1–MT6 messages are not decoded; applying them would bring single-frequency accuracy to ~1 m. |

### E. Library & API surface

`src/lib.rs` is ~20 lines; no stable embeddable API. For downstream use:
- Clean `pub` API with docstrings on `Channel`, `Receiver`, `Ephemeris`.
- A `Measurement` type exposing `(sv, t_tx, pseudorange, carrier_phase, cn0, doppler)` per epoch — the atom every GNSS algorithm consumes.
- `#[no_std]` compatibility for embedded targets (long-term).
