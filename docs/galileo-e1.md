# Galileo E1-B — implementation notes

> **Status: COMPLETE.** E1-B acquires, tracks, decodes I/NAV, and produces
> Galileo-only and mixed GPS+Galileo fixes. This records what was built, in
> what order, and the one genuinely subtle failure mode (the half-symbol-rate
> Costas false lock). Mixed-solve timing/weighting is in
> [gps-galileo-timing.md](gps-galileo-timing.md), OSNMA in
> [osnma.md](osnma.md), the stepping architecture in
> [multi-signal-stepping.md](multi-signal-stepping.md).

Reference: Galileo OS SIS ICD (ESA GSC), Tables 4 and 24–29 for I/NAV,
Annex C for the E1 codes.

## What makes E1-B different from GPS L1 C/A

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

Every implementation step below falls out of one of these rows.

## The pieces

**Codes** ([`galileo_e1_codes.rs`](../src/galileo_e1_codes.rs),
[`code.rs`](../src/code.rs)). The 4092-chip E1-B/E1-C primary memory codes
are embedded as hex (OS SIS ICD Annex C, 50 PRNs); `e1_primary_code` decodes
them to ±1 and `spreading_code()` wraps them in `boc11()`, which produces the
`[+code, −code]` per-chip replica. Tested by `e1_primary_codes_are_valid`
(bipolar, balanced, strong autocorrelation peak).

**Correlator** ([`channel.rs`](../src/channel.rs)). Because `boc11()` returns
a replica in the same shape the resampler consumes, the existing
acquisition/tracking path runs E1-B as-is — the loops are code-period-generic
(a GPS-only `n == 10` DLL assertion was the only casualty). The signal's 4 ms
period drives the channel's stepping grid.

**Receiver wiring.** No separate `--galileo` flag: selecting the signal
(`--sig E1B`, or the bandwidth-gated default families) drives it.
`get_sat_list` tags every selected PRN `Constellation::Galileo` when the
signal is Galileo — E1 PRNs 1–36 overlap GPS, so the *signal*, not the
number, decides.

**Ephemeris fields** ([`ephemeris.rs`](../src/ephemeris.rs)). `Ephemeris` is
constellation-agnostic; the Keplerian/clock fields are shared as-is.
BGD(E1,E5a) is stored in the existing `tgd` field (the E1 group delay for
E1-only reception). `is_valid()` gates on `week != 0` plus an orbit-size
bound that passes both GPS and Galileo MEO.

**I/NAV decode** ([`galileo_inav.rs`](../src/galileo_inav.rs)). Per-4 ms
symbols → preamble sync (with polarity *and* the (−1)ⁿ half-rate ambiguity,
below) → de-interleave → Viterbi (rate-1/2, K=7 — the hardest single piece;
shared with SBAS via [`fec.rs`](../src/fec.rs)) → even/odd page assembly →
CRC-24Q → 128-bit word. `decode_ephemeris_word` fills the orbit/clock per
word type:

- **Word 1**: IODnav, t0e (60 s LSB), M0, e, √a
- **Word 2**: Ω0, i0, ω, i_dot
- **Word 3**: Ω_dot, Δn, C_uc/C_us/C_rc/C_rs
- **Word 4**: C_ic/C_is, t0c (60 s LSB), a_f0/a_f1/a_f2 (2⁻³⁴/⁻⁴⁶/⁻⁵⁹)
- **Word 5**: BGD(E1,E5a)→`tgd`, **GST week** (the only word carrying it),
  GST TOW — the transmit anchor is pinned on a word-5 page so the TOW and
  code-period count are captured together.

`decodes_inav_words_into_a_valid_ephemeris` locks every offset, scale, the
60 s LSBs, and signedness, hermetically. Word 10 (GGTO) and the word-5 iono
inputs (ai0/ai1/ai2 + storm flags) are also decoded — used by the mixed-fix
work ([gps-galileo-timing.md](gps-galileo-timing.md)).

**Time and solver** ([`solver.rs`](../src/solver.rs),
[`constants.rs`](../src/constants.rs)). `EARTH_MU_GAL` (GTRF µ) is selected
per constellation in the Kepler solve and the relativistic clock term;
absolute toe/toc/tow epochs are built from GST week+TOW via
`Epoch::from_time_of_week(.., TimeScale::GST)` (hifitime carries the
GST↔GPST offset, so the solver's absolute-duration math needs no manual
GGTO). `group_delay` returns `eph.tgd` = BGD(E1,E5a) — no Galileo-specific
branch needed for E1-only reception.

## The half-symbol-rate Costas false lock

The one failure mode that cost real diagnosis time. On the ION LimeSDR
capture, three of five SVs (E01/E04/E19 — E01 the *strongest*, 47.8 dB-Hz)
held a continuous 60 s lock yet decoded **zero** CRC-valid words. Their
carrier had settled into a Costas-loop false lock at **half the symbol rate**
(±125 Hz = 250 sym/s ÷ 2): a π-per-symbol rotation that the `atan(Q/I)`
prompt discriminator and the `atan(cross/dot)` FLL both fold to zero error,
so tracking is perfectly happy while every symbol picks up a (−1)ⁿ flip —
deterministic per SV, independent of C/N0.

GPS L1 C/A is immune: its PLL updates 20× per data bit, so its discriminator
is not data-ambiguous at the bit rate. The hazard is specific to E1-B's
one-symbol-per-code-period layout.

Fixed at two layers:

1. **Decoder**: `InavDecoder::match_preamble` tries the de-alternated stream
   alongside the polarity hypotheses at frame sync — covers the seconds
   before (2) engages and any transient slip.
2. **Tracking (root cause)**: `Channel::correct_half_rate_false_lock`
   ([channel.rs](../src/channel.rs), in the code-carrier consistency monitor)
   compares the PLL Doppler to the *code-implied* Doppler from the
   transmit-phase slope (`d(t_tx)/d(t_rx) = 1 + dopp/fc`; the code/DLL loop
   is immune to the carrier aliasing) and snaps the carrier onto the nearest
   half-rate step (1/(2·code_sec) = 125 Hz), pulling it onto the true lock
   within ~5 s. This makes the carrier Doppler itself correct — needed for
   velocity / carrier-phase work, not just the pseudorange fix.

Verified on LimeSDR: E01/E04/E19 each correct exactly once (+125 Hz) with no
lock loss, and the PLL vs code-implied Doppler gap drops from ~−125 Hz to ~0
(`GAL_DOPP_CHECK=1` logs the comparison).

## Results

- ION LimeSDR (2017, 5 IOV/FOC SVs): all 5 ephemerides complete (GST week
  947, a ≈ 29 600 km), Galileo-only fix ~110 m from the 52.177, 4.488 site
  truth at the first calibration — ~148 m with the corrected BOC loop gain
  (see [dll-group-delay.md](dll-group-delay.md): the early τ≈1.95 s "BOC
  needs a huge time constant" calibration was actually compensating the
  then-undiagnosed 2.000 s I/NAV anchor latency; once the anchor was fixed,
  the BOC gain equals BPSK's 3.18 and the Galileo residual spread fell
  1554 → 8 m).
- The hermetic Galileo twin `synthetic_e1_geometry_solves_to_truth` pins the
  E1 pipeline at ~4 m against exact truth ([testing.md](testing.md)).
- FGI 2023, tuni2025: Galileo-only and mixed fixes; OSNMA authentication on
  tuni2025 ([osnma.md](osnma.md)).
- `validate_fix.py` gates the I/NAV decode chain on every present recording
  and the LimeSDR Galileo-only fix permanently.
