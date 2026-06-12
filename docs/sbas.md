# SBAS L1 — message decode, corrections, ionospheric grid

> **Status: COMPLETE** (decode + fast/long-term corrections + iono grid, all
> applied live in the solver). Reference: RTCA DO-229 (MOPS); message types
> cited below. SBAS is on by default (`--no-sbas` opts out); the GEOs also
> feed the mixed GPS+Galileo solve with EGNOS corrections
> ([multi-signal-stepping.md](multi-signal-stepping.md)).

Code: [`sbas_l1.rs`](../src/sbas_l1.rs) (decode),
[`sbas_corr.rs`](../src/sbas_corr.rs) (fast + long-term corrections),
[`sbas_iono.rs`](../src/sbas_iono.rs) (iono grid), application in
[`solver.rs`](../src/solver.rs).

## Message decode (`sbas_l1.rs`)

SBAS L1 shares the C/A code family (PRNs 120–158) at the GPS chip rate but
carries 500 symbols/s: one symbol per **2 C/A code periods**. The chain:
2 ms symbols → continuous K=7 Viterbi (rate-1/2, shared with Galileo I/NAV
via [`fec.rs`](../src/fec.rs)) → 250-bit messages framed on the 0x53/9A/C6
preamble → CRC-24Q.

Two pieces of bookkeeping carry the yield:

- **Code-phase wraps**: the 2-periods-per-symbol grid must survive a
  `code_off` wrap, which inserts or deletes one code period from the prompt
  stream. `wrap_drop`/`wrap_repeat` mirror the wrap into the symbol grid
  (the same invariant the LNAV decoder keeps — any decoder running a
  multi-period grid on top of the code-period stream needs this).
- **Stall watchdog**: if the locked symbol alignment dies (the GEO kept
  tracking but messages stop), the hypothesis search re-opens.

Together these took CTTC from ~30% to ~97% message yield (32 → 194 messages
in 100 s from EGNOS S120+S126; MT 0/1/2/3/4/7/9/10/12/17/18/24/25/26/27).

GEO handling: MT9 GEO positions feed the sky plot (el/az vs the current
fix); EGNOS GEOs stay **out of the fix pool** — flagged do-not-use-for-
ranging. Corrections state is fed from a **single GEO source** (the first to
decode): EGNOS broadcasts test-mode and operational streams from different
GEOs with different IODP/IODI generations, and mixing them wipes the shared
mask/grid state mid-assembly.

Search throttles (the block is on by default, so its mostly-absent PRNs must
stay cheap): GEOs are stationary, so once the receiver LO offset is known
from any tracking channel the Doppler search shrinks to LO ± 3 kHz; idle
longer between attempts; stop searching once 2 GEOs track (every GEO of a
system broadcasts the same corrections).

## Fast + long-term corrections (`sbas_corr.rs`)

- **MT1** PRN mask, IODP-versioned — all correction slots are positions into
  this mask.
- **MT2–5/24** fast corrections (PRC per slot): 30 s freshness bound;
  UDREI ≥ 14 ("do not use") and PRC 0x800 clear the slot; **MT0 is parsed as
  MT2** per the EGNOS test-mode convention (test-mode streams carry real
  corrections in the type-0 frame).
- **MT24/25** long-term corrections: δposition/δclock halves, velocity codes
  0 and 1, IODE-gated against the flown ephemeris.

Applied per GPS SV at the pseudorange level in the solver:

```
pr += PRC + c·δclk − û·δpos      # û: line-of-sight unit vector
```

On CTTC: 13 fast + 14 long-term assembled, per-SV corrections −0.4..−2.6 m
with ~2 m differential spread. The fix moves by a couple of metres there —
but CTTC's "truth" is its own single-frequency NMEA solution, too coarse to
certify an improvement. The certifying judge is the **hermetic regression**
`sbas_fast_corrections_recover_broadcast_clock_errors`
([receiver.rs](../src/receiver.rs)): `GeoFeed::new_diverged` broadcasts
±10 m per-SV clock errors the signal doesn't have (the fix corrupts to
~15 m), and synthetic MT1+MT2 with PRC = −c·ε through the production path
must recover < 3.5 m — sign and magnitude locked against exact truth
(measured 1.81 m when landed; 1.2–2.4 m on current main). This bench also
priced the DLL integrator's noise ([dll-pi-loop.md](dll-pi-loop.md)).

EGNOS coverage spans three captures: CTTC S120+S126, nov3 S136 (54 msgs/60 s
incl. MT1/2/3/4 + MT18/26), ION LimeSDR S120+S123 (171 msgs/60 s). The
pre-fix probes that found "no SBAS" were the broken symbol pairing making
tracked GEOs look dead.

## Ionospheric grid (`sbas_iono.rs`)

- **MT18** IGP masks per band (DO-229 Annex A geometry, bands 0–8) and
  **MT26** vertical delays are assembled into a live grid in `GnssState`.
- MT26s arriving before their band's mask are **buffered and replayed** —
  the real-capture order: MT26 every few seconds, MT18 only every ~300 s.
- The solver prefers the SBAS grid over Klobuchar: pierce point at 350 km →
  4-point bilinear interpolation → obliquity factor.

Exercised end to end on CTTC: 5 MT18 + 21 MT26 → 75–78 IGPs, the solver
applies 2.5–3.7 m of slant delay per GPS SV — the first iono correction this
receiver applied on a real capture (Klobuchar's LNAV page 18 never arrives
in ≤100 s captures). The net fix delta there is small (~0.4 m horizontal): a
calm morning iono is mostly common-mode and the clock bias absorbs it.

Open: bands 9–10 (|lat| > 55°), the 3-point interpolation fallback,
GIVEI-weighted use. The UDRE/GIVE variances do feed the WLS as per-SV priors.
