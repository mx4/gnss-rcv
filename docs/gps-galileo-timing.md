# The mixed GPS+Galileo fix — anchors, weighting, ISB

> **Status: LIVE.** A flag-less run on a wideband capture solves a mixed
> GPS+Galileo(+EGNOS) pool through one WLS with an inter-system-bias state;
> `validate_fix.py`'s "Mixed fix" gate pins it on tuni2025. This documents
> the timing and weighting problems that had to fall first — each one
> isolated by a dedicated experiment, most of them still alive as regression
> tests. The receiver-side architecture (one pass over the samples, per-
> family stepping grids) is [multi-signal-stepping.md](multi-signal-stepping.md).

## The two-pass merge experiment

Before the receiver could run both families in one pass, the solver-side
question — *can one solve digest a mixed GPST+GST pool at all?* — was
de-risked by `combined_gps_galileo_fix_from_two_passes`
([receiver.rs](../src/receiver.rs), ION LimeSDR): run the capture once as
L1CA, once as E1B, merge the `(Measurement, Ephemeris)` pools at a common
receive instant, solve. It exposed every problem below.

## Timing blocker 1 — the GPS week rollover

LNAV broadcasts a 10-bit week; `+2048` pins it to 2019–2038, putting
pre-2019 captures **1024 weeks off the Galileo timeline** (GST has a 12-bit
week). The 2017 LimeSDR merge lands ~20 years apart between constellations.
The production fix is a live rebase in the solver: in mixed pools each GPS
SV's epochs (toe/toc/tow, tx time) snap onto the GST timeline by the nearest
1024-week multiple. A date-anchored resolver for GPS-only pre-2019 captures
(cosmetic: the UI clock shows the un-rebased week) remains open.

## Timing blocker 2 — the 1.840 s anchor-latency difference

The LNAV and I/NAV transmit-time anchors disagreed by exactly **1.8400 s**.
Both decoders pair the broadcast TOW with the *decode-completion* phase, and
each completes late by its structural latency: LNAV 0.16 s (the 8
next-preamble bits that confirm a subframe), I/NAV 2.0 s (the page carrying
word 5). The difference was measured *directly* against synthetic ground
truth — `tx_anchor_latency_measured_against_synthetic_truth`, per SV,
solver-free — and aligned in `nav_anchor_tx`:

- **LNAV stays the reference convention, deliberately uncorrected.** The
  whole validated pipeline is self-consistent at it; making t_tx absolutely
  true was *measured to degrade* the synthetic fix 2.45 m → 184 m (an open
  solver frame-semantics question, see below). The anchor instrument labels
  the convention "true − 0.16 s" — bookkeeping confined to the instrument.
- **I/NAV adds the 1.840 s difference**, bringing it onto the LNAV
  convention. The merge then measures a **0.0000 s native
  inter-constellation offset**.

## Epoch sensitivity — the solver exonerated

Mixed solves looked alarmingly sensitive to the anchor epoch (~700 m per
second of common epoch offset). `epoch_sensitivity_probe` settled it: a
frame-free hand-rolled WLS matches the production solver to the decimetre at
every common anchor-epoch shift, so there is no coordinate/frame integration
issue — the sensitivity is plain orbital physics (per-SV range-rate spread
times the common epoch offset), and the whole system — generator, decoders,
both solvers, real gps-sdr-sim data — is self-consistent exactly at the
anchor convention (2.6 m at the minimum).

## Weighting and the inter-system bias

With timing aligned, the merged LimeSDR solve still degraded to ~300 m (vs
~140 m GPS-only): the upstream solver weighted every pseudorange equally and
carried a single clock state, while the 5 Galileo measurements were ~50×
noisier *on that capture* (self-calibrated σ: GPS 14 m, GAL 721 m) and their
~−125 m common bias leaked into position. The fix (`wls_fix` tests, then
`wls_solve` as the production solver):

- **weighted Gauss-Newton** with per-constellation σ self-calibrated from
  residual RMS (plus SBAS UDRE/GIVE variances as per-SV priors,
  [sbas.md](sbas.md));
- an **inter-system-bias (ISB) state** — a second clock unknown for Galileo.

It recovers 145 m from the full mixed pool and cross-checks the upstream
solver exactly on GPS-only (140 m = 140 m). `GNSS_SOLVER=rtk` selects the
upstream solver, which remains the fallback.

**The Galileo noise itself was then solved**: σ_GAL = 721 m was the old BOC
DLL gain (0.256) double-compensating the already-fixed anchor latency
([dll-group-delay.md](dll-group-delay.md)). With the corrected gain the
LimeSDR pool measures σ GPS 6 m / GAL 3 m, GAL-only 148 m, combined =
GPS-only = 140 m.

## What the residual ISB decomposes into

With timing, weighting and loop gain right, the remaining ISB is physics.
I/NAV **word 10 (GGTO)** is decoded (`decode_ephemeris_word` arm 10,
`ggto_at`): on the 2017 LimeSDR SIS all 5 SVs broadcast A0G +2.71 ns /
A1G −9.8e-15, evaluating to **+2.92 ns** at the capture epoch. The measured
−10.4 ns ISB therefore decomposes into GGTO (+2.9 ns) and a **−7.5 ns
receiver inter-signal hardware delay** (BPSK vs BOC path) — actual
GGTO/hardware class, printed by the merge test. On the live stepping
pipeline the tuni2025 mixed ISB lands at +2.2 ns and ion-lime at −10.5 ns
(matching the historical −10.4 ns).

One stepping-specific trap belongs here for completeness: families snapshot
measurements on different grids (C/A every 1 ms, E1 every 4 ms), so a mixed
pool mixes snapshot ages. Without de-staling each pseudorange to the
freshest snapshot, the up-to-3 ms gap lands in the ISB (+3 000 002 ns
observed = 3 ms + the true 2.3 ns) — the solver carries
`Measurement.ts_sec` for exactly this.

## Iono in mixed pools

Galileo's broadcast iono inputs (word-5 ai0/ai1/ai2 + storm flags) are
decoded. The Klobuchar correction applies to *all* SVs when GPS page-18
coefficients are present, so mixed runs are covered on long captures; the
SBAS grid covers the EGNOS footprint ([sbas.md](sbas.md)). Galileo-only runs
still have no model — the NeQuick-G port (ESA reference ~4k lines + CCIR
tables) is the sized remaining item, its broadcast inputs ready. Page 18
arrives once per 12.5-min master frame, so short captures never see
Klobuchar coefficients either — iono matters for long runs and live
operation.

## Open

- The anchor-instrument 0.16 s reference bookkeeping (cosmetic).
- The solver frame-semantics question behind the "absolutely-true t_tx
  degrades the fix" observation.
- NeQuick-G; the GPS-only week-display resolver.

**Best live validation capture: tuni2025** (Tampere 2025, 50 MHz): 8 Galileo
E1B + 16 GPS L1CA SVs, each fixing independently at 61.450, 23.856 —
modern, rollover-free, and the permanent mixed-fix gate.
