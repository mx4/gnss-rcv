# DLL code-loop group delay and the transmit-time compensation

> **Attribution superseded** (8b53dc5, 2026-06-12): the compensation below
> survives unchanged and is still load-bearing, but it is **not** loop group
> delay — with the DLL's rate-trim integrator active the loop holds no
> Doppler-proportional lag (trim ≈ 0 across gpssim's ±3 kHz), yet the term
> is still required (σ 1.6 → 48 m without it), and at 25 Msps (SJTU) it
> *injects* bias. The slope behaves like a ~0.157 s epoch latency with a
> sample-rate dependence; mechanism hunt open. See
> [dll-pi-loop.md](dll-pi-loop.md) § "What the trim revealed about
> `dll_lag`". The experiments and calibration below remain the record of
> how the constant was measured.

## Summary

The dominant per-satellite pseudorange error in the receiver was the **group delay
of the code-tracking (DLL) loop**. A first-order DLL tracking a Doppler-induced
code-rate ramp settles with a steady lag, so the tracked code phase trails the true
one by an amount proportional to the line-of-sight Doppler. That lag biases each
SV's transmit time, hence its pseudorange, per-SV and ∝ Doppler.

We compensate it deterministically at the point the transmit-time code phase is
captured:

```
code_off += (doppler / fc) · τ        # τ = code-loop group delay
```

with `τ = 0.25 / (B_DLL · DLL_DISC_GAIN)` derived from the loop bandwidth plus one
signal-specific discriminator gain. This took the **GPS fix from ~165 m → ~4 m**
and the **Galileo E1 fix from ~3.4 km → ~110 m**.

Code: `src/channel.rs` — `B_DLL`, `DLL_DISC_GAIN_{BPSK,BOC}`, `dll_tau()`, the
`code_off_sec` capture (the compensation), and `run_dll` (the loop). This document
records the experiments and the reasoning behind the constants.

## The diagnostic: the truth residual

Given a known antenna position via `GNSS_TRUTH_ECEF`, the solver logs a per-SV

```
RESID = pr_m + clk_m − geom
```

where `pr_m` is the pseudorange, `clk_m` the SV clock correction, and `geom` the
true geometric range (SV-at-transmit to the known antenna, Sagnac-corrected). If
the transmit-time / pseudorange model is correct, **every SV's residual equals the
same common-mode value** `c · dT_rx` (the receiver clock bias). Any *spread* across
SVs is the part the position solve must absorb as geometry error. So:

- flat residual across SVs → good;
- a *structured* spread → a per-SV model error, and its structure names the cause.

Run it (GPS, gpssim fixture, truth = Geneva):

```sh
GNSS_TRUTH_ECEF="4396463.3,474169.7,4581510.0" RUST_LOG=warn \
  ./target/release/gnss-rcv -f resources/gpssim_2xi16 -t 2xi16 \
  --sats 1,2,3,4,6,9,17,19,28,31 -x 2>&1 | grep -E "RESID|position fix"
```

## GPS: the residual is linear in Doppler

Per-SV residuals at one fix epoch, before any compensation:

| SV  | Doppler (Hz) | resid (km) |
|-----|-------------:|-----------:|
| G09 |        +3060 |    565.800 |
| G04 |        +1285 |    565.849 |
| G19 |         +306 |    565.878 |
| G03 |        −1321 |    565.930 |
| G01 |        −2768 |    565.973 |

Spread ≈ 173 m (→ ~165 m fix error). It fits a straight line in Doppler at
**−0.0297 m/Hz** to within ~2 m (e.g. G04/G03 predicted 565.850/565.928 vs actual
565.849/565.930). It does **not** correlate with code phase or C/N0 — only Doppler.

## Decisive test: instantaneous, not accumulating

A Doppler-proportional error could be either:

1. a carrier-aiding **rate** error accumulating since the nav-message anchor — would
   grow with elapsed time (∝ Doppler × time); or
2. an instantaneous loop **lag** (∝ Doppler), constant in time.

Run *without* `-x` and watch the spread across successive fix epochs (~2 s apart):

| fix epoch | spread |
|-----------|-------:|
| 1         | 0.173 km |
| 2         | 0.175 km |
| 3         | 0.175 km |

The spread is **constant**; only the common-mode drifts (the receiver clock). So it
is an instantaneous loop lag — case (2) — which rules out a carrier-aiding rate bug
and points squarely at the DLL group delay.

## The mechanism: a first-order DLL's steady-state lag

`run_dll` corrects the code phase by

```
code_off −= (B_DLL / 0.25) · err_code · code_sec · n        every  T_DLL = n·code_sec
```

so the correction *rate* is `(B_DLL/0.25) · err_code`. Writing the discriminator
output as `err_code = G_disc · e` (`G_disc` = effective normalized early-late slope,
`e` = true code error in seconds), the loop is first-order:

```
d(code_off)/dt = −K · (true − code_off),     K = (B_DLL/0.25) · G_disc
```

A first-order loop cannot track a ramp with zero error. The code-Doppler ramp has
rate `R = doppler / fc` (carrier Doppler ÷ carrier frequency, the fractional code
rate), so the steady-state lag is

```
e_ss = R / K = (doppler/fc) · τ,     τ = 1/K = 0.25 / (B_DLL · G_disc)
```

The captured code phase lags the true one by `e_ss`; we add it back. With
`B_DLL = 0.5 Hz` and the measured `τ ≈ 0.157 s`, `K = 1/τ ≈ 6.37 = 2·G_disc`, so the
effective `G_disc ≈ 3.18` (`DLL_DISC_GAIN_BPSK`).

## GPS result

After compensation, on gpssim:

- residual spread **173 m → 4 m**;
- fix **46.207524, 6.155659** (~4 m from truth 46.2075, 6.1557).

And on a real GPS capture (CTTC, Castelldefels, Spain):

- fix **41.274836, 1.987405** — ~20 m from the documented antenna (~41.27498,
  1.98754).

τ is a property of the loop (and the correlator), not of the recording, so it
generalizes from the simulated fixture to real signals.

## Galileo E1: the same effect, ~12× larger

E1 reuses the same DLL, but tracks a **BOC(1,1)** waveform whose autocorrelation
main peak is ~½ chip wide, versus ~1 chip for the BPSK L1CA triangle. The early/late
correlator spacing (`SP_CORR`, tuned for the wide BPSK peak) straddles the narrow
BOC peak too widely, so the *effective* discriminator slope `G_disc` is much
shallower → `τ = 0.25/(B_DLL·G_disc)` is much larger.

Truth for the ION LimeSDR site (52.177°N, 4.488°E, h≈40 m):

```python
import math
a=6378137.0; f=1/298.257223563; e2=2*f-f*f
lat=math.radians(52.177); lon=math.radians(4.488); h=40.0
N=a/math.sqrt(1-e2*math.sin(lat)**2)
print(f"{(N+h)*math.cos(lat)*math.cos(lon):.1f},"
      f"{(N+h)*math.cos(lat)*math.sin(lon):.1f},"
      f"{(N*(1-e2)+h)*math.sin(lat):.1f}")   # 3907428.7,306697.9,5014936.2
```

With the **L1CA** τ (0.157 s) applied to E1, the residual is still Doppler-correlated
(~−0.34 m/Hz, but with ~±200 m scatter on only 5 SVs), implying an *effective*
τ ≈ 1.95 s — about 12× the L1CA value:

| SV  | Doppler (Hz) | resid (km) |
|-----|-------------:|-----------:|
| E11 |        −3078 |  −2224.040 |
| E19 |        −2484 |  −2224.502 |
| E04 |          −95 |  −2225.223 |
| E01 |        +1820 |  −2225.952 |
| E09 |        +2080 |  −2225.881 |

### Calibration sweep

Because the per-SV slope is noisy, we calibrate against the **fix error vs truth**
(which integrates over all SVs) rather than the slope. Sweeping E1's τ:

| τ_E1 (s)     | fix                 | error  |
|--------------|---------------------|-------:|
| 0.157 (L1CA) | 52.150684, 4.462991 | ~3.4 km |
| 1.0          | 52.162875, 4.475438 | ~1.8 km |
| 1.8          | 52.174436, 4.487252 | ~290 m |
| **1.95**     | **52.176649, 4.489514** | **~110 m** |
| 2.2          | 52.180215, 4.493164 | ~500 m |

A clean bracketed minimum at **τ_E1 ≈ 1.95 s**, i.e.
`G_BOC = 0.25/(B_DLL·τ_E1) = 0.25/(0.5·1.95) ≈ 0.256` (`DLL_DISC_GAIN_BOC`). The
remaining ~110 m is at the precision of the truth coordinates we have for that site
(3 decimals, ~±100 m; height assumed).

## The constants

| signal           | modulation | main peak | `DLL_DISC_GAIN` | τ = 0.25/(B_DLL·G) |
|------------------|------------|-----------|----------------:|-------------------:|
| L1CA / SBAS      | BPSK       | ~1 chip   | 3.18            | ~0.157 s |
| Galileo E1-B/C   | BOC(1,1)   | ~½ chip   | 0.256           | ~1.95 s  |

`B_DLL = 0.5 Hz`. The gain is picked per channel via `Signal::is_boc11()`. Deriving
τ from `B_DLL` (rather than hardcoding it) means a loop-bandwidth retune carries
through automatically; only the discriminator gain is calibrated.

## Reproducing / recalibrating

1. Build release, then run the truth-residual diagnostic for the signal (commands
   above; for E1 use `--sig E1B` and the LimeSDR truth ECEF).
2. To recalibrate a signal's gain, vary `DLL_DISC_GAIN_{BPSK,BOC}` and **minimize the
   fix error vs truth** (or null the RESID Doppler slope). The fix error is the more
   robust objective — the per-SV slope is noisy with few SVs.
3. `scripts/validate_fix.py` asserts both the GPS and Galileo fixes against their
   truth coordinates, so it guards against a regression in these constants.

## Caveats and future work

- The constants are loop+correlator properties, calibrated on one recording per
  signal (gpssim for L1CA, ION LimeSDR for E1). The Galileo figure is at the
  truth-coordinate precision (~±100 m), so it can't be tightened further on that
  recording alone.
- `DLL_DISC_GAIN_BOC` is a *workaround* for tracking BOC with a BPSK-tuned
  correlator. A proper **BOC correlator** (narrow / double-delta spacing) would
  sharpen E1 code tracking directly and remove the need for the large BOC τ — and
  is also what a future BeiDou B1C (MBOC) would want.

## References (code)

- `src/channel.rs` — `B_DLL`, `DLL_DISC_GAIN_{BPSK,BOC}`, `dll_tau()`, `run_dll`,
  and the `code_off_sec` capture (the compensation).
- `src/solver.rs` — the `RESID` diagnostic (printed when `GNSS_TRUTH_ECEF` is set).
- `scripts/validate_fix.py` — the GPS / Galileo fix-vs-truth regression checks.

## Resolution (2026-06-12): the BPSK τ was the LNAV anchor's orbit-epoch error

The "DLL group delay" compensation is now **removed**; the mechanism is
closed. Three facts pinned it:

1. **The loop holds no lag.** With carrier aiding plus the rate-trim
   integrator (the DLL is a PI loop since 8b53dc5), both the discriminator
   and the trim sit at zero across gpssim's ±3 kHz Doppler spread (trim
   within ±15 ns/s) — yet the raw residual slope with the term off was still
   −0.0293 m/Hz.
2. **The raw slope is sample-rate independent.** Regenerating the gpssim
   scenario at 2.046 / 2.6 / 3.069 / 4.092 / 6.138 / 8.184 / 12.276 Msps
   (integer and non-integer samples-per-chip) gives −0.0292 ± 0.0002 m/Hz at
   every rate. That rules out every sampling-grid story (replica ZOH
   quantization, whole-sample correlator shifts) and is the signature of a
   constant *epoch* error.
3. **0.157 s ≈ 0.160 s = `LNAV_DECODE_LATENCY_SEC`.** The LNAV anchor paired
   the broadcast TOW with the decode-moment phase *without* adding the 8
   preamble bits (0.160 s) between them, leaving every GPS t_tx 0.160 s
   early "by convention" (I/NAV was aligned to the same convention). The
   time-of-day part of that offset does fold into the receiver clock bias —
   but t_tx is also the epoch at which the solver evaluates the **SV
   orbit**, and an orbit evaluated Δt early is displaced v_sv·Δt
   along-track, whose line-of-sight projection is range-rate·Δt =
   **λ·doppler·0.160 ≈ 0.030 m/Hz** — exactly the bias τ = 0.157 s was
   nulling. Same algebra as the I/NAV 2.000 s story below. (The earlier
   "correcting the latency degrades the fix 2.45 m → 184 m" experiment had
   the dll_lag term still active — a double correction — which is also why
   the epoch-sensitivity probe's minimum at the old convention was
   misread as proof the offset was harmless.)

The fix: both decoders anchor at their full structural latency (LNAV
0.160 s, I/NAV 2.000 s), making t_tx absolutely correct, and the
measurement-path `code_off += doppler/fc·τ` term is gone. On gpssim the raw
slope is +0.001 m/Hz with no correction at all (σ 4 m, fix ~2 m, identical
at 12.276 Msps); the tx-anchor truth instrument measures 0.000 s anchor
delta; the CTTC first fix moves ~2 m (re-pinned 41.274836, 1.987583, σ
1.0 m — slightly closer to the documented antenna). `DLL_DISC_GAIN_*`
remain solely as loop-tuning constants (pull-in / rate-trim gears), and
`GNSS_DLL_LAG` is retired. Everything above this section is the historical
calibration trail of a correction that no longer exists.

## Correction (2026-06-11): the E1/BOC calibration was a mis-attribution

The τ_E1 ≈ 1.95 s / `G_BOC = 0.256` calibration above is **wrong about the
mechanism**, and with hindsight the numbers say so: the bias it nulled
(λ_L1 · 2.0 s = 0.381 m/Hz of Doppler) matches the line-of-sight error of a
**2.000-second orbit-epoch offset** — which is exactly the I/NAV word-5
transmit-time anchor latency that was diagnosed and fixed later (see
`nav_anchor_tx`: the broadcast TOW names the start of the 2 s page, but the
decoder emits the word at the page's end). τ = 1.95 s mimics it at
0.371 m/Hz — within 2.5%. The DLL was never the source.

Once the anchor was corrected, keeping `G_BOC = 0.256` double-compensated:
on the ION LimeSDR capture it alone accounted for a self-calibrated Galileo
measurement noise of ~720 m (vs 14 m for GPS), a 502 m Galileo-only fix and
a 145 m apparent inter-system bias. Restoring the BPSK gain (3.18, τ ≈
0.157 s) for BOC gives, on the same capture: Galileo residual spread
1554 → 8 m, Galileo-only 502 → 148 m, GPS+Galileo combined = GPS-only =
140 m, inter-system bias +9.4 ns.

The shape story was also tested directly and refuted: a static-correlation
probe (`dll_discriminator_shape_probe`, channel.rs) shows a front-end
low-pass barely changes the E−L slope, and a noisy synthetic BOC scene at
42 dB-Hz still prefers 3.18 (9 m fix) over 0.256 (1570 m). The loop's
effective discriminator gain is, within measurement error, the same for
BPSK and BOC(1,1) at our 0.5-code-unit spacing. `GNSS_DLL_GAIN_BOC`
remains as an experimental override only.
