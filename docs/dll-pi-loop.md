# The DLL PI loop — rate trim and the pull-in gear

> **Status: LANDED** (8b53dc5, 2026-06-12). This document explains why the
> code-tracking loop became proportional-integral, how each piece is sized,
> and what the integrator experiment revealed about the then-present
> `dll_lag` measurement-path term — the trim experiment here disproved the
> group-delay attribution, and the mechanism hunt it opened has since
> closed: the term compensated the LNAV anchor's 0.160 s orbit-epoch error
> and is now **removed** (resolution in
> [dll-group-delay.md](dll-group-delay.md); see
> [§ What the trim revealed](#what-the-trim-revealed-about-dll_lag)).

Code: `src/channel.rs` — `run_dll` (the loop), `Tracking::code_rate_trim` and
`Tracking::hot_disc_streak` (the new state), `dll_tau()` (the time constant
both new gains derive from), and `monitor_code_carrier_consistency` (the
divergence guard whose baseline timing changed).

## Summary

The DLL was a first-order (proportional-only) loop. A first-order loop can
*hold position* against noise but cannot track a sustained **code-rate
error** without a permanent offset — and its slew saturates entirely once
the required rate approaches its authority. On the SJTU capture the front
end's sample-clock vs LO skew demands ~850 ns/s, ~90% of the loop's ~980
ns/s authority: every MEO channel pegged its discriminator, slid off the
correlation peak, and died every ~23 s. `run_dll` now carries:

1. a **rate-trim integrator** (`code_rate_trim`) that learns the residual
   code rate, taking the steady-state discriminator back to zero;
2. a **pull-in gear** — a half-deadbeat kick, gated on *persistent*
   discriminator saturation — that holds the prompt near the peak while the
   integrator winds up;
3. a later **divergence-guard baseline** (`T_FPULLIN + 4 s`) so the trim's
   windup transient cannot poison the guard's healthy-divergence
   self-calibration.

Result: SJTU went from "no channel survives 23 s" to 60 s unbroken locks,
6 ephemerides and 17/17 successful fixes — the first fix ever on that
capture — with the synthetic GEO bench *improving* (1.2–2.4 m vs a 1.47 m
pre-change baseline) and gpssim/CTTC unchanged at the metre level.

## The failure: SJTU's 23-second sawtooth

Symptoms (all measured with `GNSS_DLL_DEBUG=1`, which logs the
discriminator, `code_off`, the trim and C/N0 every ~100 ms):

- a channel locks at ~50 dB-Hz, C/N0 decays ~1 dB/s, the divergence guard
  fires near 30 dB-Hz, the channel instantly re-acquires at 50 dB-Hz —
  period ~23 s, deterministic;
- during the decay the early-late discriminator is **pegged at ~+0.85**
  from t ≈ 2 s — the loop is fighting at full deflection, not blind;
- GEO channels (SBAS) hold 55 s unbroken: only channels with appreciable
  Doppler die.

The quantity the loop is fighting is the front end's **sample-clock vs LO
skew**. The divergence monitor's healthy baseline measures exactly this
gap (code-implied Doppler minus carrier Doppler): +1.3–1.4 kHz on SJTU,
i.e. ~0.85 ppm, i.e. **~850 ns/s of code-phase motion the carrier aiding
cannot account for**. Carrier aiding moves `code_off` at `doppler/fc` per
second, which covers geometric Doppler; a clock-skew term is invisible to
it, so the DLL alone must supply the missing 850 ns/s.

## Why a first-order DLL cannot do that

Per DLL update (`t_u` = `T_DLL` = 10 ms for L1CA):

```
disc      = (E − L) / (E + L)
err_code  = disc/2 · T_chip            # offset estimate, seconds
Δcode_off = −(B_DLL/0.25) · err_code · t_u
```

so the correction *rate* is `(B_DLL/0.25) · disc/2 · T_chip` per second —
at full deflection (|disc| = 1, T_chip = 977.5 ns, B_DLL = 0.5 Hz) about
**±980 ns/s**. Tracking a rate `r` therefore settles at

```
disc_eq = r / 980 ns/s    →    850/980 ≈ 0.87
```

The measured pegged value was 0.85 — the model closes. At that deflection
the prompt sits ~130 ns off the peak, the discriminator is compressing
(the normalized E−L curve flattens toward saturation), and the leftover
~30 ns/s deficit walks the prompt the rest of the way off. C/N0 decays
until the guard removes the channel. The guard was the **coroner, not the
killer**: re-acquisition resets `code_off`, and the cycle repeats.

A loop with an integrator has no such equilibrium offset: the integrator
accumulates whatever constant rate the input carries, and the
discriminator returns to zero. That is the entire fix; everything below is
sizing and guard rails.

## The design

### D1 — rate-trim integrator

```
trim     −= (B_DLL / 4τ) · err_learn · t_u     # err_learn: see D1b
code_off += trim · t_u                          # applied every update
```

`τ = dll_tau(disc_gain) = 0.25/(B_DLL·G)` is the existing loop time
constant (0.157 s for L1CA), so the integrator gain rides any future loop
retune automatically.

**Why so slow.** With ζ = 1/√2 against the proportional gain (the textbook
choice, `ki = 2·B_DLL/τ`) the trim converges in ~0.3 s — and its random
walk under discriminator noise feeds *straight into the pseudoranges*.
The hermetic synthetic GEO bench (6 SVs, 45 dB-Hz, ±10 m injected clock
errors, SBAS corrections must recover < 3.5 m) priced the noise: ~9 m at
the textbook gain (entangled with the pre-persistence kick glitches of
D2), 3.43 m at `B_DLL/2τ`, **2.42 m at `B_DLL/4τ`** — against a 1.47 m
pre-change baseline. At `B_DLL/4τ`, SJTU's 850 ns/s still winds up in
~13 s (measured: −185 ns/s at 2.6 s, −855 ns/s by 15 s, discriminator
< 0.1 from ~10 s), bridged by the pull-in gear meanwhile. Trim noise on
quiet captures is ±5–10 ns/s — sub-metre.

**D1b — clamped learning, not a learning gate.** The integrator's input is
the discriminator clamped to its linear range:

```
err_learn = clamp(disc, −0.3, +0.3) / 2 · T_chip
```

A saturated discriminator measures *direction*, not rate, so learning from
its raw value overshoots; but a hard learning **gate** (`|disc| ≤ 0.3`,
the first attempt) deadlocks: on SJTU the kick/linear equilibrium parks
exactly at the pull-in threshold, the gate never opens, and the trim stays
unlearned forever (observed: disc pinned at ~0.5 for a full 50 s run,
trim = 0). The clamp keeps the windup direction under saturation and
bounds what any single outlier can teach.

**D1c — no learning during the FLL second.** Before `T_FPULLIN` (1 s) the
carrier loop is an FLL still converging; its transients masquerade as
code-rate error and the integrator would faithfully memorize them.

**D1d — the trim survives re-locks.** The rate it learns is dominated by
the front end's clock skew — a *receiver* constant, not a per-pass
quantity. Re-winding from zero leaves every re-lock a ~10 s biased tail;
on SJTU (where weak channels churn) that cost ~30 m of fix error. The trim
is zeroed only on a **code-carrier divergence drop**, the one path where
the loop's state is known-bad (the walk-off may have been steered by a bad
learned rate).

### D2 — pull-in gear

While the trim winds up (or after any real transient), the linear gain is
too weak to recover a large offset quickly. When the loop is genuinely
behind, kick out half the estimated offset per update:

```
deadbeat gain = τ / t_u          # removes the full estimated offset
pull-in gain  = τ / (2·t_u)      # half: damped, no overshoot
```

engaged only after **3 consecutive updates with |disc| > 0.5**
(`hot_disc_streak`). The persistence gate is load-bearing: the
discriminator throws isolated *one-update* outliers (|disc| up to ~0.9 at
53 dB-Hz on the synth bench, roughly one per second per SV). The linear
gain absorbs such a spike as a ~9 ns blip; a deadbeat kick would turn it
into a ~35 ns pseudorange glitch — and the first implementation (a flat
×100 boost at |disc| > 0.3, ~6× deadbeat) both overshot real offsets
(bang-bang oscillation between ±0.3) and amplified every spike, tripping
the divergence guard at full signal strength. A genuine pull-in episode
saturates the discriminator for thousands of updates; three is plenty to
tell the two apart.

### D3 — divergence-guard baseline after the windup

The guard self-calibrates a "healthy" code-carrier divergence per lock and
drops the channel when the measured value deviates persistently
(`DIV_GATE_HZ`). Its baseline used to be taken at `T_FPULLIN + 1 s` — now
**inside** the trim windup. The transient bends the early transmit-phase
slope, so the baseline calibrated to a value the settled loop then
"deviates" from: on the synth GEO bench a clean 52 dB-Hz channel
(baseline −103 Hz vs true ~0) was dropped 17 s into the run, right before
the first fix. The baseline now waits until `T_FPULLIN + 4 s`, past the
windup at the D1 gain.

## What the trim revealed about `dll_lag`

The measurement path used to add a Doppler-proportional transmit-time
correction (`dll_lag = doppler/fc · τ`, the subject of
[dll-group-delay.md](dll-group-delay.md)), historically attributed to the
DLL's group delay: a first-order loop chasing the Doppler code-rate ramp
should lag by `rate · τ`.

The integrator falsifies that attribution. With the PI loop active, the
trim converges to **~0 (±15 ns/s) across gpssim's ±3 kHz Doppler spread**
— the carrier aiding already covers code Doppler, so the loop chases
nothing and holds no Doppler-proportional lag. Yet removing the `dll_lag`
term still costs gpssim σ 1.6 m → 48 m. Therefore the 0.03 m/Hz slope it
nulls originates **outside the loop**, behaving like a ~0.157 s epoch
latency (λ·Δt ≈ 0.03 m/Hz — the same algebraic trap as the Galileo
"τ = 1.95 s" story, which turned out to be the 2.000 s I/NAV anchor
latency). At the time it also *appeared* sample-rate dependent: present on
2.046 Msps gpssim and 4 Msps CTTC, seemingly absent at SJTU's 25 Msps,
where the term degraded σ 37 → 73 m in a `GNSS_DLL_LAG=off` A/B.

**Hunt closed (2026-06-12).** A gpssim regeneration sweep measured the raw
slope *identical* (−0.0292 ± 0.0002 m/Hz) from 2.046 to 12.276 Msps — not
sample-rate dependent at all, so not a sampling-grid effect. The source is
the **orbit-epoch error of a t_tx anchored 0.160 s early** (the LNAV
decode latency, once left uncorrected "by convention"; 0.157 ≈ 0.160 s).
Both nav anchors now pin at their full structural latency, the `dll_lag`
term and `GNSS_DLL_LAG` are removed, and the SJTU A/B is explained as
confounded: with the root cause fixed, SJTU σ drops to **1–3 m** — better
than either arm of that A/B by an order of magnitude. Resolution details:
[dll-group-delay.md](dll-group-delay.md).

## Validation

| Gate | Before | After |
| --- | --- | --- |
| SJTU (`-t 4bit --fs 25e6 --fi 6.25e6`, flagless) | lock churn every ~23 s, 0 ephemerides, no fix | 60 s unbroken locks, 6 ephemerides, **17/17 fixes** at SJTU Minhang (31.026, 121.440) |
| Synth GEO bench (`sbas_fast_corrections_recover_…`) | 1.47 m | 1.20–2.42 m (must be < 3.5 m) |
| gpssim exact-truth | σ 1.6–1.9 m | fix error 1.6 m, σ 2.5–3.0 m |
| CTTC first fix | 41.274817, 1.987575 | re-pinned 41.274820, 1.987562 (~1 m; every code correction changed, hence the bit path) |
| `validate_fix.py` (GPS / Galileo / SBAS / Mixed) | PASS | PASS |
| Hermetic `--ignored` tier, `just check` | PASS | PASS |

## Loose ends

- ~~SJTU σ is still tens of metres~~ — resolved by the tx-anchor-latency
  fix (σ 1–3 m; see "What the trim revealed" above).
- ~~The `dll_lag` mechanism hunt~~ — closed: the LNAV anchor's 0.160 s
  orbit-epoch error ([dll-group-delay.md](dll-group-delay.md)).
