# Multi-signal stepping — design

Goal: one session runs every signal family the front end can carry — GPS/QZSS
L1 C/A and SBAS L1 (1 ms code periods) together with Galileo E1 (4 ms) — so a
single pass over the samples produces a mixed GPS+Galileo+EGNOS fix. This is
the last structural wall before the all-signals-by-default end state; the
solver side (weighted+ISB WLS, SBAS corrections, anchor alignment) is already
in place and signal-agnostic (`(Measurement, Ephemeris)` pairs).

Non-goals here: multi-frequency (everything is the 1575.42 MHz band),
GLONASS (FDMA breaks the single-IF assumption), decimation (separate perf
item; this design must not preclude it).

## Current contract (what changes)

The receiver is single-family: `period_sp`/`code_period_sec` are globals
chosen from `--sig`; `fetch_samples_msec` keeps a 2-period cache and hands
every channel the same `(2 code periods, ts = start of last period)` window;
`process_step` = fetch → `par_iter` channels → fix → OSNMA. SBAS/QZSS are
dropped from E1 sessions because their C/A channels cannot run on a 4 ms
step (the tuni2025 UI crash).

## Decisions

**D1 — base step = 1 ms.** The GCD of all enabled code periods (C/A, SBAS,
QZSS: 1 ms; E1: 4 ms). The receiver advances the stream one block per step.

**D2 — receiver-owned ring buffer.** The scheduler keeps the last
`2 × max(enabled period)` of samples (8 ms when E1 is on) plus the running
block index and tail time. Channels hold **no** sample buffers; they read
slices of the ring. Rayon stays as-is: immutable ring view, mutable channels.

**D3 — fixed per-family grids.** Family *f* with period *p_f* blocks
processes on every block where `(block_idx + 1) % p_f == 0`, reading the last
`2·p_f` blocks. No per-channel alignment: the correlation window's position
relative to each SV's code phase is *already arbitrary* today and absorbed by
`code_off` (circular correlation) — a fixed grid loses nothing and keeps the
wrap bookkeeping (`wrap_drop`/`wrap_repeat`, ±1 of the channel's own period)
untouched.

**D4 — per-channel time semantics preserved.** A family-step hands its
channels `ts = ring_tail_time − p_f` (start of the last full period), exactly
today's convention per family. `trk_phase` remains `num_trk_samples ×
code_sec` per channel; the anchor, transmit-time and solve math are
unchanged.

**D5 — `scheduler.rs` owns the stepping.** New module between receiver and
channels: the ring, the block counter, the family grids, and (M5) the
acquisition FFT cache. `Receiver::process_step` becomes: scheduler ingests
1 ms → for each family due this block, step its channels (one `par_iter`
across all due channels) → fix → OSNMA.

**D6 — acquisition orchestration moves into the scheduler.** Per family
step, if ≥1 channel of that family is searching, compute the per-Doppler-bin
carrier-mixed forward FFTs **once** and share them; channels do only
multiply(conj code FFT) + IFFT + peak integration. This is the deferred
"shared per-bin FFT" perf item (~2× on what remains of acquisition) landing
in its natural home. Carrier tables stay keyed by `(fs, fi, code_sp)` — one
shared set per family.

**D7 — per-family receiver state.** `period_sp`, carriers, and the OSNMA
gate (`E1 channels exist` instead of `cfg.sig == E1B`) become per-family;
`get_sat_list` builds the union of enabled families instead of dropping
SBAS/QZSS on E1 sessions.

**D8 — bandwidth gating decides the enabled set.** From `fs` alone:
C/A-family needs ≥ 2.046 Msps; E1 BOC(1,1) needs ≥ ~4.092 Msps (±2.046 MHz
main lobes). Defaults after M6: every family the bandwidth admits is on;
`--sats` and `--no-sbas`-style flags become opt-outs. `--sig` remains only as
a front-end description override.

**D9 — solve cadence unchanged.** `compute_fix` keeps its 2 s gate; the
snapshot pairs are already cadence-independent per channel, so mixed-cadence
channels need no solve-side changes (this is what the Measurement split
bought).

## Migration plan (each commit gated on the pinned baseline)

- **M1** `scheduler.rs` skeleton: ring + 1 ms blocks, single family driven
  through it; all single-signal baselines digit-exact (CTTC first fix
  41.274818, 1.987583, h=65.0; validate sweep all-PASS; perf bench within
  noise of 29.7 s / 1.12 GB).
- **M2** per-family state (`period_sp`, carriers, OSNMA gate); C/A and E1
  sessions both through the scheduler, still one family per session.
- **M3** mixed families in one session: `get_sat_list` union, UI signal
  column/gating rework, E1 channels on the 4 ms grid beside C/A channels on
  the 1 ms grid. First mixed acceptance: tuni2025 GPS+Galileo dual fix.
- **M4** SBAS/QZSS inside E1-bearing sessions (un-drop them); EGNOS
  corrections feeding a mixed solve. Acceptance: tuni2025 GPS+Galileo+EGNOS,
  ISB printed, σ per constellation.
- **M5** shared acquisition FFT cache in the scheduler. Gates: `acq_corrs`
  drops ~2× at equal yield; baselines hold; bench improves.
- **M6** defaults flip (D8). `validate_fix.py` gains a mixed-fix check.

## Risks and their tripwires

- *Time semantics drift* → the anchor instrument
  (`tx_anchor_latency_measured_against_synthetic_truth`) and the digit-exact
  CTTC gate catch any `ts`/`trk_phase` convention slip immediately.
- *Wrap handling under the shared grid* → the LNAV/SBAS wrap unit tests plus
  the E1 hermetic fix (4 ms family) cover both period lengths.
- *Rayon aliasing on the ring* → slices are immutable; the only mutable state
  per step is the channels themselves (unchanged) and the FFT cache, which is
  built before the channel fan-out.
- *Throughput regression from double-stepping E1* → E1 channels run on every
  4th block only; the bench (tuni 15 s) is the gate.

## Out of scope, unlocked after

Decimation (per-family resampling slots naturally between the ring and the
channels), E1C pilot tracking (a second family sharing the 4 ms grid),
multi-epoch position filtering.
