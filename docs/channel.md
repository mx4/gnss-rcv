# Channel — architecture and processing logic

`src/channel.rs` is the heart of gnss-rcv.  Every satellite under search or
track is represented by one `Channel` instance.  The receiver's main loop calls
`ch.process_samples(iq_vec, ts_sec)` once per code period (1 ms for GPS L1 C/A,
4 ms for Galileo E1-B/C); from that single entry point the channel state machine
runs acquisition, tracking, or idle backoff for that period.

---

## Contents

1. [Data structures](#1-data-structures)
2. [Constants](#2-constants)
3. [State machine](#3-state-machine)
4. [Initialization (`Channel::new`)](#4-initialization)
5. [Acquisition](#5-acquisition)
6. [Tracking — full processing sequence](#6-tracking)
7. [Carrier generation by phase recurrence](#7-carrier-generation)
8. [Transmit-phase anchoring and pseudoranges](#8-transmit-phase-anchoring)
9. [DLL group-delay compensation](#9-dll-group-delay-compensation)
10. [Half-rate false-lock detection (Galileo)](#10-half-rate-false-lock-detection)
11. [History ring buffers](#11-history-ring-buffers)
12. [Publishing to the shared UI state](#12-publishing-to-the-shared-ui-state)
13. [Per-channel statistics (`ChannelStats`)](#13-per-channel-statistics)

---

## 1  Data structures

### `Channel` (the outer container)

| Field | Type | Description |
|---|---|---|
| `sv` | `SV` | Satellite identifier (constellation + PRN) |
| `state` | `State` | Current state: `Acquisition`, `Tracking`, or `Idle` |
| `fc` | `f64` | Carrier frequency (Hz), e.g. 1 575 420 000 for L1 |
| `fs` | `f64` | ADC sample rate (Hz) |
| `fi` | `f64` | Intermediate frequency (Hz); 0 for baseband IQ |
| `code_sec` | `f64` | Code period in seconds (1 ms for L1 C/A, 4 ms for E1) |
| `code_len` | `usize` | Chips per period (1023 L1 C/A, 8184 E1 BOC sub-chips) |
| `code_sp` | `usize` | Samples per code period = `fs × code_sec` |
| `tau_dll` | `f64` | DLL group-delay constant for this signal (see §9) |
| `num_trk_samples` | `usize` | Correlation-buffer count; used for loop-update cadence |
| `num_tx_codes` | `f64` | Continuous transmit-phase code counter (see §8) |
| `num_acq_samples` | `usize` | Periods accumulated in the current acquisition attempt |
| `num_idl_samples` | `usize` | Periods waited while idle |
| `num_acq_fails` | `u32` | Consecutive failed acquisition attempts since last lock |
| `acq_cn0` | `f64` | Peak C/N₀ of the most recent acquisition attempt |
| `trk` | `Tracking` | All per-tracking-period state (see below) |
| `acq` | `Acquisition` | Acquisition scratchpad and pre-computed data |
| `hist` | `History` | Ring-buffer diagnostics (code phase, Doppler, prompt corr) |
| `nav` | `Navigation` | Decoded ephemeris + per-signal decoder state |
| `pub_state` | `Arc<Mutex<GnssState>>` | Shared state for the UI thread |

### `Tracking`

| Field | Description |
|---|---|
| `prn_code` | Upsampled replica (length `code_sp`, complex) |
| `sig_buf` | Reused scratch buffer for the Doppler-mixed code period |
| `doppler_hz` | Current Doppler estimate (Hz) — updated by FLL/PLL |
| `code_off_sec` | Sub-period fractional code-phase offset (seconds) |
| `cn0` | Current C/N₀ estimate (dB-Hz) |
| `adr` | Accumulated Doppler range (cycles) |
| `phi` | Carrier phase accumulator (fractional cycles); fed to correlator |
| `err_phase` | Previous PLL phase error (for the second-order PLL derivative term) |
| `sum_corr_e/l/p/n` | Running sums for DLL (E, L), C/N₀ numerator (P), C/N₀ denominator (N) |
| `txp_ts0`, `txp0` | Baseline for Galileo half-rate false-lock detection (see §10) |

### `Acquisition`

| Field | Description |
|---|---|
| `prn_code_fft` | Forward FFT of the upsampled replica — pre-computed once |
| `sum_p` | `[DOPPLER_SPREAD_BINS][code_sp]` non-coherent accumulator |
| `carriers` | Pre-computed carrier replica for each Doppler bin — avoids `sin/cos` per sample on each search step |

---

## 2  Constants

### Acquisition

| Constant | Value | Meaning |
|---|---|---|
| `DOPPLER_SPREAD_HZ` | 12 000 Hz | ±12 kHz search window; covers ±5 kHz true GPS Doppler plus several kHz of front-end LO offset |
| `DOPPLER_SPREAD_BINS` | 75 | Number of Doppler bins; step ≈ 320 Hz |
| `T_ACQ` | 0.01 s | Non-coherent integration duration (10 ms for L1 C/A) |
| `CN0_THRESHOLD_LOCKED` | 35 dB-Hz | Acquisition C/N₀ required to enter tracking |
| `CN0_THRESHOLD_LOST` | 29 dB-Hz | Tracking C/N₀ below which the channel drops back to idle |
| `ACQ_FAIL_GRACE` | 20 | Consecutive failures before idle-backoff begins |
| `T_IDLE` | 3.0 s | Normal wait between acquisition attempts |
| `T_IDLE_MAX` | 30.0 s | Maximum backoff wait for PRNs likely not in view |

### Tracking loop bandwidths

| Constant | Value | Meaning |
|---|---|---|
| `T_FPULLIN` | 1.0 s | Duration of the initial FLL pull-in phase |
| `T_NPULLIN` | 1.5 s | Time before navigation decoding is enabled |
| `B_FLL_WIDE` | 10 Hz | FLL bandwidth during the first T_FPULLIN / 2 |
| `B_FLL_NARROW` | 2 Hz | FLL bandwidth after pull-in |
| `B_PLL` | 10 Hz | PLL bandwidth (second-order Costas) |
| `B_DLL` | 0.5 Hz | DLL bandwidth |
| `T_DLL` | 0.01 s | Non-coherent DLL integration window (10 periods for L1 C/A) |
| `T_CN0` | 1.0 s | C/N₀ averaging window |
| `SP_CORR` | 0.5 chips | Early/late correlator spacing (in chips) |

### Correlator discriminator gains (DLL group delay)

The DLL loop's finite bandwidth creates a steady-state code-phase lag proportional to the Doppler ramp rate.  The lag is `code_Doppler × τ` where `τ = 0.25 / (B_DLL × disc_gain)`.

| Constant | Value | Signal |
|---|---|---|
| `DLL_DISC_GAIN_BPSK` | 3.18 | L1 C/A, SBAS (triangular correlation peak, steep discriminator, small τ ≈ 0.157 s) |
| `DLL_DISC_GAIN_BOC` | 0.256 | Galileo E1-B/C (narrow BOC peak, shallow discriminator, τ ≈ 1.95 s) |

See `dll_tau()` and §9 for how the lag is compensated at pseudorange output.

### Diagnostics

| Constant | Value | Meaning |
|---|---|---|
| `HISTORY_NUM` | 20 000 | Capacity of each `History` ring buffer (one entry per code period ≈ 20 s at L1 C/A) |

---

## 3  State machine

```
          ┌──────────────────────┐
          │     Acquisition      │◄──────────────────────┐
          │ (FFT search 10 ms)   │                       │
          └──────────┬───────────┘                       │
         C/N₀ ≥ 35?  │  no → idle_start()               │ T_IDLE elapsed
                     │                                   │
                     ▼ yes → tracking_start()            │
          ┌──────────────────────┐          ┌────────────────────┐
          │       Tracking       │          │       Idle         │
          │  FLL→PLL/DLL loops   │          │  exponential back- │
          │  nav decode, C/N₀    │──────────│  off up to 30 s    │
          └──────────────────────┘          └────────────────────┘
          C/N₀ < 29? → idle_start()
```

- **Acquisition**: entered at startup and after every lock loss.  The channel
  accumulates `T_ACQ / code_sec` code periods of non-coherent correlation energy
  across all Doppler bins, then picks the peak cell.  If C/N₀ ≥ 35 dB-Hz it
  transitions to Tracking; otherwise to Idle.
- **Tracking**: the normal operating state.  All loop updates (FLL→PLL/DLL),
  C/N₀ estimation, nav decoding, and pseudorange bookkeeping happen here.
- **Idle**: the channel waits `T_IDLE` (= 3 s) then tries again.  After
  `ACQ_FAIL_GRACE` (= 20) consecutive failures the wait grows linearly
  (`T_IDLE × (fails - 20 + 1)`) up to `T_IDLE_MAX` (= 30 s) to stop burning
  FFT searches on PRNs clearly not in view.

---

## 4  Initialization

`Channel::new(sig, sv, fs, fi, plots, pub_state)` performs the one-time setup:

1. **PRN code generation** — `sig.spreading_code(sv.prn)` returns the bipolar
   (±1) code at chip resolution (1023 chips for L1 C/A, 8184 BOC sub-chips for
   E1).

2. **Arbitrary-rate resampling** — the chip-rate code is upsampled to `code_sp =
   fs × code_sec` samples using nearest-neighbour chip selection:
   ```
   let chip = i * code_len / code_sp;
   prn_code[i] = code_buf[chip]
   ```
   This makes any sampling rate work without a dedicated table per rate.

3. **Pre-computed acquisition FFT** — `prn_code_fft` = forward FFT of the
   upsampled replica.  Used during every acquisition step (§5) to avoid
   re-computing it for each code period.

4. **Pre-computed Doppler bin carriers** — one `Vec<Complex64>` per Doppler bin
   built with `doppler_shifted_carrier` (phase recurrence, see §7).  The bin
   frequency is `fi + doppler_hz` so that non-baseband recordings (`fi ≠ 0`) are
   mixed correctly.  Pre-computing avoids a sin/cos call per sample per bin per
   period, which was the dominant CPU cost during search.

5. **`tau_dll`** — the DLL group-delay constant for this signal, derived from
   `DLL_DISC_GAIN_BPSK` or `DLL_DISC_GAIN_BOC` depending on `sig.is_boc11()`.

6. The channel is inserted into `pub_state.channels` (for the UI) and its initial
   state is set to `Acquisition`.

---

## 5  Acquisition

`acquisition_process(iq_vec)` is called once per code period while the channel is
in Acquisition state.

### Step 1 — per-Doppler-bin FFT correlation

For each of the 75 Doppler bins the function calls
`acquisition_integrate_correlation(iq_vec_slice, bin)`:

```
received slice * pre-computed carrier[bin] → FFT → multiply conjugate(prn_code_fft)
    → IFFT → |.|² → non-coherent sum into sum_p[bin]
```

The carrier at bin `i` has frequency `fi + (−12000 + i × step_hz)`.  The FFT
convolution computes the circular correlation at all code-phase offsets
simultaneously in O(N log N) instead of O(N²).

### Step 2 — non-coherent integration

After each code period the power surface `sum_p[bin][code_phase]` is updated.
Integration runs for `T_ACQ / code_sec` periods (10 for L1 C/A, 2 for E1-B).
Non-coherent accumulation averages out noise while the signal peak grows
proportionally to the number of periods.

### Step 3 — peak detection and C/N₀ estimate

After `T_ACQ`:

1. Find the 2D peak: `(idx, code_offset_idx) = argmax(sum_p[bin][phase])`.
   The peak is found by **highest peak value**, not total bin power — selecting
   by total power biases toward interference-heavy bins (empirically observed with
   multi-SV captures).

2. The winning Doppler is the bin's left edge frequency:
   `doppler_hz = -DOPPLER_SPREAD_HZ + idx × step_hz`.
   Using the bin centre (+½ step) would seed tracking ~160 Hz off the actual
   correlation peak.

3. C/N₀ estimate (signal-to-noise ratio normalized to 1 Hz bandwidth):
   ```
   p_avg = total_power / (code_sp × DOPPLER_SPREAD_BINS)
   cn0 = 10 × log10((p_peak - p_avg) / p_avg / code_sec)   [dB-Hz]
   ```

4. If `cn0 ≥ CN0_THRESHOLD_LOCKED` → `tracking_start()`; otherwise increment
   `num_acq_fails` and `idle_start()`.

---

## 6  Tracking

`tracking_process(iq_vec)` is called once per code period while tracking.  The
sequence is fixed and order-sensitive:

```
get_code_and_carrier_phase()      ← update code/carrier phase for this period
tracking_compute_correlation()    ← 4-correlator (P, E, L, N)
update trk_phase / code_off_sec   ← pseudorange bookkeeping
[FLL or PLL]                      ← carrier loop (FLL for first T_FPULLIN, then PLL)
run_dll()                         ← code loop
update_cn0()                      ← C/N₀ running estimate
nav_decode()                      ← navigation message decode (after T_NPULLIN)
correct_half_rate_false_lock()    ← Galileo-only Costas ambiguity check
log_periodically()                ← TRCK log line every 3 s
[cn0 < threshold → idle_start()]  ← lock-loss check
```

### 6.1  Phase update — `get_code_and_carrier_phase()`

Before correlating, the carrier and code phase pointers are advanced by one code
period:

```rust
adr       += doppler_hz × code_sec           // accumulated Doppler range
code_off  -= (doppler_hz / fc) × code_sec    // carrier-aided code offset
```

The second line is the **carrier-aided code loop**: the code phase is corrected
at the theoretical code-Doppler ratio `(doppler / fc)`, so the code replica stays
aligned with the received signal even without a DLL update every period.

**Code-phase wrap handling** keeps `code_off_sec` in `[0, code_sec)`.  When the
offset wraps:
- `code_off ≥ code_sec` (positive Doppler, SV approaching): the replica is ahead
  by one period → `code_off -= code_sec`, `num_tx_codes += 1`, pop one entry
  from `hist.corr_p` to keep the buffer aligned.
- `code_off < 0` (negative Doppler, SV receding): the replica is behind →
  `code_off += code_sec`, `num_tx_codes -= 1`, duplicate the last corr_p entry.

The carrier phase for this period is:
```
phi = fi × code_sec + adr + (fi + doppler) × code_off / fs
```
`phi` (in fractional cycles) is passed to `doppler_shift` as the initial phase of
the mixing carrier (see §7).

### 6.2  Fused 4-correlator — `tracking_compute_correlation()`

The code period to correlate is extracted from the 2× sliding window using
`code_off_sec` as the sample offset:

```
lo = code_off_sec × fs   (or lo = code_sp + code_off_sec × fs when negative)
sig_buf = iq_vec[lo .. lo + code_sp]
doppler_shift(sig_buf, fi + doppler_hz, phi, fs)
```

Mixing uses `fi + doppler_hz`, not just `doppler_hz` — for non-zero intermediate
frequency the signal sits at `fi + doppler`; mixing by `doppler` alone leaves an
`fi` residual that wipes out the correlation peak.

Then a **single fused loop** over `code_sp` samples computes four correlators
simultaneously, reading `sig[j]` and `code[j]` once each:

| Correlator | Code offset | Purpose |
|---|---|---|
| Prompt (P) | 0 | Carrier and code phase error, C/N₀ numerator |
| Early (E) | +`pos` samples | DLL discriminator numerator |
| Late (L) | −`pos` samples (via `sig[j+pos]`) | DLL discriminator denominator |
| Neutral (N) | +80 samples (fixed) | C/N₀ denominator (noise floor reference) |

`pos = SP_CORR × code_sec × fs / code_len` converts the 0.5-chip spacing into
samples.  The neutral correlator is intentionally placed far enough off-peak to
measure noise without signal energy.

### 6.3  Frequency-locked loop — `run_fll()`

Active for the first `T_FPULLIN` = 1.0 s.  Cross-dot discriminator between
consecutive prompt correlators:

```
dot   = Re(c1)·Re(c2) + Im(c1)·Im(c2)
cross = Re(c1)·Im(c2) − Im(c1)·Re(c2)
err_freq = atan(cross / dot) / (2π)           [cycles/period]
doppler_hz -= (B / 0.25) × err_freq
```

Bandwidth switches from `B_FLL_WIDE` = 10 Hz to `B_FLL_NARROW` = 2 Hz at
`T_FPULLIN / 2` = 0.5 s for a fast-then-stable pull-in.

### 6.4  Phase-locked loop — `run_pll()`

Active after `T_FPULLIN`.  Costas (data-insensitive) atan discriminator:

```
err_phase = atan(Im(c_p) / Re(c_p)) / (2π)   [cycles]
```

Second-order loop filter:
```
ω  = B_PLL / 0.53   ≈ 18.9 rad/s
doppler_hz += 1.4·ω·(err_phase - err_phase_prev) + ω²·err_phase·code_sec
```

The two terms are proportional (current error) and derivative (change in error).

### 6.5  Delay-locked loop — `run_dll()`

Non-coherent early-minus-late envelope discriminator, updated every `n =
max(1, T_DLL / code_sec)` periods (= 10 for L1 C/A, 2 for E1-B):

```rust
sum_corr_e += |c_e|
sum_corr_l += |c_l|
// every n periods:
err_code = (E - L) / (E + L) / 2 × code_sec / code_len    [seconds]
code_off_sec -= (B_DLL / 0.25) × err_code × code_sec × n
```

The denominator `(E + L)` normalises the discriminator so its gain is independent
of signal level.

### 6.6  C/N₀ estimation — `update_cn0()`

Narrowband–wideband estimator.  Each period:
```
sum_corr_p += |c_p|²    ← signal + noise power (prompt)
sum_corr_n += |c_n|²    ← noise power (neutral, off-peak)
```
Every `T_CN0 / code_sec` periods (= 1000 for L1 C/A):
```
cn0_inst = 10 × log10(sum_p / sum_n / code_sec)    [dB-Hz]
cn0 += 0.5 × (cn0_inst - cn0)                      ← exponential smoothing (α = 0.5)
```

Lock is lost when `cn0 < CN0_THRESHOLD_LOST` = 29 dB-Hz.

### 6.7  Navigation decode — `nav_decode()`

Enabled after `T_NPULLIN` = 1.5 s (once the PLL is stable enough to read bits).
Dispatches by constellation:

```
Galileo → nav_decode_inav()   (src/galileo_inav.rs, I/NAV Viterbi + deinterleaver)
SBAS    → nav_decode_sbas()   (src/sbas_l1.rs, 250-bit CRC-24 messages)
GPS/QZSS→ nav_decode_gps_lnav() (src/gps_lnav.rs, 300-bit LNAV word, parity)
```

Each decoder calls `Re(c_p)` as the soft-symbol input (prompt in-phase).  When a
complete subframe/message decodes cleanly the ephemeris fields in `nav.eph` are
updated and `nav.eph.is_valid()` becomes true.

---

## 7  Carrier generation by phase recurrence

All carrier mixing (both in acquisition and tracking) uses **phase recurrence**
rather than computing `sin/cos` per sample:

```rust
// carrier[n] = exp(-j·(2π·f·n/fs + 2π·φ))
let step = Complex64::from_polar(1.0, -2π·f / fs);
let mut c = Complex64::from_polar(1.0, -2π·φ);
for s in signal.iter_mut() {
    *s *= c;
    c *= step;
}
```

Each sample costs one complex multiply (4 real multiplies + 2 adds) instead of a
`sin` + `cos` call (~20–100 ns each).  This is critical in `tracking_compute_correlation`,
which runs once per locked satellite per code period.

The trade-off is phase drift from floating-point accumulation errors.  Over one
code period (`code_sp` ≈ 2046 samples at 2.046 MHz, or ≈ 48 000 at 48 MHz) the
error is negligible.

---

## 8  Transmit-phase anchoring and pseudoranges

The pseudorange is the range expressed as a signal travel time: `ρ = (t_rx -
t_tx) × c`.  `t_tx` must be an absolute GPS/Galileo time, recovered from the
ephemeris TOW plus a fractional part derived from the code phase.

### Two counters for two different jobs

`num_trk_samples` is the correlator-buffer counter.  It reflects *received* code
periods: it gains an extra step on a `code_off ≥ code_sec` wrap (Doppler > 0, SV
approaching) and loses one on a `code_off < 0` wrap (SV receding).  Its purpose
is keeping `hist.corr_p` aligned; it is reused for loop-update cadence checks.

`num_tx_codes` is the *transmit-phase* counter.  It too increments by +1 per
processing period and by ±1 at code-phase wraps — but with the *same* sign as
`num_trk_samples`, so it advances at the *received* code rate.  The transmit phase
is then:

```
t_tx_frac = num_trk_samples × code_sec − code_off_sec
```

This gives `d(t_tx)/d(t_rx) = 1 + doppler/fc`, the relativistically correct sign
(approaching SV → positive Doppler → faster ticking transmit clock).  The earlier
formulation `num_tx_codes × code_sec + code_off_sec` had the opposite sign,
yielding `1 − doppler/fc` and pseudoranges that moved in the wrong direction.

### Snapshot for the solver

At each tracking period, before calling the nav decoder:

```rust
nav.eph.trk_phase   = num_trk_samples as f64 × code_sec
nav.eph.code_off_sec = code_off_sec + dll_lag          // DLL lag compensated (§9)
```

The solver reads `trk_phase − code_off_sec` as the fractional transmit time and
adds the integer TOW decoded from the navigation message.

---

## 9  DLL group-delay compensation

The DLL's finite loop bandwidth means it cannot track an instantaneous code-phase
step — it has a **group delay** `τ = 0.25 / (B_DLL × disc_gain)`.  Under a
constant Doppler ramp (satellite moving at constant velocity), the tracked code
phase lags the true code phase by `code_Doppler × τ`, where `code_Doppler =
doppler_hz / fc`.

For GPS L1 C/A (BPSK, `disc_gain = 3.18`):
```
τ ≈ 0.157 s   →   lag ≈ 0.157 × (doppler / 1.575×10⁹)   [s]
  ≈ 0.03 m/Hz for a typical 3 km/s pass
```

For Galileo E1-B (BOC(1,1), `disc_gain = 0.256`):
```
τ ≈ 1.95 s    →   lag ≈ 0.19 m/Hz
  ≈ 10× larger than L1 C/A
```

The compensation applied at every tracking period:

```rust
let dll_lag = trk.doppler_hz / fc × tau_dll;
nav.eph.code_off_sec = trk.code_off_sec + dll_lag;
```

This adds back the lag so the solver sees the *true* code phase.  Without it, the
Galileo pseudoranges have a per-SV bias of hundreds of metres correlated with
Doppler, breaking the position fix.

---

## 10  Half-rate false-lock detection

**Galileo E1-B only.**  The I/NAV data symbols are 4 ms long (one code period
each), so the Costas PLL discriminator `atan(Q/I)` is data-ambiguous at the
per-symbol level: the loop can settle a multiple of `1/(2 × code_sec)` = ±125 Hz
away from the true carrier frequency while still producing a strong, stable
correlation peak.  The symptom is that every symbol in the I/NAV stream picks up a
`(−1)ⁿ` sign flip — the decoder handles this by itself, but the position fix
suffers if the carrier Doppler is wrong.

The DLL is **not** susceptible to this because it uses envelope (non-coherent)
detection.  So `num_trk_samples × code_sec − code_off_sec` gives the *true*
transmit phase, and its slope is the true Doppler:

```
code_dopp = ((txp − txp0) / dt − 1) × fc
```

`correct_half_rate_false_lock()` runs after each tracking period (Galileo only):

1. Waits until after `T_FPULLIN + 1 s` for carrier pull-in to complete, then
   records the baseline `(txp0, txp_ts0)`.
2. After `HALF_RATE_WINDOW` = 3 s, computes the code-implied Doppler and
   compares it with the PLL's `doppler_hz`.
3. If the difference is a non-zero multiple `k` of `step = 1/(2 × code_sec)` =
   125 Hz, corrects the Doppler:
   ```rust
   doppler_hz += k × step
   err_phase = 0.0   // prevent a derivative spike in the next PLL update
   ```
4. Resets the baseline for the next 3-second window.

---

## 11  History ring buffers

`History` stores the last `HISTORY_NUM` = 20 000 samples (≈ 20 s at L1 C/A rate)
of four time series:

| Field | Type | Content |
|---|---|---|
| `code_phase_offset` | `VecDeque<f64>` | `code_off_sec × fs` (samples) — used by `tracking_compute_correlation` to pick the correlation window |
| `phi_error` | `VecDeque<f64>` | PLL phase error in radians — used by diagnostic plots |
| `doppler_hz` | `VecDeque<f64>` | Doppler estimate history — used by plots |
| `corr_p` | `VecDeque<Complex64>` | Prompt correlator — used by FLL (last two values) and plots |

`VecDeque` makes `pop_front` O(1); a `Vec` would need O(N) memmove on every code
period.  `History::trim()` enforces the `HISTORY_NUM` cap after each push.

---

## 12  Publishing to the shared UI state

The UI thread reads `Arc<Mutex<GnssState>>` every ~50 ms.  The `publish()` helper
consolidates the lock, mutation, and repaint callback in one place:

```rust
fn publish<F: FnOnce(&mut ChannelState)>(&self, update: F) {
    let tracking = {
        let mut st = self.pub_state.lock().unwrap();
        let cs = st.channels.get_mut(&self.sv).unwrap();
        update(cs);
        cs.state == State::Tracking
    };
    if tracking { (self.pub_state.lock().unwrap().update_func.func)(); }
}
```

The UI callback (`ctx.request_repaint_after_secs(0.05)`) is only fired when the
channel is tracking, avoiding spurious repaints during acquisition.

State transitions call `set_state()`, which fires the callback only on
`Idle ↔ Tracking` transitions (not on `Acquisition ↔ Idle`).

Fields pushed to `ChannelState` each tracking period:
- `phi` — carrier phase (display as `(phi % 1.0) × 2π`)
- `code_idx` — `code_phase_offset.back()` (samples)
- `doppler_hz`
- `cn0`
- `has_eph` — set by the navigation decoder when ephemeris is valid

---

## 13  Per-channel statistics

`ChannelStats` accumulates diagnostic counters with no locking (updated only on
the channel's own thread):

| Field | Meaning |
|---|---|
| `acq_attempts` | Completed acquisition attempts (each = `T_ACQ` block) |
| `acq_corrs` | FFT correlation pairs done (= `acq_attempts × DOPPLER_SPREAD_BINS`) |
| `locks` | Successful acquisitions entering tracking |
| `lock_losses` | C/N₀-triggered drops back to idle |
| `trk_periods` | Total code periods correlated while tracking |
| `trk_streak` | Current uninterrupted lock run (periods) |
| `max_trk_streak` | Longest lock run (periods); best proxy for "real SV vs false lock" |
| `first_lock_ts` | `ts_sec` of first ever lock (0 = never acquired) |
| `peak_cn0` | Best C/N₀ observed while tracking |
| `subframes` | LNAV subframes / SBAS messages decoded with valid parity/CRC |
| `parity_errors` | LNAV parity failures |
| `used_in_fix` | True if this SV contributed to at least one solved position fix |

These are printed in a summary table at the end of each run by the receiver.

---

## Processing flow summary (GPS L1 C/A, steady-state tracking)

```
Receiver::run_loop()  [1 ms master period]
  └─ ch.process_samples(&iq_vec, ts_sec)
       └─ tracking_process()
            ├─ get_code_and_carrier_phase()
            │    ├─ adr  += doppler × 1 ms
            │    ├─ code_off -= (doppler/fc) × 1 ms   [carrier-aided]
            │    ├─ [wrap code_off, adjust num_tx_codes]
            │    └─ compute phi, push to hist
            │
            ├─ tracking_compute_correlation()
            │    ├─ extract sig_buf[lo..lo+code_sp] from 2× window
            │    ├─ doppler_shift(sig_buf, fi+doppler, phi, fs)  [phase recurrence]
            │    └─ fused loop → (c_p, c_e, c_l, c_n)
            │
            ├─ push c_p → hist.corr_p,  num_trk_samples++
            ├─ nav.eph.trk_phase   = num_trk_samples × code_sec
            ├─ nav.eph.code_off_sec = code_off + dll_lag         [DLL lag comp]
            │
            ├─ [FLL or PLL]   ← update doppler_hz
            ├─ run_dll()      ← update code_off_sec (every 10 periods)
            ├─ update_cn0()   ← update trk.cn0 (every 1000 periods)
            │
            ├─ nav_decode()   [after 1.5 s]
            │    └─ gps_lnav / galileo_inav / sbas_l1
            │
            ├─ correct_half_rate_false_lock()   [Galileo only]
            ├─ hist.trim()
            ├─ update_all_plots()
            ├─ publish cn0/doppler/phi/code_idx → GnssState
            └─ [cn0 < 29 → idle_start()]
```
