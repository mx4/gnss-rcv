# Ingest decimation

> **Status: LANDED** (7fc81ba, 2026-06-12). Wideband file captures are
> FIR-filtered and integer-downsampled to the lowest rate that still carries
> every enabled signal, before anything else sees the samples.

Code: `src/decimate.rs` — `decimation_factor()` (the policy) and
`DecimatingReader` (the filter + reader wrapper); wiring in
`Receiver::new` (`src/receiver.rs`). Disable with `GNSS_DECIM=off` for A/B
runs.

## Summary

Everything in this receiver scales with the sample rate `fs` — acquisition
FFTs, carrier tables, correlators, the per-channel search grids — but the
signals themselves occupy only `|fi| ± 2.346 MHz`. A 50 Msps capture
therefore costs ~8× more CPU and memory than the information warrants.
`DecimatingReader` wraps the file reader and presents the stream at
`fs / k`; the receiver downstream is **entirely unaware** — it is simply
constructed with the lower rate. On the tuni2025 mixed GPS+Galileo session
(50 Msps → 6.25 Msps) this took the cost from **17.2 CPU-s per data-second
/ 3.59 GB peak RSS to ~1.5 CPU-s / 646 MB** (~11× CPU, ~5.6× memory), and
σ(Galileo) *improved* slightly — the anti-alias filter also removes
out-of-band noise before correlation.

## Why a filter, not sample-dropping

Plain sample-dropping (take every k-th sample) aliases: the noise (and any
interference) in the discarded `(k−1)/k` of the band folds onto the signal,
costing up to `10·log10(k)` dB of effective C/N0. Averaging blocks of k is
just a crude low-pass with poor stopband. A proper anti-alias low-pass
keeps the noise floor flat and the signal untouched; its only side effect
is a **constant group delay** ((taps−1)/2 input samples), which shifts
every satellite identically and therefore folds into the receiver clock
bias like any common delay — invisible to the position solve.

## Picking the factor — `decimation_factor(fs, fi)`

The band that must survive is one-sided
`|fi| + 2.346 MHz` (`SIGNAL_HALF_BW_HZ`): the E1 BOC(1,1) main lobes span
±2.046 MHz (±1.023 MHz subcarrier ± 1.023 MHz code; L1 C/A is narrower and
fits inside), plus 0.3 MHz of front-end LO slack. The factor is the largest
`k` such that

1. `fs / k ≥ 2.66 ×` the one-sided band — passband plus transition room
   for a realizable FIR (the 0.41/0.5 cutoff ratio below), and
2. `fs / k` keeps a **whole number of samples per 1 ms base block** — the
   scheduler's block grid (see
   [multi-signal-stepping.md](multi-signal-stepping.md)) requires it.

`k = 1` means no decimation and no wrapper. Examples (unit-tested):

| Capture | fs | fi | k | out |
| --- | --- | --- | --- | --- |
| tuni2025 | 50 Msps | 0 | 8 | 6.25 Msps |
| SJTU | 25 Msps | 6.25 MHz | 1 (IF eats the budget) | — |
| ion-lime | 10 Msps | 420 kHz | 1 | — |
| narrowband (2.046/4 Msps) | — | 0 | 1 | — |

Note the asymmetry: a non-zero IF wastes budget because the signal sits
off-centre and the low-pass must keep everything out to `|fi| + band`.
(Mixing to baseband first would recover that budget — not implemented;
captures with large IFs are rare and small.)

## The filter and the reader — `DecimatingReader`

- **Filter**: windowed-sinc (Hamming) low-pass, `8k + 1` taps, cutoff at
  0.41 of the *output* rate (0.82 of the output Nyquist) — the remaining
  18% is the transition band. `8k+1` taps put the first sidelobe well
  below the 4–16-bit quantization floors of our captures. Taps are
  computed once in the constructor.
- **Polyphase evaluation**: output sample `n` is the filter evaluated at
  input sample `n·k` — only every k-th output is computed, so the cost is
  `(8k+1)/k ≈ 8` multiplies per *output* sample regardless of `k`.
- **Statelessness**: `get_iq_data(off, num)` is random-access, like every
  `IQReader`. Each call re-reads the `(taps − 1)`-sample overlap before
  the requested window — noise next to the block itself — and the start
  of the stream is zero-padded, so identical offsets always produce
  identical samples with no carried filter state.
- **Group delay**: the taps are applied to the window *preceding* each
  output point (constant `(taps−1)/2` input samples), common to all SVs —
  absorbed by the receiver clock bias (see above).
- **Duration passthrough**: decimation lowers the sample *rate*, not the
  wall-clock length; `duration_sec()` forwards to the inner reader so the
  UI progress bar is unaffected.

## Where it sits

Wired in `Receiver::new`, before anything consumes samples:

```rust
let decim_ok = !cfg.use_device && cfg.hostname.is_empty()
    && !std::env::var("GNSS_DECIM").is_ok_and(|v| v == "off");
if decim_ok {
    let k = decimate::decimation_factor(cfg.fs, cfg.fi);
    if k > 1 {
        iq_feed = Box::new(DecimatingReader::new(iq_feed, k));
        fs = cfg.fs / k as f64;
    }
}
```

- **File feeds only**: streaming feeds (RTL-SDR device, network) have no
  random access for the overlap re-reads. They could be supported with a
  small ring of carried state; not needed yet.
- The receiver (channels, scheduler, carrier tables, plots) is constructed
  with the decimated `fs` and never learns the capture's native rate; the
  recording metadata keeps the original.
- `GNSS_DECIM=off` preserves the native-rate path for A/B comparisons —
  decimated vs native runs differ at the ULP/noise level, not structurally.

## Tests

`cargo test --release decimate` (hermetic, in `src/decimate.rs`):

1. **factor table** — the policy: band coverage, 1 ms grid, IF handling
   (the examples above);
2. **passband tone** — a 0.5 MHz complex tone at 16 → 8 Msps survives with
   amplitude within ±5% and coherent phase after the group delay;
3. **stopband rejection** — a tone past the output Nyquist (5 MHz at
   16 → 8 Msps) is attenuated to < 1% power, not aliased in.

The end-to-end gate is the tuni2025 mixed fix in `validate_fix.py`, which
runs flag-less and therefore exercises the decimated path.
