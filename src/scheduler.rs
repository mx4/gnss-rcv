//! The multi-signal stepping scheduler (design:
//! [docs/multi-signal-stepping.md](../docs/multi-signal-stepping.md)).
//!
//! The receiver advances the sample stream in 1 ms base blocks — the GCD of
//! every supported code period — into a ring holding the last two periods of
//! the *longest* enabled family. Each signal family steps on a fixed grid
//! (C/A-family channels every block, E1 every 4th): when a family's period
//! boundary lands on the ingested block, its channels process the ring's
//! last two family periods. The window's position relative to each SV's code
//! phase is arbitrary either way — `code_off` absorbs it (that is how the
//! single-signal receiver has always worked) — so the fixed grid costs
//! nothing and the per-channel wrap bookkeeping is untouched.
//!
//! M1 scope: one family per session (the configured `--sig`), stepping
//! through the ring exactly as the old per-period fetch did — behaviour is
//! gated digit-exact against the pinned baselines. The multi-family grids,
//! per-family carrier sets and the shared acquisition-FFT cache layer on in
//! M2-M5.

use crate::code::Signal;
use rustfft::num_complex::Complex32;

/// Base block length: 1 ms, the GCD of all supported code periods.
const BLOCK_SEC: f64 = 1e-3;

pub struct Scheduler {
    /// Samples per base block.
    block_sp: usize,
    /// Each enabled family's code period in blocks (1 = C/A family, 4 = E1),
    /// in the order they were registered.
    periods: Vec<usize>,
    /// The last `2 × max(period)` of samples; every family's window is a
    /// tail slice of it.
    ring: Vec<Complex32>,
    /// Base blocks ingested since the start of the stream.
    blocks: u64,
    fs: f64,
}

impl Scheduler {
    pub fn new(fs: f64, sigs: &[Signal]) -> Self {
        let block_sp = (fs * BLOCK_SEC) as usize;
        let periods: Vec<usize> = sigs
            .iter()
            .map(|s| (s.code_period_sec() / BLOCK_SEC).round() as usize)
            .collect();
        assert!(!periods.is_empty());
        Self {
            block_sp,
            periods,
            ring: Vec::new(),
            blocks: 0,
            fs,
        }
    }

    pub fn block_sp(&self) -> usize {
        self.block_sp
    }

    /// Ingest one base block. Returns the indices of the families due on
    /// this block: their period boundary lands here and the ring holds two
    /// of their periods (so each family's first step happens once its window
    /// is primed, exactly like the old 2-period prefetch).
    pub fn ingest(&mut self, mut block: Vec<Complex32>) -> Vec<usize> {
        debug_assert_eq!(block.len(), self.block_sp);
        self.ring.append(&mut block);
        self.blocks += 1;
        let cap = 2 * self.periods.iter().max().unwrap() * self.block_sp;
        if self.ring.len() > cap {
            let excess = self.ring.len() - cap;
            self.ring.drain(0..excess);
        }
        self.periods
            .iter()
            .enumerate()
            .filter(|&(_, &p)| {
                self.blocks.is_multiple_of(p as u64) && self.ring.len() >= 2 * p * self.block_sp
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// Family `fam`'s window — its last two code periods — and the stream
    /// time at the *start of the last period* (the per-channel `ts`
    /// convention the tracking/anchor code expects):
    /// `[...code...][...code...]`
    /// `            ^ ts`
    pub fn window(&self, fam: usize) -> (&[Complex32], f64) {
        let p = self.periods[fam];
        let n = 2 * p * self.block_sp;
        let tail_ts = self.blocks as f64 * self.block_sp as f64 / self.fs;
        let period_sec = p as f64 * self.block_sp as f64 / self.fs;
        (&self.ring[self.ring.len() - n..], tail_ts - period_sec)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blocks(s: &mut Scheduler, n: usize, start: usize) -> Vec<bool> {
        (0..n)
            .map(|k| {
                let v = vec![Complex32::new((start + k) as f32, 0.0); s.block_sp()];
                !s.ingest(v).is_empty()
            })
            .collect()
    }

    #[test]
    fn ca_family_steps_every_block_after_priming() {
        // 1 ms family at 2.046 Msps: due on every block once two periods are in.
        let mut s = Scheduler::new(2_046_000.0, &[Signal::L1ca]);
        let due = blocks(&mut s, 4, 0);
        assert_eq!(due, vec![false, true, true, true]);
        let (w, ts) = s.window(0);
        assert_eq!(w.len(), 2 * s.block_sp());
        // After 4 blocks the window is blocks 2..4; ts = start of block 3.
        assert_eq!(w[0].re, 2.0);
        assert!((ts - 3e-3).abs() < 1e-12);
    }

    #[test]
    fn mixed_families_step_on_their_own_grids() {
        // C/A (1 ms) + E1 (4 ms) sharing one ring: C/A due from block 2 on
        // every block, E1 due from block 8 on every 4th; windows are tail
        // slices of the same ring with their own lengths and ts.
        let mut s = Scheduler::new(4_092_000.0, &[Signal::L1ca, Signal::GalileoE1b]);
        let mut due_log = Vec::new();
        for k in 0..12 {
            let v = vec![Complex32::new(k as f32, 0.0); s.block_sp()];
            due_log.push(s.ingest(v));
        }
        assert_eq!(due_log[0], Vec::<usize>::new()); // priming
        assert_eq!(due_log[1], vec![0]); // C/A primed
        assert_eq!(due_log[3], vec![0]); // E1 boundary but not primed
        assert_eq!(due_log[7], vec![0, 1]); // both due at block 8
        assert_eq!(due_log[8], vec![0]);
        assert_eq!(due_log[11], vec![0, 1]);
        let (wca, tsca) = s.window(0);
        let (we1, tse1) = s.window(1);
        assert_eq!(wca.len(), 2 * s.block_sp());
        assert_eq!(we1.len(), 8 * s.block_sp());
        assert_eq!(wca[0].re, 10.0); // last 2 blocks of 12
        assert_eq!(we1[0].re, 4.0); // last 8 blocks of 12
        assert!((tsca - 11e-3).abs() < 1e-12);
        assert!((tse1 - 8e-3).abs() < 1e-12);
    }

    #[test]
    fn e1_family_steps_every_fourth_block() {
        // 4 ms family: due when the boundary lands AND two periods (8 blocks)
        // are in — so the first step is at block 8, then every 4th.
        let mut s = Scheduler::new(4_092_000.0, &[Signal::GalileoE1b]);
        let due = blocks(&mut s, 13, 0);
        let expect: Vec<bool> = (1..=13).map(|b| b >= 8 && b % 4 == 0).collect();
        assert_eq!(due, expect);
        let (w, ts) = s.window(0);
        assert_eq!(w.len(), 8 * s.block_sp());
        // After 13 blocks (last due at 12): window = blocks 4..12, but the
        // 13th block has been ingested — the ring slides per block, so the
        // window is blocks 5..13 and ts marks the start of its last period.
        assert_eq!(w[0].re, 5.0);
        let tail = 13e-3;
        assert!((ts - (tail - 4e-3)).abs() < 1e-12);
    }
}
