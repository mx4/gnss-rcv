//! SBAS L1 message decoder (WAAS / EGNOS / MSAS / GAGAN / SDCM).
//!
//! SBAS broadcasts on L1 (1575.42 MHz) with the same 1023-chip C/A code as GPS,
//! so acquisition and tracking are shared (PRN 120-158); only the navigation
//! message differs. Each second carries a **250-bit message**: an 8-bit preamble
//! (a 24-bit word `0x53 0x9A 0xC6` split across three successive messages), a
//! 6-bit message type, 212 data bits, and a 24-bit **CRC-24Q**. The data are
//! *continuously* rate-1/2 convolutionally encoded (K=7, the [`crate::fec`] code)
//! → 500 symbols/s, i.e. **two C/A code periods per symbol**.
//!
//! Fed one prompt-I value per 1 ms code period, [`SbasL1Channel`] forms 2 ms
//! symbols, streaming-Viterbi-decodes the continuous code, frames 250-bit
//! messages on the preamble, and checks CRC-24Q. The unknown period→bit alignment
//! and the second-symbol (G2) convention are searched in parallel; the BPSK 180°
//! polarity is resolved from the preamble (the K=7 generators have odd weight, so
//! a symbol-polarity flip simply inverts the decoded bits). CRC is the final gate.

use crate::fec::{G1, G2, crc24q, parity};
use std::collections::VecDeque;

const MSG_BITS: usize = 250;
const PRE_BITS: usize = 8;
const TYPE_BITS: usize = 6;
const CRC_BITS: usize = 24;
/// Bits covered by the CRC: preamble + type + data (everything but the CRC field).
const DATA_END: usize = MSG_BITS - CRC_BITS; // 226
/// The three 8-bit preamble values, cycling over successive 1 s messages.
const PREAMBLES: [u8; 3] = [0x53, 0x9A, 0xC6];

const NS: usize = 64; // 2^(K-1) convolutional states
/// Streaming traceback depth (~7×K); long enough for the survivors to merge.
const TB_DEPTH: usize = 48;
/// Bits the framer may slide past without a CRC-valid message before the locked
/// hypothesis is declared misaligned and the search re-opens. A healthy stream
/// frames back-to-back (slide 0); each noise-corrupted message costs ~250, so
/// four slots distinguishes noise from a broken period→bit alignment.
const STALL_BITS: usize = 4 * MSG_BITS;

/// A decoded, CRC-valid SBAS L1 message: its type (0-63) and 250 bits.
pub struct SbasMessage {
    pub mtype: u8,
    pub bits: [u8; MSG_BITS],
}

/// Continuous hard-decision Viterbi decoder for the rate-1/2 K=7 code, releasing
/// one data bit per symbol pair at a fixed `TB_DEPTH` decision lag.
struct Viterbi {
    pm: [u32; NS],
    tb: VecDeque<[(u8, u8); NS]>, // per step: (decided input bit, predecessor state)
    next: [[u8; 2]; NS],
    expect: [[(u8, u8); 2]; NS],
}

impl Viterbi {
    fn new(invert_g2: u8) -> Self {
        let mut next = [[0u8; 2]; NS];
        let mut expect = [[(0u8, 0u8); 2]; NS];
        for s in 0..NS {
            for b in 0..2usize {
                let reg = ((b as u32) << 6) | s as u32;
                next[s][b] = (reg >> 1) as u8;
                expect[s][b] = (parity(reg & G1), parity(reg & G2) ^ invert_g2);
            }
        }
        Self {
            pm: [0; NS], // continuous decode: every start state equally likely
            tb: VecDeque::new(),
            next,
            expect,
        }
    }

    /// Feed the two coded symbols of one data bit; return the bit released from
    /// the far end of the traceback window, once it has filled.
    fn push(&mut self, r1: u8, r2: u8) -> Option<u8> {
        const INF: u32 = u32::MAX / 2;
        let mut npm = [INF; NS];
        let mut step = [(0u8, 0u8); NS];
        for s in 0..NS {
            for b in 0..2usize {
                let (o1, o2) = self.expect[s][b];
                let metric = (o1 ^ r1) as u32 + (o2 ^ r2) as u32; // Hamming distance
                let ns = self.next[s][b] as usize;
                let cand = self.pm[s] + metric;
                if cand < npm[ns] {
                    npm[ns] = cand;
                    step[ns] = (b as u8, s as u8);
                }
            }
        }
        // Renormalize so the path metrics stay bounded.
        let min = npm.iter().copied().min().unwrap();
        for v in npm.iter_mut() {
            *v -= min;
        }
        self.pm = npm;
        self.tb.push_back(step);
        if self.tb.len() <= TB_DEPTH {
            return None;
        }
        // Trace back from the best current state to the oldest step and release it.
        let mut s = (0..NS).min_by_key(|&i| self.pm[i]).unwrap() as u8;
        let mut bit = 0;
        for row in self.tb.iter().rev() {
            let (b, prev) = row[s as usize];
            bit = b;
            s = prev;
        }
        self.tb.pop_front();
        Some(bit)
    }
}

/// One decode hypothesis: a fixed period→bit alignment (`skip` lead-in periods)
/// and a G2 convention. Turns the prompt-I stream into framed messages.
struct Hypothesis {
    skip: usize,
    seen: usize,
    half: Option<f64>, // first of the 2 code periods of the current symbol
    sym0: Option<u8>,  // first of the 2 symbols of the current bit
    vit: Viterbi,
    bits: VecDeque<u8>, // decoded data bits awaiting framing
    slid: usize,        // bits slid past by the framer since the last message
}

impl Hypothesis {
    fn new(skip: usize, invert_g2: u8) -> Self {
        Self {
            skip,
            seen: 0,
            half: None,
            sym0: None,
            vit: Viterbi::new(invert_g2),
            bits: VecDeque::new(),
            slid: 0,
        }
    }

    fn push_period(&mut self, prompt_i: f64) -> Option<SbasMessage> {
        if self.seen < self.skip {
            self.seen += 1;
            return None;
        }
        // 2 code periods -> 1 hard symbol.
        let sym = match self.half.take() {
            None => {
                self.half = Some(prompt_i);
                return None;
            }
            Some(h) => ((h + prompt_i) >= 0.0) as u8,
        };
        // 2 symbols -> 1 data bit (rate 1/2).
        let bit = match self.sym0.take() {
            None => {
                self.sym0 = Some(sym);
                return None;
            }
            Some(s0) => self.vit.push(s0, sym)?,
        };
        self.bits.push_back(bit);
        self.frame()
    }

    /// Frame a 250-bit message at the front of the bit buffer, sliding one bit at
    /// a time until the preamble and CRC agree.
    fn frame(&mut self) -> Option<SbasMessage> {
        while self.bits.len() >= MSG_BITS {
            if let Some(msg) = self.try_message() {
                self.bits.drain(..MSG_BITS);
                self.slid = 0;
                return Some(msg);
            }
            self.bits.pop_front();
            self.slid += 1;
        }
        None
    }

    fn try_message(&self) -> Option<SbasMessage> {
        let mut m: [u8; MSG_BITS] = std::array::from_fn(|i| self.bits[i]);
        let pre = bits_to_u32(&m[..PRE_BITS]) as u8;
        // Resolve the BPSK 180° ambiguity: odd-weight K=7 generators mean a symbol
        // polarity flip inverts the decoded bits, so a complemented preamble means
        // the whole message is inverted.
        if PREAMBLES.contains(&pre) {
            // upright
        } else if PREAMBLES.contains(&!pre) {
            for b in m.iter_mut() {
                *b ^= 1;
            }
        } else {
            return None;
        }
        if crc24q(&m[..DATA_END]) != bits_to_u32(&m[DATA_END..]) {
            return None;
        }
        let mtype = bits_to_u32(&m[PRE_BITS..PRE_BITS + TYPE_BITS]) as u8;
        Some(SbasMessage { mtype, bits: m })
    }
}

/// Streaming SBAS L1 decoder for one channel. Searches the period→bit alignment
/// (4 phases) and the G2 convention (2) in parallel, then locks onto the first
/// hypothesis that yields a CRC-valid message. The lock is provisional: if the
/// framer slides [`STALL_BITS`] without another valid message the alignment is
/// taken as broken and the search re-opens — every hypothesis is kept fed in
/// the background, so the replacement alignment is warm when that happens.
///
/// The alignment hinges on the channel feeding exactly one prompt per
/// *transmitted* code period: a code-phase wrap re-correlates or skips one, and
/// the channel reports it via [`wrap_drop`](Self::wrap_drop) /
/// [`wrap_repeat`](Self::wrap_repeat) (mirroring `LnavState`) so the
/// 2-periods-per-symbol grid keeps its phase and no message is lost. An
/// unreported wrap shifts the grid and breaks the locked hypothesis — that is
/// what the stall watchdog recovers from, at the cost of a few messages.
pub struct SbasL1Channel {
    hyps: Vec<Hypothesis>,
    locked: Option<usize>,
    last: Option<f64>,            // last consumed prompt, the wrap_repeat stand-in
    drop_next: bool,              // next prompt re-correlates the period just consumed
    pending: Option<SbasMessage>, // message framed inside wrap_repeat
}

impl Default for SbasL1Channel {
    fn default() -> Self {
        Self::new()
    }
}

impl SbasL1Channel {
    pub fn new() -> Self {
        let mut hyps = Vec::with_capacity(8);
        for invert_g2 in [0u8, 1] {
            for skip in 0..4 {
                hyps.push(Hypothesis::new(skip, invert_g2));
            }
        }
        Self {
            hyps,
            locked: None,
            last: None,
            drop_next: false,
            pending: None,
        }
    }

    /// The channel re-correlated the same transmitted code period (positive
    /// code-phase wrap): the prompt just consumed arrives again next period —
    /// ignore that re-delivery to keep the symbol grid in phase.
    pub fn wrap_drop(&mut self) {
        if self.last.is_some() {
            self.drop_next = true;
        }
    }

    /// The channel skipped a transmitted code period (negative code-phase
    /// wrap): stand in the last prompt for the missing period. A message that
    /// frames here is held for the next `push_period`.
    pub fn wrap_repeat(&mut self) {
        if let Some(p) = self.last
            && let Some(msg) = self.feed(p)
        {
            self.pending = Some(msg);
        }
    }

    /// Feed one prompt-I value (one C/A code period). Returns a CRC-valid message
    /// when one frames.
    pub fn push_period(&mut self, prompt_i: f64) -> Option<SbasMessage> {
        self.last = Some(prompt_i);
        if std::mem::take(&mut self.drop_next) {
            return self.pending.take();
        }
        let msg = self.feed(prompt_i);
        self.pending.take().or(msg)
    }

    fn feed(&mut self, prompt_i: f64) -> Option<SbasMessage> {
        // Every hypothesis is fed even while locked (8 small Viterbis at 250
        // bit/s are trivial next to the correlators): each keeps its pairing
        // phase synced to the stream, so when the locked alignment breaks, its
        // replacement is already warm. Only the correct alignment can pass the
        // preamble + CRC gate, so at most one hypothesis fires per period.
        let mut hit: Option<(usize, SbasMessage)> = None;
        for (j, h) in self.hyps.iter_mut().enumerate() {
            if let Some(m) = h.push_period(prompt_i)
                && hit.is_none()
            {
                hit = Some((j, m));
            }
        }
        match self.locked {
            Some(i) => {
                // Locked: surface only the locked hypothesis's messages, so a
                // chance CRC collision in a misaligned stream can't slip
                // through. If its framing stalls, the alignment broke for good
                // (a missed code-phase wrap): re-open the search.
                let msg = hit.and_then(|(j, m)| (j == i).then_some(m));
                if msg.is_none() && self.hyps[i].slid > STALL_BITS {
                    self.locked = None;
                }
                msg
            }
            None => {
                let (j, m) = hit?;
                self.locked = Some(j); // CRC passed: commit to this hypothesis
                Some(m)
            }
        }
    }
}

fn bits_to_u32(bits: &[u8]) -> u32 {
    bits.iter()
        .fold(0u32, |acc, &b| (acc << 1) | (b & 1) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::conv_encode;

    fn put_bits(m: &mut [u8], pos: usize, len: usize, val: u32) {
        for i in 0..len {
            m[pos + i] = ((val >> (len - 1 - i)) & 1) as u8;
        }
    }

    // Build one valid 250-bit SBAS message (preamble for `seq`, given type).
    fn build_message(seq: usize, mtype: u8) -> [u8; MSG_BITS] {
        let mut m = [0u8; MSG_BITS];
        put_bits(&mut m, 0, PRE_BITS, PREAMBLES[seq % 3] as u32);
        put_bits(&mut m, PRE_BITS, TYPE_BITS, mtype as u32);
        for (i, b) in m[14..DATA_END].iter_mut().enumerate() {
            *b = (i * 7 + seq).is_multiple_of(3) as u8;
        }
        let crc = crc24q(&m[..DATA_END]);
        put_bits(&mut m, DATA_END, CRC_BITS, crc);
        m
    }

    // Continuously encode `messages`, then expand to 2 code periods per symbol,
    // optionally polarity-flipped and prefixed with `lead` junk periods.
    fn to_periods(
        messages: &[[u8; MSG_BITS]],
        invert_g2: u8,
        polarity: f64,
        lead: usize,
    ) -> Vec<f64> {
        let mut bits = Vec::new();
        for m in messages {
            bits.extend_from_slice(m);
        }
        let syms = conv_encode(&bits, invert_g2); // continuous, no per-message tail
        let mut periods = vec![0.3f64; lead]; // arbitrary lead-in
        for &s in &syms {
            let v = if s == 0 { 1.0 } else { -1.0 } * polarity;
            periods.push(v);
            periods.push(v);
        }
        periods
    }

    #[test]
    fn decodes_a_continuous_stream_of_messages() {
        let sent: Vec<u8> = (0..10).map(|k| (k as u8 * 9 + 5) & 0x3f).collect();
        let msgs: Vec<_> = sent
            .iter()
            .enumerate()
            .map(|(k, &t)| build_message(k, t))
            .collect();

        // The encoder's G2 convention is unknown to the decoder, as is the BPSK
        // polarity and the period→bit alignment: it must recover all of them.
        for invert_g2 in [0u8, 1] {
            for polarity in [1.0f64, -1.0] {
                for lead in [0usize, 1, 2, 3] {
                    let periods = to_periods(&msgs, invert_g2, polarity, lead);
                    let mut ch = SbasL1Channel::new();
                    let got: Vec<u8> = periods
                        .iter()
                        .filter_map(|&p| ch.push_period(p).map(|m| m.mtype))
                        .collect();

                    // Warm-up (Viterbi lag + frame search) costs the first message
                    // or two, and the lag can drop the last; the recovered ones
                    // must be a contiguous run of `sent`, in order.
                    assert!(
                        got.len() >= 5,
                        "g2={invert_g2} pol={polarity} lead={lead}: only {got:?}"
                    );
                    let run =
                        (0..=sent.len() - got.len()).any(|st| sent[st..st + got.len()] == got[..]);
                    assert!(
                        run,
                        "g2={invert_g2} pol={polarity} lead={lead}: {got:?} not a run of {sent:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn wrap_callbacks_preserve_the_symbol_alignment() {
        let sent: Vec<u8> = (0..12).map(|k| (k as u8 * 5 + 3) & 0x3f).collect();
        let msgs: Vec<_> = sent
            .iter()
            .enumerate()
            .map(|(k, &t)| build_message(k, t))
            .collect();
        let periods = to_periods(&msgs, 0, 1.0, 0);

        // A positive code-phase wrap re-correlates one transmitted period (the
        // channel re-delivers it after wrap_drop); a negative wrap skips one
        // (wrap_repeat stands the last prompt in). One of each, mid-stream,
        // after the decoder has locked. Without the callbacks each wrap shifts
        // the 2-periods-per-symbol pairing and kills the stream for good.
        let wrap_pos = 4 * 2 * 2 * MSG_BITS + 1; // inside message 4
        let wrap_neg = 8 * 2 * 2 * MSG_BITS + 3; // inside message 8
        let mut ch = SbasL1Channel::new();
        let mut got: Vec<u8> = Vec::new();
        let mut i = 0;
        while i < periods.len() {
            if i == wrap_neg {
                ch.wrap_repeat(); // transmitted period i is never delivered
                i += 1;
                continue;
            }
            got.extend(ch.push_period(periods[i]).map(|m| m.mtype));
            if i == wrap_pos {
                ch.wrap_drop(); // the same period re-correlates and re-delivers
                got.extend(ch.push_period(periods[i]).map(|m| m.mtype));
            }
            i += 1;
        }

        // Both wraps must cost nothing: a contiguous run reaching past them.
        let run = (0..=sent.len() - got.len()).any(|st| sent[st..st + got.len()] == got[..]);
        assert!(run, "{got:?} not a run of {sent:?}");
        assert!(
            got.contains(&sent[10]),
            "no message decoded after the wraps: {got:?}"
        );
    }

    #[test]
    fn an_unreported_slip_unlocks_and_the_search_recovers() {
        let sent: Vec<u8> = (0..16).map(|k| (k as u8 * 3 + 1) & 0x3f).collect();
        let msgs: Vec<_> = sent
            .iter()
            .enumerate()
            .map(|(k, &t)| build_message(k, t))
            .collect();
        let periods = to_periods(&msgs, 0, 1.0, 0);

        // One duplicated period the channel never reports: the locked
        // hypothesis's alignment breaks. The stall watchdog must re-open the
        // hypothesis search and resume decoding within ~STALL_BITS plus
        // re-lock warm-up (a handful of messages), not stay dead.
        let slip = 5 * 2 * 2 * MSG_BITS + 7;
        let mut ch = SbasL1Channel::new();
        let mut got: Vec<u8> = Vec::new();
        for (i, &p) in periods.iter().enumerate() {
            if i == slip {
                got.extend(ch.push_period(p).map(|m| m.mtype));
            }
            got.extend(ch.push_period(p).map(|m| m.mtype));
        }
        let recovered = got.iter().filter(|&&t| sent[12..].contains(&t)).count();
        assert!(
            recovered >= 2,
            "no recovery after the unreported slip: {got:?}"
        );
    }

    #[test]
    fn rejects_a_corrupted_message() {
        let msgs: Vec<_> = (0..8).map(|k| build_message(k, 7)).collect();
        let mut periods = to_periods(&msgs, 0, 1.0, 0);
        // Flip a scattering of periods: corrupted messages must fail CRC, not
        // surface as bogus type-7 messages.
        for i in (50..periods.len()).step_by(37) {
            periods[i] = -periods[i];
        }
        let mut ch = SbasL1Channel::new();
        let bad = periods
            .iter()
            .filter_map(|&p| ch.push_period(p))
            .any(|m| crc24q(&m.bits[..DATA_END]) != bits_to_u32(&m.bits[DATA_END..]));
        assert!(!bad, "every emitted message must be CRC-valid");
    }
}
