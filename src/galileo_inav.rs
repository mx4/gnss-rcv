//! Galileo E1-B I/NAV decoder.
//!
//! Each I/NAV page part is 250 symbols at 250 sym/s: a 10-symbol sync pattern
//! then 240 data symbols. The 240 symbols are **block-interleaved** (30 columns ×
//! 8 rows) and **rate-1/2 convolutionally encoded** (K=7, G1=171₈, G2=133₈ with
//! G2 inverted), carrying 120 bits (114 data + 6 tail). A **CRC-24Q** protects the
//! assembled page. The shared convolutional code and CRC live in [`crate::fec`];
//! this module adds the Galileo-specific de-interleaver, Viterbi decoder, page
//! assembly and ephemeris extraction ([`InavDecoder`] / [`decode_ephemeris_word`]).

use crate::constants::{
    P2_5, P2_19, P2_29, P2_31, P2_32, P2_33, P2_34, P2_43, P2_46, P2_59, SC2RAD,
};
use crate::ephemeris::Ephemeris;
use crate::fec::{G1, G2, conv_encode, crc24q, parity};

/// Galileo inverts the G2 output symbol of the shared K=7 convolutional code.
const G2_INVERTED: u8 = 1;

/// Hard-decision Viterbi decode of `syms` (two symbols per bit) back to `n_bits`
/// bits, for the zero-terminated trellis above (start and end at state 0).
pub fn viterbi_decode(syms: &[u8], n_bits: usize) -> Vec<u8> {
    assert_eq!(syms.len(), 2 * n_bits);
    const NSTATES: usize = 64;
    const INF: u32 = u32::MAX / 2;

    // Per-(state, input-bit) transition: next state and expected (o1, o2).
    let mut next = [[0usize; 2]; NSTATES];
    let mut expect = [[(0u8, 0u8); 2]; NSTATES];
    for s in 0..NSTATES {
        for b in 0..2usize {
            let reg = ((b as u32) << 6) | s as u32;
            next[s][b] = (reg >> 1) as usize;
            expect[s][b] = (parity(reg & G1), parity(reg & G2) ^ G2_INVERTED);
        }
    }

    let mut pm = vec![INF; NSTATES];
    pm[0] = 0;
    // Per step, for each surviving state: the (input bit, predecessor state).
    let mut tb: Vec<[(u8, u8); NSTATES]> = Vec::with_capacity(n_bits);
    for t in 0..n_bits {
        let (r1, r2) = (syms[2 * t], syms[2 * t + 1]);
        let mut npm = vec![INF; NSTATES];
        let mut step = [(0u8, 0u8); NSTATES];
        for s in 0..NSTATES {
            if pm[s] >= INF {
                continue;
            }
            for b in 0..2 {
                let (o1, o2) = expect[s][b];
                let metric = (o1 ^ r1) as u32 + (o2 ^ r2) as u32; // Hamming distance
                let ns = next[s][b];
                let cand = pm[s] + metric;
                if cand < npm[ns] {
                    npm[ns] = cand;
                    step[ns] = (b as u8, s as u8);
                }
            }
        }
        pm = npm;
        tb.push(step);
    }

    // Traceback from the terminated state 0.
    let mut state = 0usize;
    let mut bits = vec![0u8; n_bits];
    for t in (0..n_bits).rev() {
        let (b, prev) = tb[t][state];
        bits[t] = b;
        state = prev as usize;
    }
    bits
}

/// Galileo I/NAV block interleaver: 30 columns × 8 rows. Write the 240 input
/// symbols column-by-column, read row-by-row.
pub fn interleave(input: &[u8]) -> Vec<u8> {
    assert_eq!(input.len(), 240);
    let mut out = vec![0u8; 240];
    for (i, &v) in input.iter().enumerate() {
        out[(i % 8) * 30 + i / 8] = v; // row = i%8, col = i/8 -> row-major read
    }
    out
}

/// Inverse of [`interleave`].
pub fn deinterleave(input: &[u8]) -> Vec<u8> {
    assert_eq!(input.len(), 240);
    let mut out = vec![0u8; 240];
    for (j, &v) in input.iter().enumerate() {
        out[(j % 30) * 8 + j / 30] = v;
    }
    out
}

/// The 10-symbol I/NAV page-part synchronisation pattern (preamble).
const PREAMBLE: [u8; 10] = [0, 1, 0, 1, 1, 0, 0, 0, 0, 0];
/// Each page part is 250 symbols: the preamble then 240 FEC symbols.
const PAGE_PART_SYMBOLS: usize = 250;

/// A decoded, CRC-valid I/NAV word: its 6-bit word type and 128 data bits.
pub struct InavWord {
    pub word_type: u8,
    pub bits: [u8; 128],
}

/// Streaming Galileo E1-B I/NAV page decoder. Fed one symbol per 4 ms code
/// period (the sign of prompt-I), it aligns to the page-part preamble,
/// FEC-decodes each 250-symbol part, joins the even+odd parts into a 228-bit
/// page, checks CRC-24Q, and emits the 128-bit word.
#[derive(Default)]
pub struct InavDecoder {
    buf: Vec<u8>,
    n_seen: usize,                 // total symbols ever pushed (for absolute parity)
    pending_even: Option<Vec<u8>>, // the 114-bit even page-part frame, awaiting its odd
}

/// Carrier-recovery ambiguity of a page part: a constant polarity flip (`pol`,
/// the Costas 180° ambiguity) and an alternating (−1)ⁿ symbol flip (`alt`).
///
/// The (−1)ⁿ case is a Costas-loop **false lock at half the symbol rate**: a
/// residual carrier error of ±125 Hz (= 250 sym/s ÷ 2) rotates the constellation
/// by π every 4 ms symbol, which the prompt `atan(Q/I)` discriminator and the
/// `atan(cross/dot)` FLL both fold to zero error — so tracking holds a strong,
/// stable lock while every symbol is multiplied by (−1)ⁿ. It is deterministic
/// per SV (acquisition is deterministic) and independent of C/N0, so even the
/// strongest SV can land in it. Code/DLL tracking is unaffected; only the symbol
/// stream is, so we resolve it here at frame sync rather than in the loops.
#[derive(Clone, Copy)]
struct Ambiguity {
    pol: u8, // 0 = upright, 1 = carrier-inverted
    alt: u8, // 1 = symbols carry a (−1)ⁿ half-rate flip to undo
}

impl InavDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one I/NAV symbol (0/1); returns a word once an even+odd page passes CRC.
    pub fn push_symbol(&mut self, sym: u8) -> Option<InavWord> {
        self.buf.push(sym & 1);
        self.n_seen += 1;
        if self.buf.len() < PAGE_PART_SYMBOLS {
            return None;
        }
        // Absolute index (hence (−1)ⁿ parity) of the symbol now at the front.
        let parity0 = ((self.n_seen - self.buf.len()) & 1) as u8;
        // The front must be a preamble — upright or carrier-inverted, and with or
        // without the half-rate (−1)ⁿ flip; else slide one symbol and retry.
        let Some(amb) = match_preamble(&self.buf[..PREAMBLE.len()], parity0) else {
            self.buf.remove(0);
            return None;
        };
        let part: Vec<u8> = self.buf.drain(..PAGE_PART_SYMBOLS).collect();
        self.assemble(decode_page_part(&part, amb, parity0))
    }

    fn assemble(&mut self, frame: Vec<u8>) -> Option<InavWord> {
        if frame[0] == 0 {
            // even page part: hold it until the matching odd part arrives.
            self.pending_even = Some(frame);
            return None;
        }
        // odd part: join with the held even part -> 228-bit I/NAV page.
        let mut page = self.pending_even.take()?; // 114 bits
        page.extend_from_slice(&frame); // + 114 -> 228
        if crc24q(&page[..196]) != bits_to_u32(&page[196..220]) {
            return None;
        }
        let mut bits = [0u8; 128];
        bits[..112].copy_from_slice(&page[2..114]); // Data k (1/2)
        bits[112..].copy_from_slice(&page[116..132]); // Data j (2/2)
        Some(InavWord {
            word_type: bits_to_u32(&bits[..6]) as u8,
            bits,
        })
    }
}

/// Match the 10-symbol front against the preamble under every carrier-recovery
/// ambiguity: upright/inverted (`pol`) × with/without the half-rate (−1)ⁿ flip
/// (`alt`). `parity0` is the absolute (−1)ⁿ parity of the first symbol, so the
/// flip lines up with the same parity used when the whole part is decoded. The
/// upright (alt = 0) hypotheses are tried first so a clean lock is undisturbed;
/// CRC-24Q is the real gate, so the extra hypotheses only cost the rare wasted
/// decode on a chance match.
fn match_preamble(syms: &[u8], parity0: u8) -> Option<Ambiguity> {
    for alt in [0u8, 1] {
        for pol in [0u8, 1] {
            if syms
                .iter()
                .zip(PREAMBLE)
                .enumerate()
                .all(|(j, (&a, b))| a ^ pol ^ (alt & ((parity0 + j as u8) & 1)) == b)
            {
                return Some(Ambiguity { pol, alt });
            }
        }
    }
    None
}

/// Strip the preamble, undo the carrier ambiguity (constant polarity and any
/// half-rate (−1)ⁿ flip), de-interleave and Viterbi-decode one 250-symbol page
/// part into its 114-bit frame (120 decoded bits less 6 tail). `parity0` is the
/// absolute (−1)ⁿ parity of `part[0]`; the FEC symbols start `PREAMBLE.len()`
/// (an even count) later, so they share that parity.
fn decode_page_part(part: &[u8], amb: Ambiguity, parity0: u8) -> Vec<u8> {
    let encoded: Vec<u8> = part[PREAMBLE.len()..]
        .iter()
        .enumerate()
        .map(|(j, &s)| s ^ amb.pol ^ (amb.alt & ((parity0 + j as u8) & 1)))
        .collect();
    let mut bits = viterbi_decode(&deinterleave(&encoded), 120);
    bits.truncate(114);
    bits
}

fn bits_to_u32(bits: &[u8]) -> u32 {
    bits.iter()
        .fold(0u32, |acc, &b| (acc << 1) | (b & 1) as u32)
}

/// Read `len` unsigned bits (MSB first) from the one-bit-per-byte `bits`, at
/// 0-indexed `pos`.
fn ubits(bits: &[u8], pos: usize, len: usize) -> u32 {
    bits[pos..pos + len]
        .iter()
        .fold(0u32, |a, &b| (a << 1) | (b & 1) as u32)
}

/// Like [`ubits`] but two's-complement sign-extended to `i32`.
fn sbits(bits: &[u8], pos: usize, len: usize) -> i32 {
    let shift = 32 - len;
    ((ubits(bits, pos, len) << shift) as i32) >> shift
}

/// Fill `eph` from one CRC-valid I/NAV word: types 1-4 carry the orbit+clock,
/// type 5 the BGD and the GST week. Galileo broadcasts the same Keplerian set as
/// GPS LNAV but with its own bit layout and scale factors (finer af0/af1/af2,
/// t0e/t0c in 60 s units, week taken from the GST). Word types 0 / 6-10 / 16
/// carry spare / UTC / almanac / reduced-CED data and are skipped here. Once
/// types 1-5 have all landed, `eph.is_valid()` holds and the solver can use it.
///
/// Field offsets and scales follow the Galileo OS SIS ICD (cross-checked against
/// gnss-sdr's `Galileo_INAV.h`); positions there are 1-indexed in the 128-bit word.
pub fn decode_ephemeris_word(eph: &mut Ephemeris, word: &InavWord) {
    let b = &word.bits[..];
    let u = |start: usize, len: usize| ubits(b, start - 1, len);
    let s = |start: usize, len: usize| sbits(b, start - 1, len) as f64;
    match word.word_type {
        1 => {
            eph.iode = u(7, 10); // IODnav
            eph.toe = u(17, 14) * 60; // t0e, 60 s LSB
            eph.m0 = s(31, 32) * P2_31 * SC2RAD;
            eph.ecc = u(63, 32) as f64 * P2_33;
            let sqrt_a = u(95, 32) as f64 * P2_19;
            eph.a = sqrt_a * sqrt_a;
        }
        2 => {
            eph.omg0 = s(17, 32) * P2_31 * SC2RAD;
            eph.i0 = s(49, 32) * P2_31 * SC2RAD;
            eph.omg = s(81, 32) * P2_31 * SC2RAD;
            eph.i_dot = s(113, 14) * P2_43 * SC2RAD;
        }
        3 => {
            eph.omg_dot = s(17, 24) * P2_43 * SC2RAD;
            eph.deln = s(41, 16) * P2_43 * SC2RAD;
            eph.cuc = s(57, 16) * P2_29;
            eph.cus = s(73, 16) * P2_29;
            eph.crc = s(89, 16) * P2_5;
            eph.crs = s(105, 16) * P2_5;
        }
        4 => {
            eph.cic = s(23, 16) * P2_29;
            eph.cis = s(39, 16) * P2_29;
            eph.toc = u(55, 14) * 60; // t0c, 60 s LSB
            eph.f0 = s(69, 31) * P2_34;
            eph.f1 = s(100, 21) * P2_46;
            eph.f2 = s(121, 6) * P2_59;
        }
        5 => {
            eph.tgd = s(48, 10) * P2_32; // BGD(E1,E5a): the E1 group delay
            eph.week = u(74, 12); // GST week number
            eph.tow = u(86, 20); // GST time of week [s]
        }
        _ => {}
    }
}

// ---- I/NAV encoder (inverse of the decoder; used by the synthetic generator) ----

/// Write `len` bits (MSB first) of `val` at 1-indexed `start` in the 128-bit word.
/// Signed values are two's-complement (the low `len` bits of `val as u64`).
fn set_word_bits(bits: &mut [u8; 128], start: usize, len: usize, val: i64) {
    let u = val as u64;
    for (i, b) in bits[start - 1..start - 1 + len].iter_mut().enumerate() {
        *b = ((u >> (len - 1 - i)) & 1) as u8;
    }
}

/// Encode `eph` into the 128-bit I/NAV word of the given type — the exact inverse
/// of [`decode_ephemeris_word`], same ICD offsets and scales.
pub fn encode_ephemeris_word(eph: &Ephemeris, word_type: u8) -> InavWord {
    let mut bits = [0u8; 128];
    let r = |x: f64| x.round() as i64;
    set_word_bits(&mut bits, 1, 6, word_type as i64);
    match word_type {
        1 => {
            set_word_bits(&mut bits, 7, 10, eph.iode as i64);
            set_word_bits(&mut bits, 17, 14, (eph.toe / 60) as i64);
            set_word_bits(&mut bits, 31, 32, r(eph.m0 / (P2_31 * SC2RAD)));
            set_word_bits(&mut bits, 63, 32, r(eph.ecc / P2_33));
            set_word_bits(&mut bits, 95, 32, r(eph.a.sqrt() / P2_19));
        }
        2 => {
            set_word_bits(&mut bits, 7, 10, eph.iode as i64);
            set_word_bits(&mut bits, 17, 32, r(eph.omg0 / (P2_31 * SC2RAD)));
            set_word_bits(&mut bits, 49, 32, r(eph.i0 / (P2_31 * SC2RAD)));
            set_word_bits(&mut bits, 81, 32, r(eph.omg / (P2_31 * SC2RAD)));
            set_word_bits(&mut bits, 113, 14, r(eph.i_dot / (P2_43 * SC2RAD)));
        }
        3 => {
            set_word_bits(&mut bits, 7, 10, eph.iode as i64);
            set_word_bits(&mut bits, 17, 24, r(eph.omg_dot / (P2_43 * SC2RAD)));
            set_word_bits(&mut bits, 41, 16, r(eph.deln / (P2_43 * SC2RAD)));
            set_word_bits(&mut bits, 57, 16, r(eph.cuc / P2_29));
            set_word_bits(&mut bits, 73, 16, r(eph.cus / P2_29));
            set_word_bits(&mut bits, 89, 16, r(eph.crc / P2_5));
            set_word_bits(&mut bits, 105, 16, r(eph.crs / P2_5));
        }
        4 => {
            set_word_bits(&mut bits, 7, 10, eph.iode as i64);
            set_word_bits(&mut bits, 23, 16, r(eph.cic / P2_29));
            set_word_bits(&mut bits, 39, 16, r(eph.cis / P2_29));
            set_word_bits(&mut bits, 55, 14, (eph.toc / 60) as i64);
            set_word_bits(&mut bits, 69, 31, r(eph.f0 / P2_34));
            set_word_bits(&mut bits, 100, 21, r(eph.f1 / P2_46));
            set_word_bits(&mut bits, 121, 6, r(eph.f2 / P2_59));
        }
        5 => {
            set_word_bits(&mut bits, 48, 10, r(eph.tgd / P2_32));
            set_word_bits(&mut bits, 74, 12, eph.week as i64);
            set_word_bits(&mut bits, 86, 20, eph.tow as i64);
        }
        _ => {}
    }
    InavWord { word_type, bits }
}

/// Render one CRC-valid I/NAV page carrying `word` to its 500-symbol stream (the
/// even + odd page parts, each `preamble + interleave(conv_encode(frame+tail))`).
/// Inverse of `InavDecoder::push_symbol` → `assemble`.
pub fn encode_inav_page(word: &InavWord) -> Vec<u8> {
    let mut page = [0u8; 228];
    page[2..114].copy_from_slice(&word.bits[..112]); // Data k → even part
    page[114] = 1; // odd page-part flag
    page[116..132].copy_from_slice(&word.bits[112..]); // Data j → odd part
    let crc = crc24q(&page[..196]);
    for (i, p) in page[196..220].iter_mut().enumerate() {
        *p = ((crc >> (23 - i)) & 1) as u8;
    }
    let render = |frame: &[u8]| -> Vec<u8> {
        let mut b = frame.to_vec();
        b.extend([0u8; 6]); // convolutional tail
        let mut part = PREAMBLE.to_vec();
        part.extend(interleave(&conv_encode(&b, G2_INVERTED)));
        part
    };
    let mut syms = render(&page[..114]);
    syms.extend(render(&page[114..]));
    syms
}

/// The I/NAV symbol stream (one symbol per 4 ms code period) broadcasting `eph` as
/// word types 1-5, repeated `reps` times — for the synthetic E1 generator.
pub fn encode_inav_stream(eph: &Ephemeris, reps: usize) -> Vec<u8> {
    let mut syms = Vec::new();
    for _ in 0..reps {
        for wt in 1..=5u8 {
            syms.extend(encode_inav_page(&encode_ephemeris_word(eph, wt)));
        }
    }
    syms
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_bits(n: usize) -> Vec<u8> {
        (0..n).map(|i| ((i * 37 + 11) % 7 < 3) as u8).collect()
    }

    #[test]
    fn fec_roundtrip_corrects_symbol_errors() {
        // 114 data + 6 zero tail -> conv encode -> interleave == 240 symbols.
        let data = sample_bits(114);
        let mut padded = data.clone();
        padded.extend([0u8; 6]);
        let coded = conv_encode(&padded, G2_INVERTED);
        assert_eq!(coded.len(), 240);

        let mut rx = interleave(&coded);
        // a few channel symbol errors the rate-1/2 code should fix.
        rx[3] ^= 1;
        rx[57] ^= 1;
        rx[199] ^= 1;

        let decoded = viterbi_decode(&deinterleave(&rx), padded.len());
        assert_eq!(&decoded[..114], &data[..], "Viterbi must recover the data");
    }

    #[test]
    fn interleave_is_invertible() {
        let x = sample_bits(240);
        assert_eq!(deinterleave(&interleave(&x)), x);
        // it actually permutes (not identity).
        assert_ne!(interleave(&x), x);
    }

    // Build an even+odd page pair carrying a known 128-bit word with a valid CRC,
    // render them to symbols (preamble + FEC + interleave), and check the streaming
    // decoder recovers the word -- under every carrier-recovery ambiguity: both
    // polarities (Costas 180°) and with/without the half-rate (−1)ⁿ false-lock flip.
    #[test]
    fn inav_decoder_recovers_a_crc_valid_word() {
        let mut word = [0u8; 128];
        for (k, w) in word[..6].iter_mut().enumerate() {
            *w = (4u8 >> (5 - k)) & 1; // word type 4
        }
        for (k, w) in word.iter_mut().enumerate().skip(6) {
            *w = ((k * 5 + 1) % 3 == 0) as u8;
        }

        // 228-bit page: even[0..114] then odd[0..114].
        let mut page = vec![0u8; 228];
        page[0] = 0; // even/odd = even
        page[2..114].copy_from_slice(&word[..112]); // Data k (1/2)
        page[114] = 1; // even/odd = odd
        page[116..132].copy_from_slice(&word[112..]); // Data j (2/2)
        for (k, p) in page.iter_mut().enumerate().take(196).skip(132) {
            *p = ((k * 3) % 2) as u8; // reserved/SAR/spare filler
        }
        let crc = crc24q(&page[..196]);
        for k in 0..24 {
            page[196 + k] = ((crc >> (23 - k)) & 1) as u8;
        }

        // Render a 114-bit frame to a 250-symbol page part.
        let make_part = |frame: &[u8]| -> Vec<u8> {
            let mut bits = frame.to_vec();
            bits.extend([0u8; 6]); // convolutional tail
            let mut part = PREAMBLE.to_vec();
            part.extend(interleave(&conv_encode(&bits, G2_INVERTED)));
            part
        };
        let even = make_part(&page[..114]);
        let odd = make_part(&page[114..]);

        for invert in [false, true] {
            for half_rate in [false, true] {
                let mut dec = InavDecoder::new();
                let mut got = None;
                // half_rate multiplies the symbol stream by (−1)ⁿ over the absolute
                // symbol index, the Costas half-symbol-rate false lock the decoder
                // must transparently undo.
                for (k, &s) in even.iter().chain(odd.iter()).enumerate() {
                    let flip = half_rate as u8 & (k & 1) as u8;
                    if let Some(w) = dec.push_symbol(s ^ invert as u8 ^ flip) {
                        got = Some(w);
                    }
                }
                let w = got.expect("a CRC-valid word");
                assert_eq!(w.word_type, 4, "invert={invert} half_rate={half_rate}");
                assert_eq!(w.bits, word, "invert={invert} half_rate={half_rate}");
            }
        }
    }

    // Pack `val` into one-bit-per-byte `bits` (MSB first) at 0-indexed `pos`.
    fn set_ubits(bits: &mut [u8], pos: usize, len: usize, val: u32) {
        for (i, b) in bits[pos..pos + len].iter_mut().enumerate() {
            *b = ((val >> (len - 1 - i)) & 1) as u8;
        }
    }

    // Build an I/NAV word of `word_type` with fields at their 1-indexed {start,len}.
    fn make_word(word_type: u8, fields: &[(usize, usize, u32)]) -> InavWord {
        let mut bits = [0u8; 128];
        set_ubits(&mut bits, 0, 6, word_type as u32);
        for &(start, len, val) in fields {
            set_ubits(&mut bits, start - 1, len, val);
        }
        InavWord { word_type, bits }
    }

    // Feed canned word types 1-5 (raw fields -> realistic Galileo MEO ephemeris)
    // through the extractor and lock the bit offsets, scales, signedness, and the
    // is_valid() gate.
    #[test]
    fn decodes_inav_words_into_a_valid_ephemeris() {
        let mut eph = Ephemeris::default();

        let e_raw = 2_000_000u32; // ecc = e_raw * 2^-33 ~ 2.3e-4
        let sqrta_raw = 2_852_500_000u32; // a ~ 29.6e6 m (Galileo MEO)
        let i0_raw = 668_504_000u32; // i0 ~ 0.978 rad (~56 deg)
        let omgdot_raw = (-2000_i32 as u32) & 0x00FF_FFFF; // negative, 24-bit

        // Word 1: IODnav, t0e (60 s LSB), M0, e, sqrtA.
        decode_ephemeris_word(
            &mut eph,
            &make_word(
                1,
                &[
                    (7, 10, 100),
                    (17, 14, 600), // -> toe 36000 s
                    (31, 32, 0x1000_0000),
                    (63, 32, e_raw),
                    (95, 32, sqrta_raw),
                ],
            ),
        );
        // Word 2: Omega0, i0, omega, iDOT.
        decode_ephemeris_word(
            &mut eph,
            &make_word(
                2,
                &[
                    (17, 32, 0x2000_0000),
                    (49, 32, i0_raw),
                    (81, 32, 0x0800_0000),
                    (113, 14, 8),
                ],
            ),
        );
        // Word 3: OmegaDot (signed, negative), deltaN, Cuc/Cus/Crc/Crs.
        decode_ephemeris_word(
            &mut eph,
            &make_word(
                3,
                &[
                    (17, 24, omgdot_raw),
                    (41, 16, 300),
                    (57, 16, 100),
                    (73, 16, 100),
                    (89, 16, 200),
                    (105, 16, 200),
                ],
            ),
        );
        // Word 4: Cic, Cis, t0c (60 s LSB), af0, af1, af2.
        decode_ephemeris_word(
            &mut eph,
            &make_word(
                4,
                &[
                    (23, 16, 50),
                    (39, 16, 50),
                    (55, 14, 600), // -> toc 36000 s
                    (69, 31, 1000),
                    (100, 21, 10),
                    (121, 6, 1),
                ],
            ),
        );
        // Word 5: BGD(E1,E5a), GST week, GST TOW.
        decode_ephemeris_word(
            &mut eph,
            &make_word(5, &[(48, 10, 20), (74, 12, 1300), (86, 20, 36_000)]),
        );

        // The channel timestamps the first word in; emulate so is_valid() passes.
        eph.ts_sec = 1.0;
        assert!(
            eph.is_valid(),
            "word types 1-5 must yield a valid ephemeris"
        );

        // Offsets + 60 s LSB:
        assert_eq!(eph.iode, 100);
        assert_eq!(eph.toe, 36_000);
        assert_eq!(eph.toc, 36_000);
        assert_eq!(eph.week, 1300); // GST week, from word 5 only
        assert_eq!(eph.tow, 36_000);
        // Scales:
        let sqrt_a = sqrta_raw as f64 * P2_19;
        assert!((eph.a - sqrt_a * sqrt_a).abs() < 1.0);
        assert!((29.0e6..30.0e6).contains(&eph.a), "a={}", eph.a);
        assert!((eph.ecc - e_raw as f64 * P2_33).abs() < 1e-18);
        assert!((eph.i0 - i0_raw as f64 * P2_31 * SC2RAD).abs() < 1e-12);
        assert!((eph.f0 - 1000.0 * P2_34).abs() < 1e-18);
        // Signedness: OmegaDot decoded as negative.
        assert!(eph.omg_dot < 0.0, "omg_dot={}", eph.omg_dot);
    }

    // A plausible Galileo MEO ephemeris, used by the encoder round-trip.
    fn sample_eph() -> Ephemeris {
        use gnss_rs::{constellation::Constellation, sv::SV};
        let mut e = Ephemeris::new(SV::new(Constellation::Galileo, 1));
        e.iode = 42;
        e.toe = 36_000;
        e.toc = 36_000;
        e.week = 1234;
        e.tow = 36_030;
        e.a = 29_600_000.0;
        e.ecc = 1.5e-4;
        e.i0 = 0.96;
        e.m0 = 0.5;
        e.omg0 = 0.3;
        e.omg = -1.0;
        e.omg_dot = -5.0e-9;
        e.deln = 2.6e-9;
        e.cuc = 1.0e-6;
        e.cus = 2.0e-6;
        e.crc = 110.0;
        e.crs = 50.0;
        e.cic = 1.0e-7;
        e.cis = -1.0e-7;
        e.i_dot = 1.0e-10;
        e.f0 = 1.0e-4;
        e.f1 = 1.0e-12;
        e.f2 = 0.0;
        e.tgd = 5.0e-9;
        e
    }

    // Encode an ephemeris to the I/NAV symbol stream, push it through the *real*
    // streaming decoder, and check the fields come back within their quantization.
    #[test]
    fn inav_encode_decode_roundtrips_an_ephemeris() {
        let eph = sample_eph();
        let syms = encode_inav_stream(&eph, 2);
        let mut dec = InavDecoder::new();
        let mut got = Ephemeris::default();
        let mut words = 0;
        for &s in &syms {
            if let Some(w) = dec.push_symbol(s) {
                decode_ephemeris_word(&mut got, &w);
                words += 1;
            }
        }
        assert_eq!(words, 10, "two reps of word types 1-5 must decode");
        got.ts_sec = 1.0;
        assert!(got.is_valid(), "round-tripped ephemeris must be valid");
        assert_eq!(got.week, eph.week);
        assert_eq!(got.toe, eph.toe);
        assert_eq!(got.toc, eph.toc);
        assert_eq!(got.tow, eph.tow);
        assert!((got.a - eph.a).abs() < 1.0, "a {} vs {}", got.a, eph.a);
        assert!((got.ecc - eph.ecc).abs() < 1e-9);
        assert!((got.i0 - eph.i0).abs() < 1e-9);
        assert!((got.m0 - eph.m0).abs() < 1e-9);
        assert!((got.omg0 - eph.omg0).abs() < 1e-9);
        assert!((got.omg_dot - eph.omg_dot).abs() < 5e-13); // > 1 LSB (P2_43·SC2RAD)
        assert!((got.crc - eph.crc).abs() < 0.1);
        assert!((got.f0 - eph.f0).abs() < 1e-9);
    }
}
