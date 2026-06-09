//! Galileo E1-B I/NAV forward-error-correction layer.
//!
//! Each I/NAV page part is 250 symbols at 250 sym/s: a 10-symbol sync pattern
//! then 240 data symbols. The 240 symbols are **block-interleaved** (30 columns ×
//! 8 rows) and **rate-1/2 convolutionally encoded** (K=7, G1=171₈, G2=133₈ with
//! G2 inverted), carrying 120 bits (114 data + 6 tail). A **CRC-24Q** protects the
//! assembled page. This module is those primitives -- the de-interleaver, the
//! Viterbi decoder, and the CRC -- with the page/word assembly and the ephemeris
//! field extraction still to come on top.

/// Galileo I/NAV convolutional code, K=7, rate 1/2.
const G1: u32 = 0o171; // 1111001
const G2: u32 = 0o133; // 1011011
/// Galileo inverts the G2 output symbol.
const G2_INVERTED: u8 = 1;

fn parity(x: u32) -> u8 {
    (x.count_ones() & 1) as u8
}

/// Convolutionally encode `bits` (rate 1/2). The register starts at 0; to
/// terminate the trellis the caller appends 6 zero tail bits. Output is two
/// symbols per input bit (G1 then the inverted G2).
pub fn conv_encode(bits: &[u8]) -> Vec<u8> {
    let mut state: u32 = 0; // 6 history bits in positions 5..0
    let mut out = Vec::with_capacity(bits.len() * 2);
    for &b in bits {
        let reg = ((b as u32 & 1) << 6) | state; // 7-bit window: current bit + history
        out.push(parity(reg & G1));
        out.push(parity(reg & G2) ^ G2_INVERTED);
        state = reg >> 1;
    }
    out
}

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

/// CRC-24Q (poly 0x1864CFB, init 0), MSB-first over a bit slice -- the check
/// Galileo I/NAV (and GPS L2C/L5, RTCM) use. CRC of a message with its own CRC
/// appended is 0.
pub fn crc24q(bits: &[u8]) -> u32 {
    const POLY: u32 = 0x0086_4CFB; // 0x1864CFB without the implicit x^24 term
    let mut crc: u32 = 0;
    for &b in bits {
        let feedback = ((crc >> 23) & 1) ^ (b as u32 & 1);
        crc = (crc << 1) & 0xFF_FFFF;
        if feedback != 0 {
            crc ^= POLY;
        }
    }
    crc
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
    pending_even: Option<Vec<u8>>, // the 114-bit even page-part frame, awaiting its odd
}

impl InavDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one I/NAV symbol (0/1); returns a word once an even+odd page passes CRC.
    pub fn push_symbol(&mut self, sym: u8) -> Option<InavWord> {
        self.buf.push(sym & 1);
        if self.buf.len() < PAGE_PART_SYMBOLS {
            return None;
        }
        // The front must be a preamble (normal or carrier-inverted); else slide one.
        let pol = match preamble_polarity(&self.buf[..PREAMBLE.len()]) {
            Some(p) => p,
            None => {
                self.buf.remove(0);
                return None;
            }
        };
        let part: Vec<u8> = self.buf.drain(..PAGE_PART_SYMBOLS).collect();
        self.assemble(decode_page_part(&part, pol))
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

fn preamble_polarity(syms: &[u8]) -> Option<u8> {
    if syms.iter().zip(PREAMBLE).all(|(&a, b)| a == b) {
        Some(0)
    } else if syms.iter().zip(PREAMBLE).all(|(&a, b)| a ^ 1 == b) {
        Some(1)
    } else {
        None
    }
}

/// Strip the preamble, undo the carrier polarity, de-interleave and Viterbi-decode
/// one 250-symbol page part into its 114-bit frame (120 decoded bits less 6 tail).
fn decode_page_part(part: &[u8], pol: u8) -> Vec<u8> {
    let encoded: Vec<u8> = part[PREAMBLE.len()..].iter().map(|&s| s ^ pol).collect();
    let mut bits = viterbi_decode(&deinterleave(&encoded), 120);
    bits.truncate(114);
    bits
}

fn bits_to_u32(bits: &[u8]) -> u32 {
    bits.iter()
        .fold(0u32, |acc, &b| (acc << 1) | (b & 1) as u32)
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
        let coded = conv_encode(&padded);
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

    #[test]
    fn crc24q_is_self_consistent() {
        let data = sample_bits(196);
        let crc = crc24q(&data);
        let mut full = data.clone();
        for k in (0..24).rev() {
            full.push(((crc >> k) & 1) as u8);
        }
        // CRC over (message || CRC) is zero for CRC-24Q.
        assert_eq!(crc24q(&full), 0);
        // a single flipped bit changes the CRC.
        let mut bad = data.clone();
        bad[0] ^= 1;
        assert_ne!(crc24q(&bad), crc);
    }

    // Build an even+odd page pair carrying a known 128-bit word with a valid CRC,
    // render them to symbols (preamble + FEC + interleave), and check the streaming
    // decoder recovers the word -- at both carrier polarities.
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
            part.extend(interleave(&conv_encode(&bits)));
            part
        };
        let even = make_part(&page[..114]);
        let odd = make_part(&page[114..]);

        for invert in [false, true] {
            let mut dec = InavDecoder::new();
            let mut got = None;
            for &s in even.iter().chain(odd.iter()) {
                if let Some(w) = dec.push_symbol(s ^ invert as u8) {
                    got = Some(w);
                }
            }
            let w = got.expect("a CRC-valid word");
            assert_eq!(w.word_type, 4, "invert={invert}");
            assert_eq!(w.bits, word, "invert={invert}");
        }
    }
}
