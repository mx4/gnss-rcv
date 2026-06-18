//! Galileo E5a-I F/NAV decoder — transport + demodulation layer.
//!
//! Each F/NAV page is 500 symbols at 50 sym/s (10 s): a 12-symbol unencoded
//! synchronisation pattern then 488 FEC symbols. The 488 symbols are
//! **block-interleaved** (61 columns × 8 rows) and **rate-1/2 convolutionally
//! encoded** (K=7, G1=171₈, G2=133₈ with G2 inverted — the same code as I/NAV),
//! carrying 244 bits: the 238-bit F/NAV word (a 6-bit page type, 208-bit nav
//! data, and a **CRC-24Q** over those 214 bits) plus 6 zero tail bits. Each
//! page is self-contained (no even/odd split). The shared convolutional code,
//! Viterbi decoder and CRC live in [`crate::fec`] / [`crate::galileo_inav`]; this
//! module adds the F/NAV-specific interleaver, page codec and streaming
//! sync-finder. The page-type → ephemeris field extraction is a later step.
//!
//! Source: Galileo OS SIS ICD Issue 2.1, §4.1.4 / §4.2.

use crate::fec::{conv_encode, crc24q};
use crate::galileo_inav::viterbi_decode;

/// Galileo inverts the G2 output symbol of the shared K=7 convolutional code
/// (matches `viterbi_decode`'s built-in convention).
const G2_INVERTED: u8 = 1;

/// F/NAV page synchronisation pattern (12 symbols, transmitted *unencoded*).
pub const FNAV_SYNC: [u8; 12] = [1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0];
/// Symbols in a whole F/NAV page (sync + FEC), one per 20 ms → 10 s.
pub const FNAV_PAGE_SYMBOLS: usize = 500;
/// FEC symbols after the sync pattern (= interleave(conv_encode(244 bits))).
const FNAV_DATA_SYMBOLS: usize = 488;
const FNAV_TYPE_BITS: usize = 6;
const FNAV_DATA_BITS: usize = 208;
/// CRC-protected body: page type + nav data (the CRC is computed over these).
const FNAV_BODY_BITS: usize = FNAV_TYPE_BITS + FNAV_DATA_BITS; // 214
/// Full F/NAV word: body + CRC-24.
const FNAV_WORD_BITS: usize = FNAV_BODY_BITS + 24; // 238
const FNAV_TAIL_BITS: usize = 6;

/// F/NAV block interleaver: 61 columns × 8 rows (ICD Table 25). Write the 488
/// input symbols column-by-column, read row-by-row — the I/NAV 30×8 convention
/// at F/NAV's dimensions.
fn interleave(input: &[u8]) -> Vec<u8> {
    assert_eq!(input.len(), FNAV_DATA_SYMBOLS);
    let mut out = vec![0u8; FNAV_DATA_SYMBOLS];
    for (i, &v) in input.iter().enumerate() {
        out[(i % 8) * 61 + i / 8] = v;
    }
    out
}

/// Inverse of [`interleave`].
fn deinterleave(input: &[u8]) -> Vec<u8> {
    assert_eq!(input.len(), FNAV_DATA_SYMBOLS);
    let mut out = vec![0u8; FNAV_DATA_SYMBOLS];
    for (j, &v) in input.iter().enumerate() {
        out[(j % 61) * 8 + j / 61] = v;
    }
    out
}

/// A decoded, CRC-valid F/NAV page: its 6-bit page type and 208 nav-data bits
/// (MSB first, one bit per byte). The page-type layouts that turn `data` into an
/// ephemeris are decoded in a later step.
#[derive(Clone)]
pub struct FnavWord {
    pub page_type: u8,
    pub data: [u8; FNAV_DATA_BITS],
}

fn bits_to_u8(bits: &[u8]) -> u8 {
    bits.iter().fold(0u8, |acc, &b| (acc << 1) | (b & 1))
}

/// Encode one F/NAV page into its 500 symbols: build the 214-bit body (page type
/// then data), append CRC-24Q over it and 6 zero tail bits, convolutionally
/// encode, interleave, and prepend the (unencoded) sync pattern. For the
/// round-trip test and the future E5a synth.
pub fn encode_fnav_page(page_type: u8, data: &[u8]) -> Vec<u8> {
    assert_eq!(data.len(), FNAV_DATA_BITS);
    let mut body = Vec::with_capacity(FNAV_BODY_BITS);
    for k in (0..FNAV_TYPE_BITS).rev() {
        body.push((page_type >> k) & 1);
    }
    body.extend_from_slice(data);
    let crc = crc24q(&body);
    let mut bits = body; // becomes the 244-bit FEC input
    for k in (0..24).rev() {
        bits.push(((crc >> k) & 1) as u8); // CRC, MSB first -> 238
    }
    bits.extend(std::iter::repeat_n(0u8, FNAV_TAIL_BITS)); // zero tail -> 244
    debug_assert_eq!(bits.len(), FNAV_WORD_BITS + FNAV_TAIL_BITS);

    let mut page = FNAV_SYNC.to_vec();
    page.extend(interleave(&conv_encode(&bits, G2_INVERTED)));
    debug_assert_eq!(page.len(), FNAV_PAGE_SYMBOLS);
    page
}

/// Decode one 488-symbol F/NAV data block (the page *after* its sync pattern):
/// de-interleave, Viterbi-decode to 244 bits, and CRC-validate the 238-bit word.
/// `None` if the CRC fails.
pub fn decode_fnav_page(data_syms: &[u8]) -> Option<FnavWord> {
    assert_eq!(data_syms.len(), FNAV_DATA_SYMBOLS);
    let bits = viterbi_decode(&deinterleave(data_syms), FNAV_WORD_BITS + FNAV_TAIL_BITS);
    let word = &bits[..FNAV_WORD_BITS]; // 238 = body + CRC; valid ⇒ remainder 0
    if crc24q(word) != 0 {
        return None;
    }
    let mut data = [0u8; FNAV_DATA_BITS];
    data.copy_from_slice(&word[FNAV_TYPE_BITS..FNAV_BODY_BITS]);
    Some(FnavWord {
        page_type: bits_to_u8(&word[..FNAV_TYPE_BITS]),
        data,
    })
}

/// Match the 12-symbol front against the F/NAV sync pattern. `Some(0)` upright,
/// `Some(1)` carrier-inverted (the Costas 180° ambiguity flips every symbol),
/// `None` otherwise.
fn match_sync(front: &[u8]) -> Option<u8> {
    if front == FNAV_SYNC {
        Some(0)
    } else if front.iter().zip(FNAV_SYNC).all(|(&a, b)| a != b) {
        Some(1)
    } else {
        None
    }
}

/// Streaming F/NAV page decoder. Fed one symbol per F/NAV symbol period (the sign
/// of prompt-I on E5a-I, 50 sym/s), it slides to the sync pattern, then decodes
/// each 500-symbol page and emits the CRC-valid word. The carrier 180° ambiguity
/// is resolved at sync (the whole page is flipped if inverted); the half-rate
/// (−1)ⁿ Costas false lock (cf. I/NAV) is left to real-signal validation — the
/// E5a-Q pilot avoids it entirely.
#[derive(Default)]
pub struct FnavDecoder {
    buf: Vec<u8>,
}

impl FnavDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one F/NAV symbol (0/1); returns a word once a page passes CRC.
    pub fn push_symbol(&mut self, sym: u8) -> Option<FnavWord> {
        self.buf.push(sym & 1);
        if self.buf.len() < FNAV_PAGE_SYMBOLS {
            return None;
        }
        let Some(pol) = match_sync(&self.buf[..FNAV_SYNC.len()]) else {
            self.buf.remove(0); // not a page boundary: slide one and retry
            return None;
        };
        let page: Vec<u8> = self.buf.drain(..FNAV_PAGE_SYMBOLS).collect();
        let mut data_syms = page[FNAV_SYNC.len()..].to_vec();
        if pol == 1 {
            for s in data_syms.iter_mut() {
                *s ^= 1; // undo the carrier inversion before FEC
            }
        }
        decode_fnav_page(&data_syms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_bits(n: usize, seed: u64) -> Vec<u8> {
        // Cheap deterministic pseudo-random bits.
        let mut x = seed;
        (0..n)
            .map(|_| {
                x = x
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((x >> 33) & 1) as u8
            })
            .collect()
    }

    #[test]
    fn interleave_is_invertible() {
        let x = sample_bits(FNAV_DATA_SYMBOLS, 1);
        assert_eq!(deinterleave(&interleave(&x)), x);
        assert_ne!(interleave(&x), x); // it actually permutes
    }

    #[test]
    fn page_round_trips_with_crc() {
        for (pt, seed) in [(1u8, 11), (4, 22), (63, 33)] {
            let data = sample_bits(FNAV_DATA_BITS, seed);
            let page = encode_fnav_page(pt, &data);
            assert_eq!(page.len(), FNAV_PAGE_SYMBOLS);
            assert_eq!(&page[..FNAV_SYNC.len()], &FNAV_SYNC);
            let word = decode_fnav_page(&page[FNAV_SYNC.len()..]).expect("CRC valid");
            assert_eq!(word.page_type, pt);
            assert_eq!(word.data.as_slice(), data.as_slice());
        }
    }

    #[test]
    fn corruption_fails_crc() {
        let data = sample_bits(FNAV_DATA_BITS, 7);
        let mut page = encode_fnav_page(2, &data);
        // Flip a long burst — well past the rate-1/2 K=7 code's correction power,
        // so the CRC must catch it (a single flip the Viterbi would just fix).
        for s in page[50..130].iter_mut() {
            *s ^= 1;
        }
        assert!(decode_fnav_page(&page[FNAV_SYNC.len()..]).is_none());
    }

    #[test]
    fn decoder_streams_pages_both_polarities() {
        // Two pages back to back, the stream prefixed with junk and the whole
        // thing carrier-inverted, so the decoder must find sync mid-buffer and
        // undo the 180° flip.
        let d1 = sample_bits(FNAV_DATA_BITS, 101);
        let d2 = sample_bits(FNAV_DATA_BITS, 202);
        let mut stream = vec![1u8, 1, 0, 1, 0]; // junk before the first sync
        stream.extend(encode_fnav_page(1, &d1));
        stream.extend(encode_fnav_page(4, &d2));
        for s in stream.iter_mut() {
            *s ^= 1; // invert the entire stream (carrier 180°)
        }

        let mut dec = FnavDecoder::new();
        let mut got = Vec::new();
        for s in stream {
            if let Some(w) = dec.push_symbol(s) {
                got.push((w.page_type, w.data));
            }
        }
        assert_eq!(got.len(), 2, "should recover both pages");
        assert_eq!(got[0].0, 1);
        assert_eq!(got[0].1.as_slice(), d1.as_slice());
        assert_eq!(got[1].0, 4);
        assert_eq!(got[1].1.as_slice(), d2.as_slice());
    }
}
