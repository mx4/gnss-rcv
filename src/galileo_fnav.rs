//! Galileo E5a-I F/NAV decoder.
//!
//! Each F/NAV page is 500 symbols at 50 sym/s (10 s): a 12-symbol unencoded
//! synchronisation pattern then 488 FEC symbols. The 488 symbols are
//! **block-interleaved** (61 columns × 8 rows) and **rate-1/2 convolutionally
//! encoded** (K=7, G1=171₈, G2=133₈ with G2 inverted — the same code as I/NAV),
//! carrying 244 bits: the 238-bit F/NAV word (a 6-bit page type, 208-bit nav
//! data, and a **CRC-24Q** over those 214 bits) plus 6 zero tail bits. Each
//! page is self-contained (no even/odd split). The shared convolutional code,
//! Viterbi decoder and CRC live in [`crate::fec`] / [`crate::galileo_inav`]; this
//! module adds the F/NAV-specific interleaver, page codec, streaming
//! sync-finder, and the page-type → ephemeris field extraction.
//!
//! Source: Galileo OS SIS ICD Issue 2.1, §4.1.4 / §4.2.

use crate::constants::{
    P2_2, P2_5, P2_8, P2_15, P2_19, P2_29, P2_31, P2_32, P2_33, P2_34, P2_35, P2_43, P2_46, P2_51,
    P2_59, SC2RAD,
};
use crate::ephemeris::Ephemeris;
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
/// (MSB first, one bit per byte). [`decode_fnav_ephemeris`] turns the per-type
/// `data` layouts into an [`Ephemeris`].
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

// ---- F/NAV ephemeris extraction (page types 1-4) ----

fn ubits(bits: &[u8], pos: usize, len: usize) -> u32 {
    bits[pos..pos + len]
        .iter()
        .fold(0u32, |a, &b| (a << 1) | (b & 1) as u32)
}

/// Like [`ubits`] but two's-complement sign-extended.
fn sbits(bits: &[u8], pos: usize, len: usize) -> i32 {
    let shift = 32 - len;
    ((ubits(bits, pos, len) << shift) as i32) >> shift
}

/// F/NAV structural decode latency: the WN/TOW names the GST at the *start* of
/// the 10 s page carrying it, and [`FnavDecoder`] emits the word at the page's
/// last symbol — 10.000 s later. Anchored at the full latency like LNAV/I-NAV
/// (see [`crate::galileo_inav::INAV_DECODE_LATENCY_SEC`] for why an uncorrected
/// latency is a real t_tx error, not absorbed by the receiver clock).
pub const FNAV_DECODE_LATENCY_SEC: f64 = FNAV_PAGE_SYMBOLS as f64 * 20e-3; // 10.0 s

/// Fill `eph` from one CRC-valid F/NAV page. Page types 1-4 carry the clock,
/// ionosphere, BGD and GST (type 1), the Keplerian ephemeris 1/3..3/3 (types
/// 2-4) and the GST-GPS offset / GGTO (type 4). Galileo broadcasts the same
/// parameter set and scale factors as I/NAV — only the bit layout differs (ICD
/// §4.2, 1-indexed within the 208-bit nav-data field). Almanac pages 5/6 are
/// skipped. The inverse is [`encode_fnav_ephemeris`].
pub fn decode_fnav_ephemeris(eph: &mut Ephemeris, word: &FnavWord) {
    let d = &word.data[..];
    let u = |start: usize, len: usize| ubits(d, start - 1, len);
    let s = |start: usize, len: usize| sbits(d, start - 1, len) as f64;
    if (1..=4).contains(&word.page_type) {
        eph.eph_mask |= 1 << word.page_type; // types 1-4 → complete = 0b11110
    }
    match word.page_type {
        1 => {
            eph.iode = u(7, 10); // IODnav
            eph.toc = u(17, 14) * 60; // t0c, 60 s LSB
            eph.f0 = s(31, 31) * P2_34;
            eph.f1 = s(62, 21) * P2_46;
            eph.f2 = s(83, 6) * P2_59;
            eph.sva = u(89, 8); // SISA(E1,E5a)
            eph.ai0 = u(97, 11) as f64 * P2_2;
            eph.ai1 = s(108, 11) * P2_8;
            eph.ai2 = s(119, 14) * P2_15;
            eph.iono_storm = (u(133, 5) as u8).reverse_bits() >> 3; // bit 0 = Region 1
            eph.gal_iono_valid = true;
            eph.tgd = s(138, 10) * P2_32; // BGD(E1,E5a)
            eph.svh = u(148, 2); // E5aHS
            eph.week = u(150, 12); // GST week
            eph.tow = u(162, 20); // GST TOW [s]
        }
        2 => {
            eph.iode = u(1, 10);
            eph.m0 = s(11, 32) * P2_31 * SC2RAD;
            eph.omg_dot = s(43, 24) * P2_43 * SC2RAD;
            eph.ecc = u(67, 32) as f64 * P2_33;
            let sqrt_a = u(99, 32) as f64 * P2_19;
            eph.a = sqrt_a * sqrt_a;
            eph.omg0 = s(131, 32) * P2_31 * SC2RAD;
            eph.i_dot = s(163, 14) * P2_43 * SC2RAD;
            eph.week = u(177, 12);
            eph.tow = u(189, 20);
        }
        3 => {
            eph.iode = u(1, 10);
            eph.i0 = s(11, 32) * P2_31 * SC2RAD;
            eph.omg = s(43, 32) * P2_31 * SC2RAD;
            eph.deln = s(75, 16) * P2_43 * SC2RAD;
            eph.cuc = s(91, 16) * P2_29;
            eph.cus = s(107, 16) * P2_29;
            eph.crc = s(123, 16) * P2_5;
            eph.crs = s(139, 16) * P2_5;
            eph.toe = u(155, 14) * 60; // t0e, 60 s LSB
            eph.week = u(169, 12);
            eph.tow = u(181, 20);
        }
        4 => {
            eph.iode = u(1, 10);
            eph.cic = s(11, 16) * P2_29;
            eph.cis = s(27, 16) * P2_29;
            // GST-UTC conversion (A0/A1/leap seconds, bits 43-141) is not used
            // for positioning and is skipped; the GST-GPS offset (GGTO) is.
            eph.t0g = u(142, 8) * 3600;
            eph.a0g = s(150, 16) * P2_35;
            eph.a1g = s(166, 12) * P2_51;
            eph.wn0g = u(178, 6);
            eph.ggto_valid = true;
            eph.tow = u(184, 20);
        }
        _ => {}
    }
}

/// Write `len` bits (MSB first) of `val` at 1-indexed `start` in the 208-bit
/// nav-data field. Signed values are two's-complement (low `len` bits).
fn set_data_bits(data: &mut [u8; FNAV_DATA_BITS], start: usize, len: usize, val: i64) {
    let u = val as u64;
    for (i, b) in data[start - 1..start - 1 + len].iter_mut().enumerate() {
        *b = ((u >> (len - 1 - i)) & 1) as u8;
    }
}

/// Encode `eph` into the 208-bit nav-data field of F/NAV page type 1-4 — the
/// exact inverse of [`decode_fnav_ephemeris`], same ICD offsets and scales. For
/// the round-trip test and the future E5a synth. (UTC-conversion and almanac
/// fields this receiver does not use are left zero.)
pub fn encode_fnav_ephemeris(eph: &Ephemeris, page_type: u8) -> [u8; FNAV_DATA_BITS] {
    let mut d = [0u8; FNAV_DATA_BITS];
    let r = |x: f64| x.round() as i64;
    match page_type {
        1 => {
            set_data_bits(&mut d, 1, 6, eph.sv.prn as i64); // SVID
            set_data_bits(&mut d, 7, 10, eph.iode as i64);
            set_data_bits(&mut d, 17, 14, (eph.toc / 60) as i64);
            set_data_bits(&mut d, 31, 31, r(eph.f0 / P2_34));
            set_data_bits(&mut d, 62, 21, r(eph.f1 / P2_46));
            set_data_bits(&mut d, 83, 6, r(eph.f2 / P2_59));
            set_data_bits(&mut d, 89, 8, eph.sva as i64);
            set_data_bits(&mut d, 97, 11, r(eph.ai0 / P2_2));
            set_data_bits(&mut d, 108, 11, r(eph.ai1 / P2_8));
            set_data_bits(&mut d, 119, 14, r(eph.ai2 / P2_15));
            set_data_bits(
                &mut d,
                133,
                5,
                ((eph.iono_storm << 3).reverse_bits()) as i64,
            );
            set_data_bits(&mut d, 138, 10, r(eph.tgd / P2_32));
            set_data_bits(&mut d, 148, 2, eph.svh as i64);
            set_data_bits(&mut d, 150, 12, eph.week as i64);
            set_data_bits(&mut d, 162, 20, eph.tow as i64);
        }
        2 => {
            set_data_bits(&mut d, 1, 10, eph.iode as i64);
            set_data_bits(&mut d, 11, 32, r(eph.m0 / (P2_31 * SC2RAD)));
            set_data_bits(&mut d, 43, 24, r(eph.omg_dot / (P2_43 * SC2RAD)));
            set_data_bits(&mut d, 67, 32, r(eph.ecc / P2_33));
            set_data_bits(&mut d, 99, 32, r(eph.a.sqrt() / P2_19));
            set_data_bits(&mut d, 131, 32, r(eph.omg0 / (P2_31 * SC2RAD)));
            set_data_bits(&mut d, 163, 14, r(eph.i_dot / (P2_43 * SC2RAD)));
            set_data_bits(&mut d, 177, 12, eph.week as i64);
            set_data_bits(&mut d, 189, 20, eph.tow as i64);
        }
        3 => {
            set_data_bits(&mut d, 1, 10, eph.iode as i64);
            set_data_bits(&mut d, 11, 32, r(eph.i0 / (P2_31 * SC2RAD)));
            set_data_bits(&mut d, 43, 32, r(eph.omg / (P2_31 * SC2RAD)));
            set_data_bits(&mut d, 75, 16, r(eph.deln / (P2_43 * SC2RAD)));
            set_data_bits(&mut d, 91, 16, r(eph.cuc / P2_29));
            set_data_bits(&mut d, 107, 16, r(eph.cus / P2_29));
            set_data_bits(&mut d, 123, 16, r(eph.crc / P2_5));
            set_data_bits(&mut d, 139, 16, r(eph.crs / P2_5));
            set_data_bits(&mut d, 155, 14, (eph.toe / 60) as i64);
            set_data_bits(&mut d, 169, 12, eph.week as i64);
            set_data_bits(&mut d, 181, 20, eph.tow as i64);
        }
        4 => {
            set_data_bits(&mut d, 1, 10, eph.iode as i64);
            set_data_bits(&mut d, 11, 16, r(eph.cic / P2_29));
            set_data_bits(&mut d, 27, 16, r(eph.cis / P2_29));
            set_data_bits(&mut d, 142, 8, (eph.t0g / 3600) as i64);
            set_data_bits(&mut d, 150, 16, r(eph.a0g / P2_35));
            set_data_bits(&mut d, 166, 12, r(eph.a1g / P2_51));
            set_data_bits(&mut d, 178, 6, eph.wn0g as i64);
            set_data_bits(&mut d, 184, 20, eph.tow as i64);
        }
        _ => {}
    }
    d
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

    #[test]
    fn ephemeris_round_trips_through_pages_1_to_4() {
        use gnss_rs::constellation::Constellation;
        use gnss_rs::sv::SV;

        // A representative quantised Galileo ephemeris (Keplerian + clock + iono
        // + BGD + GGTO); every value is within its field's range.
        let mut eph = Ephemeris::new(SV::new(Constellation::Galileo, 11));
        eph.iode = 42;
        eph.toe = 36_000;
        eph.toc = 36_000;
        eph.week = 1300;
        eph.tow = 35_990;
        eph.m0 = 0.30;
        eph.ecc = 2.0e-4;
        eph.a = 29_600_000.0;
        eph.omg0 = -1.20;
        eph.i0 = 0.96;
        eph.omg = 0.50;
        eph.omg_dot = -5.0e-9;
        eph.i_dot = 1.0e-10;
        eph.deln = 3.0e-9;
        (eph.cuc, eph.cus) = (1.0e-6, 2.0e-6);
        (eph.crc, eph.crs) = (150.0, -80.0);
        (eph.cic, eph.cis) = (1.0e-7, -2.0e-7);
        (eph.f0, eph.f1, eph.f2) = (1.0e-4, 1.0e-12, 0.0);
        (eph.sva, eph.svh) = (3, 0);
        (eph.ai0, eph.ai1, eph.ai2) = (50.0, 0.1, 0.01);
        eph.tgd = 1.5e-9;
        (eph.a0g, eph.a1g, eph.t0g, eph.wn0g) = (2.0e-9, 1.0e-15, 36_000, 1300 % 64);

        // Encode pages 1-4, then decode them into a fresh ephemeris (same SV).
        let pages: Vec<[u8; FNAV_DATA_BITS]> =
            (1..=4).map(|pt| encode_fnav_ephemeris(&eph, pt)).collect();
        let mut got = Ephemeris::new(SV::new(Constellation::Galileo, 11));
        for (pt, data) in (1u8..=4).zip(&pages) {
            decode_fnav_ephemeris(
                &mut got,
                &FnavWord {
                    page_type: pt,
                    data: *data,
                },
            );
        }
        got.ts_sec = 1.0; // the channel timestamps on first decode; stand in here

        assert_eq!(got.eph_mask, 0b1_1110, "all 4 ephemeris pages collected");
        assert!(got.is_valid(), "decoded ephemeris should be valid");

        // Integer fields are exact — they directly confirm the bit positions.
        assert_eq!(got.iode, 42);
        assert_eq!(got.toe, 36_000);
        assert_eq!(got.toc, 36_000);
        assert_eq!(got.week, 1300);
        assert_eq!(got.tow, 35_990);
        assert_eq!(got.wn0g, 1300 % 64);
        assert!(got.ggto_valid && got.gal_iono_valid);

        // Float fields survive quantisation (within ~1 LSB of their scale).
        assert!((got.a - eph.a).abs() < 1.0, "a {} vs {}", got.a, eph.a);
        assert!((got.ecc - eph.ecc).abs() < 1e-9);
        assert!((got.m0 - eph.m0).abs() < 1e-8);
        assert!((got.i0 - eph.i0).abs() < 1e-8);
        assert!((got.crc - eph.crc).abs() < 0.1);
        assert!((got.f0 - eph.f0).abs() < 1e-10);
        assert!((got.tgd - eph.tgd).abs() < P2_32); // BGD LSB

        // Decode is the exact inverse of encode: re-encoding the decoded
        // ephemeris reproduces every page bit-for-bit (pins positions+scales).
        for (pt, data) in (1u8..=4).zip(&pages) {
            assert_eq!(
                &encode_fnav_ephemeris(&got, pt),
                data,
                "page {pt} re-encode"
            );
        }
    }
}
