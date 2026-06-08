//! Minimal LNAV (GPS L1 C/A nav message) **encoder** -- the inverse of the parity
//! decoder in [navigation.rs](crate::navigation). Produces parity-correct,
//! frame-syncable subframes carrying a canned, valid ephemeris, so the synthetic
//! signal generator and tests can exercise the receiver's decode path (parity →
//! field parse → ephemeris assembly) with no recording.
//!
//! `nav_test_lnav_parity` resets its parity register at the start of every
//! subframe, so each subframe is encoded independently with D29* = D30* = 0. That
//! keeps every transmitted preamble clean (0x8B) and needs no end-of-subframe
//! "t-bit" solving.

use crate::util::{getbitu, setbitu, xor_bits};

// Same parity masks as the decoder (navigation.rs::nav_test_lnav_parity).
const MASK: [u32; 6] = [
    0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
];
const PREAMBLE: u32 = 0x8b;

/// Encode one 30-bit word from its 24 source data bits (low 24 bits, D1 = bit23)
/// and the previous word's transmitted (D29*, D30*). Returns the 30 transmitted
/// bits (D1 = bit29) and the new (D29, D30).
fn encode_word(source24: u32, d29star: u32, d30star: u32) -> (u32, u32, u32) {
    // Parity input mirrors the decoder's (data>>6) layout: source data in bits
    // 0..23, then D30* at bit 24 and D29* at bit 25.
    let pv = (source24 & 0xFF_FFFF) | (d30star << 24) | (d29star << 25);
    let mut parity = 0u32;
    for m in MASK {
        parity = (parity << 1) | xor_bits(pv & m) as u32; // D25..D30
    }
    // Transmitted data = source XOR D30* (complemented when D30* == 1); the
    // decoder undoes this via the same D30*.
    let tx_data = if d30star == 1 {
        (!source24) & 0xFF_FFFF
    } else {
        source24 & 0xFF_FFFF
    };
    let word = (tx_data << 6) | parity;
    (word, (parity >> 1) & 1, parity & 1) // new D29 = P4, D30 = P5
}

/// Encode a subframe whose data bits are set (byte-packed, `setbitu`-style, at
/// the documented offsets; the 6 parity bits per word left 0) into the 300
/// transmitted bits, one bit per byte -- the form the receiver's bit stream and
/// parity decoder consume.
pub fn encode_subframe(source: &[u8]) -> [u8; 300] {
    let mut out = [0u8; 300];
    let (mut d29, mut d30) = (0u32, 0u32);
    for i in 0..10 {
        let s24 = getbitu(source, 30 * i, 24); // 24 source data bits, D1 = MSB
        let (word, n29, n30) = encode_word(s24, d29, d30);
        for j in 0..30 {
            out[30 * i + j] = ((word >> (29 - j)) & 1) as u8;
        }
        (d29, d30) = (n29, n30);
    }
    out
}

/// Set a value split across two bit ranges (high bits then low bits), the layout
/// `getbitu2`/`getbits2` read.
fn set_split(s: &mut [u8; 300], p_hi: usize, l_lo: usize, p_lo: usize, v: u32) {
    setbitu(s, p_hi, 8, v >> l_lo);
    setbitu(s, p_lo, l_lo, v & ((1 << l_lo) - 1));
}

/// The three source subframes (1, 2, 3) of a canned, valid GPS ephemeris. Field
/// offsets/scales match [ephemeris.rs](crate::ephemeris)'s decoders. Decoded
/// values: a ≈ 26,560 km, ecc ≈ 0.005, i0 ≈ 0.97 rad, omg_dot ≈ -8e-9 rad/s,
/// week 2348, toc/toe 36,000 s.
pub fn canned_ephemeris_subframes() -> [[u8; 300]; 3] {
    let p2 = |n: i32| 2f64.powi(n);
    let sc2rad = std::f64::consts::PI; // 1 semicircle = π rad

    let mut sf = [[0u8; 300]; 3];
    for (k, s) in sf.iter_mut().enumerate() {
        setbitu(s, 0, 8, PREAMBLE);
        setbitu(s, 30, 17, 100 + k as u32); // HOW TOW count (×6 s)
        setbitu(s, 49, 3, k as u32 + 1); // subframe id
    }

    // subframe 1 — week + clock reference time
    let s = &mut sf[0];
    setbitu(s, 60, 10, 300); // week (+2048 -> 2348)
    setbitu(s, 218, 16, 2250); // toc (×16 -> 36,000 s)

    // subframe 2 — orbit size/shape + ephemeris reference time
    let s = &mut sf[1];
    setbitu(s, 60, 8, 10); // iode
    set_split(s, 166, 24, 180, (0.005 * p2(33)).round() as u32); // ecc
    set_split(
        s,
        226,
        24,
        240,
        (26_560_000f64.sqrt() * p2(19)).round() as u32,
    ); // sqrt_a
    setbitu(s, 270, 16, 2250); // toe (×16 -> 36,000 s)

    // subframe 3 — inclination + rate of right ascension
    let s = &mut sf[2];
    set_split(s, 136, 24, 150, (0.97 / sc2rad * p2(31)).round() as u32); // i0
    setbitu(
        s,
        240,
        24,
        ((-8e-9 / sc2rad * p2(43)).round() as i32 as u32) & 0xFF_FFFF,
    ); // omg_dot
    setbitu(s, 270, 8, 10); // iode

    sf
}

/// A continuous ±1 nav-bit stream (50 bps) cycling the canned subframes 1→2→3
/// `repeats` times -- feed as `SynthSv.nav_bits` to drive the full decode path.
pub fn ephemeris_nav_bits(repeats: usize) -> Vec<i8> {
    let sfs = canned_ephemeris_subframes();
    let mut out = Vec::with_capacity(repeats * 3 * 300);
    for _ in 0..repeats {
        for sf in &sfs {
            for &b in encode_subframe(sf).iter() {
                out.push(if b == 1 { 1 } else { -1 });
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel::Channel;
    use crate::ephemeris::Ephemeris;
    use gnss_rs::constellation::Constellation;
    use gnss_rs::sv::SV;

    // Hermetic decode-pipeline test: the canned subframes must pass the receiver's
    // *real* parity decoder, recover their exact source bits, and parse into a
    // valid ephemeris -- in <1 ms, no recording. (Acquire/track is covered by the
    // synth tests; a full signal→fix needs geometry-consistent code phases.)
    #[test]
    fn encodes_a_parity_valid_decodable_ephemeris() {
        let sv = SV::new(Constellation::GPS, 5);
        let mut eph = Ephemeris::new(sv);
        eph.ts_sec = 1.0; // the channel timestamps the first subframe; emulate it

        for (k, source) in canned_ephemeris_subframes().iter().enumerate() {
            let tx = encode_subframe(source);
            let mut nav_data = vec![0u8; 300];
            assert!(
                Channel::nav_test_lnav_parity(&tx, &mut nav_data),
                "subframe {} must pass the receiver's parity check",
                k + 1
            );
            assert_eq!(
                &nav_data[..],
                &source[..],
                "subframe {} data must round-trip through encode→parity-decode",
                k + 1
            );
            match k {
                0 => eph.nav_decode_lnav_subframe1(&nav_data, sv),
                1 => eph.nav_decode_lnav_subframe2(&nav_data, sv),
                2 => eph.nav_decode_lnav_subframe3(&nav_data, sv),
                _ => unreachable!(),
            }
        }

        assert!(eph.is_valid(), "decoded ephemeris should be valid");
        assert_eq!(eph.week, 2348);
        assert!((eph.a - 26_560_000.0).abs() < 50_000.0, "a = {}", eph.a);
        assert!((eph.ecc - 0.005).abs() < 1e-4, "ecc = {}", eph.ecc);
        assert!((eph.i0 - 0.97).abs() < 1e-3, "i0 = {}", eph.i0);
        assert!(
            eph.omg_dot < 0.0 && eph.omg_dot.abs() > 1e-9,
            "omg_dot = {}",
            eph.omg_dot
        );
    }
}
