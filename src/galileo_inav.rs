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
}
