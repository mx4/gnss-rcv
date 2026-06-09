//! Forward-error-correction primitives shared by the navigation-message decoders.
//!
//! The CCSDS K=7 rate-1/2 convolutional code (G1=171₈, G2=133₈) is used by both
//! Galileo E1-B I/NAV and SBAS L1; CRC-24Q (poly 0x1864CFB) by Galileo I/NAV,
//! SBAS L1, GPS L2C/L5 CNAV and RTCM. The constellation-specific framing (block
//! interleaving, page assembly, message layout) lives in the per-signal modules.

/// Generator polynomials of the K=7, rate-1/2 convolutional code.
pub const G1: u32 = 0o171; // 1111001
pub const G2: u32 = 0o133; // 1011011

/// Parity (XOR of bits) of `x` — one convolutional output symbol.
#[inline]
pub fn parity(x: u32) -> u8 {
    (x.count_ones() & 1) as u8
}

/// Rate-1/2 convolutional encode of `bits`. The 6-bit register starts at 0; the
/// caller appends 6 zero tail bits to terminate the trellis. Two output symbols
/// per input bit: G1, then G2 XORed with `invert_g2`. Galileo I/NAV inverts the
/// second symbol (`1`); SBAS L1 does not (`0`).
pub fn conv_encode(bits: &[u8], invert_g2: u8) -> Vec<u8> {
    let mut state: u32 = 0; // 6 history bits in positions 5..0
    let mut out = Vec::with_capacity(bits.len() * 2);
    for &b in bits {
        let reg = ((b as u32 & 1) << 6) | state; // 7-bit window: current bit + history
        out.push(parity(reg & G1));
        out.push(parity(reg & G2) ^ invert_g2);
        state = reg >> 1;
    }
    out
}

/// CRC-24Q (poly 0x1864CFB, init 0), MSB-first over a bit slice. The CRC of a
/// message with its own 24-bit CRC appended is 0.
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

    #[test]
    fn conv_encode_doubles_and_terminates() {
        let data = sample_bits(20);
        let mut padded = data.clone();
        padded.extend([0u8; 6]); // tail
        let coded = conv_encode(&padded, 0);
        assert_eq!(coded.len(), 2 * padded.len());
        // The inverted-G2 variant differs only in every second symbol.
        let coded_inv = conv_encode(&padded, 1);
        for (i, (a, b)) in coded.iter().zip(&coded_inv).enumerate() {
            assert_eq!(a == b, i % 2 == 0, "symbol {i}");
        }
    }
}
