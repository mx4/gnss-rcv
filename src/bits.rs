//! Navigation-message bit-field helpers: MSB-first packed-bit readers and
//! writers (single and split fields, signed and unsigned), parity XOR and
//! bit-vector comparisons shared by the LNAV/I/NAV/SBAS decoders.

pub fn getbitu(buf: &[u8], pos: usize, len: usize) -> u32 {
    assert!(len <= 32);
    let mut bits = 0;
    for i in pos..pos + len {
        bits = (bits << 1) | ((buf[i / 8] >> (7 - i % 8)) & 1) as u32;
    }
    bits
}

/// Read `len` bits at `pos` as a two's-complement signed value.
pub fn getbits(buf: &[u8], pos: usize, len: usize) -> i32 {
    let bits = getbitu(buf, pos, len);

    // Sign-extend the len-bit field: `mask` covers bit (len-1) — the field's
    // sign bit — and every bit above it. Negative values get all of them set
    // (the two's-complement extension); positive values get them cleared (a
    // no-op, since getbitu zero-fills above the field). The shift pair builds
    // the mask without overflowing at len == 32 (where `1 << len` would).
    let sign = (1 << (len - 1)) & bits;
    let mask = (0xffffffff >> (len - 1)) << (len - 1);
    let res = if sign != 0 { bits | mask } else { bits & !mask };
    res as i32
}

pub fn getbitu2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> u32 {
    assert!(l1 + l2 <= 32);
    let hi = getbitu(buf, p1, l1);
    let lo = getbitu(buf, p2, l2);
    (hi << l2) + lo
}

/// Read a signed value split across two bit ranges (nav messages split several
/// fields across word boundaries): the high part carries the sign — extend it,
/// shift it up, and append the unsigned low part.
pub fn getbits2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> i32 {
    assert!(l1 + l2 <= 32);
    if getbitu(buf, p1, 1) != 0 {
        (getbits(buf, p1, l1) << l2) + getbitu(buf, p2, l2) as i32
    } else {
        getbitu2(buf, p1, l1, p2, l2) as i32
    }
}

pub fn hex_str(data: &[u8]) -> String {
    let num_bits = data.len();
    let mut s = String::new();
    let num = num_bits.div_ceil(8);
    for v in data.iter().take(num) {
        let n = format!("{:02x}", *v);
        s.push_str(&n);
    }
    s
}

pub fn xor_bits(v: u32) -> u8 {
    const XOR_8B: [u8; 256] = [
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    ];

    let bytes = v.to_le_bytes().map(|v| v as usize);
    XOR_8B[bytes[0]] ^ XOR_8B[bytes[1]] ^ XOR_8B[bytes[2]] ^ XOR_8B[bytes[3]]
}

pub fn bits_opposed(bits0: &[u8], bits1: &[u8]) -> bool {
    let bits1_rev: Vec<_> = bits1.iter().map(|v| 1 - v).collect();
    bits_equal(bits0, bits1_rev.as_slice())
}

pub fn bits_equal(bits0: &[u8], bits1: &[u8]) -> bool {
    assert_eq!(bits0.len(), bits1.len());
    bits0 == bits1
}

pub fn setbitu(buf: &mut [u8], pos: usize, len: usize, data: u32) {
    let mut mask = 1u32 << (len - 1);
    if len > 32 {
        return;
    }
    for i in pos..pos + len {
        let bit = 1u8 << (7 - i % 8);
        if data & mask != 0 {
            buf[i / 8] |= bit;
        } else {
            buf[i / 8] &= !bit;
        }
        mask >>= 1;
    }
}
