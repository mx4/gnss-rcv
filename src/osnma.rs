//! Galileo OSNMA (Open Service Navigation Message Authentication).
//!
//! A thin adapter over the [`galileo_osnma`] crate. It turns our decoded I/NAV
//! words (bit arrays from [`crate::galileo_inav`]) into the crate's byte-packed
//! inputs and tracks which satellites currently have cryptographically
//! authenticated navigation data.
//!
//! OSNMA authenticates the Galileo I/NAV ephemeris, clock and health using a
//! TESLA chain: each 30 s subframe carries MACs over the navigation data, and
//! the MAC key is disclosed one subframe later. The chain's root key is
//! ECDSA-signed, and that public key is bound to a Merkle tree root the GSC
//! publishes out of band — the single trust anchor a receiver needs.
//!
//! Usage: feed every decoded page (its 128 data bits + 40 OSNMA bits) with the
//! GST at which it was transmitted; once a subframe plus the delayed key arrive,
//! [`OsnmaVerifier::is_authenticated`] turns true for that satellite.

use crate::galileo_inav::InavWord;
use galileo_osnma::storage::FullStorage;
use galileo_osnma::{Gst, InavBand, MerkleTreeNode, Osnma, PublicKey, Svn};
use p256::ecdsa::VerifyingKey;

/// Galileo OSNMA ECDSA P-256 public key, PKID 1, in force throughout 2023 (until
/// the 2024-01-15 Merkle-tree renewal that re-issued the keys). Compressed SEC1
/// point, as published by the GSC and by Daniel Estévez (destevez.net), and
/// confirmed directly by the DSM-PKR in the FGI 2023 recording (`new_public_key_id
/// = 1`). The trust anchor for captures from that epoch — e.g. the FGI 2023 dataset.
/// Hex: `0374A925CFA0FF1805E5C5A58FDBA31BF0145D5B5BE2F062D3F8BB2EE98F0F6DB0`.
pub const PUBKEY_2023_PKID1: [u8; 33] = [
    0x03, 0x74, 0xA9, 0x25, 0xCF, 0xA0, 0xFF, 0x18, 0x05, 0xE5, 0xC5, 0xA5, 0x8F, 0xDB, 0xA3, 0x1B,
    0xF0, 0x14, 0x5D, 0x5B, 0x5B, 0xE2, 0xF0, 0x62, 0xD3, 0xF8, 0xBB, 0x2E, 0xE9, 0x8F, 0x0F, 0x6D,
    0xB0,
];

/// Galileo OSNMA Merkle tree root in force throughout 2023 (until the 2024-01-15
/// renewal), as published by the GSC and by Daniel Estévez. Loading it lets the
/// verifier authenticate DSM-PKR (public-key) messages in the stream, and pairs
/// with [`PUBKEY_2023_PKID1`] as the trust anchor for that epoch.
/// Hex: `0E63F552C8021709043C239032EFFE941BF22C8389032F5F2701E0FBC80148B8`.
pub const MERKLE_ROOT_2023: [u8; 32] = [
    0x0E, 0x63, 0xF5, 0x52, 0xC8, 0x02, 0x17, 0x09, 0x04, 0x3C, 0x23, 0x90, 0x32, 0xEF, 0xFE, 0x94,
    0x1B, 0xF2, 0x2C, 0x83, 0x89, 0x03, 0x2F, 0x5F, 0x27, 0x01, 0xE0, 0xFB, 0xC8, 0x01, 0x48, 0xB8,
];

/// One decoded I/NAV page awaiting OSNMA processing: the word plus the GST
/// (Galileo week + time-of-week in seconds) at which it was transmitted.
pub struct OsnmaPage {
    pub week: u16,
    pub tow: u32,
    pub word: InavWord,
}

/// Verifies Galileo OSNMA over a stream of decoded I/NAV pages.
pub struct OsnmaVerifier {
    osnma: Osnma<FullStorage>,
}

impl OsnmaVerifier {
    /// New verifier anchored on the GSC Merkle tree root (32 bytes).
    ///
    /// No ECDSA public key is supplied, so the verifier recovers it from a
    /// DSM-PKR message in the stream. Those are broadcast only every 6 hours, so
    /// short captures cannot bootstrap the DSM-KROOT this way — they need the
    /// public key provided up front ([`from_p256_pubkey`](Self::from_p256_pubkey)
    /// or [`from_merkle_and_p256`](Self::from_merkle_and_p256)).
    pub fn new(merkle_root: MerkleTreeNode) -> Self {
        // only_slowmac = false: process all ADKDs, not just Slow MAC (ADKD=12).
        OsnmaVerifier {
            osnma: Osnma::from_merkle_tree(merkle_root, None, false),
        }
    }

    /// New verifier trusting a single ECDSA P-256 public key directly (a SEC1
    /// point, compressed or uncompressed), bypassing the Merkle tree. This is the
    /// right mode for short captures: the DSM-KROOT (reassembled every ~30 s) is
    /// verified straight from this key, with no need for a DSM-PKR (every 6 h).
    /// `pkid` must match the public key ID the stream's DSM-KROOT references.
    /// Returns `None` if `sec1` is not a valid P-256 point.
    pub fn from_p256_pubkey(sec1: &[u8], pkid: u8) -> Option<Self> {
        let vk = VerifyingKey::from_sec1_bytes(sec1).ok()?;
        let pubkey = PublicKey::from_p256(vk, pkid).force_valid();
        Some(OsnmaVerifier {
            osnma: Osnma::from_pubkey(pubkey, false),
        })
    }

    /// New verifier anchored on a Merkle tree root *and* a known ECDSA P-256
    /// public key. The root lets DSM-PKR messages be authenticated; the public
    /// key lets the DSM-KROOT (and hence the TESLA chain) be verified straight
    /// away, without waiting for a DSM-PKR. Returns `None` if `sec1` is invalid.
    pub fn from_merkle_and_p256(
        merkle_root: MerkleTreeNode,
        sec1: &[u8],
        pkid: u8,
    ) -> Option<Self> {
        let vk = VerifyingKey::from_sec1_bytes(sec1).ok()?;
        let pubkey = PublicKey::from_p256(vk, pkid).force_valid();
        Some(OsnmaVerifier {
            osnma: Osnma::from_merkle_tree(merkle_root, Some(pubkey), false),
        })
    }

    /// New verifier for the 2023 OSNMA epoch — anchors captures from before the
    /// 2024-01-15 Merkle-tree renewal (e.g. the FGI 2023 dataset). Loads both the
    /// 2023 Merkle root and the PKID-1 public key, so it can verify the stream's
    /// DSM-PKR *and* its DSM-KROOT. See [`MERKLE_ROOT_2023`] / [`PUBKEY_2023_PKID1`].
    pub fn galileo_2023() -> Self {
        Self::from_merkle_and_p256(MERKLE_ROOT_2023, &PUBKEY_2023_PKID1, 1)
            .expect("built-in 2023 OSNMA anchor is valid")
    }

    /// Feed one decoded I/NAV page transmitted by `prn` (Galileo E-number, 1..=36)
    /// at the given GST — Galileo week number and time-of-week in seconds, taken
    /// at the *start* of the page transmission.
    ///
    /// Non-E1B bands and out-of-range PRNs are ignored. The 40-bit OSNMA field is
    /// only fed when non-zero (all-zero means the SV carries no OSNMA on this page).
    pub fn feed(&mut self, prn: u8, gst_week: u16, gst_tow: u32, word: &InavWord) {
        let Ok(svn) = Svn::try_from(prn) else { return };
        let gst = Gst::new(gst_week, gst_tow);
        self.osnma
            .feed_inav(&pack_msb::<16>(&word.bits), svn, gst, InavBand::E1B);
        if word.osnma.iter().any(|&b| b != 0) {
            self.osnma.feed_osnma(&pack_msb::<5>(&word.osnma), svn, gst);
        }
    }

    /// Whether satellite `prn` currently has authenticated CED + health data
    /// (ADKD=0/12) in the OSNMA store.
    pub fn is_authenticated(&self, prn: u8) -> bool {
        Svn::try_from(prn)
            .map(|svn| self.osnma.get_ced_and_status(svn).is_some())
            .unwrap_or(false)
    }
}

/// Pack `bits` (one bit per byte) into `N` bytes, most-significant bit first:
/// bit 0 becomes the MSB of byte 0. This is the layout the `galileo_osnma` crate
/// expects for both the 16-byte I/NAV word and the 5-byte OSNMA data message,
/// matching the Galileo ICD's bit numbering (bit 0 = first transmitted).
fn pack_msb<const N: usize>(bits: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    for (i, &b) in bits.iter().enumerate().take(N * 8) {
        out[i / 8] |= (b & 1) << (7 - (i % 8));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::Ephemeris;
    use crate::galileo_inav::encode_ephemeris_word;
    use gnss_rs::{constellation::Constellation, sv::SV};

    // The built-in 2023 trust anchor decodes to a valid P-256 key (compressed
    // point decompression works under our lean p256 feature set).
    #[test]
    fn the_2023_pubkey_parses() {
        assert!(OsnmaVerifier::from_p256_pubkey(&PUBKEY_2023_PKID1, 1).is_some());
        let _ = OsnmaVerifier::galileo_2023();
    }

    // Packing is MSB-first: bit 0 -> MSB of byte 0.
    #[test]
    fn pack_msb_is_msb_first() {
        let mut bits = [0u8; 16];
        bits[0] = 1; // -> 0x80 in byte 0
        bits[9] = 1; // -> 0x40 in byte 1
        let packed = pack_msb::<2>(&bits);
        assert_eq!(packed, [0x80, 0x40]);
    }

    // The crate identifies a word's type as `inav_word[0] >> 2` (top 6 bits of
    // byte 0). Our packing of bits[..6] (the word type, MSB first) must agree, or
    // every word we feed would be mis-typed. This locks the decoder<->crate
    // byte-order contract without needing any crypto.
    #[test]
    fn packed_word_type_matches_the_crate_convention() {
        let eph = Ephemeris::new(SV::new(Constellation::Galileo, 1));
        for wt in 1..=5u8 {
            let word = encode_ephemeris_word(&eph, wt);
            let packed = pack_msb::<16>(&word.bits);
            assert_eq!(packed[0] >> 2, wt, "word type {wt} must survive packing");
        }
    }

    // The plumbing path runs end to end without panicking, and a verifier with a
    // dummy trust anchor authenticates nothing (no valid TESLA chain).
    #[test]
    fn feeding_pages_without_a_valid_chain_authenticates_nothing() {
        let eph = Ephemeris::new(SV::new(Constellation::Galileo, 1));
        let mut v = OsnmaVerifier::new([0u8; 32]);
        for tow in 0..15u32 {
            let word = encode_ephemeris_word(&eph, (tow % 5 + 1) as u8);
            v.feed(1, 1262, tow * 2, &word);
        }
        assert!(!v.is_authenticated(1));
    }
}
