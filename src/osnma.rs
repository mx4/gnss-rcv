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
use galileo_osnma::{Gst, InavBand, MerkleTreeNode, Osnma, Svn};

/// Verifies Galileo OSNMA over a stream of decoded I/NAV pages.
pub struct OsnmaVerifier {
    osnma: Osnma<FullStorage>,
}

impl OsnmaVerifier {
    /// New verifier anchored on the GSC Merkle tree root (32 bytes).
    ///
    /// No ECDSA public key is supplied, so the verifier recovers it from a
    /// DSM-PKR message in the stream. Those are broadcast only every 6 hours, so
    /// short captures cannot bootstrap this way — they need the public key
    /// provided up front (a future `with_pubkey` constructor).
    pub fn new(merkle_root: MerkleTreeNode) -> Self {
        // only_slowmac = false: process all ADKDs, not just Slow MAC (ADKD=12).
        OsnmaVerifier {
            osnma: Osnma::from_merkle_tree(merkle_root, None, false),
        }
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
