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
use galileo_osnma::subframe::CollectSubframe;
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

/// Galileo OSNMA Merkle tree root after the **2024-01-15 renewal** (a new tree,
/// distinct from [`MERKLE_ROOT_2023`]). The PKID-1 and PKID-2 publications share
/// this root; only the public key rotates. GSC value.
/// Hex: `832E15EDE55655EAC6E399A539477B7C034CCE24C3C93FFC904ACD9BF842F04E`.
pub const MERKLE_ROOT_RENEWED: [u8; 32] = [
    0x83, 0x2E, 0x15, 0xED, 0xE5, 0x56, 0x55, 0xEA, 0xC6, 0xE3, 0x99, 0xA5, 0x39, 0x47, 0x7B, 0x7C,
    0x03, 0x4C, 0xCE, 0x24, 0xC3, 0xC9, 0x3F, 0xFC, 0x90, 0x4A, 0xCD, 0x9B, 0xF8, 0x42, 0xF0, 0x4E,
];

/// Galileo OSNMA ECDSA P-256 public key, **PKID 1**, in force from the 2024-01-15
/// renewal until the 2025-12-10 rotation. Compressed SEC1 point, GSC value.
/// Hex: `0397EB43789AA0F6D052A638468ECF5278E6F6DF8465ECB8D8B84B8C7A3501F73B`.
pub const PUBKEY_2024_PKID1: [u8; 33] = [
    0x03, 0x97, 0xEB, 0x43, 0x78, 0x9A, 0xA0, 0xF6, 0xD0, 0x52, 0xA6, 0x38, 0x46, 0x8E, 0xCF, 0x52,
    0x78, 0xE6, 0xF6, 0xDF, 0x84, 0x65, 0xEC, 0xB8, 0xD8, 0xB8, 0x4B, 0x8C, 0x7A, 0x35, 0x01, 0xF7,
    0x3B,
];

/// Galileo OSNMA ECDSA P-256 public key, **PKID 2**, in force from 2025-12-10.
/// Compressed SEC1 point, GSC value.
/// Hex: `02219204B5CA6C46B623EEED6CDD2CDDB1F7D6A7532767E5B8DA0DE1EBD695FC99`.
pub const PUBKEY_2025_PKID2: [u8; 33] = [
    0x02, 0x21, 0x92, 0x04, 0xB5, 0xCA, 0x6C, 0x46, 0xB6, 0x23, 0xEE, 0xED, 0x6C, 0xDD, 0x2C, 0xDD,
    0xB1, 0xF7, 0xD6, 0xA7, 0x53, 0x27, 0x67, 0xE5, 0xB8, 0xDA, 0x0D, 0xE1, 0xEB, 0xD6, 0x95, 0xFC,
    0x99,
];

/// A GSC OSNMA trust anchor: the Merkle tree root + the in-force ECDSA P-256
/// public key (and its PKID) for a range of GST weeks. The Merkle tree was
/// renewed on 2024-01-15 (new root, PKID reset), and the public key rotated to
/// PKID 2 on 2025-12-10 under that renewed tree.
struct Anchor {
    name: &'static str,
    /// First GST week this anchor applies to (until the next, newer one).
    from_gst_week: u16,
    merkle_root: [u8; 32],
    pubkey_sec1: [u8; 33],
    pkid: u8,
}

/// Published OSNMA trust anchors, newest first. Week boundaries are the GSC
/// applicability dates: 2024-01-15 = GST week 1273, 2025-12-10 = GST week 1372.
/// See the GSC OSNMA MT/PKI products (<https://www.gsc-europa.eu/gsc-products/OSNMA>).
const ANCHORS: &[Anchor] = &[
    Anchor {
        name: "2025 (PKID 2)",
        from_gst_week: 1372,
        merkle_root: MERKLE_ROOT_RENEWED,
        pubkey_sec1: PUBKEY_2025_PKID2,
        pkid: 2,
    },
    Anchor {
        name: "2024 (PKID 1)",
        from_gst_week: 1273,
        merkle_root: MERKLE_ROOT_RENEWED,
        pubkey_sec1: PUBKEY_2024_PKID1,
        pkid: 1,
    },
    Anchor {
        name: "2023 (PKID 1)",
        from_gst_week: 0,
        merkle_root: MERKLE_ROOT_2023,
        pubkey_sec1: PUBKEY_2023_PKID1,
        pkid: 1,
    },
];

/// The trust anchor in force at GST `week` (ANCHORS is newest-first).
fn anchor_for_gst_week(week: u16) -> &'static Anchor {
    ANCHORS
        .iter()
        .find(|a| week >= a.from_gst_week)
        .unwrap_or(&ANCHORS[ANCHORS.len() - 1])
}

/// One decoded I/NAV page awaiting OSNMA processing: the word plus the GST
/// (Galileo week + time-of-week in seconds) at which it was transmitted.
pub struct OsnmaPage {
    pub week: u16,
    pub tow: u32,
    pub word: InavWord,
}

/// DSM-KROOT block count for the 4-bit NB field (OSNMA SIS ICD v1.1 §3.2.1.1;
/// same table the `galileo_osnma` crate uses). 0 = reserved / unknown.
fn kroot_blocks_for_nb(nb: u8) -> u8 {
    match nb {
        1 => 7,
        2 => 8,
        3 => 9,
        4 => 10,
        5 => 11,
        6 => 12,
        7 => 13,
        8 => 14,
        _ => 0,
    }
}

/// Verifies Galileo OSNMA over a stream of decoded I/NAV pages.
pub struct OsnmaVerifier {
    osnma: Osnma<FullStorage>,
    // The DSM-KROOT (TESLA root key) is broadcast in blocks, one per 30 s
    // subframe; the verifier can only check the chain once every block has
    // arrived. The crate collects them but hides the partial state, so we run a
    // parallel subframe assembler and track the block bitmap ourselves purely to
    // surface assembly progress in the UI.
    subframe: CollectSubframe,
    kroot_dsm_id: Option<u8>,
    kroot_received: u16, // bitmap of received block IDs
    kroot_needed: u8,    // total blocks (from block 0's NB field); 0 = unknown
}

impl OsnmaVerifier {
    /// Wrap a constructed `Osnma` with a fresh DSM-KROOT progress tracker.
    fn wrap(osnma: Osnma<FullStorage>) -> Self {
        OsnmaVerifier {
            osnma,
            subframe: CollectSubframe::new(),
            kroot_dsm_id: None,
            kroot_received: 0,
            kroot_needed: 0,
        }
    }

    /// New verifier anchored on the GSC Merkle tree root (32 bytes).
    ///
    /// No ECDSA public key is supplied, so the verifier recovers it from a
    /// DSM-PKR message in the stream. Those are broadcast only every 6 hours, so
    /// short captures cannot bootstrap the DSM-KROOT this way — they need the
    /// public key provided up front ([`from_p256_pubkey`](Self::from_p256_pubkey)
    /// or [`from_merkle_and_p256`](Self::from_merkle_and_p256)).
    pub fn new(merkle_root: MerkleTreeNode) -> Self {
        // only_slowmac = false: process all ADKDs, not just Slow MAC (ADKD=12).
        Self::wrap(Osnma::from_merkle_tree(merkle_root, None, false))
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
        Some(Self::wrap(Osnma::from_pubkey(pubkey, false)))
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
        Some(Self::wrap(Osnma::from_merkle_tree(
            merkle_root,
            Some(pubkey),
            false,
        )))
    }

    /// New verifier for the 2023 OSNMA epoch — anchors captures from before the
    /// 2024-01-15 Merkle-tree renewal (e.g. the FGI 2023 dataset). Loads both the
    /// 2023 Merkle root and the PKID-1 public key, so it can verify the stream's
    /// DSM-PKR *and* its DSM-KROOT. See [`MERKLE_ROOT_2023`] / [`PUBKEY_2023_PKID1`].
    pub fn galileo_2023() -> Self {
        Self::from_merkle_and_p256(MERKLE_ROOT_2023, &PUBKEY_2023_PKID1, 1)
            .expect("built-in 2023 OSNMA anchor is valid")
    }

    /// New verifier using the built-in GSC trust anchor in force at GST `week`,
    /// picking the epoch automatically (2023 / 2024 / 2025). This is what a live
    /// receiver wants: the decoded GST week selects the right root + public key.
    pub fn for_gst_week(week: u16) -> Self {
        let a = anchor_for_gst_week(week);
        Self::from_merkle_and_p256(a.merkle_root, &a.pubkey_sec1, a.pkid)
            .expect("built-in OSNMA anchor is a valid P-256 point")
    }

    /// Name of the trust anchor [`for_gst_week`](Self::for_gst_week) selects for
    /// `week` (for logging).
    pub fn anchor_name(week: u16) -> &'static str {
        anchor_for_gst_week(week).name
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
            let osnma_msg = pack_msb::<5>(&word.osnma);
            self.osnma.feed_osnma(&osnma_msg, svn, gst);
            // Mirror the feed into our own subframe assembler to watch DSM-KROOT
            // blocks accumulate. On a completed subframe, HKROOT byte 1 is the
            // DSM header (id + block id) and byte 2 is the block's first byte
            // (block 0 carries NB). Copy those two out so the borrow ends before
            // we touch the tracker.
            let dsm = self
                .subframe
                .feed(&osnma_msg, svn, gst)
                .map(|(hkroot, _, _)| (hkroot[1], hkroot[2]));
            if let Some((dsm_header, block_first)) = dsm {
                self.note_dsm_block(dsm_header, block_first);
            }
        }
    }

    /// Update the DSM-KROOT block tracker from one completed subframe's HKROOT.
    /// `dsm_header` (HKROOT byte 1) splits into DSM id (high nibble) and block id
    /// (low nibble); ids ≥ 12 are DSM-PKR and ignored here. For block 0,
    /// `block_first` (HKROOT byte 2) carries the NB field in its high nibble.
    fn note_dsm_block(&mut self, dsm_header: u8, block_first: u8) {
        let dsm_id = dsm_header >> 4;
        let block_id = dsm_header & 0x0F;
        if dsm_id >= 12 {
            return; // DSM-PKR — not the KROOT we report progress for
        }
        if self.kroot_dsm_id != Some(dsm_id) {
            // A different DSM-KROOT is now on air — restart collection.
            self.kroot_dsm_id = Some(dsm_id);
            self.kroot_received = 0;
            self.kroot_needed = 0;
        }
        self.kroot_received |= 1u16 << block_id;
        if block_id == 0 {
            self.kroot_needed = kroot_blocks_for_nb(block_first >> 4);
        }
    }

    /// DSM-KROOT assembly progress as `(blocks_received, blocks_total)`, once the
    /// first block (which carries the total) has been seen — else `None`. When
    /// the two are equal the KROOT is complete and the TESLA chain can verify.
    pub fn kroot_progress(&self) -> Option<(u8, u8)> {
        let total = self.kroot_needed;
        if total == 0 {
            return None; // block 0 not yet received → total unknown
        }
        let mask = (1u16 << total) - 1;
        Some(((self.kroot_received & mask).count_ones() as u8, total))
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

    // Every built-in anchor decodes to a valid P-256 key, and the GST week selects
    // the right epoch (2024-01-15 = week 1273, 2025-12-10 = week 1372).
    #[test]
    fn anchors_parse_and_select_by_gst_week() {
        for a in ANCHORS {
            assert!(
                OsnmaVerifier::from_p256_pubkey(&a.pubkey_sec1, a.pkid).is_some(),
                "{}",
                a.name
            );
        }
        assert_eq!(OsnmaVerifier::anchor_name(1262), "2023 (PKID 1)"); // FGI recording
        assert_eq!(OsnmaVerifier::anchor_name(1272), "2023 (PKID 1)"); // day before renewal
        assert_eq!(OsnmaVerifier::anchor_name(1273), "2024 (PKID 1)"); // renewal
        assert_eq!(OsnmaVerifier::anchor_name(1371), "2024 (PKID 1)");
        assert_eq!(OsnmaVerifier::anchor_name(1372), "2025 (PKID 2)"); // rotation
        assert_eq!(OsnmaVerifier::anchor_name(1500), "2025 (PKID 2)");
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
