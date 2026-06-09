//! Generic per-channel navigation decoding.
//!
//! `Channel::nav_decode` is the signal-agnostic entry point; it dispatches by
//! constellation to the message decoder for that signal:
//!   - GPS / QZSS L1 C/A LNAV → [gps_lnav](crate::gps_lnav),
//!   - Galileo E1-B I/NAV → [galileo_inav](crate::galileo_inav),
//!   - SBAS L1 (EGNOS/WAAS…) → [sbas_l1](crate::sbas_l1).
//!
//! The shared, generic output is `Navigation::eph` (the decoded ephemeris the
//! solver consumes); each signal keeps its own decoder state next to it.

use crate::channel::Channel;
use crate::ephemeris::Ephemeris;
use crate::galileo_inav::{InavDecoder, decode_ephemeris_word};
use crate::gps_lnav::LnavState;
use crate::sbas_l1::SbasL1Channel;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Epoch, TimeScale};

/// Per-channel navigation state: the decoded ephemeris (generic, consumed by the
/// solver) plus the per-signal decoder state.
pub struct Navigation {
    pub eph: Ephemeris,
    pub(crate) lnav: LnavState,     // GPS / QZSS L1 C/A
    pub(crate) inav: InavDecoder,   // Galileo E1-B
    pub(crate) sbas: SbasL1Channel, // SBAS L1
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            eph: Ephemeris::new(sv),
            lnav: LnavState::new(),
            inav: InavDecoder::new(),
            sbas: SbasL1Channel::new(),
        }
    }

    /// Reset the per-signal decode state on (re)acquisition; `eph` is preserved.
    pub fn init(&mut self) {
        self.lnav.reset();
        self.inav = InavDecoder::new();
        self.sbas = SbasL1Channel::new();
    }
}

impl Channel {
    /// Decode the navigation message for this channel's signal. Generic entry
    /// point: dispatches by constellation to the signal-specific decoder.
    pub fn nav_decode(&mut self) {
        match self.sv.constellation {
            Constellation::Galileo => self.nav_decode_inav(),
            Constellation::SBAS => self.nav_decode_sbas(),
            _ => self.nav_decode_gps_lnav(), // GPS, QZSS
        }
    }

    /// SBAS L1 (EGNOS/WAAS/MSAS…): feed prompt-I per 1 ms code period to the
    /// streaming decoder; on a CRC-valid 250-bit message, log its type.
    fn nav_decode_sbas(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        if let Some(msg) = self.nav.sbas.push_period(c_p.re) {
            self.stats.subframes += 1;
            log::warn!("{}: SBAS L1 message type {} (CRC ok)", self.sv, msg.mtype);
        }
    }

    /// Galileo E1-B I/NAV: feed one symbol per 4 ms code period (the sign of
    /// prompt-I) to the page decoder; each CRC-valid word fills the ephemeris
    /// (orbit/clock from word types 1-4, BGD + GST week from type 5). Once every
    /// orbit/clock field is in, `eph.is_valid()` holds — logged once.
    fn nav_decode_inav(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        let sym = (c_p.re >= 0.0) as u8;
        let Some(word) = self.nav.inav.push_symbol(sym) else {
            return;
        };
        self.stats.subframes += 1;
        if self.nav.eph.ts_sec == 0.0 {
            self.nav.eph.ts_sec = self.ts_sec; // timestamp the first word in
        }
        let was_valid = self.nav.eph.is_valid();
        decode_ephemeris_word(&mut self.nav.eph, &word);
        log::warn!("{}: I/NAV word type {} (CRC ok)", self.sv, word.word_type);
        if !was_valid && self.nav.eph.is_valid() {
            log::warn!(
                "{}: Galileo ephemeris complete (GST week {}, toe {} s)",
                self.sv,
                self.nav.eph.week,
                self.nav.eph.toe
            );
        }

        // Build the absolute GST epochs once the week (word 5) is known, and pin
        // the transmit anchor on a word-type-5 page — the only one carrying a
        // fresh TOW, so the captured code-period count matches it. The fixed
        // offset between a page's TOW reference and when its word decodes is the
        // same for every Galileo SV, so it folds into the receiver clock bias.
        if word.word_type == 5 && self.nav.eph.week != 0 {
            let w = self.nav.eph.week;
            let sow_ns = |sow: u32| (sow as u64) * 1_000_000_000;
            self.nav.eph.tow_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.tow), TimeScale::GST);
            self.nav.eph.toe_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.toe), TimeScale::GST);
            self.nav.eph.toc_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.toc), TimeScale::GST);
            self.nav_anchor_tx();
        }
    }

    /// After a signal decoder has filled the ephemeris and set its tow/toe/toc
    /// epochs, mark it usable to the solver and pin the transmit-time anchor: the
    /// integer code-period count (`trk_phase`) at the current TOW edge. Shared by
    /// GPS LNAV (per subframe) and Galileo I/NAV (per word-5 page) — both carry
    /// the TOW at the call site, so the pinned phase matches the epoch.
    pub(crate) fn nav_anchor_tx(&mut self) {
        if self.is_ephemeris_complete() {
            self.pub_state
                .lock()
                .unwrap()
                .channels
                .get_mut(&self.sv)
                .unwrap()
                .has_eph = true;
        }
        self.nav.eph.ts_sec = self.ts_sec;
        // Pin once. Re-anchoring would reset the reference and turn the absolute
        // inter-SV range (carried in code_off_sec) into a relative one.
        if self.is_ephemeris_complete() && !self.nav.eph.tx_anchored {
            self.nav.eph.tow_trk_phase = self.nav.eph.trk_phase;
            self.nav.eph.tx_tow_gpst = self.nav.eph.tow_gpst;
            self.nav.eph.tx_anchor_ts_sec = self.ts_sec;
            self.nav.eph.tx_anchored = true;
            log::warn!(
                "{}: tx anchored tow={:?} phase={:.6}",
                self.sv,
                self.nav.eph.tx_tow_gpst,
                self.nav.eph.tow_trk_phase
            );
        }
        self.update_gpst_time(self.nav.eph.tow_gpst);
    }

    /// Publish the current time-of-week to the shared UI state.
    pub(crate) fn update_gpst_time(&mut self, tow_gpst: Epoch) {
        self.pub_state.lock().unwrap().tow_gpst = tow_gpst;
        (self.pub_state.lock().unwrap().update_func.func)();
    }
}
