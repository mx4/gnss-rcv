//! Generic per-channel navigation decoding.
//!
//! `Channel::nav_decode` is the signal-agnostic entry point; it dispatches by
//! constellation to the message decoder for that signal:
//!   - GPS / QZSS L1 C/A LNAV → [gps_lnav](crate::gps_lnav),
//!   - Galileo E1-B I/NAV → [galileo_inav](crate::galileo_inav),
//!   - SBAS → a stub for now.
//!
//! The shared, generic output is `Navigation::eph` (the decoded ephemeris the
//! solver consumes); each signal keeps its own decoder state next to it.

use crate::channel::Channel;
use crate::ephemeris::Ephemeris;
use crate::galileo_inav::InavDecoder;
use crate::gps_lnav::LnavState;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;

/// Per-channel navigation state: the decoded ephemeris (generic, consumed by the
/// solver) plus the per-signal decoder state.
pub struct Navigation {
    pub eph: Ephemeris,
    pub(crate) lnav: LnavState,   // GPS / QZSS L1 C/A
    pub(crate) inav: InavDecoder, // Galileo E1-B
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            eph: Ephemeris::new(sv),
            lnav: LnavState::new(),
            inav: InavDecoder::new(),
        }
    }

    /// Reset the per-signal decode state on (re)acquisition; `eph` is preserved.
    pub fn init(&mut self) {
        self.lnav.reset();
        self.inav = InavDecoder::new();
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

    fn nav_decode_sbas(&mut self) {
        log::warn!("{}: SBAS frame", self.sv);
    }

    /// Galileo E1-B I/NAV: feed one symbol per 4 ms code period (the sign of
    /// prompt-I) to the page decoder; on a CRC-valid word, log its type. (The
    /// ephemeris field extraction from word types 1-5 is the next step.)
    fn nav_decode_inav(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        let sym = (c_p.re >= 0.0) as u8;
        if let Some(word) = self.nav.inav.push_symbol(sym) {
            self.stats.subframes += 1;
            log::warn!("{}: I/NAV word type {} (CRC ok)", self.sv, word.word_type);
        }
    }
}
