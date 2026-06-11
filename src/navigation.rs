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
use crate::galileo_inav::{INAV_DECODE_LATENCY_SEC, InavDecoder, decode_ephemeris_word};
use crate::gps_lnav::LnavState;
use crate::osnma::OsnmaPage;
use crate::sbas_l1::SbasL1Channel;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Duration, Epoch, TimeScale};

/// Per-channel navigation state: the decoded ephemeris (generic, consumed by the
/// solver) plus the per-signal decoder state.
pub struct Navigation {
    pub eph: Ephemeris,
    pub(crate) lnav: LnavState,     // GPS / QZSS L1 C/A
    pub(crate) inav: InavDecoder,   // Galileo E1-B
    pub(crate) sbas: SbasL1Channel, // SBAS L1
    /// Galileo OSNMA: the GST anchor `(tow, ts_sec)` from the last word-5 page,
    /// and decoded pages buffered for the receiver-level verifier to drain.
    pub(crate) osnma_anchor: Option<(u32, f64)>,
    pub(crate) osnma_pages: Vec<OsnmaPage>,
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            eph: Ephemeris::new(sv),
            lnav: LnavState::new(sv),
            inav: InavDecoder::new(),
            sbas: SbasL1Channel::new(),
            osnma_anchor: None,
            osnma_pages: Vec::new(),
        }
    }

    /// Reset the per-signal decode state on (re)acquisition; `eph` is preserved.
    pub fn init(&mut self) {
        self.lnav.reset();
        self.inav = InavDecoder::new();
        self.sbas = SbasL1Channel::new();
        // The I/NAV symbol stream restarts, so the page-timing anchor is stale.
        self.osnma_anchor = None;
        self.osnma_pages.clear();
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
    /// streaming decoder; each CRC-valid 250-bit message updates the shared
    /// ionospheric grid (MT18 masks + MT26 delays), the fast/long-term
    /// correction state (MT1-5/24/25), and the UI message counter.
    fn nav_decode_sbas(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        if let Some(msg) = self.nav.sbas.push_period(c_p.re) {
            self.stats.subframes += 1;
            let detail = {
                let mut st = self.pub_state.lock().unwrap();
                let iono = st.sbas_iono.feed(&msg);
                let corr = st.sbas_corr.feed(&msg, self.ts_sec);
                if let Some(cs) = st.channels.get_mut(&self.sv) {
                    cs.sbas_msgs = self.stats.subframes;
                }
                if iono {
                    format!(" — iono grid: {} IGPs", st.sbas_iono.len())
                } else if corr {
                    format!(
                        " — corrections: {} fast / {} long-term",
                        st.sbas_corr.fast_len(),
                        st.sbas_corr.long_len()
                    )
                } else {
                    String::new()
                }
            };
            log::warn!(
                "{}: SBAS L1 message type {} (CRC ok) ts={:.3}{detail}",
                self.sv,
                msg.mtype,
                self.ts_sec
            );
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
        let had_ggto = self.nav.eph.ggto_valid;
        let had_iono = self.nav.eph.gal_iono_valid;
        decode_ephemeris_word(&mut self.nav.eph, &word);
        if !had_iono && self.nav.eph.gal_iono_valid {
            log::warn!(
                "{}: NeQuick-G iono (word 5): ai0={:.2} sfu ai1={:+.3} ai2={:+.4} storm=0b{:05b}",
                self.sv,
                self.nav.eph.ai0,
                self.nav.eph.ai1,
                self.nav.eph.ai2,
                self.nav.eph.iono_storm
            );
            self.pub_state.lock().unwrap().gal_iono_az =
                Some([self.nav.eph.ai0, self.nav.eph.ai1, self.nav.eph.ai2]);
        }
        if !had_ggto && self.nav.eph.ggto_valid {
            log::warn!(
                "{}: GGTO (GST-GPST) decoded: A0G={:+.2} ns A1G={:+.2e} s/s t0G={} WN0G={}",
                self.sv,
                self.nav.eph.a0g * 1e9,
                self.nav.eph.a1g,
                self.nav.eph.t0g,
                self.nav.eph.wn0g
            );
        }
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
            // Word 5 carries a fresh GST tow; (re)anchor the OSNMA page clock to it.
            self.nav.osnma_anchor = Some((self.nav.eph.tow, self.ts_sec));
            self.nav_anchor_tx(INAV_DECODE_LATENCY_SEC);
        }

        // Buffer the page for the receiver-level OSNMA verifier, timestamped with
        // the GST at which it was transmitted: the word-5 anchor plus real elapsed
        // time. The verifier floors GST to the 30 s subframe, so sub-second
        // accuracy suffices; pages before the run's first word 5 can't be
        // timestamped yet (no week/tow), so they are skipped.
        if let Some((anchor_tow, anchor_ts)) = self.nav.osnma_anchor {
            let off = (self.ts_sec - anchor_ts).round() as i64;
            let (mut week, mut tow) = (self.nav.eph.week as i64, anchor_tow as i64 + off);
            while tow >= 604800 {
                tow -= 604800;
                week += 1;
            }
            while tow < 0 {
                tow += 604800;
                week -= 1;
            }
            self.nav.osnma_pages.push(OsnmaPage {
                week: week as u16,
                tow: tow as u32,
                word,
            });
        }
    }

    /// After a signal decoder has filled the ephemeris and set its tow/toe/toc
    /// epochs, mark it usable to the solver and pin the transmit-time anchor: the
    /// integer code-period count (`trk_phase`) at the current TOW edge. Shared by
    /// GPS LNAV (per subframe) and Galileo I/NAV (per word-5 page).
    ///
    /// `decode_latency_sec` is the signal's *structural* gap between the
    /// transmit instant the broadcast TOW names and the moment the decoder can
    /// emit the frame that carries it (LNAV: the next subframe's 8 preamble
    /// bits the frame check requires, 0.16 s; I/NAV: TOW names the start of
    /// its own 2 s page, emitted at page end). The pinned phase belongs to the
    /// decode moment, so the paired epoch is `tow + latency`. A wrong latency
    /// is invisible within one constellation (common across its SVs, it folds
    /// into the receiver clock bias — which is how it went unnoticed) but
    /// shifts that constellation's absolute t_tx, breaking mixed
    /// GPS+Galileo solves by the LNAV/INAV difference (measured 1.84 s on the
    /// ION LimeSDR capture). Both constants are verified directly against
    /// synthetic ground truth by `tx_anchor_latency_measured_against_synthetic_truth`.
    pub(crate) fn nav_anchor_tx(&mut self, decode_latency_sec: f64) {
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
            self.nav.eph.tx_tow_gpst =
                self.nav.eph.tow_gpst + Duration::from_seconds(decode_latency_sec);
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
