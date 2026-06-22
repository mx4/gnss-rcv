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
use crate::code::Signal;
use crate::ephemeris::{Ephemeris, Measurement};
use crate::galileo_fnav::{
    FNAV_DECODE_LATENCY_SEC, FnavDecoder, FnavSymbolizer, decode_fnav_ephemeris,
};
use crate::galileo_inav::{INAV_DECODE_LATENCY_SEC, InavDecoder, decode_ephemeris_word};
use crate::gps_lnav::LnavState;
use crate::models::compute_sv_position_ecef;
use crate::osnma::OsnmaPage;
use crate::rinex_nav::fresh_eph;
use crate::sbas_l1::SbasL1Channel;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Duration, Epoch, TimeScale, Vector3};

/// Per-channel navigation state: the decoded ephemeris (generic, consumed by the
/// solver) plus the per-signal decoder state.
pub struct Navigation {
    pub eph: Ephemeris,
    /// The per-epoch tracking measurement, snapshotted by the channel each
    /// code period; paired with `eph` for the solve.
    pub meas: Measurement,
    pub(crate) lnav: LnavState,   // GPS / QZSS L1 C/A
    pub(crate) inav: InavDecoder, // Galileo E1-B
    pub(crate) fnav: FnavDecoder, // Galileo E5a-I (page decoder)
    /// E5a-I CS20 secondary symboliser: tiers 20 primary-epoch prompts into one
    /// 20 ms F/NAV symbol for [`Navigation::fnav`].
    pub(crate) fnav_sym: FnavSymbolizer,
    pub(crate) sbas: SbasL1Channel, // SBAS L1
    /// Galileo OSNMA: the GST anchor `(tow, ts_sec)` from the last word-5 page,
    /// and decoded pages buffered for the receiver-level verifier to drain.
    pub(crate) osnma_anchor: Option<(u32, f64)>,
    pub(crate) osnma_pages: Vec<OsnmaPage>,
    /// Latest TOW-bearing message's `(tow_gpst + decode latency, trk_phase)`
    /// pair, remembered even before the ephemeris completes. `trk_phase`
    /// counts transmitted code periods continuously while tracking, so the
    /// pair stays valid until lock is lost — letting the transmit anchor pin
    /// *retroactively* the moment the ephemeris completes instead of waiting
    /// for the next TOW message (up to a 30 s word-5 cycle on Galileo).
    pub(crate) tow_pair: Option<(Epoch, f64)>,
    /// Assisted-GNSS (Tier-1): the downloaded broadcast orbit injected for this
    /// SV. Copied into `eph` at channel creation so the channel can anchor the
    /// transmit time on the *first* decoded TOW instead of waiting out the full
    /// on-air ephemeris (~30 s LNAV, ~120 s F/NAV). Kept here as the baseline for
    /// the injected-vs-decoded orbit cross-check. `None` without `--eph`.
    pub(crate) assist_eph: Option<Ephemeris>,
    /// All injected brdc issues for this SV (sorted by `toe`). The initial
    /// `assist_eph` is a coarse pick; once the channel decodes its first TOW it
    /// re-selects the issue nearest that GPST time — so the injected orbit is the
    /// right one without needing the recording's time-of-day. Empty without `--eph`.
    pub(crate) assist_set: Vec<Ephemeris>,
    /// True once the injected orbit has been cross-checked against the fully
    /// signal-decoded one (fires once, when the on-air ephemeris completes).
    pub(crate) assist_crosschecked: bool,
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            eph: Ephemeris::new(sv),
            meas: Measurement::default(),
            lnav: LnavState::new(sv),
            inav: InavDecoder::new(),
            fnav: FnavDecoder::new(),
            fnav_sym: FnavSymbolizer::new(),
            sbas: SbasL1Channel::new(),
            osnma_anchor: None,
            osnma_pages: Vec::new(),
            tow_pair: None,
            assist_eph: None,
            assist_set: Vec::new(),
            assist_crosschecked: false,
        }
    }

    /// Reset the per-signal decode state on (re)acquisition; `eph` is preserved.
    pub fn init(&mut self) {
        self.lnav.reset();
        self.inav = InavDecoder::new();
        self.fnav = FnavDecoder::new();
        self.fnav_sym = FnavSymbolizer::new();
        self.sbas = SbasL1Channel::new();
        // The I/NAV symbol stream restarts, so the page-timing anchor is stale.
        self.osnma_anchor = None;
        self.osnma_pages.clear();
        self.tow_pair = None; // phase continuity broken with the re-lock
    }
}

impl Channel {
    /// Decode the navigation message for this channel's signal. Generic entry
    /// point: dispatches by constellation to the signal-specific decoder.
    pub fn nav_decode(&mut self) {
        // Dataless pilots (E1-C, E5a-Q) carry no nav data — nothing to decode;
        // they only improve tracking. The nav data is on the data components.
        if matches!(self.sig, Signal::GalileoE1c | Signal::GalileoE5aQ) {
            return;
        }
        match self.sv.constellation {
            // Galileo splits by band: E1-B carries I/NAV, E5a-I carries F/NAV.
            Constellation::Galileo if self.sig == Signal::GalileoE5aI => self.nav_decode_fnav(),
            Constellation::Galileo => self.nav_decode_inav(),
            Constellation::SBAS => self.nav_decode_sbas(),
            _ => self.nav_decode_gps_lnav(), // GPS, QZSS
        }
    }

    /// Galileo E5a-I F/NAV: each 1 ms primary-epoch prompt-I is tiered by the CS20
    /// symboliser into a 20 ms symbol (50 sym/s); each CRC-valid page fills the
    /// ephemeris — clock/iono/BGD/GST (page type 1), the Keplerian orbit 1/3..3/3
    /// (types 2-4) and the GGTO (type 4). The F/NAV twin of [`nav_decode_inav`];
    /// the TOW names the GST at the page start, decoded 10 s later
    /// ([`FNAV_DECODE_LATENCY_SEC`]).
    fn nav_decode_fnav(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        let Some(sym) = self.nav.fnav_sym.push_prompt(c_p.re) else {
            return;
        };
        let Some(word) = self.nav.fnav.push_symbol(sym) else {
            return;
        };
        self.stats.subframes += 1;
        if self.nav.eph.ts_sec == 0.0 {
            self.nav.eph.ts_sec = self.ts_sec; // timestamp the first page in
        }
        let was_valid = self.nav.eph.is_valid();
        let had_ggto = self.nav.eph.ggto_valid;
        decode_fnav_ephemeris(&mut self.nav.eph, &word);
        log::warn!("{}: F/NAV page type {} (CRC ok)", self.sv, word.page_type);
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
        // A page can be the one completing the ephemeris — anchor right away.
        self.try_anchor_tx();
        if !was_valid && self.nav.eph.is_valid() {
            log::warn!(
                "{}: Galileo E5a ephemeris complete (GST week {}, toe {} s)",
                self.sv,
                self.nav.eph.week,
                self.nav.eph.toe
            );
        }
        // Every F/NAV page carries a fresh GST WN/TOW; build the absolute epochs
        // and pin the transmit anchor. The fixed page-decode offset (10 s) is the
        // same for every Galileo SV, so it folds into the receiver clock bias.
        if self.nav.eph.week != 0 {
            let w = self.nav.eph.week;
            let sow_ns = |sow: u32| (sow as u64) * 1_000_000_000;
            self.nav.eph.tow_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.tow), TimeScale::GST);
            self.nav.eph.toe_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.toe), TimeScale::GST);
            self.nav.eph.toc_gpst =
                Epoch::from_time_of_week(w, sow_ns(self.nav.eph.toc), TimeScale::GST);
            self.nav_anchor_tx(FNAV_DECODE_LATENCY_SEC);
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
                // One GEO feeds the correction state (see GnssState::sbas_source).
                // Lock onto the first to decode the PRN mask (MT1): the fast and
                // long-term corrections index into it, so a GEO that emits only
                // fast/iono messages first would lock in unusable while a
                // fully-provisioned GEO sits idle. No usable source exists until
                // some GEO has the mask, so nothing is fed before then.
                if st.sbas_source.is_none() && msg.mtype == 1 {
                    st.sbas_source = Some(self.sv);
                }
                let (iono, corr) = if st.sbas_source == Some(self.sv) {
                    (
                        st.sbas_iono.feed(&msg),
                        st.sbas_corr.feed(&msg, self.ts_sec),
                    )
                } else {
                    (false, false)
                };
                // MT9 (GEO navigation): place this GEO on the sky plot. Its
                // ECEF position + the current fix give elevation/azimuth —
                // the solver never computes them for SBAS channels (no
                // ranging, so they are not in the fix pool).
                let elaz = (msg.mtype == 9 && st.latitude != 0.0).then(|| {
                    let (x, y, z) = map_3d::geodetic2ecef(
                        st.latitude.to_radians(),
                        st.longitude.to_radians(),
                        st.height,
                        map_3d::Ellipsoid::WGS84,
                    );
                    let geo = crate::sbas_corr::mt9_geo_position_ecef(&msg.bits);
                    let (el, az) = crate::models::elevation_azimuth(
                        Vector3::new(x, y, z),
                        (geo[0], geo[1], geo[2]),
                    );
                    (el.to_degrees(), az.to_degrees())
                });
                if let Some(cs) = st.channels.get_mut(&self.sv) {
                    cs.sbas_msgs = self.stats.subframes;
                    cs.sbas_msg_mask |= 1u64 << msg.mtype.min(63);
                    if let Some((el, az)) = elaz {
                        cs.elevation_deg = el;
                        cs.azimuth_deg = az;
                    }
                }
                if iono {
                    format!(" — iono grid: {} IGPs", st.sbas_iono.len())
                } else if corr {
                    format!(
                        " — corrections: {} fast / {} long-term",
                        st.sbas_corr.fast_len(),
                        st.sbas_corr.long_len()
                    )
                } else if let Some((el, az)) = elaz {
                    format!(" — GEO at el={el:.1}° az={az:.1}°")
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
        // Words 1-4 can be the ones completing the ephemeris: anchor from the
        // remembered word-5 pair instead of waiting for the next word 5.
        self.try_anchor_tx();
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
            let week_secs = crate::constants::SECONDS_PER_WEEK as i64;
            let (mut week, mut tow) = (self.nav.eph.week as i64, anchor_tow as i64 + off);
            while tow >= week_secs {
                tow -= week_secs;
                week += 1;
            }
            while tow < 0 {
                tow += week_secs;
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
        // First TOW: now that the exact GPST time is known, swap the coarsely
        // injected assist orbit for the brdc issue nearest it (no recording
        // time-of-day needed). Once anchored the issue is settled, so do it once.
        if !self.nav.meas.tx_anchored {
            self.refine_assist_eph();
        }
        self.nav.eph.ts_sec = self.ts_sec;
        // Remember this TOW edge's (epoch, phase) pair; the anchor can then pin
        // from it at any later moment of the same continuous track.
        self.nav.tow_pair = Some((
            self.nav.eph.tow_gpst + Duration::from_seconds(decode_latency_sec),
            self.nav.meas.trk_phase,
        ));
        self.try_anchor_tx();
        self.assist_crosscheck();
        self.update_gpst_time(self.nav.eph.tow_gpst);
    }

    /// Re-select the injected orbit to the brdc issue nearest the just-decoded
    /// TOW. The initial injection (at channel creation) is a coarse pick from the
    /// day's set; the signal's own time is the exact reference, so this lands on
    /// the issue the SV is actually broadcasting — no recording time-of-day
    /// needed. Keeps the decoded TOW fields; resets decode progress so the
    /// cross-check re-confirms against the freshly adopted issue.
    fn refine_assist_eph(&mut self) {
        if self.nav.assist_set.is_empty() {
            return; // not assisted
        }
        let Some(best) = fresh_eph(&self.nav.assist_set, self.nav.eph.tow_gpst) else {
            // Even the nearest issue is stale — a brdc gap for this SV (the
            // eccentric E14/E18 are the usual culprits). Don't anchor on a stale
            // orbit; drop the assist and let the SV decode its own ephemeris.
            self.nav.eph = Ephemeris::new(self.sv);
            self.nav.assist_eph = None;
            self.nav.assist_set.clear();
            if let Some(cs) = self.pub_state.lock().unwrap().channels.get_mut(&self.sv) {
                cs.assist = false;
            }
            log::warn!(
                "{}: A-GNSS orbit too stale (brdc gap) — decoding on-air instead",
                self.sv
            );
            return;
        };
        if self.nav.assist_eph.map(|a| a.toe) == Some(best.toe) {
            return; // already on the nearest (fresh) issue
        }
        let (tow, tow_gpst) = (self.nav.eph.tow, self.nav.eph.tow_gpst);
        self.nav.eph = best; // complete brdc issue (eph_mask resets to 0)
        self.nav.eph.tow = tow;
        self.nav.eph.tow_gpst = tow_gpst;
        self.nav.assist_eph = Some(best);
        log::warn!(
            "{}: A-GNSS injected orbit refined to toe {} (nearest the decoded TOW)",
            self.sv,
            best.toe
        );
    }

    /// Assisted-GNSS correctness validator: once the on-air ephemeris has fully
    /// decoded (every message part), compare the signal-decoded orbit against the
    /// injected (downloaded) one at the decoded `toe` and log the 3-D difference.
    /// Same broadcast issue (IODE/toe) ⇒ they agree within quantisation; a stale
    /// injected `toe` shows up as the larger, bounded extrapolation gap. A free
    /// check that the right brdc was downloaded/parsed/injected. Fires once.
    fn assist_crosscheck(&mut self) {
        if self.nav.assist_crosschecked {
            return;
        }
        let Some(assist) = self.nav.assist_eph else {
            return;
        };
        // Per-signal "all ephemeris parts decoded" mask (eph_mask bit n = message
        // part n): GPS LNAV SF1-3, Galileo I/NAV words 1-5, F/NAV pages 1-4.
        let complete: u8 = match self.sig {
            Signal::GalileoE1b | Signal::GalileoE1c => 0b0011_1110,
            Signal::GalileoE5aI | Signal::GalileoE5aQ => 0b0001_1110,
            _ => 0b0000_1110, // GPS / QZSS LNAV
        };
        if self.nav.eph.eph_mask & complete != complete {
            return; // on-air ephemeris not fully decoded yet
        }
        let t = self.nav.eph.toe_gpst;
        let (dx, dy, dz) = compute_sv_position_ecef(&self.nav.eph, t);
        let (ax, ay, az) = compute_sv_position_ecef(&assist, t);
        let d = ((dx - ax).powi(2) + (dy - ay).powi(2) + (dz - az).powi(2)).sqrt();
        log::warn!(
            "{}: A-GNSS orbit cross-check: injected vs decoded |Δ|={:.1} m (toe {} vs {}, iode {} vs {})",
            self.sv,
            d,
            assist.toe,
            self.nav.eph.toe,
            assist.iode,
            self.nav.eph.iode,
        );
        // Publish for the UI: the per-SV figure and the session-worst.
        {
            let mut st = self.pub_state.lock().unwrap();
            if let Some(cs) = st.channels.get_mut(&self.sv) {
                cs.assist_delta_m = Some(d);
            }
            if let Some(a) = st.agnss.as_mut() {
                a.max_delta_m = Some(a.max_delta_m.map_or(d, |m| m.max(d)));
            }
        }
        self.nav.assist_crosschecked = true;
    }

    /// Pin the transmit anchor from the remembered TOW pair once the ephemeris
    /// is complete — called both from TOW-bearing messages (via
    /// [`nav_anchor_tx`]) and from any message that completes the ephemeris,
    /// so completion never waits for the *next* TOW message. Pins once:
    /// re-anchoring would reset the reference and turn the absolute inter-SV
    /// range (carried in code_off_sec) into a relative one.
    pub(crate) fn try_anchor_tx(&mut self) {
        if !self.is_ephemeris_complete() {
            return;
        }
        {
            let mut st = self.pub_state.lock().unwrap();
            let cs = st.channels.get_mut(&self.sv).unwrap();
            cs.has_eph = true;
        }
        if self.nav.meas.tx_anchored {
            return;
        }
        let Some((tow_gpst, phase)) = self.nav.tow_pair else {
            return;
        };
        self.nav.meas.tow_trk_phase = phase;
        self.nav.meas.tx_tow_gpst = tow_gpst;
        self.nav.meas.tx_anchored = true;
        if let Some(cs) = self.pub_state.lock().unwrap().channels.get_mut(&self.sv) {
            cs.tx_anchored = true;
        }
        log::warn!(
            "{}: tx anchored tow={:?} phase={:.6}",
            self.sv,
            self.nav.meas.tx_tow_gpst,
            self.nav.meas.tow_trk_phase
        );
    }

    /// Publish the current time-of-week to the shared UI state.
    pub(crate) fn update_gpst_time(&mut self, tow_gpst: Epoch) {
        self.pub_state.lock().unwrap().tow_gpst = tow_gpst;
        (self.pub_state.lock().unwrap().update_func.func)();
    }
}
