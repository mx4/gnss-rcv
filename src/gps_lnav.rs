//! GPS L1 C/A (and QZSS) LNAV codec.
//!
//! **Decode** (driven from the generic `Channel::nav_decode` dispatch in
//! [navigation](crate::navigation)): 50 bps bit sync, frame sync on the 0x8B
//! preamble, the 30-bit-word Hamming parity, and the subframe-1..5 parsers that
//! fill the ephemeris/almanac.
//!
//! **Encode** (test / synthesis): the inverse parity encoder plus a canned valid
//! ephemeris, so tests and the synthetic generator can exercise the decode path
//! (parity → field parse → ephemeris assembly) with no recording.

use crate::{
    bits::{
        bits_equal, bits_opposed, getbits, getbits2, getbitu, getbitu2, hex_str, setbitu, xor_bits,
    },
    channel::Channel,
    constants::{P2_5, P2_19, P2_24, P2_27, P2_29, P2_30, P2_31, P2_33, P2_43, P2_55, SC2RAD},
    ephemeris::Ephemeris,
};
use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Epoch, TimeScale};
use std::collections::VecDeque;

use crate::constants::SECONDS_PER_WEEK;

/// Rolling nav-bit window (bits): one 300-bit subframe plus the *next*
/// subframe's 8-bit preamble — exactly what frame sync and decode look back at
/// (a subframe is accepted only when bracketed by a preamble at both ends).
const NAV_BIT_WINDOW: usize = 308;

/// Bit-edge (50 bps sync) acceptance gate: a candidate edge is taken when the
/// ±-pattern correlation explains the energy (|p| >= r in `nav_sync_symbol`)
/// AND the mean normalized prompt-I energy `r` itself clears this floor — i.e.
/// the bits are strong, not just self-consistent noise.
const THRESHOLD_SYNC: f64 = 0.4;
/// Bit-quality gate while synced: a decoded 20 ms bit whose |mean prompt-I|
/// falls below this is "weak" (the coherent sum nearly cancelled); see
/// `WEAK_BIT_LIMIT` for the hysteresis before bit sync is declared lost.
const THRESHOLD_LOST: f64 = 0.03;
/// Consecutive weak 50 bps bits (|mean prompt-I| < THRESHOLD_LOST) tolerated before
/// declaring bit sync lost. One weak bit is usually a momentary carrier wobble, not
/// a real dropout; the per-subframe parity is the real data gate, so we ride through
/// brief dips (keeping bit sync and the 300-bit alignment) instead of hard-resetting
/// and forcing a full ~7 s re-sync — which on marginal recordings prevented the 3
/// consecutive clean subframes an ephemeris needs.
const WEAK_BIT_LIMIT: usize = 25; // ~0.5 s at 50 bps

/// LNAV 30-bit-word parity masks (IS-GPS-200), shared by the parity check and the
/// encoder.
const MASK: [u32; 6] = [
    0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
];
/// LNAV TLM preamble (8 bits).
const PREAMBLE: u32 = 0x8b;
/// The TLM preamble, MSB first, one bit per byte (= `PREAMBLE` unpacked).
const PREAMBLE_BITS: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
/// One LNAV data bit spans 20 C/A code periods (50 bps over 1 ms codes).
const PERIODS_PER_BIT: usize = 20;
/// Structural decode latency of the LNAV frame: the HOW TOW names the start of
/// the *next* subframe, but the decoder emits a subframe only once that next
/// subframe's 8-bit preamble is demodulated — 8 bits × 20 ms after the instant
/// the TOW names. The anchor pairs the decode-moment phase with `tow +` this
/// latency, making t_tx absolutely correct (verified against synthetic truth
/// by the tx-anchor test).
///
/// An uncorrected latency is NOT harmless: only its time-of-day part folds
/// into the receiver clock bias. t_tx is also the epoch at which the solver
/// evaluates the SV *orbit*, and an orbit evaluated Δt early is displaced by
/// v_sv·Δt along-track, whose line-of-sight projection is range-rate·Δt =
/// λ·doppler·Δt — a per-SV, Doppler-proportional pseudorange bias (0.030 m/Hz
/// for Δt = 0.16 s) that the position solve cannot absorb. That bias was
/// long misattributed to DLL group delay and compensated by a calibrated
/// `doppler/fc·τ` (τ ≈ 0.157 s ≈ this latency) term on the measured code
/// phase; anchoring at the true latency removes the cause (and the term).
pub(crate) const LNAV_DECODE_LATENCY_SEC: f64 = 8.0 * PERIODS_PER_BIT as f64 * 1e-3;
/// Prompt window kept by the decoder: bit-edge detection reads 2·19 periods
/// straddling a candidate edge (see `sync_bit`), plus slack for wrap repeats.
const PROMPT_WINDOW: usize = 40;

#[derive(PartialEq, Debug, Default)]
pub(crate) enum SyncState {
    #[default]
    Normal,
    Reversed,
    None,
}

/// One step of the LNAV decoder (the return of [`LnavState::push_prompt`]).
pub(crate) enum LnavOut {
    None,
    /// A parity-valid subframe: 300 parity-stripped bits, one per byte.
    /// Boxed so the common `None` arm isn't 300 bytes wide (it is returned
    /// every code period; a subframe only every 6 s).
    Subframe(Box<[u8; 300]>),
    /// A frame-synced subframe failed the word parity check.
    ParityError,
}

/// Streaming GPS/QZSS L1 C/A LNAV decoder. Fed one *normalized* prompt
/// (re/|c|) per code period via [`push_prompt`](Self::push_prompt), it finds
/// the 50 bps bit boundary, demodulates bits (20 periods each, with weak-bit
/// hysteresis), frame-syncs on the 0x8B preamble bracketing a 300-bit
/// subframe, checks the 30-bit-word Hamming parity, and emits the subframe.
/// Self-contained: no access to the channel's tracking state. The channel
/// mirrors its code-phase-wrap bookkeeping via [`wrap_drop`](Self::wrap_drop)
/// / [`wrap_repeat`](Self::wrap_repeat) so the prompt stream stays aligned to
/// *transmitted* code periods — the grid the 20-period bits divide.
pub(crate) struct LnavState {
    sv: SV,
    prompts: VecDeque<f64>, // last PROMPT_WINDOW normalized prompts
    count: usize,           // transmitted code periods seen (wrap-corrected)
    bit_sync: usize,        // `count` at a navigation-bit boundary (0 = none)
    nav_sync: usize,        // `count` at the last decoded frame end (0 = none)
    sync_state: SyncState,
    bits: Vec<u8>,      // rolling NAV_BIT_WINDOW navigation bits
    bits_filled: usize, // bits demodulated since the last (re)bit-sync
    weak_bits: usize,   // consecutive weak bits (hysteresis, see WEAK_BIT_LIMIT)
}

impl LnavState {
    pub fn new(sv: SV) -> Self {
        Self {
            sv,
            prompts: VecDeque::with_capacity(PROMPT_WINDOW + 1),
            count: 0,
            bit_sync: 0,
            nav_sync: 0,
            sync_state: SyncState::Normal,
            bits: vec![0; NAV_BIT_WINDOW],
            bits_filled: 0,
            weak_bits: 0,
        }
    }

    pub fn reset(&mut self) {
        self.prompts.clear();
        self.count = 0;
        self.bit_sync = 0;
        self.nav_sync = 0;
        self.sync_state = SyncState::Normal;
        self.bits.fill(0);
        self.bits_filled = 0;
        self.weak_bits = 0;
    }

    /// The channel re-correlated the same transmitted code period (positive
    /// code-phase wrap): withdraw the just-pushed prompt; its replacement
    /// arrives next period.
    pub fn wrap_drop(&mut self) {
        if self.prompts.pop_back().is_some() {
            self.count -= 1;
        }
    }

    /// The channel skipped a transmitted code period (negative code-phase
    /// wrap): repeat the last prompt to keep the period grid continuous.
    pub fn wrap_repeat(&mut self) {
        if let Some(&v) = self.prompts.back() {
            self.prompts.push_back(v);
            if self.prompts.len() > PROMPT_WINDOW {
                self.prompts.pop_front();
            }
            self.count += 1;
        }
    }

    /// Feed one normalized prompt (one code period). Returns a parity-valid
    /// subframe / a parity failure on a frame boundary, `None` otherwise.
    pub fn push_prompt(&mut self, p_norm: f64, ts_sec: f64) -> LnavOut {
        self.prompts.push_back(p_norm);
        if self.prompts.len() > PROMPT_WINDOW {
            self.prompts.pop_front();
        }
        self.count += 1;

        if !self.sync_bit(ts_sec) {
            return LnavOut::None;
        }

        if self.nav_sync > 0 {
            #[allow(clippy::comparison_chain)]
            if self.count == self.nav_sync + 300 * PERIODS_PER_BIT {
                let sync = self.frame_sync_state(ts_sec);
                if sync == self.sync_state {
                    return self.decode_frame(sync);
                }
            } else if self.count > self.nav_sync + 300 * PERIODS_PER_BIT {
                // Missed the frame boundary: drop frame sync and re-find the
                // preamble, but KEEP bit sync — a frame-sync slip shouldn't cost
                // the full ~7 s bit-resync. (Parity errors likewise keep bit sync.)
                self.nav_sync = 0;
                self.sync_state = SyncState::Normal;
            }
        } else if self.bits_filled >= NAV_BIT_WINDOW {
            // Only search for the frame once the 308-bit window holds bits
            // demodulated under the *current* bit sync, not zero-fill/stale ones.
            let sync = self.frame_sync_state(ts_sec);
            if sync != SyncState::None {
                return self.decode_frame(sync);
            }
        }
        LnavOut::None
    }

    /// Advance bit sync by one period. Returns true when a navigation bit was
    /// demodulated this period (i.e. the frame logic should run).
    fn sync_bit(&mut self, ts_sec: f64) -> bool {
        const N: usize = PERIODS_PER_BIT - 1;
        if self.bit_sync == 0 {
            // Edge detection: correlate the last 2N prompts against a -/+ step.
            // Fires only when the step exactly explains the window (|p| >= r)
            // and the signal level itself is strong (r >= THRESHOLD_SYNC).
            if self.prompts.len() < 2 * N {
                return false;
            }
            let len = self.prompts.len();
            let (mut p, mut r) = (0.0, 0.0);
            for i in 0..2 * N {
                let code = if i < N { -1.0 } else { 1.0 };
                let v = self.prompts[len - 2 * N + i];
                p += v * code;
                r += v.abs();
            }
            p /= (2 * N) as f64;
            r /= (2 * N) as f64;

            if p.abs() >= r && r >= THRESHOLD_SYNC {
                self.bit_sync = self.count - N;
                log::info!(
                    "{}: SYNC: p={:.5} ssync={} ts={ts_sec:.3}",
                    self.sv,
                    p,
                    self.bit_sync
                );
            }
        } else if (self.count - self.bit_sync).is_multiple_of(PERIODS_PER_BIT) {
            let p = self.mean_ip(PERIODS_PER_BIT);
            if p.abs() >= THRESHOLD_LOST {
                self.weak_bits = 0; // strong bit: clear the weak streak
            } else {
                // Weak bit (the 20 ms coherent sum nearly cancelled). Ride out a
                // brief run — keep bit sync and still add the bit (its sign),
                // preserving the 300-bit alignment — but declare loss after a
                // sustained streak. Parity is the real gate for the noisy bits.
                self.weak_bits += 1;
                if self.weak_bits >= WEAK_BIT_LIMIT {
                    self.bit_sync = 0;
                    self.sync_state = SyncState::Normal;
                    self.weak_bits = 0;
                    self.bits_filled = 0;
                    log::info!("{}: SYNC {} p={}", self.sv, "LOST".to_string().red(), p);
                    return false;
                }
            }
            self.add_bit(if p >= 0.0 { 1 } else { 0 });
            return true;
        }
        false
    }

    /// Mean of the last `n` prompts (already normalized to re/|c|).
    fn mean_ip(&self, n: usize) -> f64 {
        let len = self.prompts.len();
        let n = n.min(len);
        self.prompts.iter().skip(len - n).sum::<f64>() / n as f64
    }

    fn add_bit(&mut self, bit: u8) {
        self.bits.rotate_left(1);
        *self.bits.last_mut().unwrap() = bit;
        self.bits_filled += 1;
    }

    /// Check for the preamble bracketing the bit window (upright or inverted).
    fn frame_sync_state(&self, ts_sec: f64) -> SyncState {
        let bits = self.bits.as_slice();
        let bits_beg = &bits[0..PREAMBLE_BITS.len()];
        let bits_end = &bits[300..300 + PREAMBLE_BITS.len()];
        let mut sync_state = SyncState::None;

        if bits_equal(&PREAMBLE_BITS, bits_beg) && bits_equal(&PREAMBLE_BITS, bits_end) {
            sync_state = SyncState::Normal;
        } else if bits_opposed(&PREAMBLE_BITS, bits_beg) && bits_opposed(&PREAMBLE_BITS, bits_end) {
            sync_state = SyncState::Reversed;
        }
        if sync_state != SyncState::None {
            log::info!("{}: FRAME SYNC {sync_state:?}: ts={ts_sec:.3}", self.sv);
        }

        sync_state
    }

    /// Parity-check the frame-synced 300-bit subframe and strip the parity.
    fn decode_frame(&mut self, sync: SyncState) -> LnavOut {
        let rev = if sync == SyncState::Normal { 0 } else { 1 };
        // The subframe is the window minus the trailing (next) preamble.
        let bits: Vec<u8> = self.bits[..NAV_BIT_WINDOW - 8]
            .iter()
            .map(|v| v ^ rev)
            .collect();
        let mut nav_data = [0u8; 300];

        if test_lnav_parity(&bits, &mut nav_data) {
            self.nav_sync = self.count;
            self.sync_state = sync;
            LnavOut::Subframe(Box::new(nav_data))
        } else {
            self.nav_sync = 0;
            self.sync_state = SyncState::Normal;
            LnavOut::ParityError
        }
    }
}

impl Channel {
    /// GPS/QZSS LNAV decode entry: feed the streaming decoder one normalized
    /// prompt per code period; on a parity-valid subframe, parse it into the
    /// ephemeris/almanac (the receiver-side bookkeeping the decoder is free of).
    pub(crate) fn nav_decode_gps_lnav(&mut self) {
        let Some(&c_p) = self.hist.corr_p.back() else {
            return;
        };
        // Feed re/|c| (cos of the carrier phase error): corr_p's absolute scale
        // varies with IQ format and recording level (the f32 reader passes raw
        // amplitudes through; int formats only bound them to ~[-1, 1]), while
        // the decoder's sync thresholds are fixed constants in normalized units.
        match self.nav.lnav.push_prompt(c_p.re / c_p.norm(), self.ts_sec) {
            LnavOut::None => {}
            LnavOut::ParityError => {
                self.stats.parity_errors += 1;
                log::warn!("{}: PARITY ERROR", self.sv);
            }
            LnavOut::Subframe(nav_data) => {
                self.stats.subframes += 1;
                let id = self.nav_decode_lnav_subframe(&nav_data[..]);
                log::info!("{}: LNAV: id={id} -- {}", self.sv, hex_str(&nav_data[..]));
            }
        }
    }

    fn nav_decode_lnav_subframe4(&mut self, buf: &[u8]) {
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 6);

        if data_id == 1 {
            let pub_state = &mut self.pub_state.lock().unwrap();
            let alm_array = &mut pub_state.almanac;

            if (25..=32).contains(&svid) {
                let alm = alm_array.get_mut(svid as usize - 1).unwrap();
                alm.nav_decode_alm(buf, svid);
                log::warn!("{}: {:?}", self.sv, alm);
            } else if svid == 63 {
                /* page 25 */
                const ARRAY_SVCONF_IDX: [usize; 32] = [
                    68, 72, 76, 80, 90, 94, 98, 102, 106, 110, 120, 124, 128, 132, 136, 140, 150,
                    154, 158, 162, 166, 170, 180, 184, 188, 192, 196, 200, 210, 214, 218, 222,
                ];

                for sv in 1..=32 {
                    let alm = alm_array.get_mut(sv - 1).unwrap();
                    let pos = ARRAY_SVCONF_IDX[sv - 1];

                    alm.svconf = getbitu(buf, pos, 4);
                }

                const ARRAY_SVH_IDX: [usize; 8] = [228, 240, 246, 252, 258, 270, 276, 282];
                for sv in 25..=32 {
                    let alm = alm_array.get_mut(sv - 1).unwrap();
                    let pos = ARRAY_SVH_IDX[sv - 25];
                    alm.svh = getbitu(buf, pos, 6);
                    if alm.svh != 0 {
                        log::warn!("{}: sv {} is unhealthy", self.sv, sv)
                    }
                }
            } else if svid == 55 {
                // page 17: special message -- 22 eight-bit ASCII characters
                // packed across words 3-10 (two in word 3, three each in words
                // 4-9, two in word 10).
                const MSG_BITS: [usize; 22] = [
                    68, 76, 90, 98, 106, 120, 128, 136, 150, 158, 166, 180, 188, 196, 210, 218,
                    226, 240, 248, 256, 270, 278,
                ];
                let msg: String = MSG_BITS
                    .iter()
                    .map(|&p| getbitu(buf, p, 8) as u8)
                    .map(|c| {
                        if (0x20..0x7f).contains(&c) {
                            c as char
                        } else {
                            '.'
                        }
                    })
                    .collect();
                log::warn!(
                    "{}: {} (SF4 p17): \"{}\"",
                    self.sv,
                    "special msg".blue(),
                    msg
                );
            } else if svid == 56 {
                /* page 18 */
                // handle iono, utc and leap seconds
                let mut ion = [0.0; 8];

                ion[0] = getbits(buf, 68, 8) as f64 * P2_30;
                ion[1] = getbits(buf, 76, 8) as f64 * P2_27;
                ion[2] = getbits(buf, 90, 8) as f64 * P2_24;
                ion[3] = getbits(buf, 98, 8) as f64 * P2_24;
                ion[4] = getbits(buf, 106, 8) as f64 * 2.0_f64.powi(11);
                ion[5] = getbits(buf, 120, 8) as f64 * 2.0_f64.powi(14);
                ion[6] = getbits(buf, 128, 8) as f64 * 2.0_f64.powi(16);
                ion[7] = getbits(buf, 136, 8) as f64 * 2.0_f64.powi(16);

                pub_state.iono_alpha = [ion[0], ion[1], ion[2], ion[3]];
                pub_state.iono_beta = [ion[4], ion[5], ion[6], ion[7]];
                pub_state.ion_adj = true;

                // Page 18 also carries the GPS->UTC conversion (A0/A1, ref time
                // tot/WNt) and the leap-second count: dt_ls is the current
                // GPS-UTC offset; if it differs from dt_lsf a leap second is
                // scheduled at week wn_lsf, day-of-week dn.
                let a1 = getbits(buf, 150, 24) as f64 * 2.0_f64.powi(-50);
                let a0 = getbits2(buf, 180, 24, 210, 8) as f64 * P2_30;
                let tot = getbitu(buf, 218, 8) * 4096;
                let wnt = getbitu(buf, 226, 8);
                let dt_ls = getbits(buf, 240, 8);
                let wn_lsf = getbitu(buf, 248, 8);
                let dn = getbitu(buf, 256, 8);
                let dt_lsf = getbits(buf, 270, 8);
                pub_state.utc_adj = true;
                let leap = if dt_ls != dt_lsf {
                    format!(" -- LEAP SECOND scheduled: {dt_lsf}s at week {wn_lsf} day {dn}")
                } else {
                    String::new()
                };
                log::warn!(
                    "{}: {} (SF4 p18): leap={dt_ls}s A0={a0:+.3e}s A1={a1:+.3e}s/s tot={tot} WNt={wnt}{leap}",
                    self.sv,
                    "UTC/leap".blue(),
                );
            }
        }

        log::warn!(
            "{}: {}: data_id={data_id} svid={svid} tow={}",
            self.sv,
            "subframe-4".blue(),
            self.nav.eph.tow
        );
    }

    fn nav_decode_lnav_subframe5(&mut self, buf: &[u8]) {
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 6);
        let alm_array = &mut self.pub_state.lock().unwrap().almanac;

        if data_id == 1 {
            if (1..=24).contains(&svid) {
                let alm = alm_array.get_mut(svid as usize - 1).unwrap();
                alm.nav_decode_alm(buf, svid);
                log::warn!("{}: {:?}", self.sv, alm);
            } else if svid == 51 {
                let toas = getbitu(buf, 68, 8) * 4096;
                // The almanac reference week WNa is only 8 bits (mod 256), so
                // unlike subframe 1's 10-bit field "+2048" reconstructs the full
                // week only for weeks 2048..2303 (2019-04 through 2024-03);
                // later captures get a stale week here. Almanac data is
                // display-only (the fix uses the ephemeris), so the skew is
                // cosmetic; the proper fix is anchoring WNa to subframe 1's week
                // (same "GPS week rollover" roadmap item).
                let week = getbitu(buf, 76, 8) + 2048;

                const ARRAY_SVH_IDX: [usize; 24] = [
                    90, 96, 102, 108, 120, 126, 132, 138, 150, 156, 162, 168, 180, 186, 192, 198,
                    210, 216, 222, 228, 240, 246, 252, 258,
                ];
                for sv in 1..=24 {
                    let alm = alm_array.get_mut(sv - 1).unwrap();
                    let pos = ARRAY_SVH_IDX[sv - 1];
                    alm.svh = getbitu(buf, pos, 6);
                    if alm.svh != 0 {
                        log::warn!("{}: sv {} is unhealthy", self.sv, sv)
                    }
                }
                for sv in 1..=32 {
                    let alm = alm_array.get_mut(sv - 1).unwrap();
                    alm.week = week;
                    alm.toas = toas;
                }
            } else {
                log::warn!("XXX unknown svid={}", svid);
            }
        }

        log::warn!(
            "{}: {}: data_id={data_id} svid={svid} tow={}",
            self.sv,
            "subframe-5".blue(),
            self.nav.eph.tow
        );
    }

    fn nav_subframe_post(&mut self) {
        // GPS LNAV carries the TOW in every subframe's HOW, so build the GPST
        // epochs and run the shared transmit-time anchor each subframe (the
        // anchor itself pins only once — see Channel::nav_anchor_tx).
        if self.nav.eph.week != 0 {
            let week_to_secs = self.nav.eph.week * SECONDS_PER_WEEK;
            self.nav.eph.tow_gpst =
                Epoch::from_gpst_seconds((week_to_secs + self.nav.eph.tow).into());
            self.nav.eph.toe_gpst =
                Epoch::from_gpst_seconds((week_to_secs + self.nav.eph.toe).into());
            self.nav.eph.toc_gpst =
                Epoch::from_gpst_seconds((week_to_secs + self.nav.eph.toc).into());
            log::warn!(
                "{}: tow={:?} tgd={:+.3e} toe={:?}",
                self.sv,
                self.nav.eph.tow_gpst,
                self.nav.eph.tgd,
                self.nav.eph.toe_gpst
            );
            self.nav_anchor_tx(LNAV_DECODE_LATENCY_SEC);
        }
    }

    fn nav_decode_lnav_subframe(&mut self, buf: &[u8]) -> u32 {
        let preamble = getbitu(buf, 0, 8);
        assert_eq!(preamble, 0x8b);
        self.nav.eph.tlm = getbitu(buf, 8, 14);
        let _isf = getbitu(buf, 22, 1);
        let _rsvd = getbitu(buf, 23, 1);
        let _alert = getbitu(buf, 47, 1);
        let _anti_spoof = getbitu(buf, 48, 1);
        let subframe_id = getbitu(buf, 49, 3);
        let zero = getbitu(buf, 58, 2);
        assert_eq!(zero, 0);

        match subframe_id {
            1 => decode_lnav_subframe1(&mut self.nav.eph, buf, self.sv, self.week_anchor),
            2 => decode_lnav_subframe2(&mut self.nav.eph, buf, self.sv),
            3 => decode_lnav_subframe3(&mut self.nav.eph, buf, self.sv),
            4 => self.nav_decode_lnav_subframe4(buf),
            5 => self.nav_decode_lnav_subframe5(buf),
            _ => log::warn!("{}: invalid subframe id={subframe_id}", self.sv),
        }

        self.nav_subframe_post();

        subframe_id
    }
}

/// Check the 30-bit-word Hamming parity of a 300-bit subframe (one bit per
/// byte) and write the 240 parity-stripped data bits into `nav_data`
/// (`setbitu` layout, parity bits zeroed). Returns false on the first bad word.
pub(crate) fn test_lnav_parity(bits: &[u8], nav_data: &mut [u8]) -> bool {
    assert_eq!(bits.len(), 300);

    let mut data: u32 = 0;
    for i in 0..10 {
        for j in 0..30 {
            data = (data << 1) | bits[i * 30 + j] as u32;
        }
        if data & (1 << 30) != 0 {
            data ^= 0x3FFFFFC0;
        }
        #[allow(clippy::needless_range_loop)]
        for j in 0..6 {
            let v0 = (data >> 6) & MASK[j];
            let v1: u8 = ((data >> (5 - j)) & 1) as u8;
            if xor_bits(v0) != v1 {
                return false;
            }
        }
        setbitu(nav_data, 30 * i, 24, (data >> 6) & 0xFFFFFF);
        setbitu(nav_data, 30 * i + 24, 6, 0);
    }
    true
}

// --- LNAV subframe field parsers --------------------------------------------
//
// Fill the generic `Ephemeris` (constellation-agnostic orbit+clock data) from
// the GPS LNAV subframe bit fields (IS-GPS-200). Galileo's I/NAV parser fills the
// same struct from its own word layout.

/// Reconstruct the full GPS week from the broadcast 10-bit field (mod 1024,
/// IS-GPS-200 20.3.3.3.1.1). That field alone is ambiguous every 1024 weeks
/// (~19.6 yr), so we anchor to `anchor_week` (the receiver's known time — the
/// system clock, or `--eph-date`) and pick the most recent 1024-week era that is
/// *not in the future*, since a recording can never be from the future. Correct
/// for any capture within ~19.6 yr before the anchor (older than that is
/// genuinely ambiguous). The +1 week guards against minor clock skew. Pass
/// `u32::MAX` to disable the resolver (raw + 2048 = the latest era).
pub(crate) fn resolve_gps_week(raw10: u32, anchor_week: u32) -> u32 {
    let mut week = raw10 + 2048; // newest era: weeks 2048.. = 2019-04-07 onward
    while week > anchor_week.saturating_add(1) {
        week -= 1024;
    }
    week
}

/// The current GPS week from the system clock — the default rollover anchor.
/// Falls back to the newest era's first week if the clock is unavailable.
pub fn current_gps_week() -> u32 {
    // to_time_of_week() counts weeks from the epoch's *time-scale* reference, so
    // rebase to GPST (1980-01-06) first — otherwise Epoch::now() (UTC/TAI) yields
    // weeks since ~1900 and the rollover anchor is uselessly far in the future.
    Epoch::now()
        .map(|e| e.to_time_scale(TimeScale::GPST).to_time_of_week().0)
        .unwrap_or(2048)
}

/// Subframe 1: GPS week + clock (af0/af1/af2, tgd, toc). `anchor_week` resolves
/// the 10-bit week's rollover (see [`resolve_gps_week`]).
pub(crate) fn decode_lnav_subframe1(eph: &mut Ephemeris, buf: &[u8], sv: SV, anchor_week: u32) {
    eph.eph_mask |= 1 << 1; // decode progress: subframe 1 of 3 (clock)
    eph.tow = getbitu(buf, 30, 17) * 6;
    eph.week = resolve_gps_week(getbitu(buf, 60, 10), anchor_week);
    eph.code = getbitu(buf, 70, 2);
    eph.sva = getbitu(buf, 72, 4);
    eph.svh = getbitu(buf, 76, 6);
    eph.iodc = getbitu2(buf, 82, 2, 210, 8);
    eph.flag = getbitu(buf, 90, 1);
    eph.tgd = getbits(buf, 196, 8) as f64 * P2_31;
    eph.toc = getbitu(buf, 218, 16) * 16;
    eph.f2 = getbits(buf, 240, 8) as f64 * P2_55;
    eph.f1 = getbits(buf, 248, 16) as f64 * P2_43;
    eph.f0 = getbits(buf, 270, 22) as f64 * P2_31;
    log::warn!(
        "{sv}: {} tow={} week={} code={} sva={} svh={} iodc={} tgd={:+.3e} toc={} f0={:+.3e} f1={:+.3e} f2={:+.3e}",
        "subframe-1".blue(),
        eph.tow,
        eph.week,
        eph.code,
        eph.sva,
        eph.svh,
        eph.iodc,
        eph.tgd,
        eph.toc,
        eph.f0,
        eph.f1,
        eph.f2
    );
}

/// Subframe 2: orbit size/shape (M0, e, √A, Crs, Cuc, Cus, Δn, toe).
pub(crate) fn decode_lnav_subframe2(eph: &mut Ephemeris, buf: &[u8], sv: SV) {
    eph.eph_mask |= 1 << 2; // decode progress: subframe 2 of 3 (orbit size/shape)
    eph.tow = getbitu(buf, 30, 17) * 6;
    eph.iode = getbitu(buf, 60, 8);
    eph.crs = getbits(buf, 68, 16) as f64 * P2_5;
    eph.deln = getbits(buf, 90, 16) as f64 * P2_43 * SC2RAD;
    eph.m0 = getbits2(buf, 106, 8, 120, 24) as f64 * P2_31 * SC2RAD;
    eph.ecc = getbitu2(buf, 166, 8, 180, 24) as f64 * P2_33;
    eph.cuc = getbits(buf, 150, 16) as f64 * P2_29;
    eph.cus = getbits(buf, 210, 16) as f64 * P2_29;
    let sqrt_a = getbitu2(buf, 226, 8, 240, 24) as f64 * P2_19;
    eph.a = sqrt_a * sqrt_a;
    eph.toe = getbitu(buf, 270, 16) * 16;
    eph.fit = getbitu(buf, 286, 1);
    log::warn!(
        "{sv}: {} tow={} a={:.2} iode={} crs={} crc={} cuc={:+.3e} cus={:+.3e} ecc={:+.3e} m0={:+.4e} toe={}",
        "subframe-2".blue(),
        eph.tow,
        eph.a,
        eph.iode,
        eph.crs,
        eph.crc,
        eph.cuc,
        eph.cus,
        eph.ecc,
        eph.m0,
        eph.toe,
    );
}

/// Subframe 3: orientation (Ω0, i0, ω, ΩDOT, IDOT, Cic, Cis, Crc).
pub(crate) fn decode_lnav_subframe3(eph: &mut Ephemeris, buf: &[u8], sv: SV) {
    eph.eph_mask |= 1 << 3; // decode progress: subframe 3 of 3 (orientation)
    eph.tow = getbitu(buf, 30, 17) * 6;
    eph.cic = getbits(buf, 60, 16) as f64 * P2_29;
    eph.cis = getbits(buf, 120, 16) as f64 * P2_29;
    eph.omg0 = getbits2(buf, 76, 8, 90, 24) as f64 * P2_31 * SC2RAD;
    eph.i0 = getbits2(buf, 136, 8, 150, 24) as f64 * P2_31 * SC2RAD;
    eph.crc = getbits(buf, 180, 16) as f64 * P2_5;
    eph.omg = getbits2(buf, 196, 8, 210, 24) as f64 * P2_31 * SC2RAD;
    eph.omg_dot = getbits(buf, 240, 24) as f64 * P2_43 * SC2RAD;
    eph.iode = getbitu(buf, 270, 8);
    eph.i_dot = getbits(buf, 278, 14) as f64 * P2_43 * SC2RAD;
    log::warn!(
        "{sv}: {} tow={} cic={:+e} cis={:+e} omg={:.3} omg0={:.3} omgd={:+.3e} i0={:+.3e} idot={:+.3e}",
        "subframe-3".blue(),
        eph.tow,
        eph.cic,
        eph.cis,
        eph.omg,
        eph.omg0,
        eph.omg_dot,
        eph.i0,
        eph.i_dot
    );
}

// --- LNAV encoder (the inverse of nav_test_lnav_parity) ---------------------
//
// Produces parity-correct, frame-syncable subframes carrying a canned ephemeris.
// `nav_test_lnav_parity` resets its parity register per subframe, so each subframe
// is encoded independently with D29* = D30* = 0; that keeps every transmitted
// preamble clean (0x8B) and needs no end-of-subframe "t-bit" solving.

/// Encode one 30-bit word from its 24 source data bits (low 24 bits, D1 = bit23)
/// and the previous word's transmitted (D29*, D30*). Returns the 30 transmitted
/// bits (D1 = bit29) and the new (D29, D30).
fn encode_word(source24: u32, d29star: u32, d30star: u32) -> (u32, u32, u32) {
    // Parity input mirrors the decoder's (data>>6) layout: source data in bits
    // 0..23, then D30* at bit 24 and D29* at bit 25.
    let pv = (source24 & 0xFF_FFFF) | (d30star << 24) | (d29star << 25);
    let mut parity = 0u32;
    for m in MASK {
        parity = (parity << 1) | xor_bits(pv & m) as u32; // D25..D30
    }
    // Transmitted data = source XOR D30* (complemented when D30* == 1); the
    // decoder undoes this via the same D30*.
    let tx_data = if d30star == 1 {
        (!source24) & 0xFF_FFFF
    } else {
        source24 & 0xFF_FFFF
    };
    let word = (tx_data << 6) | parity;
    (word, (parity >> 1) & 1, parity & 1) // new D29 = P4, D30 = P5
}

/// Encode a subframe whose data bits are set (byte-packed, `setbitu`-style, at
/// the documented offsets; the 6 parity bits per word left 0) into the 300
/// transmitted bits, one bit per byte -- the form the receiver's bit stream and
/// parity decoder consume.
pub fn encode_subframe(source: &[u8]) -> [u8; 300] {
    let mut out = [0u8; 300];
    let (mut d29, mut d30) = (0u32, 0u32);
    for i in 0..10 {
        let s24 = getbitu(source, 30 * i, 24); // 24 source data bits, D1 = MSB
        let (word, n29, n30) = encode_word(s24, d29, d30);
        for j in 0..30 {
            out[30 * i + j] = ((word >> (29 - j)) & 1) as u8;
        }
        (d29, d30) = (n29, n30);
    }
    out
}

/// Set a value split across two bit ranges (high bits then low bits), the layout
/// `getbitu2`/`getbits2` read.
fn set_split(s: &mut [u8; 300], p_hi: usize, l_lo: usize, p_lo: usize, v: u32) {
    setbitu(s, p_hi, 8, v >> l_lo);
    setbitu(s, p_lo, l_lo, v & ((1 << l_lo) - 1));
}

/// Write the low `len` two's-complement bits of `val` at `pos` (the layout
/// `getbits` sign-extends back). Works for unsigned values too.
fn set_i(s: &mut [u8; 300], pos: usize, len: usize, val: i64) {
    setbitu(s, pos, len, (val as u64 & ((1u64 << len) - 1)) as u32);
}

/// Like [`set_i`] but split across the (8-bit hi, `l_lo`-bit lo) ranges that
/// `getbits2`/`getbitu2` read.
fn set_i_split(s: &mut [u8; 300], p_hi: usize, p_lo: usize, l_lo: usize, val: i64) {
    let bits = (val as u64 & ((1u64 << (8 + l_lo)) - 1)) as u32;
    set_split(s, p_hi, l_lo, p_lo, bits);
}

/// Encode `eph` into one LNAV subframe *source* layout (24 data bits per word
/// at the documented offsets, parity bits zeroed — run [`encode_subframe`] on
/// the result for the transmitted bits). `how_field` is the 17-bit HOW TOW
/// count: the TOW of the *next* subframe's start divided by 6 (IS-GPS-200
/// 20.3.3.2 — the convention gps-sdr-sim transmits and the receiver decodes).
/// Field offsets and scales are the exact inverse of
/// `decode_lnav_subframe{1,2,3}`.
pub fn encode_lnav_subframe_source(eph: &Ephemeris, subframe_id: u8, how_field: u32) -> [u8; 300] {
    let r = |x: f64| x.round() as i64;
    let mut s = [0u8; 300];
    setbitu(&mut s, 0, 8, PREAMBLE);
    setbitu(&mut s, 30, 17, how_field);
    setbitu(&mut s, 49, 3, subframe_id as u32);
    match subframe_id {
        1 => {
            setbitu(&mut s, 60, 10, eph.week - 2048);
            setbitu(&mut s, 70, 2, eph.code);
            setbitu(&mut s, 72, 4, eph.sva);
            setbitu(&mut s, 76, 6, eph.svh);
            setbitu(&mut s, 82, 2, eph.iodc >> 8);
            setbitu(&mut s, 210, 8, eph.iodc & 0xFF);
            setbitu(&mut s, 90, 1, eph.flag);
            set_i(&mut s, 196, 8, r(eph.tgd / P2_31));
            setbitu(&mut s, 218, 16, eph.toc / 16);
            set_i(&mut s, 240, 8, r(eph.f2 / P2_55));
            set_i(&mut s, 248, 16, r(eph.f1 / P2_43));
            set_i(&mut s, 270, 22, r(eph.f0 / P2_31));
        }
        2 => {
            setbitu(&mut s, 60, 8, eph.iode);
            set_i(&mut s, 68, 16, r(eph.crs / P2_5));
            set_i(&mut s, 90, 16, r(eph.deln / (P2_43 * SC2RAD)));
            set_i_split(&mut s, 106, 120, 24, r(eph.m0 / (P2_31 * SC2RAD)));
            set_i(&mut s, 150, 16, r(eph.cuc / P2_29));
            set_i_split(&mut s, 166, 180, 24, r(eph.ecc / P2_33));
            set_i(&mut s, 210, 16, r(eph.cus / P2_29));
            set_i_split(&mut s, 226, 240, 24, r(eph.a.sqrt() / P2_19));
            setbitu(&mut s, 270, 16, eph.toe / 16);
            setbitu(&mut s, 286, 1, eph.fit);
        }
        3 => {
            set_i(&mut s, 60, 16, r(eph.cic / P2_29));
            set_i_split(&mut s, 76, 90, 24, r(eph.omg0 / (P2_31 * SC2RAD)));
            set_i(&mut s, 120, 16, r(eph.cis / P2_29));
            set_i_split(&mut s, 136, 150, 24, r(eph.i0 / (P2_31 * SC2RAD)));
            set_i(&mut s, 180, 16, r(eph.crc / P2_5));
            set_i_split(&mut s, 196, 210, 24, r(eph.omg / (P2_31 * SC2RAD)));
            set_i(&mut s, 240, 24, r(eph.omg_dot / (P2_43 * SC2RAD)));
            setbitu(&mut s, 270, 8, eph.iode);
            set_i(&mut s, 278, 14, r(eph.i_dot / (P2_43 * SC2RAD)));
        }
        _ => unreachable!("only subframes 1-3 carry the ephemeris"),
    }
    s
}

/// Round-trip `eph` through the LNAV encode → field-parse path: the result is
/// the ephemeris exactly as a receiver will decode it, every field quantized
/// to its broadcast LSB. The geometric signal generator flies *this* orbit, so
/// generator and solver use bit-identical ephemerides.
pub fn quantize_via_lnav(eph: &Ephemeris) -> Ephemeris {
    let mut q = Ephemeris::new(eph.sv);
    decode_lnav_subframe1(
        &mut q,
        &encode_lnav_subframe_source(eph, 1, 1),
        eph.sv,
        u32::MAX,
    );
    decode_lnav_subframe2(&mut q, &encode_lnav_subframe_source(eph, 2, 1), eph.sv);
    decode_lnav_subframe3(&mut q, &encode_lnav_subframe_source(eph, 3, 1), eph.sv);
    q.ts_sec = 1.0;
    q
}

/// The three source subframes (1, 2, 3) of a canned, valid GPS ephemeris. Field
/// offsets/scales match [ephemeris.rs](crate::ephemeris)'s decoders. Decoded
/// values: a ≈ 26,560 km, ecc ≈ 0.005, i0 ≈ 0.97 rad, omg_dot ≈ -8e-9 rad/s,
/// week 2348, toc/toe 36,000 s.
pub fn canned_ephemeris_subframes() -> [[u8; 300]; 3] {
    let p2 = |n: i32| 2f64.powi(n);
    let sc2rad = std::f64::consts::PI; // 1 semicircle = π rad

    let mut sf = [[0u8; 300]; 3];
    for (k, s) in sf.iter_mut().enumerate() {
        setbitu(s, 0, 8, PREAMBLE);
        setbitu(s, 30, 17, 100 + k as u32); // HOW TOW count (×6 s)
        setbitu(s, 49, 3, k as u32 + 1); // subframe id
    }

    // subframe 1 — week + clock reference time
    let s = &mut sf[0];
    setbitu(s, 60, 10, 300); // week (+2048 -> 2348)
    setbitu(s, 218, 16, 2250); // toc (×16 -> 36,000 s)
    setbitu(
        s,
        270,
        22,
        ((-1e-4 * p2(31)).round() as i32 as u32) & 0x3F_FFFF,
    ); // af0 ~ -1e-4 s

    // subframe 2 — orbit size/shape + ephemeris reference time
    let s = &mut sf[1];
    setbitu(s, 60, 8, 10); // iode
    set_split(s, 166, 24, 180, (0.005 * p2(33)).round() as u32); // ecc
    set_split(
        s,
        226,
        24,
        240,
        (26_560_000f64.sqrt() * p2(19)).round() as u32,
    ); // sqrt_a
    setbitu(s, 270, 16, 2250); // toe (×16 -> 36,000 s)

    // subframe 3 — inclination + rate of right ascension
    let s = &mut sf[2];
    set_split(s, 136, 24, 150, (0.97 / sc2rad * p2(31)).round() as u32); // i0
    setbitu(
        s,
        240,
        24,
        ((-8e-9 / sc2rad * p2(43)).round() as i32 as u32) & 0xFF_FFFF,
    ); // omg_dot
    setbitu(s, 270, 8, 10); // iode

    sf
}

/// A continuous ±1 nav-bit stream (50 bps) cycling the canned subframes 1→2→3
/// `repeats` times -- feed as `SynthSv.nav_bits` to drive the full decode path.
pub fn ephemeris_nav_bits(repeats: usize) -> Vec<i8> {
    let sfs = canned_ephemeris_subframes();
    let mut out = Vec::with_capacity(repeats * 3 * 300);
    for _ in 0..repeats {
        for sf in &sfs {
            for &b in encode_subframe(sf).iter() {
                out.push(if b == 1 { 1 } else { -1 });
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_week_rollover_resolves_to_latest_past_era() {
        let now = 2424; // ~mid-2026
        // a recent (2025, week ~2360) capture: no rollover, raw + 2048.
        assert_eq!(resolve_gps_week(312, now), 2360);
        // a 2017 capture (true week 1960, raw = 1960 mod 1024 = 936): the naive
        // +2048 gives 2984 (year 2037, in the future) → subtract one 1024-week
        // era back to 1960, the correct past era.
        assert_eq!(resolve_gps_week(936, now), 1960);
        // a 2013 capture (true week 1740, raw 716): 2764 → 1740.
        assert_eq!(resolve_gps_week(716, now), 1740);
        // a capture exactly at the anchor is kept (no spurious subtraction).
        assert_eq!(resolve_gps_week(376, now), 2424);
        // u32::MAX disables the resolver (latest era), as the tests rely on.
        assert_eq!(resolve_gps_week(936, u32::MAX), 2984);
    }
    use gnss_rs::constellation::Constellation;

    // Hermetic decode-pipeline test: the canned subframes must pass the receiver's
    // *real* parity decoder, recover their exact source bits, and parse into a
    // valid ephemeris -- in <1 ms, no recording. (Acquire/track is covered by the
    // synth tests; a full signal→fix needs geometry-consistent code phases.)
    #[test]
    fn encodes_a_parity_valid_decodable_ephemeris() {
        let sv = SV::new(Constellation::GPS, 5);
        let mut eph = Ephemeris::new(sv);
        eph.ts_sec = 1.0; // the channel timestamps the first subframe; emulate it

        for (k, source) in canned_ephemeris_subframes().iter().enumerate() {
            let tx = encode_subframe(source);
            let mut nav_data = vec![0u8; 300];
            assert!(
                test_lnav_parity(&tx, &mut nav_data),
                "subframe {} must pass the receiver's parity check",
                k + 1
            );
            assert_eq!(
                &nav_data[..],
                &source[..],
                "subframe {} data must round-trip through encode→parity-decode",
                k + 1
            );
            match k {
                0 => decode_lnav_subframe1(&mut eph, &nav_data, sv, u32::MAX),
                1 => decode_lnav_subframe2(&mut eph, &nav_data, sv),
                2 => decode_lnav_subframe3(&mut eph, &nav_data, sv),
                _ => unreachable!(),
            }
        }

        assert!(eph.is_valid(), "decoded ephemeris should be valid");
        assert_eq!(eph.week, 2348);
        assert!((eph.a - 26_560_000.0).abs() < 50_000.0, "a = {}", eph.a);
        assert!((eph.ecc - 0.005).abs() < 1e-4, "ecc = {}", eph.ecc);
        assert!((eph.i0 - 0.97).abs() < 1e-3, "i0 = {}", eph.i0);
        assert!(
            eph.omg_dot < 0.0 && eph.omg_dot.abs() > 1e-9,
            "omg_dot = {}",
            eph.omg_dot
        );
    }

    // The decoder is a self-contained state machine: fed normalized prompts
    // (20 per bit, rendered from the canned encoder), it must bit-sync,
    // frame-sync and emit parity-valid subframes — no Channel, no recording.
    // This is the hermetic LNAV-pipeline test the Channel coupling used to
    // make impossible.
    #[test]
    fn lnav_decoder_decodes_subframes_from_raw_prompts() {
        let sv = SV::new(Constellation::GPS, 7);
        let mut dec = LnavState::new(sv);
        let (mut subframes, mut parity_errors) = (0, 0);

        for &b in &ephemeris_nav_bits(3) {
            for _ in 0..PERIODS_PER_BIT {
                match dec.push_prompt(b as f64, 0.0) {
                    LnavOut::Subframe(data) => {
                        subframes += 1;
                        // every emitted subframe carries the TLM preamble
                        assert_eq!(getbitu(&data[..], 0, 8), PREAMBLE);
                    }
                    LnavOut::ParityError => parity_errors += 1,
                    LnavOut::None => {}
                }
            }
        }

        // 9 subframes in the stream; bit sync + filling the 308-bit window
        // consume roughly the first one, the rest must decode cleanly.
        assert!(subframes >= 5, "decoded only {subframes} subframes");
        assert_eq!(parity_errors, 0, "clean stream must not parity-fail");
    }

    // Every ephemeris field must survive encode → parity-encode → parity-decode
    // → field-parse within its broadcast LSB. This locks the encoder to the
    // decoders bit-for-bit — the property the geometric synth (GeoFeed) relies
    // on when it flies the quantized ephemeris the receiver will decode.
    #[test]
    fn encode_lnav_subframes_roundtrip_all_fields() {
        let sv = SV::new(Constellation::GPS, 9);
        let mut e = Ephemeris::new(sv);
        e.week = 2348;
        e.code = 1;
        e.sva = 2;
        e.svh = 0;
        e.iodc = 0x2A5;
        e.flag = 1;
        e.tgd = -4.6e-9;
        e.toc = 36_000;
        e.f2 = 2.0e-16;
        e.f1 = -3.2e-12;
        e.f0 = 4.2e-4;
        e.iode = 0x55;
        e.crs = -23.4;
        e.deln = 4.5e-9;
        e.m0 = -1.234;
        e.cuc = 1.2e-6;
        e.ecc = 0.0123;
        e.cus = -7.8e-6;
        e.a = 26_559_800.0;
        e.toe = 36_000;
        e.cic = 9.3e-8;
        e.omg0 = 2.345;
        e.cis = -1.1e-7;
        e.i0 = 0.958;
        e.crc = 200.5;
        e.omg = -2.9;
        e.omg_dot = -8.1e-9;
        e.i_dot = 3.0e-10;

        // Through the full transmitted path: source -> parity encode -> parity
        // decode -> field parse.
        let mut q = Ephemeris::new(sv);
        for id in 1..=3u8 {
            let src = encode_lnav_subframe_source(&e, id, 6001);
            let tx = encode_subframe(&src);
            let mut nav = vec![0u8; 300];
            assert!(test_lnav_parity(&tx, &mut nav), "subframe {id} parity");
            assert_eq!(&nav[..], &src[..], "subframe {id} source bits round-trip");
            match id {
                1 => decode_lnav_subframe1(&mut q, &nav, sv, u32::MAX),
                2 => decode_lnav_subframe2(&mut q, &nav, sv),
                _ => decode_lnav_subframe3(&mut q, &nav, sv),
            }
        }
        q.ts_sec = 1.0;
        assert!(q.is_valid());

        // Integer fields are exact.
        assert_eq!(q.tow, 6001 * 6);
        assert_eq!(q.week, e.week);
        assert_eq!(q.toc, e.toc);
        assert_eq!(q.toe, e.toe);
        assert_eq!(q.iodc, e.iodc);
        assert_eq!(q.iode, e.iode);
        assert_eq!((q.code, q.sva, q.svh, q.flag), (1, 2, 0, 1));

        // Scaled fields within half their LSB.
        let near = |got: f64, want: f64, lsb: f64, name: &str| {
            assert!(
                (got - want).abs() <= lsb,
                "{name}: {got:e} vs {want:e} (lsb {lsb:e})"
            );
        };
        near(q.tgd, e.tgd, P2_31, "tgd");
        near(q.f2, e.f2, P2_55, "f2");
        near(q.f1, e.f1, P2_43, "f1");
        near(q.f0, e.f0, P2_31, "f0");
        near(q.crs, e.crs, P2_5, "crs");
        near(q.crc, e.crc, P2_5, "crc");
        near(q.deln, e.deln, P2_43 * SC2RAD, "deln");
        near(q.omg_dot, e.omg_dot, P2_43 * SC2RAD, "omg_dot");
        near(q.i_dot, e.i_dot, P2_43 * SC2RAD, "i_dot");
        near(q.m0, e.m0, P2_31 * SC2RAD, "m0");
        near(q.omg0, e.omg0, P2_31 * SC2RAD, "omg0");
        near(q.omg, e.omg, P2_31 * SC2RAD, "omg");
        near(q.i0, e.i0, P2_31 * SC2RAD, "i0");
        near(q.ecc, e.ecc, P2_33, "ecc");
        near(q.cuc, e.cuc, P2_29, "cuc");
        near(q.cus, e.cus, P2_29, "cus");
        near(q.cic, e.cic, P2_29, "cic");
        near(q.cis, e.cis, P2_29, "cis");
        near(q.a, e.a, 0.1, "a"); // sqrt_a LSB P2_19 -> ~2 cm in a

        // quantize_via_lnav is the same path.
        let qq = quantize_via_lnav(&e);
        assert_eq!(qq.m0, q.m0);
        assert_eq!(qq.a, q.a);
        assert_eq!(qq.week, q.week);
    }

    fn hex_to_bytes(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    // Real parity-stripped LNAV subframes captured from G01 in the gpssim_2xi16
    // fixture (the `LNAV: id=N -- <hex>` log lines). Decoding them must recover a
    // physically plausible GPS ephemeris. This locks the bit-field offsets in
    // decode_lnav_subframe{1,2,3}; a regression there would fail the ranges.
    const G01_SF1: &str =
        "8b00000130bc1805c10020000000000000000000000000000ec0174e808000ffb20096b22400";
    const G01_SF2: &str =
        "8b00000130b42405df43d00cc12000adc63303d76c000e2306f0048c28400cef4100e8080c00";
    const G01_SF3: &str =
        "8b00000130b63c0fff05603c788700fff727000a987401a420701c6e3280ffa6fb0177be6400";

    #[test]
    fn decodes_real_lnav_subframes_to_a_valid_ephemeris() {
        let pi = std::f64::consts::PI;
        let sv = SV::new(Constellation::GPS, 1);
        let mut e = Ephemeris::new(sv);
        // The receiver sets ts_sec when it timestamps the first subframe; mimic.
        e.ts_sec = 1.0;

        decode_lnav_subframe1(&mut e, &hex_to_bytes(G01_SF1), sv, u32::MAX);
        decode_lnav_subframe2(&mut e, &hex_to_bytes(G01_SF2), sv);
        decode_lnav_subframe3(&mut e, &hex_to_bytes(G01_SF3), sv);

        // Subframe 1 (clock / week).
        assert!(
            (2048..3000).contains(&e.week),
            "GPS week {} implausible",
            e.week
        );
        assert_ne!(e.toc, 0);
        assert!(e.f0.abs() < 1.0e-2, "clock bias {} out of range", e.f0);

        // Subframe 2 (orbit shape). GPS: a ~ 26560 km, near-circular.
        assert!((2.6e7..2.66e7).contains(&e.a), "semi-major axis {} m", e.a);
        assert!((0.0..0.03).contains(&e.ecc), "eccentricity {}", e.ecc);
        assert_ne!(e.toe, 0);
        assert!((-pi..=pi).contains(&e.m0), "mean anomaly {}", e.m0);

        // Subframe 3 (orientation). GPS inclination ~ 55 deg = 0.96 rad.
        assert!((0.9..1.1).contains(&e.i0), "inclination {} rad", e.i0);
        assert!((-pi..=pi).contains(&e.omg0), "ascending node {}", e.omg0);
        assert!(
            e.omg_dot < 0.0 && e.omg_dot > -1.0e-8,
            "omg_dot {}",
            e.omg_dot
        );

        // All three subframes decoded -> the ephemeris passes the solver gate.
        assert!(e.is_valid());
    }
}
