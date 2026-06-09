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
    channel::Channel,
    constants::{P2_5, P2_19, P2_24, P2_27, P2_29, P2_30, P2_31, P2_33, P2_43, P2_55, SC2RAD},
    ephemeris::Ephemeris,
    util::{
        bits_equal, bits_opposed, getbits, getbits2, getbitu, getbitu2, hex_str, setbitu, xor_bits,
    },
};
use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

const SECS_PER_WEEK: u32 = 7 * 24 * 60 * 60;
const SDR_MAX_NSYM: usize = 18000;

const THRESHOLD_SYNC: f64 = 0.4; // 0.02
const THRESHOLD_LOST: f64 = 0.03; // 0.002

/// LNAV 30-bit-word parity masks (IS-GPS-200), shared by the parity check and the
/// encoder.
const MASK: [u32; 6] = [
    0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
];
/// LNAV TLM preamble (8 bits).
const PREAMBLE: u32 = 0x8b;

#[derive(PartialEq, Debug, Default)]
pub(crate) enum SyncState {
    #[default]
    Normal,
    Reversed,
    None,
}

/// GPS L1 C/A LNAV decoder state: 50 bps bit/frame sync and the rolling symbol
/// buffer the parity decoder reads.
pub(crate) struct LnavState {
    bit_sync: usize, // beginning of a navigation bit in num_trk_samples
    nav_sync: usize, // beginning/end of a navigation frame in num_trk_samples
    sync_state: SyncState,
    bits: Vec<u8>, // navigation bits
    count_parity_err: usize,
}

impl LnavState {
    pub fn new() -> Self {
        Self {
            bit_sync: 0,
            nav_sync: 0,
            sync_state: SyncState::Normal,
            bits: vec![0; SDR_MAX_NSYM],
            count_parity_err: 0,
        }
    }

    pub fn reset(&mut self) {
        self.bit_sync = 0;
        self.nav_sync = 0;
        self.sync_state = SyncState::Normal;
        self.bits.fill(0);
    }
}

impl Channel {
    /// GPS/QZSS LNAV decode entry: advance bit sync, then on a frame boundary run
    /// frame sync and decode the subframe.
    pub(crate) fn nav_decode_gps_lnav(&mut self) {
        const PREAMBULE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
        let preambule = &PREAMBULE[0..];

        if !self.nav_sync_symbol(20) {
            return;
        }

        if self.nav.lnav.nav_sync > 0 {
            #[allow(clippy::comparison_chain)]
            if self.num_trk_samples == self.nav.lnav.nav_sync + 300 * 20 {
                let sync = self.nav_get_frame_sync_state(preambule);
                if sync == self.nav.lnav.sync_state {
                    self.nav_decode_lnav(sync);
                }
            } else if self.num_trk_samples > self.nav.lnav.nav_sync + 300 * 20 {
                self.nav.lnav.nav_sync = 0;
                self.nav.lnav.bit_sync = 0;
                self.nav.lnav.sync_state = SyncState::Normal;
            }
        } else if self.num_trk_samples >= 20 * 308 + 1000 {
            let sync = self.nav_get_frame_sync_state(preambule);
            if sync != SyncState::None {
                self.nav_decode_lnav(sync);
            }
        }
    }

    fn nav_mean_ip(&self, n: usize) -> f64 {
        let mut p = 0.0;
        let len = self.hist.corr_p.len();

        for i in 0..n {
            // weird math
            let c = self.hist.corr_p[len - n + i];
            //p += (c.re / c.norm() - p) / (1 + i) as f64;
            p += c.re / c.norm();
        }
        p / n as f64
    }

    fn nav_add_bit(&mut self, bit: u8) {
        self.nav.lnav.bits.rotate_left(1);
        *self.nav.lnav.bits.last_mut().unwrap() = bit;
    }

    fn nav_get_frame_sync_state(&self, preambule: &[u8]) -> SyncState {
        let bits = &self.nav.lnav.bits[SDR_MAX_NSYM - 308..];
        let bits_beg = &bits[0..preambule.len()];
        let bits_end = &bits[300..300 + preambule.len()];
        let mut sync_state = SyncState::None;

        if bits_equal(preambule, bits_beg) && bits_equal(preambule, bits_end) {
            sync_state = SyncState::Normal;
        } else if bits_opposed(preambule, bits_beg) && bits_opposed(preambule, bits_end) {
            sync_state = SyncState::Reversed;
        }
        if sync_state != SyncState::None {
            log::info!(
                "{}: FRAME SYNC {sync_state:?}: ts={:.3}",
                self.sv,
                self.ts_sec
            );
        }

        sync_state
    }

    fn nav_sync_symbol(&mut self, num: usize) -> bool {
        if self.nav.lnav.bit_sync == 0 {
            let n = if num <= 2 { 1 } else { num - 1 };
            let len = self.hist.corr_p.len();

            let mut p = 0.0;
            let mut r = 0.0;
            for i in 0..2 * n {
                let code = if i < n { -1.0 } else { 1.0 };
                let corr = self.hist.corr_p[len - 2 * n + i];
                let corr_re = corr.re / corr.norm(); // XXX: shouldn't be required

                p += corr_re * code;
                r += corr_re.abs();
            }

            p /= 2.0 * n as f64;
            r /= 2.0 * n as f64;

            if p.abs() >= r && r >= THRESHOLD_SYNC {
                self.nav.lnav.bit_sync = self.num_trk_samples - n;
                log::info!(
                    "{}: SYNC: p={:.5} ssync={}",
                    self.sv,
                    p,
                    self.nav.lnav.bit_sync
                );
            }
        } else if (self.num_trk_samples - self.nav.lnav.bit_sync).is_multiple_of(num) {
            let p = self.nav_mean_ip(num);
            if p.abs() >= THRESHOLD_LOST {
                let sym: u8 = if p >= 0.0 { 1 } else { 0 };
                self.nav_add_bit(sym);
                return true;
            } else {
                self.nav.lnav.bit_sync = 0;
                self.nav.lnav.sync_state = SyncState::Normal;
                log::info!("{}: SYNC {} p={}", self.sv, "LOST".to_string().red(), p)
            }
        }
        false
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

    fn update_gpst_time(&mut self, tow_gpst: Epoch) {
        self.pub_state.lock().unwrap().tow_gpst = tow_gpst;

        (self.pub_state.lock().unwrap().update_func.func)();
    }

    fn nav_subframe_post(&mut self) {
        if self.is_ephemeris_complete() {
            self.pub_state
                .lock()
                .unwrap()
                .channels
                .get_mut(&self.sv)
                .unwrap()
                .has_eph = true;
        }
        if self.nav.eph.week != 0 {
            let week_to_secs = self.nav.eph.week * SECS_PER_WEEK;
            let tow_secs_gpst = week_to_secs + self.nav.eph.tow;
            let toe_secs_gpst = week_to_secs + self.nav.eph.toe;
            let toc_secs_gpst = week_to_secs + self.nav.eph.toc;

            self.nav.eph.tow_gpst = Epoch::from_gpst_seconds(tow_secs_gpst.into());
            self.nav.eph.toe_gpst = Epoch::from_gpst_seconds(toe_secs_gpst.into());
            self.nav.eph.toc_gpst = Epoch::from_gpst_seconds(toc_secs_gpst.into());

            self.nav.eph.ts_sec = self.ts_sec;

            // Pin transmit time once when ephemeris is complete. Re-anchoring every
            // subframe reset c_anchor and made sub-ms inter-SV range track (c-c_anchor)
            // instead of absolute code phase. After re-acquisition tx_anchored is
            // cleared in tracking_init until the next nav subframe re-anchors.
            if self.is_ephemeris_complete() && !self.nav.eph.tx_anchored {
                // Anchor only the INTEGER period count at the TOW edge (a code
                // epoch). The sub-ms range must come from the absolute code_off at
                // the fix (which is on a common cross-SV reference, since all SVs
                // are acquired against the same IQ buffer). Subtracting code_off
                // here would cancel the absolute sub-ms range in the solver's
                // elapsed = phase_now - tow_trk_phase, biasing each SV by its
                // arbitrary acquisition code phase (~±0.5 ms = ±150 km).
                self.nav.eph.tow_trk_phase = self.nav.eph.trk_phase;
                self.nav.eph.tx_tow_gpst = self.nav.eph.tow_gpst;
                self.nav.eph.tx_anchor_ts_sec = self.ts_sec;
                self.nav.eph.tx_anchored = true;
                log::warn!(
                    "{}: tx anchored tow={:?} phase={:.6}",
                    self.sv,
                    self.nav.eph.tx_tow_gpst,
                    self.nav.eph.tow_trk_phase,
                );
            }

            log::warn!(
                "{}: tow={:?} tgd={:+.3e} toe={:?}",
                self.sv,
                self.nav.eph.tow_gpst,
                self.nav.eph.tgd,
                self.nav.eph.toe_gpst
            );

            self.update_gpst_time(self.nav.eph.tow_gpst);
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
            1 => decode_lnav_subframe1(&mut self.nav.eph, buf, self.sv),
            2 => decode_lnav_subframe2(&mut self.nav.eph, buf, self.sv),
            3 => decode_lnav_subframe3(&mut self.nav.eph, buf, self.sv),
            4 => self.nav_decode_lnav_subframe4(buf),
            5 => self.nav_decode_lnav_subframe5(buf),
            _ => log::warn!("{}: invalid subframe id={subframe_id}", self.sv),
        }

        self.nav_subframe_post();

        subframe_id
    }

    fn nav_decode_lnav(&mut self, sync: SyncState) {
        let rev = if sync == SyncState::Normal { 0 } else { 1 };
        let bits_len = self.nav.lnav.bits.len();
        let bits_raw = &self.nav.lnav.bits[bits_len - 308..bits_len - 8];
        let bits: Vec<_> = bits_raw.iter().map(|v| v ^ rev).collect();
        let mut nav_data = vec![0; 300];

        if Self::nav_test_lnav_parity(&bits, &mut nav_data) {
            self.nav.lnav.nav_sync = self.num_trk_samples;
            self.nav.lnav.sync_state = sync;
            self.stats.subframes += 1;

            let id = self.nav_decode_lnav_subframe(&nav_data);
            let hex_str = hex_str(&nav_data[0..300]);
            log::info!("{}: LNAV: id={id} -- {hex_str}", self.sv);
        } else {
            self.nav.lnav.nav_sync = 0;
            self.nav.lnav.sync_state = SyncState::Normal;
            self.nav.lnav.count_parity_err += 1;
            self.stats.parity_errors += 1;

            log::warn!("{}: PARITY ERROR", self.sv);
        }
    }

    pub(crate) fn nav_test_lnav_parity(bits: &[u8], nav_data: &mut [u8]) -> bool {
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
}

// --- LNAV subframe field parsers --------------------------------------------
//
// Fill the generic `Ephemeris` (constellation-agnostic orbit+clock data) from
// the GPS LNAV subframe bit fields (IS-GPS-200). Galileo's I/NAV parser fills the
// same struct from its own word layout.

/// Subframe 1: GPS week + clock (af0/af1/af2, tgd, toc).
fn decode_lnav_subframe1(eph: &mut Ephemeris, buf: &[u8], sv: SV) {
    eph.tow = getbitu(buf, 30, 17) * 6;
    eph.week = getbitu(buf, 60, 10) + 2048;
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
fn decode_lnav_subframe2(eph: &mut Ephemeris, buf: &[u8], sv: SV) {
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
fn decode_lnav_subframe3(eph: &mut Ephemeris, buf: &[u8], sv: SV) {
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
                Channel::nav_test_lnav_parity(&tx, &mut nav_data),
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
                0 => decode_lnav_subframe1(&mut eph, &nav_data, sv),
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

        decode_lnav_subframe1(&mut e, &hex_to_bytes(G01_SF1), sv);
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
