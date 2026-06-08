use crate::{
    channel::Channel,
    constants::{P2_24, P2_27, P2_30},
    ephemeris::Ephemeris,
    util::{bits_equal, bits_opposed, getbits, getbits2, getbitu, hex_str, setbitu, xor_bits},
};
use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

const SECS_PER_WEEK: u32 = 7 * 24 * 60 * 60;
const SDR_MAX_NSYM: usize = 18000;

const THRESHOLD_SYNC: f64 = 0.4; // 0.02
const THRESHOLD_LOST: f64 = 0.03; // 0.002

#[derive(PartialEq, Debug, Default)]
enum SyncState {
    #[default]
    Normal,
    Reversed,
    None,
}

pub struct Navigation {
    // pub_state: Arc<Mutex<GnssState>>,
    bit_sync: usize, // beginning of a navigation bit in num_trk_samples
    nav_sync: usize, // beginning/end of a navigation frame in num_trk_samples
    sync_state: SyncState,
    bits: Vec<u8>, // navigation bits
    count_parity_err: usize,
    pub eph: Ephemeris,
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            //       pub_state,
            bit_sync: 0,
            nav_sync: 0,
            sync_state: SyncState::Normal,
            bits: vec![0; SDR_MAX_NSYM],
            count_parity_err: 0,
            eph: Ephemeris::new(sv),
        }
    }

    pub fn init(&mut self) {
        self.bit_sync = 0;
        self.nav_sync = 0;
        self.sync_state = SyncState::Normal;
        self.bits.fill(0);
    }
}

impl Channel {
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
        self.nav.bits.rotate_left(1);
        *self.nav.bits.last_mut().unwrap() = bit;
    }

    fn nav_get_frame_sync_state(&self, preambule: &[u8]) -> SyncState {
        let bits = &self.nav.bits[SDR_MAX_NSYM - 308..];
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
        if self.nav.bit_sync == 0 {
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
                self.nav.bit_sync = self.num_trk_samples - n;
                log::info!("{}: SYNC: p={:.5} ssync={}", self.sv, p, self.nav.bit_sync);
            }
        } else if (self.num_trk_samples - self.nav.bit_sync).is_multiple_of(num) {
            let p = self.nav_mean_ip(num);
            if p.abs() >= THRESHOLD_LOST {
                let sym: u8 = if p >= 0.0 { 1 } else { 0 };
                self.nav_add_bit(sym);
                return true;
            } else {
                self.nav.bit_sync = 0;
                self.nav.sync_state = SyncState::Normal;
                log::info!("{}: SYNC {} p={}", self.sv, "LOST".to_string().red(), p)
            }
        }
        false
    }

    fn nav_decode_lnav_subframe1(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe1(buf, self.sv);
    }

    fn nav_decode_lnav_subframe2(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe2(buf, self.sv);
    }

    fn nav_decode_lnav_subframe3(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe3(buf, self.sv);
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
            1 => self.nav_decode_lnav_subframe1(buf),
            2 => self.nav_decode_lnav_subframe2(buf),
            3 => self.nav_decode_lnav_subframe3(buf),
            4 => self.nav_decode_lnav_subframe4(buf),
            5 => self.nav_decode_lnav_subframe5(buf),
            _ => log::warn!("{}: invalid subframe id={subframe_id}", self.sv),
        }

        self.nav_subframe_post();

        subframe_id
    }

    fn nav_decode_lnav(&mut self, sync: SyncState) {
        let rev = if sync == SyncState::Normal { 0 } else { 1 };
        let bits_len = self.nav.bits.len();
        let bits_raw = &self.nav.bits[bits_len - 308..bits_len - 8];
        let bits: Vec<_> = bits_raw.iter().map(|v| v ^ rev).collect();
        let mut nav_data = vec![0; 300];

        if Self::nav_test_lnav_parity(&bits, &mut nav_data) {
            self.nav.nav_sync = self.num_trk_samples;
            self.nav.sync_state = sync;
            self.stats.subframes += 1;

            let id = self.nav_decode_lnav_subframe(&nav_data);
            let hex_str = hex_str(&nav_data[0..300]);
            log::info!("{}: LNAV: id={id} -- {hex_str}", self.sv);
        } else {
            self.nav.nav_sync = 0;
            self.nav.sync_state = SyncState::Normal;
            self.nav.count_parity_err += 1;
            self.stats.parity_errors += 1;

            log::warn!("{}: PARITY ERROR", self.sv);
        }
    }

    fn nav_test_lnav_parity(bits: &[u8], nav_data: &mut [u8]) -> bool {
        const MASK: [u32; 6] = [
            0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
        ];
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

    fn nav_decode_sbas(&mut self) {
        log::warn!("{}: SBAS frame", self.sv);
    }

    pub fn nav_decode(&mut self) {
        const PREAMBULE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
        let preambule = &PREAMBULE[0..];

        if self.sv.prn >= 120 && self.sv.prn <= 158 {
            self.nav_decode_sbas();
            return;
        }

        if !self.nav_sync_symbol(20) {
            return;
        }

        if self.nav.nav_sync > 0 {
            #[allow(clippy::comparison_chain)]
            if self.num_trk_samples == self.nav.nav_sync + 300 * 20 {
                let sync = self.nav_get_frame_sync_state(preambule);
                if sync == self.nav.sync_state {
                    self.nav_decode_lnav(sync);
                }
            } else if self.num_trk_samples > self.nav.nav_sync + 300 * 20 {
                self.nav.nav_sync = 0;
                self.nav.bit_sync = 0;
                self.nav.sync_state = SyncState::Normal;
            }
        } else if self.num_trk_samples >= 20 * 308 + 1000 {
            let sync = self.nav_get_frame_sync_state(preambule);
            if sync != SyncState::None {
                self.nav_decode_lnav(sync);
            }
        }
    }
}
