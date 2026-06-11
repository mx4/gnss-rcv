//! SBAS fast and long-term corrections (DO-229 MT1, MT2-5, MT24, MT25): the
//! differential corrections that actually move the fix.
//!
//! SBAS continuously corrects each GPS satellite's broadcast clock and
//! ephemeris: **MT1** defines which PRNs the correction slots refer to (a
//! 210-bit mask, versioned by IODP); **MT2-5** carry *fast* pseudorange
//! corrections (PRC, clock-error dominated, repeated every few seconds);
//! **MT25** carries *long-term* ephemeris corrections (δx/δy/δz + δclock,
//! gated on the IODE of the ephemeris they were computed against); **MT24**
//! is half fast block, half long-term. [`SbasCorrections::feed`] assembles
//! them; the solver applies the result per SV at the pseudorange level:
//!
//! ```text
//! pr += PRC + c·δclk − û·δpos      (û = receiver→SV unit vector)
//! ```
//!
//! per DO-229 A.4.4.3 (PR_corrected = PR_measured + PRC) — the LOS projection
//! of δpos stands in for moving the modelled SV, exact to ~|δpos|²/range.
//! Unlike the iono grid these are differential (per-SV), so the receiver
//! clock state cannot absorb them. Bit layouts cross-checked against RTKLIB's
//! sbas.c. EGNOS GEOs in test mode broadcast their MT2 content as MT0
//! ("don't use" for safety-of-life — we are not, so it is parsed as MT2).

use crate::sbas_l1::SbasMessage;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use std::collections::HashMap;

/// Fast corrections expire quickly (the underlying clock error random-walks);
/// EGNOS repeats them every ~6 s, so anything older than this is stale data
/// from a decode gap, not a current correction.
const FAST_CORR_MAX_AGE_SEC: f64 = 30.0;

/// One slot's fast correction.
#[derive(Clone, Copy)]
struct FastCorr {
    prc_m: f64,
    ts_sec: f64, // receiver stream time of receipt
}

/// One satellite's long-term correction (MT24/25).
#[derive(Clone, Copy)]
struct LongCorr {
    iode: u8,
    dpos_m: [f64; 3],
    dvel_mps: [f64; 3],
    daf0_s: f64,
    daf1_ss: f64,
    /// Time of applicability as GPS seconds-of-day (velocity code 1 only).
    t0_sod: Option<f64>,
}

/// The assembled correction state of one SBAS service (all GEOs of a system
/// broadcast the same data, so one instance is fed from every channel).
#[derive(Default, Clone)]
pub struct SbasCorrections {
    /// Mask version; fast/long corrections only apply against their own mask.
    iodp: Option<u8>,
    /// MT1 mask: slot index (0-based) → PRN "mask number" 1-210
    /// (1-37 = GPS PRN, 120-138 = SBAS; others unused here).
    mask: Vec<u16>,
    fast: HashMap<u16, FastCorr>, // mask number -> fast corr
    long: HashMap<u16, LongCorr>, // mask number -> long-term corr
}

/// Read `len` bits MSB-first at `pos` from a one-bit-per-byte message.
fn bits(m: &[u8], pos: usize, len: usize) -> u32 {
    m[pos..pos + len]
        .iter()
        .fold(0, |a, &b| (a << 1) | (b & 1) as u32)
}

/// Same, sign-extended (two's complement).
fn sbits(m: &[u8], pos: usize, len: usize) -> i32 {
    let u = bits(m, pos, len);
    let sign = 1u32 << (len - 1);
    (u as i32) - if u & sign != 0 { (sign as i32) << 1 } else { 0 }
}

impl SbasCorrections {
    /// Number of satellites with a current fast correction.
    pub fn fast_len(&self) -> usize {
        self.fast.len()
    }

    /// Number of satellites with a long-term correction.
    pub fn long_len(&self) -> usize {
        self.long.len()
    }

    /// Feed one CRC-valid SBAS message at receiver stream time `ts_sec`;
    /// returns true if it updated the correction state.
    pub fn feed(&mut self, msg: &SbasMessage, ts_sec: f64) -> bool {
        match msg.mtype {
            1 => self.feed_mt1(&msg.bits),
            // EGNOS test mode: MT0 carries the MT2 payload.
            0 | 2..=5 => self.feed_fast(msg.mtype.max(2), &msg.bits, ts_sec),
            24 => self.feed_mt24(&msg.bits, ts_sec),
            25 => {
                self.feed_long_half(&msg.bits, 14);
                self.feed_long_half(&msg.bits, 120);
            }
            _ => return false,
        }
        true
    }

    /// MT1: 210-bit PRN mask (bits 14..224), IODP (224..226). A new IODP
    /// re-keys every slot, so stale corrections are dropped wholesale.
    fn feed_mt1(&mut self, m: &[u8]) {
        let iodp = bits(m, 224, 2) as u8;
        if self.iodp == Some(iodp) {
            return; // same mask version, nothing to do
        }
        self.iodp = Some(iodp);
        self.mask = (1..=210u16).filter(|&n| m[13 + n as usize] == 1).collect();
        self.fast.clear();
        self.long.clear();
    }

    /// MT2-5: IODF (14), IODP (16), 13 × 12-bit PRC ×0.125 m (18..174),
    /// 13 × 4-bit UDREI (174..226). Message type n covers mask slots
    /// 13(n−2) .. 13(n−2)+12.
    fn feed_fast(&mut self, mtype: u8, m: &[u8], ts_sec: f64) {
        if self.iodp != Some(bits(m, 16, 2) as u8) {
            return; // corrections keyed to a mask we don't have
        }
        for i in 0..13 {
            self.fast_slot(
                13 * (mtype as usize - 2) + i,
                sbits(m, 18 + 12 * i, 12),
                bits(m, 174 + 4 * i, 4),
                ts_sec,
            );
        }
    }

    /// MT24: 6 × PRC (14..86), 6 × UDREI (86..110), IODP (110), block ID
    /// (112) selecting which MT2-5 slots the 6 entries are, IODF (114);
    /// bits 120.. are a long-term half-message.
    fn feed_mt24(&mut self, m: &[u8], ts_sec: f64) {
        if self.iodp == Some(bits(m, 110, 2) as u8) {
            let block = bits(m, 112, 2) as usize;
            for i in 0..6 {
                self.fast_slot(
                    13 * block + i,
                    sbits(m, 14 + 12 * i, 12),
                    bits(m, 86 + 4 * i, 4),
                    ts_sec,
                );
            }
        }
        self.feed_long_half(m, 120);
    }

    /// Store one fast-correction slot; PRC 0x800 and UDREI ≥ 14 mean "don't
    /// use" / "not monitored" and clear the slot instead.
    fn fast_slot(&mut self, slot: usize, prc_raw: i32, udrei: u32, ts_sec: f64) {
        let Some(&mn) = self.mask.get(slot) else {
            return; // beyond the mask: slot not assigned
        };
        if prc_raw == -2048 || udrei >= 14 {
            self.fast.remove(&mn);
            return;
        }
        let prc_m = prc_raw as f64 * 0.125;
        self.fast.insert(mn, FastCorr { prc_m, ts_sec });
    }

    /// One 106-bit long-term half-message at bit `p` (MT25 carries two, MT24
    /// one): velocity code (1 bit), then per DO-229 / RTKLIB decode_longcorrh:
    /// v=0 → two corrections {mask# 6, IODE 8, δxyz 3×9 ×0.125 m, δaf0 10
    /// ×2⁻³¹}, IODP at p+103; v=1 → one correction {mask# 6, IODE 8, δxyz
    /// 3×11 ×0.125 m, δaf0 11 ×2⁻³¹, δvxyz 3×8 ×2⁻¹¹ m/s, δaf1 8 ×2⁻³⁹,
    /// t0 13 ×16 s}, IODP at p+104.
    fn feed_long_half(&mut self, m: &[u8], p: usize) {
        if bits(m, p, 1) == 0 {
            if self.iodp != Some(bits(m, p + 103, 2) as u8) {
                return;
            }
            for q in [p + 1, p + 52] {
                let mn = bits(m, q, 6) as usize;
                let Some(&prn) = self.mask.get(mn.wrapping_sub(1)) else {
                    continue; // mask number 0 = empty filler entry
                };
                self.long.insert(
                    prn,
                    LongCorr {
                        iode: bits(m, q + 6, 8) as u8,
                        dpos_m: [
                            sbits(m, q + 14, 9) as f64 * 0.125,
                            sbits(m, q + 23, 9) as f64 * 0.125,
                            sbits(m, q + 32, 9) as f64 * 0.125,
                        ],
                        dvel_mps: [0.0; 3],
                        daf0_s: sbits(m, q + 41, 10) as f64 * crate::constants::P2_31,
                        daf1_ss: 0.0,
                        t0_sod: None,
                    },
                );
            }
        } else {
            if self.iodp != Some(bits(m, p + 104, 2) as u8) {
                return;
            }
            let q = p + 1;
            let mn = bits(m, q, 6) as usize;
            let Some(&prn) = self.mask.get(mn.wrapping_sub(1)) else {
                return;
            };
            self.long.insert(
                prn,
                LongCorr {
                    iode: bits(m, q + 6, 8) as u8,
                    dpos_m: [
                        sbits(m, q + 14, 11) as f64 * 0.125,
                        sbits(m, q + 25, 11) as f64 * 0.125,
                        sbits(m, q + 36, 11) as f64 * 0.125,
                    ],
                    dvel_mps: [
                        sbits(m, q + 58, 8) as f64 * crate::constants::P2_11,
                        sbits(m, q + 66, 8) as f64 * crate::constants::P2_11,
                        sbits(m, q + 74, 8) as f64 * crate::constants::P2_11,
                    ],
                    daf0_s: sbits(m, q + 47, 11) as f64 * crate::constants::P2_31,
                    daf1_ss: sbits(m, q + 82, 8) as f64 * crate::constants::P2_39,
                    t0_sod: Some(bits(m, q + 90, 13) as f64 * 16.0),
                },
            );
        }
    }

    /// Mask number of an SV (only GPS and SBAS occupy DO-229 mask space).
    fn mask_number(sv: SV) -> Option<u16> {
        match sv.constellation {
            Constellation::GPS if (1..=37).contains(&sv.prn) => Some(sv.prn as u16),
            Constellation::SBAS => Some(sv.prn as u16),
            _ => None,
        }
    }

    /// Current fast pseudorange correction (m) for `sv`, if fresh at stream
    /// time `ts_sec`. Applied as `pr += prc` (DO-229 A.4.4.3).
    pub fn fast_prc_m(&self, sv: SV, ts_sec: f64) -> Option<f64> {
        let fc = self.fast.get(&Self::mask_number(sv)?)?;
        ((ts_sec - fc.ts_sec).abs() <= FAST_CORR_MAX_AGE_SEC).then_some(fc.prc_m)
    }

    /// Long-term correction for `sv` as `(δpos ECEF m, δclock s)`, evaluated
    /// at GPS second-of-day `gps_sod`. Gated on `iode` matching the broadcast
    /// ephemeris the receiver is flying — a mismatch means the correction was
    /// computed against a different ephemeris issue and would corrupt, not
    /// correct.
    pub fn long_term(&self, sv: SV, iode: u8, gps_sod: f64) -> Option<([f64; 3], f64)> {
        let lc = self.long.get(&Self::mask_number(sv)?)?;
        if lc.iode != iode {
            return None;
        }
        // Velocity-code-1 corrections evaluate at t − t0 (wrapped to ±43200 s
        // around the day boundary); velocity-code-0 ones are static.
        let dt = lc.t0_sod.map_or(0.0, |t0| {
            let mut dt = gps_sod - t0;
            if dt <= -43200.0 {
                dt += 86400.0;
            } else if dt > 43200.0 {
                dt -= 86400.0;
            }
            dt
        });
        let dpos = [
            lc.dpos_m[0] + lc.dvel_mps[0] * dt,
            lc.dpos_m[1] + lc.dvel_mps[1] * dt,
            lc.dpos_m[2] + lc.dvel_mps[2] * dt,
        ];
        Some((dpos, lc.daf0_s + lc.daf1_ss * dt))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put(m: &mut [u8], pos: usize, len: usize, val: u32) {
        for i in 0..len {
            m[pos + i] = ((val >> (len - 1 - i)) & 1) as u8;
        }
    }

    fn puts(m: &mut [u8], pos: usize, len: usize, val: i32) {
        put(m, pos, len, (val as u32) & ((1 << len) - 1));
    }

    const IODP: u32 = 2;

    /// MT1 with the given mask numbers set.
    fn mt1(mask_numbers: &[u16]) -> SbasMessage {
        let mut m = [0u8; 250];
        put(&mut m, 8, 6, 1);
        for &n in mask_numbers {
            m[13 + n as usize] = 1;
        }
        put(&mut m, 224, 2, IODP);
        SbasMessage { mtype: 1, bits: m }
    }

    /// MT2-5 with PRCs (m) for the message's first slots.
    fn mt_fast(mtype: u8, prcs_m: &[f64]) -> SbasMessage {
        let mut m = [0u8; 250];
        put(&mut m, 8, 6, mtype as u32);
        put(&mut m, 16, 2, IODP);
        for i in 0..13 {
            match prcs_m.get(i) {
                Some(&p) => {
                    puts(&mut m, 18 + 12 * i, 12, (p / 0.125).round() as i32);
                    put(&mut m, 174 + 4 * i, 4, 5);
                }
                None => put(&mut m, 174 + 4 * i, 4, 14), // not monitored
            }
        }
        SbasMessage { mtype, bits: m }
    }

    /// MT25 first half, velocity code 0, one correction (second slot empty).
    fn mt25_v0(mask_no: u16, iode: u8, dpos: [f64; 3], daf0: f64) -> SbasMessage {
        let mut m = [0u8; 250];
        put(&mut m, 8, 6, 25);
        let q = 15; // velocity bit at 14 = 0
        put(&mut m, q, 6, mask_no as u32);
        put(&mut m, q + 6, 8, iode as u32);
        for (i, d) in dpos.iter().enumerate() {
            puts(&mut m, q + 14 + 9 * i, 9, (d / 0.125).round() as i32);
        }
        puts(
            &mut m,
            q + 41,
            10,
            (daf0 / crate::constants::P2_31).round() as i32,
        );
        put(&mut m, 14 + 103, 2, IODP);
        // Second half: velocity code 1 with mask number 0 = unused.
        put(&mut m, 120 + 104, 2, IODP);
        put(&mut m, 120, 1, 1);
        SbasMessage { mtype: 25, bits: m }
    }

    #[test]
    fn fast_corrections_assemble_and_expire() {
        let gps = |prn| SV::new(Constellation::GPS, prn);
        let mut c = SbasCorrections::default();
        // Mask: GPS PRNs 3, 7, 31 occupy slots 0, 1, 2.
        assert!(c.feed(&mt1(&[3, 7, 31]), 0.0));
        assert!(c.feed(&mt_fast(2, &[2.0, -1.5, 0.625]), 10.0));
        assert_eq!(c.fast_len(), 3);
        assert_eq!(c.fast_prc_m(gps(3), 12.0), Some(2.0));
        assert_eq!(c.fast_prc_m(gps(7), 12.0), Some(-1.5));
        assert_eq!(c.fast_prc_m(gps(31), 12.0), Some(0.625));
        assert_eq!(c.fast_prc_m(gps(4), 12.0), None); // not in the mask
        // Stale after the age bound.
        assert_eq!(c.fast_prc_m(gps(3), 10.0 + 31.0), None);
        // MT0 carries MT2 content in EGNOS test mode.
        assert!(c.feed(&mt_fast(0, &[4.0]), 20.0));
        assert_eq!(c.fast_prc_m(gps(3), 21.0), Some(4.0));
        // A new IODP (new mask) drops everything.
        let mut remask = mt1(&[3, 7, 31]);
        put(&mut remask.bits, 224, 2, IODP + 1);
        c.feed(&remask, 22.0);
        assert_eq!(c.fast_len(), 0);
    }

    #[test]
    fn long_term_corrections_gate_on_iode() {
        let gps = |prn| SV::new(Constellation::GPS, prn);
        let mut c = SbasCorrections::default();
        c.feed(&mt1(&[3, 7]), 0.0);
        c.feed(&mt25_v0(2, 0x5A, [1.0, -0.5, 0.25], 3.0e-9), 1.0);
        assert_eq!(c.long_len(), 1);
        // Mask number 2 = second mask entry = PRN 7.
        let (dpos, dclk) = c.long_term(gps(7), 0x5A, 43000.0).unwrap();
        assert_eq!(dpos, [1.0, -0.5, 0.25]);
        assert!((dclk - 3.0e-9).abs() < crate::constants::P2_31);
        // Wrong IODE: the correction is for another ephemeris issue.
        assert_eq!(c.long_term(gps(7), 0x5B, 43000.0), None);
        assert_eq!(c.long_term(gps(3), 0x5A, 43000.0), None);
    }

    #[test]
    fn fast_corrections_require_matching_iodp() {
        let mut c = SbasCorrections::default();
        c.feed(&mt1(&[3]), 0.0);
        let mut msg = mt_fast(2, &[2.0]);
        put(&mut msg.bits, 16, 2, IODP + 1);
        c.feed(&msg, 1.0);
        assert_eq!(c.fast_len(), 0);
    }
}
