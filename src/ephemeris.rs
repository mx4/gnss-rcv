use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

/// Per-epoch tracking-measurement snapshot — everything the solver needs
/// about *this channel's signal right now*, as opposed to the broadcast
/// [`Ephemeris`] (which changes per ~2 h issue, not per code period). Kept
/// separate so multi-signal channels can snapshot at their own cadence and
/// the solve consumes `(Measurement, Ephemeris)` pairs.
#[derive(Default, Clone, Copy)]
pub struct Measurement {
    pub cn0: f64,
    /// Fractional code phase (s) paired with `trk_phase` (includes the DLL
    /// group-delay compensation; see channel.rs).
    pub code_off_sec: f64,
    /// Accumulated carrier phase (cycles) within the current continuous lock:
    /// Σ f_doppler·τ (the integrated Doppler, i.e. accumulated delta range).
    /// Resets to 0 on every (re)acquisition, so only *differences within one
    /// `lock_id`* are meaningful. Sign convention: f_doppler = −ρ̇/λ, so a range
    /// *decrease* (SV approaching, positive Doppler) makes this grow — the
    /// inter-epoch range change is Δρ = −λ·Δ(carrier_cyc). Fed to the TDCP
    /// velocity solve (the integer cycle ambiguity cancels in the difference).
    pub carrier_cyc: f64,
    /// Lock generation: the channel's successful-acquisition count at the time
    /// of this snapshot. A change between epochs means the carrier phase reset,
    /// so `carrier_cyc` must NOT be differenced across the boundary (it marks a
    /// cycle slip / re-acquisition).
    pub lock_id: u64,
    /// This signal's nominal carrier frequency (Hz) — `Signal::carrier_hz`. Per
    /// SV so the wavelength (λ = c/carrier) is correct across bands: L1 vs E5a
    /// differ by ~1.34×, which would mis-scale a non-L1 TDCP velocity.
    pub carrier_hz: f64,
    // Integer transmit-time in seconds since tracking start: num_trk_periods *
    // code_sec. Because num_trk_periods counts *transmitted* code periods
    // (carrier-aided), it advances at the SV clock rate, unlike the receiver
    // wall clock ts_sec. The absolute sub-ms code phase is kept in code_off_sec.
    pub trk_phase: f64, // current integer transmit-time
    /// Receiver stream time of this snapshot (the channel's last period
    /// step). In a mixed session families snapshot on different grids, so a
    /// solve epoch sees E1 snapshots up to 3 ms staler than C/A ones — the
    /// solver subtracts the per-SV staleness from the pseudorange (without
    /// it the offset is family-common and lands in the ISB state: measured
    /// +3 ms exactly on the first mixed tuni2025 solve).
    pub ts_sec: f64,
    pub tow_trk_phase: f64, // integer-ms phase at tx_tow_gpst (set once)
    pub tx_tow_gpst: Epoch, // GPS time pinned once the transmit anchor pins
    pub tx_anchored: bool,
}

#[derive(Default, Clone, Copy)]
pub struct Ephemeris {
    pub sv: SV,
    pub tow: u32,
    pub ts_sec: f64, // receiver time for 1st subframe
    pub tow_gpst: Epoch,
    pub toe_gpst: Epoch, // cf toe
    pub toc_gpst: Epoch,
    pub tlm: u32,

    /// Galileo GST-GPS time offset (GGTO), from I/NAV word type 10: the
    /// broadcast GST − GPST offset model `A0G + A1G·Δt`, referenced to
    /// `t0g` (s) in mod-64 GST week `wn0g`. ns-scale; used as a diagnostic
    /// (apparent inter-system bias minus GGTO = receiver hardware delay).
    pub a0g: f64,
    pub a1g: f64,
    pub t0g: u32,
    pub wn0g: u32,
    pub ggto_valid: bool,

    /// Galileo NeQuick-G ionosphere inputs, from I/NAV word type 5: the
    /// effective-ionisation coefficients Az(µ) = ai0 + ai1·µ + ai2·µ²
    /// (µ = MODIP, units sfu) plus the 5 regional storm flags (bit 0 =
    /// Region 1). Decoded and surfaced; the NeQuick-G model itself is not
    /// implemented (mixed runs use the GPS Klobuchar for all SVs).
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub iono_storm: u8,
    pub gal_iono_valid: bool,

    pub iode: u32,    // Issue of Data, Ephemeris
    pub iodc: u32,    // Issue of Data, Clock
    pub sva: u32,     // SV accuracy (URA index)
    pub svh: u32,     // SV health (0:ok)
    pub week: u32,    // GPS/QZS: gps week, GAL: galileo week
    pub code: u32,    // GPS/QZS: code on L2, GAL/CMP: data sources
    pub flag: u32,    // GPS/QZS: L2 P data flag, CMP: nav type
    pub tgd: f64,     // GPS: Estimated Group Delay Differential
    pub f0: f64,      // SV Clock Bias Correction Coefficient
    pub f1: f64,      // SV Clock Drift Correction Coefficient
    pub f2: f64,      // Drift Rate Correction Coefficient
    pub omg: f64,     // Argument of Perigee
    pub omg0: f64,    // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    pub omg_dot: f64, // Rate of Right Ascension
    pub cic: f64, // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    pub cis: f64, // Amplitude of the Sine   Harmonic Correction Term to the Angle of Inclination
    pub crc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    pub crs: f64, // Amplitude of the Sine   Harmonic Correction Term to the Orbit Radius
    pub cuc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    pub cus: f64, // Amplitude of the Sine   Harmonic Correction Term to the Argument of Latitude
    pub i_dot: f64, // Rate of Inclination Angle
    pub i0: f64,  // Inclination Angle at Reference Time
    pub m0: f64,  // Mean Anomaly at Reference Time
    pub a: f64,   // semi major axis
    pub ecc: f64, // Eccentricity
    pub deln: f64, // Mean Motion Difference From Computed Value
    pub toc: u32, // Time of Clock
    pub toe: u32, // Reference Time Ephemeris
    pub fit: u32, // fit interval (h)
    /// Decode-progress bitmask: bit *n* is set once ephemeris-bearing message
    /// *n* has been parsed into this struct — Galileo I/NAV word types 1-5, or
    /// GPS LNAV subframes 1-3. `pages()` (its `count_ones`) is how far the
    /// broadcast ephemeris has been collected; a full set (5 Galileo / 3 GPS)
    /// is a complete ephemeris. Surfaced to the UI as the per-SV progress.
    pub eph_mask: u8,
}

impl Ephemeris {
    pub fn new(sv: SV) -> Self {
        Self {
            sv,
            ..Default::default()
        }
    }
    /// True once the broadcast ephemeris has decoded into a physically plausible
    /// orbit+clock the solver can consume. Checks at least one field from each
    /// message part carrying the orbit/clock — GPS LNAV subframes 1-3, Galileo
    /// I/NAV word types 1-5 / F/NAV pages 1-4 — so a half-decoded ephemeris can't
    /// slip through, plus an eccentricity sanity bound to reject a corrupt frame.
    /// The orbit thresholds are constellation-agnostic (GPS a≈26 560 km, Galileo
    /// a≈29 600 km both pass) so this stays correct as more signals are added.
    ///
    /// Uses orbit/clock *values* (which are never legitimately zero) as the
    /// "decoded" signal, NOT `toc`/`toe`: those are seconds-of-week and are
    /// genuinely 0 at the Sunday-00:00 week boundary, so a 0-sentinel on them
    /// wrongly rejected a valid week-boundary ephemeris (real on the signal path,
    /// common when injecting a Sunday-dated A-GNSS brdc). `a` covers the
    /// subframe/word that also carries `toe`; `f0` (SV clock bias, never 0 for a
    /// real SV) covers the one that carries `toc`.
    pub fn is_valid(&self) -> bool {
        self.ts_sec != 0.0           // timestamped on first decode / injected
            && self.week != 0        // week number (LNAV SF1 / I-NAV w5 / F-NAV p1)
            && self.a >= 20_000_000.0 // semi-major axis decoded (LNAV SF2, from sqrt_a)
            && self.ecc < 0.5        // sanity: real orbits have ecc << 1
            && self.i0 != 0.0        // inclination at reference time (LNAV SF3)
            && self.omg_dot != 0.0   // rate of right ascension (LNAV SF3)
            && self.f0 != 0.0 // clock decoded (LNAV SF1 / I-NAV w4 / F-NAV clock)
    }

    /// Number of distinct ephemeris-bearing messages decoded so far — Galileo
    /// I/NAV words 1-5 or GPS LNAV subframes 1-3. The UI's per-SV decode
    /// progress; reaches 5 (Galileo) / 3 (GPS) when the ephemeris is complete.
    pub fn pages(&self) -> u8 {
        self.eph_mask.count_ones() as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gnss_rs::constellation::Constellation;

    // A fully-decoded GPS ephemeris (representative values: a ~ 26560 km,
    // i0 ~ 55 deg, small eccentricity).
    fn valid_eph() -> Ephemeris {
        let mut e = Ephemeris::new(SV::new(Constellation::GPS, 1));
        e.ts_sec = 1.0;
        e.week = 2300;
        e.toc = 100 * 16;
        e.toe = 450 * 16;
        e.a = 26_560_000.0;
        e.ecc = 0.012;
        e.i0 = 0.96;
        e.omg_dot = -8.0e-9;
        e.f0 = -1.0e-4; // SV clock bias — never 0 for a real SV (gates is_valid)
        e
    }

    #[test]
    fn fully_decoded_ephemeris_is_valid() {
        assert!(valid_eph().is_valid());
    }

    #[test]
    fn default_ephemeris_is_invalid() {
        assert!(!Ephemeris::default().is_valid());
    }

    #[test]
    fn missing_any_subframe_field_is_invalid() {
        // Zeroing any single completeness field must invalidate the ephemeris.
        // The gates are the orbit/clock *values* (never legitimately 0), one per
        // message part: week (SF1), f0 (SF1 clock), a (SF2), i0 & omg_dot (SF3).
        let check = |zero: fn(&mut Ephemeris), name: &str| {
            let mut e = valid_eph();
            zero(&mut e);
            assert!(!e.is_valid(), "{name} missing should be invalid");
        };
        check(|e| e.ts_sec = 0.0, "untimestamped");
        check(|e| e.week = 0, "sf1 week");
        check(|e| e.f0 = 0.0, "sf1 clock f0");
        check(|e| e.a = 0.0, "sf2 a");
        check(|e| e.i0 = 0.0, "sf3 i0");
        check(|e| e.omg_dot = 0.0, "sf3 omg_dot");
    }

    #[test]
    fn week_boundary_toc_toe_zero_stays_valid() {
        // toc/toe == 0 is a real value at the Sunday-00:00 week boundary (and is
        // common in an injected Sunday-dated A-GNSS brdc) — it must NOT invalidate
        // an otherwise-complete ephemeris.
        let mut e = valid_eph();
        e.toc = 0;
        e.toe = 0;
        assert!(e.is_valid(), "week-boundary toc/toe == 0 must stay valid");
    }

    #[test]
    fn corrupt_eccentricity_is_invalid() {
        let mut e = valid_eph();
        e.ecc = 0.9; // a garbled subframe-2 can decode to ecc ~ 1
        assert!(!e.is_valid());
    }
}
