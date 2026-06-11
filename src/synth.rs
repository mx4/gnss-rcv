//! Synthetic GPS L1 C/A signal generator.
//!
//! Produces baseband / low-IF complex IQ that the *real* acquisition → tracking
//! → decode path can lock onto, with no recording or network. Two uses:
//!   * hermetic, deterministic unit tests (acquire/track regressions runnable in
//!     CI — see `receiver.rs`), and
//!   * a controlled DSP bench — e.g. emit a 45 dB-Hz SV with known 50 bps bit
//!     edges to reproduce/measure the slow bit-sync issue without downloading a
//!     multi-GB capture.
//!
//! The carrier sign convention matches the receiver's de-rotation (it mixes down
//! by `fi + doppler`), i.e. the modelled signal is
//! `code · nav · exp(+j·2π·(fi+fd)·t)`, so a correct replica cancels it to the
//! bare code.

use rustfft::num_complex::{Complex32, Complex64};
use std::f64::consts::{PI, TAU};

use crate::code::{Code, E1_CODE_LEN, L1CA_CODE_LEN, Signal};
use crate::constants::{EARTH_ROTATION_RATE, L1_HZ, SPEED_OF_LIGHT};
use crate::ephemeris::Ephemeris;
use crate::gps_lnav::{encode_lnav_subframe_source, encode_subframe, quantize_via_lnav};
use crate::receiver::IQReader;
use crate::solver::{compute_sv_position_ecef, elevation_azimuth};
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Epoch, Vector3};

/// L1 C/A chip rate (1023 chips / 1 ms).
const CODE_RATE_HZ: f64 = 1_023_000.0;
/// Galileo E1 BOC(1,1) sub-chip rate (8184 sub-chips / 4 ms).
const E1_SUBCHIP_RATE_HZ: f64 = 2_046_000.0;
/// E1-B/C BOC code length in sub-chips (4092 chips × 2).
const E1_BOC_LEN: usize = 2 * E1_CODE_LEN;
/// One nav data bit lasts 20 ms (50 bps) = 20 code periods.
const NAV_BIT_SEC: f64 = 0.020;

/// One synthetic satellite in a scene.
#[derive(Clone)]
pub struct SynthSv {
    pub prn: u8,
    /// Carrier Doppler (Hz). Also stretches the code rate by `fd / L1`.
    pub doppler_hz: f64,
    /// Initial code phase (chips, may be fractional). Real SVs are never at
    /// exactly 0 — which is itself a useful edge case (first tracking step).
    pub code_phase_chips: f64,
    /// Carrier-to-noise density (dB-Hz). Used only by the *noisy* generator to
    /// scale this SV's amplitude against unit-variance AWGN.
    pub cn0_dbhz: f64,
    /// Optional 50 bps nav data bits (±1). Empty = held at +1 (no transitions),
    /// which still acquires and tracks; supply bits to exercise bit/frame sync.
    pub nav_bits: Vec<i8>,
}

impl SynthSv {
    /// A satellite with no nav-data transitions (data held at +1).
    pub fn new(prn: u8, doppler_hz: f64, code_phase_chips: f64, cn0_dbhz: f64) -> Self {
        Self {
            prn,
            doppler_hz,
            code_phase_chips,
            cn0_dbhz,
            nav_bits: Vec::new(),
        }
    }

    /// Same, but BPSK-modulated by a 50 bps nav-bit pattern (±1) — for bit/frame
    /// sync benches and tests.
    pub fn with_nav_bits(mut self, bits: Vec<i8>) -> Self {
        self.nav_bits = bits;
        self
    }
}

/// Deterministic splitmix64 PRNG + Box–Muller Gaussian, so noisy scenes are
/// reproducible bit-for-bit without depending on the `rand` crate.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        // 53-bit mantissa uniform in [0, 1).
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// One N(0, 1) sample.
    fn gauss(&mut self) -> f64 {
        let u1 = self.unit().max(1e-300);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }
}

/// Render `num_msec` of complex IQ at sample rate `fs` and IF `fi`, summing every
/// satellite in `svs`.
///
/// * `seed = None` — a clean noiseless reference: each SV at unit amplitude,
///   `cn0_dbhz` ignored. Fully deterministic (no RNG).
/// * `seed = Some(s)` — adds complex AWGN (`E|n|² = 1` per sample) and scales each
///   SV's amplitude so it sits at its `cn0_dbhz` in that noise:
///   `A = sqrt(10^(cn0/10) / fs)`.
pub fn synth_l1ca(
    svs: &[SynthSv],
    fs: f64,
    fi: f64,
    num_msec: usize,
    seed: Option<u64>,
) -> Vec<Complex32> {
    let code_sp = (fs * 1e-3) as usize;
    let n_total = code_sp * num_msec;

    let codes: Vec<Vec<i8>> = svs
        .iter()
        .map(|s| Code::gen_code("L1CA", s.prn).expect("L1CA code"))
        .collect();
    let chip_rate: Vec<f64> = svs
        .iter()
        .map(|s| CODE_RATE_HZ * (1.0 + s.doppler_hz / L1_HZ))
        .collect();
    let amp: Vec<f64> = svs
        .iter()
        .map(|s| match seed {
            None => 1.0,
            Some(_) => (10f64.powf(s.cn0_dbhz / 10.0) / fs).sqrt(),
        })
        .collect();

    let mut rng = seed.map(Rng);
    let nstd = 0.5f64.sqrt(); // per-component std so E|n|² = 1 for the complex sample
    let inv_fs = 1.0 / fs;
    let mut out = Vec::with_capacity(n_total);
    for n in 0..n_total {
        let t = n as f64 * inv_fs;
        let mut x = Complex32::new(0.0, 0.0);
        for (k, s) in svs.iter().enumerate() {
            let chip = (s.code_phase_chips + chip_rate[k] * t).rem_euclid(L1CA_CODE_LEN as f64);
            let c = codes[k][chip as usize] as f64;
            let b = if s.nav_bits.is_empty() {
                1.0
            } else {
                let bi = (t / NAV_BIT_SEC) as usize % s.nav_bits.len();
                s.nav_bits[bi] as f64
            };
            let phase = TAU * (fi + s.doppler_hz) * t;
            let sv64 = Complex64::from_polar(amp[k] * c * b, phase);
            x += Complex32::new(sv64.re as f32, sv64.im as f32);
        }
        if let Some(rng) = rng.as_mut() {
            x += Complex32::new((rng.gauss() * nstd) as f32, (rng.gauss() * nstd) as f32);
        }
        out.push(x);
    }
    out
}

/// One synthetic Galileo E1-B satellite. Like [`SynthSv`] but the spreading code
/// is the BOC(1,1) E1-B memory code and the data is the I/NAV **symbol** stream
/// (250 sym/s = one symbol per 4 ms code period), e.g. from
/// [`crate::galileo_inav::encode_inav_stream`].
#[derive(Clone)]
pub struct SynthE1Sv {
    pub prn: u8,
    pub doppler_hz: f64,
    /// Initial code phase in BOC sub-chips (0..8184, may be fractional).
    pub code_phase_subchips: f64,
    pub cn0_dbhz: f64,
    /// I/NAV symbols (0/1). Empty = held at 0 (no data), which still acquires and
    /// tracks; supply a stream to exercise frame sync / ephemeris decode.
    pub symbols: Vec<u8>,
}

impl SynthE1Sv {
    pub fn new(prn: u8, doppler_hz: f64, code_phase_subchips: f64, cn0_dbhz: f64) -> Self {
        Self {
            prn,
            doppler_hz,
            code_phase_subchips,
            cn0_dbhz,
            symbols: Vec::new(),
        }
    }

    pub fn with_symbols(mut self, symbols: Vec<u8>) -> Self {
        self.symbols = symbols;
        self
    }
}

/// Like [`synth_l1ca`] but for Galileo E1-B: BOC(1,1) code at the 2.046 Msps
/// sub-chip rate, modulated by each SV's I/NAV symbol stream (one symbol per 4 ms
/// code period). Symbol bit 0 → +1 (positive prompt-I, which the receiver reads
/// back as 0), bit 1 → −1.
pub fn synth_e1(
    svs: &[SynthE1Sv],
    fs: f64,
    fi: f64,
    num_msec: usize,
    seed: Option<u64>,
) -> Vec<Complex32> {
    let code_sp = (fs * 1e-3) as usize;
    let n_total = code_sp * num_msec;

    let codes: Vec<Vec<i8>> = svs
        .iter()
        .map(|s| Signal::GalileoE1b.spreading_code(s.prn).expect("E1-B code"))
        .collect();
    let subchip_rate: Vec<f64> = svs
        .iter()
        .map(|s| E1_SUBCHIP_RATE_HZ * (1.0 + s.doppler_hz / L1_HZ))
        .collect();
    let amp: Vec<f64> = svs
        .iter()
        .map(|s| match seed {
            None => 1.0,
            Some(_) => (10f64.powf(s.cn0_dbhz / 10.0) / fs).sqrt(),
        })
        .collect();

    let mut rng = seed.map(Rng);
    let nstd = 0.5f64.sqrt();
    let inv_fs = 1.0 / fs;
    let mut out = Vec::with_capacity(n_total);
    for n in 0..n_total {
        let t = n as f64 * inv_fs;
        let mut x = Complex32::new(0.0, 0.0);
        for (k, s) in svs.iter().enumerate() {
            let pos = s.code_phase_subchips + subchip_rate[k] * t;
            let c = codes[k][pos.rem_euclid(E1_BOC_LEN as f64) as usize] as f64;
            // One I/NAV symbol per code period (every 8184 sub-chips).
            let b = if s.symbols.is_empty() {
                1.0
            } else {
                let period = (pos / E1_BOC_LEN as f64).floor() as i64;
                let idx = period.rem_euclid(s.symbols.len() as i64) as usize;
                if s.symbols[idx] == 0 { 1.0 } else { -1.0 }
            };
            let phase = TAU * (fi + s.doppler_hz) * t;
            let sv64 = Complex64::from_polar(amp[k] * c * b, phase);
            x += Complex32::new(sv64.re as f32, sv64.im as f32);
        }
        if let Some(rng) = rng.as_mut() {
            x += Complex32::new((rng.gauss() * nstd) as f32, (rng.gauss() * nstd) as f32);
        }
        out.push(x);
    }
    out
}

// ---- Geometry-consistent L1 C/A scene ---------------------------------------

const SECS_PER_WEEK: f64 = 604_800.0;

/// One satellite of a [`GeoFeed`] scene.
struct GeoSv {
    eph: Ephemeris, // the LSB-quantized ephemeris it both broadcasts and flies
    code: Vec<i8>,  // spreading replica (chips; BOC sub-chips for E1)
    chip_rate: f64, // code units per transmit second
    data: Vec<i8>,  // ±1 data stream; item k spans t_tx = tow0 + [k, k+1)·data_period
    data_period: f64,
    amp: f64,
}

/// Geometry-consistent GPS L1 C/A IQ source (a streaming [`IQReader`]).
///
/// Each satellite's code phase, code rate, carrier Doppler and LNAV bit timing
/// all derive from the true range between `rx_ecef` and the orbit propagated
/// from its ephemeris — light-time iteration with the Earth-rotation (Sagnac)
/// convention the solver uses — so the receiver's decoded pseudoranges solve
/// back to `rx_ecef`. The generator flies the *quantized* ephemeris
/// ([`quantize_via_lnav`]): generator and solver use bit-identical orbits, so
/// broadcast-LSB rounding cancels instead of aliasing into the fix.
///
/// The receiver clock is GPST exactly: sample `n` is received at
/// `t0_sow + n/fs`, where `t0_sow` is the first ephemeris' toe (so orbit
/// extrapolation stays near its reference). The LNAV streams start at the
/// last subframe boundary (6 s grid) before the first transmit time, with the
/// HOW TOW counting true GPST — the framing gps-sdr-sim produces.
///
/// Transmit time is evaluated exactly on the 1 ms block grid and interpolated
/// linearly in between (range acceleration over 1 ms is sub-µm). Every sample
/// is a pure function of its absolute index, so the feed is stateless and
/// random-access; with `seed`, AWGN is generated per-block from the seed and
/// block index, keeping that property.
pub struct GeoFeed {
    svs: Vec<GeoSv>,
    rx: [f64; 3],
    fs: f64,
    fi: f64,
    week: u32,
    gst: bool, // epochs built on the GST timescale (Galileo scenes)
    t0_sow: f64,
    tow0: f64,
    block_sp: usize,
    total_samples: usize,
    seed: Option<u64>,
}

impl GeoFeed {
    /// Build a scene of `num_msec` over the satellites in `ephs` (all sharing
    /// one GPS week), received at `rx_ecef`. `seed = None` renders each SV
    /// clean at unit amplitude (`cn0_dbhz` ignored); `Some(s)` adds AWGN with
    /// each SV scaled to `cn0_dbhz`, as in [`synth_l1ca`].
    pub fn new(
        ephs: &[Ephemeris],
        rx_ecef: [f64; 3],
        fs: f64,
        fi: f64,
        num_msec: usize,
        cn0_dbhz: f64,
        seed: Option<u64>,
    ) -> Self {
        Self::new_diverged(ephs, ephs, rx_ecef, fs, fi, num_msec, cn0_dbhz, seed)
    }

    /// [`new`](Self::new), but the LNAV bits broadcast `bcast` while the
    /// signal timing flies `ephs` — the real-world signal-in-space error
    /// situation (the broadcast ephemeris/clock is only a fit to the truth),
    /// which is exactly what SBAS measures and corrects. `bcast[i]` must be
    /// `ephs[i]`'s satellite.
    #[allow(clippy::too_many_arguments)]
    pub fn new_diverged(
        ephs: &[Ephemeris],
        bcast: &[Ephemeris],
        rx_ecef: [f64; 3],
        fs: f64,
        fi: f64,
        num_msec: usize,
        cn0_dbhz: f64,
        seed: Option<u64>,
    ) -> Self {
        assert!(!ephs.is_empty());
        assert!(ephs.len() == bcast.len());
        let week = ephs[0].week;
        assert!(
            ephs.iter().all(|e| e.week == week),
            "one GPS week per scene"
        );
        let t0_sow = ephs[0].toe as f64;
        // LNAV stream origin: the subframe boundary (6 s grid) one subframe
        // before t0, so every transmit time in the run maps to a bit index ≥ 0.
        let tow0 = (t0_sow / 6.0).floor() * 6.0 - 6.0;
        let n_subframes = num_msec / 6000 + 3;

        let amp = match seed {
            None => 1.0,
            Some(_) => (10f64.powf(cn0_dbhz / 10.0) / fs).sqrt(),
        };
        let svs = ephs
            .iter()
            .zip(bcast.iter())
            .map(|(e, b)| {
                assert!(e.sv == b.sv, "flown/broadcast SV mismatch");
                let mut q = quantize_via_lnav(e);
                let wsec = q.week as f64 * SECS_PER_WEEK;
                q.toe_gpst = Epoch::from_gpst_seconds(wsec + q.toe as f64);
                q.toc_gpst = q.toe_gpst;
                // The bit stream — encoding the *broadcast* ephemeris:
                // subframes 1,2,3 cycling, subframe j spanning GPST
                // [tow0 + 6j, tow0 + 6j + 6), HOW = next subframe start / 6.
                let qb = quantize_via_lnav(b);
                let mut bits = Vec::with_capacity(n_subframes * 300);
                for j in 0..n_subframes {
                    let id = (j % 3) as u8 + 1;
                    let how = (tow0 as u32) / 6 + j as u32 + 1;
                    let src = encode_lnav_subframe_source(&qb, id, how);
                    bits.extend(
                        encode_subframe(&src)
                            .iter()
                            .map(|&b| if b == 1 { 1i8 } else { -1 }),
                    );
                }
                GeoSv {
                    eph: q,
                    code: Code::gen_code("L1CA", e.sv.prn).expect("L1CA code"),
                    chip_rate: CODE_RATE_HZ,
                    data: bits,
                    data_period: NAV_BIT_SEC,
                    amp,
                }
            })
            .collect();

        let block_sp = (fs * 1e-3) as usize;
        Self {
            svs,
            rx: rx_ecef,
            fs,
            fi,
            week,
            gst: false,
            t0_sow,
            tow0,
            block_sp,
            total_samples: num_msec * block_sp,
            seed,
        }
    }

    /// The Galileo E1-B twin of [`new`](Self::new): BOC(1,1) memory codes at the
    /// 2.046 Msc/s sub-chip rate, modulated by full I/NAV page streams (word
    /// types 1-5 cycling, 2 s per page). The word-5 WN/TOW follows the OS SIS
    /// ICD convention: the broadcast TOW is the GST at the start of the page
    /// carrying it. `week` fields are GST weeks; the generator builds epochs
    /// via `TimeScale::GST`, exactly as the receiver's I/NAV decoder does.
    pub fn new_e1(
        ephs: &[Ephemeris],
        rx_ecef: [f64; 3],
        fs: f64,
        fi: f64,
        num_msec: usize,
        cn0_dbhz: f64,
        seed: Option<u64>,
    ) -> Self {
        use crate::galileo_inav::{encode_ephemeris_word, encode_inav_page, quantize_via_inav};
        use gnss_rtk::prelude::TimeScale;

        assert!(!ephs.is_empty());
        let week = ephs[0].week;
        assert!(
            ephs.iter().all(|e| e.week == week),
            "one GST week per scene"
        );
        let t0_sow = ephs[0].toe as f64;
        // Stream origin: a 5-page word-cycle boundary (10 s grid) at least one
        // cycle before t0, so every transmit time maps to a data index ≥ 0.
        let tow0 = (t0_sow / 10.0).floor() * 10.0 - 10.0;
        let n_cycles = num_msec / 10_000 + 4;

        let amp = match seed {
            None => 1.0,
            Some(_) => (10f64.powf(cn0_dbhz / 10.0) / fs).sqrt(),
        };
        let svs = ephs
            .iter()
            .map(|e| {
                let mut q = quantize_via_inav(e);
                let ns = |sow: u32| (sow as u64) * 1_000_000_000;
                q.toe_gpst = Epoch::from_time_of_week(q.week, ns(q.toe), TimeScale::GST);
                q.toc_gpst = q.toe_gpst;
                // Pages: word types 1..5 cycling, 2 s each; word 5 carries the
                // GST WN/TOW of its own page's start (ICD convention).
                let mut data = Vec::with_capacity(n_cycles * 5 * 500);
                for k in 0..n_cycles {
                    for wt in 1..=5u8 {
                        let mut e_page = q;
                        if wt == 5 {
                            e_page.tow = tow0 as u32 + 10 * k as u32 + 8;
                        }
                        let page = encode_inav_page(&encode_ephemeris_word(&e_page, wt));
                        data.extend(page.iter().map(|&s| if s == 0 { 1i8 } else { -1 }));
                    }
                }
                GeoSv {
                    eph: q,
                    code: Signal::GalileoE1b
                        .spreading_code(e.sv.prn)
                        .expect("E1-B code"),
                    chip_rate: E1_SUBCHIP_RATE_HZ,
                    data,
                    data_period: 4e-3,
                    amp,
                }
            })
            .collect();

        let block_sp = (fs * 1e-3) as usize;
        Self {
            svs,
            rx: rx_ecef,
            fs,
            fi,
            week,
            gst: true,
            t0_sow,
            tow0,
            block_sp,
            total_samples: num_msec * block_sp,
            seed,
        }
    }

    fn epoch_at(&self, sow: f64) -> Epoch {
        if self.gst {
            // Mirror the receiver's I/NAV epoch construction (GST week + sow).
            use gnss_rtk::prelude::TimeScale;
            Epoch::from_time_of_week(self.week, 0, TimeScale::GST)
                + gnss_rtk::prelude::Duration::from_seconds(sow)
        } else {
            Epoch::from_gpst_seconds(self.week as f64 * SECS_PER_WEEK + sow)
        }
    }

    /// The true transmit time (as an absolute epoch) of the signal received
    /// from `sv` at scene time `ts_sec` — the generator's own ground truth,
    /// for measuring the receiver's transmit-time (anchor) accuracy directly.
    pub fn true_transmit_time(&self, sv: SV, ts_sec: f64) -> Option<Epoch> {
        let g = self.svs.iter().find(|g| g.eph.sv == sv)?;
        let t_rx = self.t0_sow + ts_sec;
        let tau = self.travel_time_sec(&g.eph, t_rx);
        Some(self.epoch_at(t_rx - tau))
    }

    /// Signal travel time from `eph`'s satellite to the receiver for a signal
    /// *received* at GPST `t_rx_sow`: fixed-point light-time iteration, with
    /// the SV rotated into the reception-instant ECEF frame (Earth turns by
    /// ωe·τ while the signal travels — the solver's Sagnac convention).
    fn travel_time_sec(&self, eph: &Ephemeris, t_rx_sow: f64) -> f64 {
        let mut tau = 0.07;
        for _ in 0..3 {
            let (x, y, z) = compute_sv_position_ecef(eph, self.epoch_at(t_rx_sow - tau));
            let w = EARTH_ROTATION_RATE * tau;
            let (cw, sw) = (w.cos(), w.sin());
            let (sx, sy) = (cw * x + sw * y, -sw * x + cw * y);
            let (dx, dy, dz) = (sx - self.rx[0], sy - self.rx[1], z - self.rx[2]);
            tau = (dx * dx + dy * dy + dz * dz).sqrt() / SPEED_OF_LIGHT;
        }
        tau
    }
}

impl IQReader for GeoFeed {
    fn get_iq_data(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        if off_samples + num_samples > self.total_samples {
            return Err("end of file".into());
        }
        let mut out = vec![Complex32::default(); num_samples];
        let b = self.block_sp;
        let inv_fs = 1.0 / self.fs;
        let end = off_samples + num_samples;

        for m in off_samples / b..=(end - 1) / b {
            let (blk_s, blk_e) = (m * b, (m + 1) * b);
            let (s0, s1) = (blk_s.max(off_samples), blk_e.min(end));
            let ts0 = blk_s as f64 * inv_fs;
            let ts1 = blk_e as f64 * inv_fs;

            for sv in &self.svs {
                // Exact transmit times at the block edges; linear in between.
                let tau0 = self.travel_time_sec(&sv.eph, self.t0_sow + ts0);
                let tau1 = self.travel_time_sec(&sv.eph, self.t0_sow + ts1);
                let dtau = (tau1 - tau0) / b as f64; // per sample
                let dtx = inv_fs - dtau;

                // State at the first generated sample of this block.
                let f0 = (s0 - blk_s) as f64;
                let ts = ts0 + f0 * inv_fs;
                let tau = tau0 + f0 * dtau;
                let t_tx = self.t0_sow + ts - tau - self.tow0; // since stream origin
                let code_len = sv.code.len() as f64;
                let mut chip = t_tx * sv.chip_rate;
                let chip_step = dtx * sv.chip_rate;
                let mut bit_pos = t_tx / sv.data_period;
                let bit_step = dtx / sv.data_period;
                // Received carrier: θ = 2π(fi·ts − fc·τ); per-sample rotation.
                // f64 recurrence (f32 would drift over a block), f32 at the sum.
                let mut car = Complex64::from_polar(sv.amp, TAU * (self.fi * ts - L1_HZ * tau));
                let rot = Complex64::from_polar(1.0, TAU * (self.fi * inv_fs - L1_HZ * dtau));

                for i in s0..s1 {
                    let c = sv.code[chip.rem_euclid(code_len) as usize] as f64;
                    let d = sv.data[bit_pos as usize] as f64;
                    let smp = car * (c * d);
                    out[i - off_samples] += Complex32::new(smp.re as f32, smp.im as f32);
                    car *= rot;
                    chip += chip_step;
                    bit_pos += bit_step;
                }
            }

            if let Some(seed) = self.seed {
                // Per-block reseed keeps samples a pure function of their index.
                let mut rng = Rng(seed ^ (m as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
                let nstd = 0.5f64.sqrt();
                for v in out[s0 - off_samples..s1 - off_samples].iter_mut() {
                    *v += Complex32::new((rng.gauss() * nstd) as f32, (rng.gauss() * nstd) as f32);
                }
            }
        }
        Ok(out)
    }
}

/// Pick `n` GPS-like orbits visible from `rx_ecef` at the ephemeris epoch:
/// circular (ecc = 0) zero-clock MEO shells scanned over an (Ω0, M0) grid,
/// keeping the best candidate per azimuth sector for spread-out geometry.
/// Circular zero-clock orbits make every SV clock term (relativistic, f0/f1/f2,
/// TGD) exactly zero, so the scene needs no clock model on either side.
pub fn pick_geo_constellation(rx_ecef: [f64; 3], week: u32, toe: u32, n: usize) -> Vec<Ephemeris> {
    pick_constellation(rx_ecef, week, toe, n, Constellation::GPS)
}

/// The Galileo twin of [`pick_geo_constellation`]: Galileo MEO shells
/// (a ≈ 29 600 km), `week` is a GST week and the epochs are built on the GST
/// timescale, exactly as the receiver's I/NAV decoder builds them.
pub fn pick_geo_constellation_gal(
    rx_ecef: [f64; 3],
    week: u32,
    toe: u32,
    n: usize,
) -> Vec<Ephemeris> {
    pick_constellation(rx_ecef, week, toe, n, Constellation::Galileo)
}

fn pick_constellation(
    rx_ecef: [f64; 3],
    week: u32,
    toe: u32,
    n: usize,
    cons: Constellation,
) -> Vec<Ephemeris> {
    use gnss_rtk::prelude::TimeScale;
    let (a, toe_gpst) = match cons {
        Constellation::Galileo => (
            29_600_000.0,
            Epoch::from_time_of_week(week, toe as u64 * 1_000_000_000, TimeScale::GST),
        ),
        _ => (
            26_560_000.0,
            Epoch::from_gpst_seconds(week as f64 * SECS_PER_WEEK + toe as f64),
        ),
    };
    let rx = Vector3::new(rx_ecef[0], rx_ecef[1], rx_ecef[2]);

    let mut cands: Vec<(f64, f64, Ephemeris)> = Vec::new();
    for i_omg0 in 0..12 {
        for i_m0 in 0..18 {
            let mut e = Ephemeris::new(SV::new(cons, 1));
            e.week = week;
            e.toe = toe;
            e.toc = toe;
            e.toe_gpst = toe_gpst;
            e.toc_gpst = toe_gpst;
            e.a = a;
            e.ecc = 0.0;
            e.i0 = 0.96; // ~55-56°, the GPS/Galileo inclination
            e.omg0 = i_omg0 as f64 * (TAU / 12.0) - PI;
            e.m0 = i_m0 as f64 * (TAU / 18.0) - PI;
            e.omg_dot = -8.0e-9; // typical nodal regression; also gates is_valid
            e.ts_sec = 1.0;
            let pos = compute_sv_position_ecef(&e, toe_gpst);
            let (el, az) = elevation_azimuth(rx, pos);
            if el > 20f64.to_radians() {
                cands.push((el, az, e));
            }
        }
    }

    // Geometry: one near-zenith SV plus a low ring spread in azimuth. The
    // zenith SV pins the vertical — an all-ring (or all-high) pick has poor
    // VDOP: measured 28 m fix error, of which 27.9 m was vertical, vs ~4 m
    // with this mix.
    let mut picked: Vec<(f64, f64, Ephemeris)> = Vec::new();
    if let Some(z) = cands.iter().max_by(|a, b| a.0.total_cmp(&b.0)) {
        picked.push(*z);
    }
    let ring_target = 30f64.to_radians();
    let sectors = n - 1;
    for k in 0..sectors {
        let lo = -PI + k as f64 * TAU / sectors as f64;
        let hi = lo + TAU / sectors as f64;
        if let Some(best) = cands
            .iter()
            .filter(|(_, az, _)| (lo..hi).contains(az))
            .filter(|(el, az, _)| !picked.iter().any(|(pe, pa, _)| pe == el && pa == az))
            .min_by(|a, b| {
                (a.0 - ring_target)
                    .abs()
                    .total_cmp(&(b.0 - ring_target).abs())
            })
        {
            picked.push(*best);
        }
    }

    let mut ephs: Vec<Ephemeris> = picked.into_iter().map(|(_, _, e)| e).collect();
    for (k, e) in ephs.iter_mut().enumerate() {
        e.sv = SV::new(cons, (k + 1) as u8);
    }
    ephs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn output_length_and_unit_amplitude_when_noiseless() {
        let fs = 2_046_000.0;
        let sig = synth_l1ca(&[SynthSv::new(5, 0.0, 0.0, 0.0)], fs, 0.0, 3, None);
        assert_eq!(sig.len(), (fs * 1e-3) as usize * 3);
        // noiseless single SV: |code · carrier| == 1 everywhere.
        assert!(sig.iter().all(|c| (c.norm() - 1.0).abs() < 1e-9));
    }

    #[test]
    fn geo_constellation_is_visible_and_distinct() {
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0]; // Geneva
        let ephs = pick_geo_constellation(truth, 2348, 36_000, 6);
        assert!(ephs.len() >= 5, "found {} SVs above the site", ephs.len());
        let rx = Vector3::new(truth[0], truth[1], truth[2]);
        for e in &ephs {
            assert!(e.is_valid(), "{} ephemeris invalid", e.sv);
            let pos = compute_sv_position_ecef(e, e.toe_gpst);
            let (el, _) = elevation_azimuth(rx, pos);
            assert!(el > 20f64.to_radians(), "{} below mask", e.sv);
        }
        // Distinct PRNs and orbits (the xcorr rejector compares m0/omg0/f0).
        for i in 0..ephs.len() {
            for j in i + 1..ephs.len() {
                assert_ne!(ephs[i].sv.prn, ephs[j].sv.prn);
                assert!(ephs[i].m0 != ephs[j].m0 || ephs[i].omg0 != ephs[j].omg0);
            }
        }
    }

    #[test]
    fn geo_feed_is_deterministic_and_bounded() {
        let truth = [4_396_463.3, 474_169.7, 4_581_510.0];
        let ephs = pick_geo_constellation(truth, 2348, 36_000, 4);
        let mut a = GeoFeed::new(&ephs, truth, 2_046_000.0, 0.0, 20, 45.0, Some(3));
        let mut b = GeoFeed::new(&ephs, truth, 2_046_000.0, 0.0, 20, 45.0, Some(3));
        let xa = a.get_iq_data(1000, 4092).unwrap();
        let xb = b.get_iq_data(1000, 4092).unwrap();
        assert_eq!(xa, xb, "same scene + seed must reproduce the same samples");
        assert_eq!(xa.len(), 4092);
        assert!(a.get_iq_data(0, 21 * 2046).is_err(), "EOF past the end");
    }

    #[test]
    fn noisy_amplitude_tracks_requested_cn0() {
        // The despread carrier power should land near 10^(cn0/10)/fs · 1023
        // (1023 = coherent gain over one code). Just sanity-check the mean noise
        // power is ~1 and the run is deterministic for a fixed seed.
        let fs = 2_046_000.0;
        let a = synth_l1ca(&[SynthSv::new(5, 1000.0, 100.0, 45.0)], fs, 0.0, 2, Some(7));
        let b = synth_l1ca(&[SynthSv::new(5, 1000.0, 100.0, 45.0)], fs, 0.0, 2, Some(7));
        assert_eq!(a, b, "same seed must reproduce the same samples");
        let mean_pwr: f64 = a.iter().map(|c| c.norm_sqr() as f64).sum::<f64>() / a.len() as f64;
        // noise power ~1 dominates the weak signal; should be close to 1.
        assert!((0.7..1.4).contains(&mean_pwr), "mean power {mean_pwr:.3}");
    }
}
