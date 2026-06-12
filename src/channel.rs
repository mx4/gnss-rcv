use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rustfft::FftPlanner;
use rustfft::num_complex::{Complex32, Complex64};
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::Mutex;

const PI: f64 = std::f64::consts::PI;

use crate::code::Signal;
use crate::navigation::Navigation;
use crate::plots::plot_channel;
use crate::state::ChannelState;
use crate::state::GnssState;
use crate::util::doppler_shift;
use crate::util::doppler_shifted_carrier;
use crate::util::get_max_with_idx;
use crate::util::{calc_correlation_inplace, finish_correlation_inplace};

/// Early/late correlator spacing, in chips (prompt ± half a chip). The
/// early-late discriminator's slope — and through it the DLL group delay —
/// depends on the correlation-peak shape *relative to this spacing*; that
/// interaction is what the per-signal `DLL_DISC_GAIN_*` constants calibrate.
const SP_CORR: f64 = 0.5;
/// Offset of the "neutral" correlator (the C/N0 noise reference), in the same
/// code units as `SP_CORR`: far enough out that the spreading-code correlation
/// sits at its off-peak floor, so the tap measures noise only. Was a fixed 80
/// *samples* — 40 chips at the default 2.046 MHz but shrinking with sample
/// rate (3.3 chips at 25 MHz, inside the peak beyond ~54 MHz); in chips it is
/// rate-invariant. Verified no C/N0 shift at 2.046/25 MHz (synth bench, 45 dB-Hz).
const NEUTRAL_CORR: f64 = 40.0;
const T_IDLE: f64 = 3.0;
// A satellite in view acquires within a handful of attempts; the 10 ms
// acquisition CN0 estimate is noisy near the threshold, so a weak-but-real SV
// may fail several times before it locks. Keep the normal retry rate for this
// many failures (well above what any visible SV needs), then back off the idle
// time so a truly-absent PRN stops burning the FFT search every few seconds.
const ACQ_FAIL_GRACE: u32 = 20;
const T_IDLE_MAX: f64 = 30.0;
const T_ACQ: f64 = 0.01; // 10msec acquisition time
// SBAS search throttles (the block is on by default, so its 19 mostly-absent
// PRNs must stay cheap): GEOs are stationary, so once the receiver-side LO
// offset is known from any tracking channel, the Doppler search shrinks to
// LO ± this spread (vs the full ±12 kHz). Idle longer between attempts (the
// 1 msg/s correction stream loses nothing to a slow first lock), and stop
// searching altogether once this many GEOs track — every GEO of a system
// broadcasts the same corrections, two covers redundancy.
const SBAS_DOPPLER_SPREAD_HZ: f64 = 3000.0;
// MEO search half-width once the LO offset is known: ±5 kHz of physical
// Doppler plus margin for LO drift since the measurement.
const ACQ_SPREAD_LO_KNOWN_HZ: f64 = 6000.0;
const T_IDLE_SBAS: f64 = 10.0;
const SBAS_TRACKED_ENOUGH: usize = 2;
// Tracking this long without one CRC-valid message marks a SBAS channel as a
// cross-correlation false lock (a real GEO frames within ~3 s at 1 msg/s).
const SBAS_MSG_TIMEOUT_SEC: f64 = 12.0;
const T_FPULLIN: f64 = 1.0;
const T_NPULLIN: f64 = 1.5; // navigation data pullin time (s)
const HALF_RATE_WINDOW: f64 = 3.0; // code-carrier Doppler slope window (s)
const T_DLL: f64 = 0.01; // non-coherent integration time for DLL
const T_CN0: f64 = 1.0; // averaging time for C/N0
const B_FLL_WIDE: f64 = 10.0; // bandwidth of FLL wide Hz
const B_FLL_NARROW: f64 = 2.0; // bandwidth of FLL narrow Hz
const B_PLL: f64 = 10.0; // bandwidth of PLL filter Hz
const B_DLL: f64 = 0.5; // bandwidth of DLL filter Hz
/// Effective early-late discriminator slope (normalized output per unit code
/// error) that sets the code-loop time constant [`dll_tau`] — a loop-tuning
/// constant only (pull-in and rate-trim gears); it no longer carries any
/// measurement-path role (history in `docs/dll-group-delay.md`).
/// - **BPSK** (L1CA / SBAS): τ ≈ 0.157 s. Historical note: this value was
///   originally calibrated to null a 0.03 m/Hz pseudorange-vs-Doppler slope
///   believed to be DLL group delay; that slope was actually the orbit-epoch
///   error of a t_tx anchored 0.160 s early (see `LNAV_DECODE_LATENCY_SEC`),
///   which τ ≈ 0.157 ≈ 0.160 s mimicked. The loop tuning it set is validated
///   and stands on its own.
const DLL_DISC_GAIN_BPSK: f64 = 3.18;
/// - **BOC(1,1)** (Galileo E1): the *same* value — the loop's effective gain
///   is shape-independent within measurement error (verified on ideal and
///   noisy synthetic BOC scenes and on the real LimeSDR capture). The earlier
///   0.256 calibration ("BOC needs τ ≈ 1.95 s") was in fact compensating the
///   then-undiagnosed 2.000 s I/NAV anchor latency: the anchor's orbit-epoch
///   error produces a Doppler-proportional LOS error of λ·2.0 = 0.381 m/Hz,
///   which τ = 1.95 s mimics (0.371 m/Hz) to 2.5%. Once the anchor was fixed
///   (`nav_anchor_tx` latency alignment), 0.256 became a double-correction —
///   it alone was the ~720 m Galileo measurement noise on LimeSDR; restoring
///   3.18 took the Galileo-only fix 502 → 148 m and the residual
///   inter-system bias to +9 ns. GNSS_DLL_GAIN_BOC still overrides it for
///   experiments.
const DLL_DISC_GAIN_BOC: f64 = 3.18;

/// Code-loop time constant τ = 1/(loop velocity gain): `run_dll` corrects
/// `code_off` at rate `(B_DLL/0.25)·disc_gain` per unit code error. The loop's
/// two derived gains follow from τ — the pull-in kick (τ/t_u is the deadbeat
/// gain) and the rate-trim integrator (B_DLL/2τ, see its noise note).
/// Derived from B_DLL so both track any loop retune.
fn dll_tau(disc_gain: f64) -> f64 {
    0.25 / (B_DLL * disc_gain)
}

// Acquisition Doppler search: +/-12 kHz so a large front-end LO offset (some
// captures sit several kHz off L1, e.g. the CTTC recording's SVs at +5..+10 kHz)
// still lands inside the window, on top of the +/-5 kHz of true GPS Doppler. The
// bin count keeps the step at ~320 Hz (2*12000/75) so resolution is unchanged.
const DOPPLER_SPREAD_HZ: f64 = 12000.0;
const DOPPLER_SPREAD_BINS: usize = 75;
// Full history depth, kept only when diagnostics are consumed (--plots or the UI
// tab). The five ring buffers at this depth are ~1.5 MB/tracking channel plus an
// equal published clone — pure diagnostic weight a headless run never reads.
const HISTORY_NUM: usize = 20000;
// Headless depth: the tracking loops read at most the last two entries (the FLL
// at run_fll reads corr_p[len-1] and [len-2]; everything else is `.back()`), so
// a tiny ring suffices when nothing is plotting. The margin over 2 is free.
const HISTORY_MIN: usize = 4;
/// Acquisition accept / tracking drop C/N0 gates (dB-Hz). The 6 dB gap is
/// hysteresis: the 10 ms acquisition estimate is noisy, so the accept gate
/// sits well above the floor where the loops still work — a marginal SV that
/// just locked isn't immediately re-dropped, and a PRN near the floor doesn't
/// churn through lock/loss cycles.
const CN0_THRESHOLD_LOCKED: f64 = 35.0;
const CN0_THRESHOLD_LOST: f64 = 29.0;

#[derive(PartialEq, Debug, Clone)]
pub enum State {
    Tracking,
    Acquisition,
    Idle,
}

#[derive(Default)]
pub struct Tracking {
    prn_code: Vec<f32>, // upsampled real (±1) code; built on lock, freed on idle
    doppler_hz: f64,
    code_off_sec: f64,
    code_rate_trim: f64,        // DLL integrator: residual code-rate error (s/s)
    disc_saturated_streak: u32, // consecutive DLL updates with the discriminator saturated
    cn0: f64,
    accum_doppler_cyc: f64,
    phi: f64,
    err_phase: f64,
    sum_corr_e: f64,
    sum_corr_l: f64,
    sum_corr_p: f64,
    sum_corr_noise: f64,
    tx_phase_ref_ts: f64, // baseline rx time for the code-implied-Doppler slope (0 = unset)
    tx_phase_ref: f64,    // baseline transmit phase paired with tx_phase_ref_ts
    divergence_ema: f64,  // self-calibrated healthy code-carrier divergence, Hz (0 = unset)
    divergence_streak: u32, // consecutive windows the divergence exceeds the gate
}

// Rolling per-channel diagnostics. These are capped ring buffers: new samples
// are pushed at the back, the oldest are dropped from the front once the buffer
// reaches HISTORY_NUM. VecDeque makes that pop_front O(1); a Vec would memmove
// the whole (HISTORY_NUM-element) buffer on every code period.
#[derive(Default, Clone)]
pub struct History {
    last_log_ts: f64,
    last_plot_ts: f64,
    // pub(crate) so the plotting lives in plots.rs (see plots::plot_channel).
    pub(crate) code_phase_offset: VecDeque<f64>,
    pub(crate) phi_error: VecDeque<f64>,
    pub(crate) doppler_hz: VecDeque<f64>,
    pub(crate) cn0: VecDeque<f64>,
    pub corr_p: VecDeque<Complex64>,
}

impl History {
    pub fn trim(&mut self, cap: usize) {
        if self.doppler_hz.len() > cap {
            self.doppler_hz.pop_front();
        }
        if self.phi_error.len() > cap {
            self.phi_error.pop_front();
        }
        if self.corr_p.len() > cap {
            self.corr_p.pop_front();
        }
        if self.code_phase_offset.len() > cap {
            self.code_phase_offset.pop_front();
        }
        if self.cn0.len() > cap {
            self.cn0.pop_front();
        }
    }
}

#[derive(Default)]
pub struct Acquisition {
    prn_code_fft: Vec<Complex32>,
    // Integration grid for the *current* attempt only: one row per searched
    // Doppler bin (row k = absolute bin `bin_range.0 + k`), freed on lock and
    // on idle. Held permanently and full-width it was 30 MB/channel at
    // 50 Msps — 1.5 GB across a default channel set, mostly for channels that
    // were long since tracking. f32: each cell accumulates only ~10 values
    // (T_ACQ / code_sec), so there is no long-sum precision concern.
    sum_p: Vec<Vec<f32>>,
    // Pre-computed carrier replica for each Doppler bin. The bins are fixed and
    // PRN-independent (they depend only on fs/fi/code_sp), so the whole set is
    // built once by the receiver ([`Channel::build_carriers`]) and *shared* across
    // every channel — at 50 MHz each set is ~240 MB, so duplicating it per PRN
    // (36×) was ~8.6 GB and OOM'd. Sharing it avoids that and the sin/cos per
    // sample per bin on every acquisition step (the dominant search-time cost).
    carriers: Arc<Vec<Vec<Complex32>>>,
    // Half-open bin range this attempt searches — the full sweep until the
    // front-end LO offset is known (see `acquisition_init`).
    bin_range: (usize, usize),
}

/// Per-block acquisition FFT cache: for each Doppler bin, the carrier-mixed,
/// forward-FFT'd window — PRN-independent, so the receiver computes each bin
/// once per family step and every searching channel reuses it (the per-PRN
/// remainder is multiply + IFFT; see `finish_correlation_inplace`). Bins
/// outside the cached range fall back to the channel's own mix+FFT.
pub struct AcqFftCache {
    pub bins: Vec<Option<Vec<Complex32>>>,
}

/// Bin range covering `center_hz ± spread_hz` of the fixed Doppler grid.
fn doppler_bin_range(center_hz: f64, spread_hz: f64) -> (usize, usize) {
    let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;
    let lo = ((center_hz - spread_hz + DOPPLER_SPREAD_HZ) / step_hz).floor() as isize;
    let hi = ((center_hz + spread_hz + DOPPLER_SPREAD_HZ) / step_hz).ceil() as isize + 1;
    (
        lo.clamp(0, DOPPLER_SPREAD_BINS as isize) as usize,
        hi.clamp(0, DOPPLER_SPREAD_BINS as isize) as usize,
    )
}

/// Per-channel work/quality counters, aggregated and printed at end of run.
/// Plain counters bumped in the hot path (no locking).
#[derive(Default, Clone)]
pub struct ChannelStats {
    pub acq_attempts: u64,   // completed acquisition attempts (T_ACQ blocks)
    pub acq_corrs: u64,      // acquisition correlations done (= FFT pairs)
    pub locks: u64,          // successful acquisitions -> tracking
    pub lock_losses: u64,    // lost track (cn0 < threshold while tracking)
    pub trk_periods: u64,    // code periods correlated while tracking (total)
    pub trk_streak: u64,     // current continuous tracking run (periods)
    pub max_trk_streak: u64, // longest continuous tracking run (periods)
    pub first_lock_ts: f64,  // ts_sec of first lock (0 = never)
    pub peak_cn0: f64,       // best C/N0 seen while tracking
    pub subframes: u64,      // LNAV subframes decoded (parity OK)
    pub parity_errors: u64,  // LNAV parity failures
    pub used_in_fix: bool,   // contributed to at least one successful fix
}

/// The per-channel scalar parameters passed to [`Channel::new`] — signal
/// identity, front-end rates, and diagnostics flags. Bundled so the
/// constructor takes the config plus its two shared handles (state, carriers)
/// rather than a long positional argument list.
pub struct ChannelConfig {
    pub sig: Signal,
    pub sv: SV,
    pub fs: f64,
    pub fi: f64,
    pub plots: bool,
    pub diagnostics: bool,
}

pub struct Channel {
    pub pub_state: Arc<Mutex<GnssState>>,
    pub sv: SV,
    sig: Signal, // kept so the tracking code / acq FFT can be rebuilt on demand
    pub stats: ChannelStats,
    plots: bool,
    // Diagnostics consumer present (--plots or the egui tab): keep the full
    // History depth and publish the 2 s snapshot. Off → the loops keep only a
    // tiny ring and skip the snapshot clone (see HISTORY_MIN, update_all_plots).
    diagnostics: bool,
    fc: f64, // carrier frequency
    fs: f64, // sampling frequency
    fi: f64, // intermediate frequency

    code_sec: f64,   // code duration in sec
    code_len: usize, // prn code len: e.g. 1023
    code_sp: usize,  // samples per upsampled code: e.g. 2046 for L1CA
    tau_dll: f64,    // code-loop time constant (pull-in / rate-trim gains)

    fft_planner: FftPlanner<f32>,
    state: State,

    pub ts_sec: f64, // current time
    pub num_trk_periods: usize,
    // Continuous count of transmitted code periods since tracking start. Unlike
    // `num_trk_periods` (whose +/-1 wraps are for correlation-buffer alignment),
    // this advances by +1 per processed code period and is corrected at code-phase
    // wraps with the sign that keeps the *transmit phase* continuous. It is the
    // source for the pseudorange transmit-time and must not be reused for buffering.
    num_tx_codes: f64,
    num_acq_periods: usize,
    num_idl_periods: usize,
    // Consecutive failed acquisitions (reset on a successful lock); drives the
    // idle backoff for PRNs that are very likely not in view.
    num_acq_fails: u32,
    // Peak C/N0 of the most recent acquisition attempt. Kept even when it is
    // below the lock threshold so a search (e.g. for weak SBAS GEOs) can tell
    // "present but weak" from "noise floor"; surfaced in the IDLE log.
    acq_cn0: f64,
    // stats.subframes at tracking start — the baseline for the SBAS
    // no-message false-lock timeout.
    subfr_at_lock: u64,

    pub hist: History,
    pub nav: Navigation,
    // One mix/correlation scratch (one code period). A channel is in exactly one
    // state, so acquisition and tracking never need their scratch at once — they
    // share this buffer instead of holding one each. Freed on idle.
    scratch: Vec<Complex32>,
    trk: Tracking,
    acq: Acquisition,
}

impl Drop for Channel {
    fn drop(&mut self) {
        if self.plots {
            self.update_all_plots(true);
        }
    }
}

impl Channel {
    /// Seconds of signal tracked (code periods correlated × code duration).
    pub fn tracked_secs(&self) -> f64 {
        self.stats.trk_periods as f64 * self.code_sec
    }

    /// Longest uninterrupted lock, in seconds (separates real SVs from the brief
    /// false locks every PRN picks up during the search).
    pub fn max_lock_secs(&self) -> f64 {
        self.stats.max_trk_streak as f64 * self.code_sec
    }

    pub fn get_cn0(&self) -> f64 {
        if self.state != State::Tracking {
            return 0.0;
        }

        self.trk.cn0
    }

    pub fn is_state_tracking(&self) -> bool {
        self.state == State::Tracking
    }

    pub fn is_ephemeris_complete(&self) -> bool {
        self.nav.eph.is_valid()
    }

    /// Apply `update` to this channel's shared `ChannelState` under one lock,
    /// then fire the UI repaint callback if the channel is tracking. Centralizes
    /// the lock / get_mut / update_func boilerplate the per-field updaters share.
    fn publish<F: FnOnce(&mut ChannelState)>(&self, update: F) {
        let tracking = {
            let mut st = self.pub_state.lock().unwrap();
            let cs = st.channels.get_mut(&self.sv).unwrap();
            update(cs);
            cs.state == State::Tracking
        };
        if tracking {
            (self.pub_state.lock().unwrap().update_func.func)();
        }
    }

    fn set_state(&mut self, state: State) {
        // Fire the UI callback only on an Idle<->Tracking transition.
        let transitioned = {
            let mut st = self.pub_state.lock().unwrap();
            let cs = st.channels.get_mut(&self.sv).unwrap();
            let old_state = cs.state.clone();
            cs.state = state.clone();
            (state == State::Tracking && old_state == State::Idle)
                || (state == State::Idle && old_state == State::Tracking)
        };
        if transitioned {
            (self.pub_state.lock().unwrap().update_func.func)();
        }
        self.state = state;
    }

    fn update_state_phi(&self) {
        let phi = self.trk.phi;
        self.publish(|cs| cs.phi = phi);
    }

    fn update_state_code_idx(&self) {
        let code_idx = *self.hist.code_phase_offset.back().unwrap();
        self.publish(|cs| cs.code_idx = code_idx);
    }

    fn update_state_doppler_hz(&self) {
        let doppler_hz = self.trk.doppler_hz;
        self.publish(|cs| cs.doppler_hz = doppler_hz);
    }

    fn update_state_cn0(&self) {
        let cn0 = self.trk.cn0;
        // Piggyback the ephemeris decode progress on the frequent cn0 publish so
        // the UI's per-SV "pages/needed" counter advances as words arrive.
        let eph_pages = self.nav.eph.pages();
        self.publish(|cs| {
            cs.cn0 = cn0;
            cs.eph_pages = eph_pages;
        });
    }

    /// Build the per-Doppler-bin carrier replicas once, to be shared (cloned
    /// `Arc`) across every channel — they depend only on `fs`/`fi`/`code_sp`, not
    /// the PRN. See [`Acquisition::carriers`].
    /// The Doppler bin range this channel's *next* acquisition pass will
    /// search, if it is searching — the receiver unions these to size the
    /// shared FFT cache.
    pub fn acq_search_range(&self) -> Option<(usize, usize)> {
        if self.state != State::Acquisition {
            return None;
        }
        Some(
            if self.acq.sum_p.len() == self.acq.bin_range.1 - self.acq.bin_range.0 {
                self.acq.bin_range
            } else {
                // First pass after being born in Acquisition: the lazy init
                // will pick the full sweep (or the LO window) when it runs.
                (0, DOPPLER_SPREAD_BINS)
            },
        )
    }

    pub fn build_carriers(fs: f64, fi: f64, code_sp: usize) -> Arc<Vec<Vec<Complex32>>> {
        let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;
        Arc::new(
            (0..DOPPLER_SPREAD_BINS)
                .map(|i| {
                    let doppler_hz = -DOPPLER_SPREAD_HZ + i as f64 * step_hz;
                    doppler_shifted_carrier(fi + doppler_hz, 0.0, fs, code_sp)
                })
                .collect(),
        )
    }

    pub fn new(
        config: ChannelConfig,
        pub_state: Arc<Mutex<GnssState>>,
        carriers: Arc<Vec<Vec<Complex32>>>,
    ) -> Self {
        let ChannelConfig {
            sig,
            sv,
            fs,
            fi,
            plots,
            diagnostics,
        } = config;
        let code_buf = sig.spreading_code(sv.prn).unwrap_or_else(|| {
            panic!(
                "no spreading code for {sig} PRN {} (Galileo E1 codes pending)",
                sv.prn
            )
        });
        let code_sec = sig.code_period_sec();
        let code_len = sig.code_len();
        let code_sp = (fs * code_sec) as usize;
        // Per-signal code-loop time constant (sets the pull-in and rate-trim
        // gains in run_dll); same gain for BPSK and BOC(1,1), see dll_tau.
        let tau_dll = dll_tau(if sig.is_boc11() {
            // GNSS_DLL_GAIN_BOC overrides the BOC loop gain for experiments
            // (history of the retired 0.256 value in docs/dll-group-delay.md).
            std::env::var("GNSS_DLL_GAIN_BOC")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(DLL_DISC_GAIN_BOC)
        } else {
            DLL_DISC_GAIN_BPSK
        });
        let mut fft_planner = FftPlanner::new();

        // Acquisition's complex FFT replica, resampled to the actual samples-per-
        // code-period (code_sp = fs * code_sec) so any sampling rate works (for
        // fs = 2.046 MHz this is exactly 2 samples/chip). The real tracking code
        // is *not* built here — it is built on lock (build_prn_code) so an
        // un-acquired channel carries none of it.
        let mut prn_code_fft: Vec<Complex32> = (0..code_sp)
            .map(|i| Complex32::new(code_buf[i * code_len / code_sp] as f32, 0.0))
            .collect();

        let fft_fw = fft_planner.plan_fft_forward(prn_code_fft.len());
        fft_fw.process(&mut prn_code_fft);

        pub_state
            .lock()
            .unwrap()
            .channels
            .insert(sv, ChannelState::default());

        Self {
            pub_state: pub_state.clone(),
            sv,
            sig,
            stats: ChannelStats::default(),
            plots,
            diagnostics,
            fft_planner,
            ts_sec: 0.0,
            fc: sig.carrier_hz(),
            fs,
            fi,
            code_sec,
            code_len,
            code_sp,
            tau_dll,

            num_acq_periods: 0,
            num_idl_periods: 0,
            num_acq_fails: 0,
            subfr_at_lock: 0,
            acq_cn0: 0.0,
            num_trk_periods: 0,
            num_tx_codes: 0.0,

            state: State::Acquisition,
            nav: Navigation::new(sv),
            hist: History::default(),
            scratch: Vec::new(),
            trk: Tracking::default(),
            acq: Acquisition {
                prn_code_fft,
                sum_p: Vec::new(),
                carriers,
                bin_range: (0, DOPPLER_SPREAD_BINS),
            },
        }
    }

    /// Resample the spreading code to `code_sp` real (±1) samples for the
    /// tracking correlator. Cheap (one resample, no FFT); built on lock so an
    /// un-acquired channel holds none of it.
    fn build_prn_code(&self) -> Vec<f32> {
        let code_buf = self
            .sig
            .spreading_code(self.sv.prn)
            .expect("spreading code present (constructed in Channel::new)");
        (0..self.code_sp)
            .map(|i| code_buf[i * self.code_len / self.code_sp] as f32)
            .collect()
    }

    /// The forward-FFT'd complex code replica for FFT acquisition. Rebuilt when a
    /// channel re-enters acquisition after a lock freed it (built in `new` for a
    /// freshly-born channel). Bit-identical to that original.
    fn build_acq_fft(&mut self) -> Vec<Complex32> {
        let mut fft: Vec<Complex32> = self
            .build_prn_code()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        self.fft_planner
            .plan_fft_forward(fft.len())
            .process(&mut fft);
        fft
    }

    fn idle_start(&mut self) {
        // Idle channels hold no search state, no mix scratch, and no tracking
        // code (rebuilt on the next lock). The acq FFT is kept: an idle channel
        // re-acquires, and dropping it from a tracking channel happens on lock.
        self.acq.sum_p = Vec::new();
        self.scratch = Vec::new();
        self.trk.prn_code = Vec::new();
        if self.state == State::Tracking {
            self.stats.lock_losses += 1;
            log::warn!(
                "{}: {} cn0={:.1} ts_sec={:.3}",
                self.sv,
                "LOST".red(),
                self.trk.cn0,
                self.ts_sec,
            );
        } else {
            // acq_cn0 is the peak C/N0 of the last (failed) acquisition: useful
            // to gauge a weak signal that didn't reach the lock threshold.
            log::info!(
                "{}: IDLE acq_cn0={:.1} ts_sec={:.3}",
                self.sv,
                self.acq_cn0,
                self.ts_sec,
            );
        }

        self.set_state(State::Idle);
        self.num_idl_periods = 0;
        self.num_trk_periods = 0;
        self.num_acq_periods = 0;
    }

    fn idle_process(&mut self) {
        self.num_idl_periods += 1;
        // Normal retry rate during the grace window; afterwards (PRN very likely
        // absent) grow the idle time linearly, capped, so the FFT search runs far
        // less often. A successful lock resets num_acq_fails, so a visible SV that
        // locks during grace is never throttled. SBAS idles longer from the start:
        // the block is on by default and a slow first lock costs nothing (the
        // correction stream repeats every second).
        let is_sbas = self.sv.constellation == Constellation::SBAS;
        let t_idle = if is_sbas { T_IDLE_SBAS } else { T_IDLE };
        let idle = if self.num_acq_fails <= ACQ_FAIL_GRACE {
            t_idle
        } else {
            (t_idle * (self.num_acq_fails - ACQ_FAIL_GRACE + 1) as f64).min(T_IDLE_MAX)
        };
        if self.num_idl_periods as f64 * self.code_sec > idle {
            // Enough GEOs already track → their corrections are flowing; the
            // remaining SBAS PRNs stop burning the FFT search entirely.
            if is_sbas
                && self
                    .pub_state
                    .lock()
                    .unwrap()
                    .channels
                    .iter()
                    .filter(|(sv, cs)| {
                        sv.constellation == Constellation::SBAS && cs.state == State::Tracking
                    })
                    .count()
                    >= SBAS_TRACKED_ENOUGH
            {
                self.num_idl_periods = 0;
                return;
            }
            self.acquisition_start();
        }
    }

    fn acquisition_init(&mut self) {
        // Rebuild the acq FFT if a prior lock freed it (a freshly-born channel
        // already has it from `new`); tracking-only channels carry none.
        if self.acq.prn_code_fft.is_empty() {
            self.acq.prn_code_fft = self.build_acq_fft();
        }
        self.num_acq_periods = 0;
        self.num_idl_periods = 0;
        self.num_trk_periods = 0;
        // The ±12 kHz sweep exists for the *unknown front-end LO offset*, not
        // for satellite dynamics — which is common-mode, so any tracking
        // channel's Doppler measures it (median, against outliers). Once one
        // is available, every search shrinks to LO ± the physical Doppler
        // span: ±6 kHz for MEO satellites (±5 kHz orbital + margin), ±3 kHz
        // for geostationary SBAS. Until then the full sweep stands.
        self.acq.bin_range = (0, DOPPLER_SPREAD_BINS);
        let dopplers: Vec<f64> = {
            let st = self.pub_state.lock().unwrap();
            st.channels
                .values()
                .filter(|cs| cs.state == State::Tracking)
                .map(|cs| cs.doppler_hz)
                .collect()
        };
        if !dopplers.is_empty() {
            let mut d = dopplers;
            d.sort_by(f64::total_cmp);
            let lo_offset = d[d.len() / 2];
            let spread = if self.sv.constellation == Constellation::SBAS {
                SBAS_DOPPLER_SPREAD_HZ
            } else {
                ACQ_SPREAD_LO_KNOWN_HZ
            };
            self.acq.bin_range = doppler_bin_range(lo_offset, spread);
        }
        // Size the grid to this attempt's searched range, reusing rows from
        // the previous attempt where possible.
        let rows = self.acq.bin_range.1 - self.acq.bin_range.0;
        self.acq.sum_p.truncate(rows);
        for row in &mut self.acq.sum_p {
            row.fill(0.0);
        }
        while self.acq.sum_p.len() < rows {
            self.acq.sum_p.push(vec![0.0; self.code_sp]);
        }
    }

    fn acquisition_start(&mut self) {
        self.acquisition_init();
        self.set_state(State::Acquisition);
    }

    fn tracking_init(&mut self) {
        // A locked channel holds no search state (sum_p was 30 MB/channel
        // full-width at 50 Msps) and no acquisition FFT — it builds the real
        // tracking code instead. The acq FFT is rebuilt if the lock is later
        // lost and the channel re-enters acquisition (acquisition_init).
        self.acq.sum_p = Vec::new();
        self.acq.prn_code_fft = Vec::new();
        self.trk.prn_code = self.build_prn_code();
        self.trk.doppler_hz = 0.0;
        self.trk.cn0 = 0.0;
        self.trk.accum_doppler_cyc = 0.0;
        self.trk.code_off_sec = 0.0;
        // code_rate_trim deliberately survives re-locks: the rate it learns
        // (dominated by the front end's sample-clock vs LO skew, ~850 ns/s on
        // SJTU) is a receiver constant, not a per-pass quantity — re-winding
        // from zero leaves every re-lock a ~10 s biased tail. It is zeroed
        // only on a code-carrier divergence drop (known-bad loop state).
        self.trk.disc_saturated_streak = 0;
        self.trk.err_phase = 0.0;
        self.trk.sum_corr_p = 0.0;
        self.trk.sum_corr_e = 0.0;
        self.trk.sum_corr_l = 0.0;
        self.trk.sum_corr_noise = 0.0;
        self.trk.tx_phase_ref_ts = 0.0;
        self.trk.tx_phase_ref = 0.0;
        self.trk.divergence_ema = 0.0;
        self.trk.divergence_streak = 0;
        self.num_trk_periods = 0;
        self.num_acq_periods = 0;
        self.num_idl_periods = 0;
        self.num_tx_codes = 0.0;
        self.subfr_at_lock = self.stats.subframes;
        self.nav.meas = crate::ephemeris::Measurement::default();
        self.publish(|cs| cs.tx_anchored = false);
        self.nav.init();
    }

    fn tracking_start(
        &mut self,
        doppler_hz: f64,
        cn0: f64,
        code_off_sec: f64,
        code_offset_idx: usize,
    ) {
        log::warn!(
            "{}: {} cn0={cn0:.1} dopp={doppler_hz:5.0} code_off={code_offset_idx:4} ts_sec={:.3}",
            self.sv,
            "LOCK".green(),
            self.ts_sec,
        );
        self.tracking_init();
        self.set_state(State::Tracking);
        self.num_acq_fails = 0; // in view: restore full-rate retry on future loss

        self.stats.locks += 1;
        self.stats.trk_streak = 0; // start a fresh continuous-tracking run
        if self.stats.first_lock_ts == 0.0 {
            self.stats.first_lock_ts = self.ts_sec;
        }

        self.trk.code_off_sec = code_off_sec;
        self.trk.doppler_hz = doppler_hz;
        self.update_state_doppler_hz();
        self.trk.cn0 = cn0;
        self.update_state_cn0();
    }

    fn update_all_plots(&mut self, force: bool) {
        if !force && self.ts_sec - self.hist.last_plot_ts <= 2.0 {
            return;
        }
        self.hist.last_plot_ts = self.ts_sec;

        if self.plots {
            plot_channel(self.sv, &self.hist, self.code_sec);
        }

        // Publish a history snapshot at 2-second cadence for the diagnostics tab.
        // Only the UI/plots consume it, and headless the History is trimmed to
        // HISTORY_MIN anyway — so skip the ~1.5 MB clone when nothing reads it.
        if self.diagnostics
            && let Ok(mut st) = self.pub_state.lock()
        {
            st.histories.insert(self.sv, self.hist.clone());
        }
    }

    fn acquisition_process(&mut self, iq_vec: &[Complex32], cache: Option<&AcqFftCache>) {
        // Channels are *born* in Acquisition without passing through
        // acquisition_start (which sizes the grid for the attempt) — set up
        // here on the first pass.
        if self.acq.sum_p.len() != self.acq.bin_range.1 - self.acq.bin_range.0 {
            self.acquisition_init();
        }
        // The feed hands us two code periods; correlate over the most recent one.
        let iq_vec_slice = &iq_vec[self.code_sp..];
        let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;
        let (bin_lo, bin_hi) = self.acq.bin_range;

        for i in bin_lo..bin_hi {
            self.stats.acq_corrs += 1;
            self.scratch.clear();
            // The bin's carrier-mixed forward FFT is PRN-independent: take it
            // from the shared per-block cache when present (every searching
            // channel reuses it; only multiply+IFFT remain per PRN), else
            // mix + FFT locally. (scratch, fft_planner, acq are disjoint fields.)
            match cache.and_then(|c| c.bins.get(i)).and_then(|b| b.as_ref()) {
                Some(fft) => {
                    self.scratch.extend_from_slice(fft);
                    finish_correlation_inplace(
                        &mut self.fft_planner,
                        &mut self.scratch,
                        &self.acq.prn_code_fft,
                    );
                }
                None => {
                    self.scratch.extend_from_slice(iq_vec_slice);
                    for (s, c) in self.scratch.iter_mut().zip(self.acq.carriers[i].iter()) {
                        *s *= *c;
                    }
                    calc_correlation_inplace(
                        &mut self.fft_planner,
                        &mut self.scratch,
                        &self.acq.prn_code_fft,
                    );
                }
            }
            for (p, v) in self.acq.sum_p[i - bin_lo]
                .iter_mut()
                .zip(self.scratch.iter())
            {
                *p += v.norm_sqr();
            }
        }

        self.num_acq_periods += 1;

        if self.num_acq_periods as f64 * self.code_sec >= T_ACQ {
            let mut code_offset_idx = 0;
            let mut idx = 0;
            let mut p_peak = 0.0;
            let mut p_total = 0.0;

            // Pick the (Doppler, code-phase) cell with the highest correlation
            // *peak* (standard acquisition: the global max of the 2D surface).
            // Selecting the bin by total integrated power (sum over all code
            // phases) instead biases toward whichever Doppler bin holds the most
            // spread interference energy; with several strong SVs present (e.g.
            // the CTTC capture) that steers acquisition to an interference bin
            // near 0 Hz and misses the real auto-correlation peak.
            for i in bin_lo..bin_hi {
                let row = &self.acq.sum_p[i - bin_lo];
                let (j_peak, v_peak) = get_max_with_idx(row);
                if v_peak > p_peak {
                    idx = i;
                    p_peak = v_peak;
                    code_offset_idx = j_peak;
                }
                p_total += row.iter().map(|&v| v as f64).sum::<f64>();
            }

            // Report the carrier frequency of the winning bin (the replicas are
            // built at -SPREAD + i*step, line ~293), not the bin *center*: a
            // +0.5*step here would seed tracking ~160 Hz off the frequency that
            // actually produced the peak, hurting carrier pull-in / bit sync.
            let doppler_hz = -DOPPLER_SPREAD_HZ + idx as f64 * step_hz;
            let code_off_sec = code_offset_idx as f64 / self.code_sp as f64 * self.code_sec;
            // C/N0 estimate from the search surface: the signal occupies ~one
            // (Doppler, code-phase) cell, so the surface mean is the noise
            // power per cell; (peak − mean)/mean is the SNR in the coherent
            // bandwidth 1/T, and dividing by T = code_sec converts to the
            // 1 Hz reference: C/N0 = 10·log10(SNR / T) dB-Hz.
            let p_avg = p_total / self.code_sp as f64 / (bin_hi - bin_lo) as f64;
            let cn0 = 10.0 * ((p_peak as f64 - p_avg) / p_avg / self.code_sec).log10();
            self.acq_cn0 = cn0;
            self.stats.acq_attempts += 1;

            if cn0 >= CN0_THRESHOLD_LOCKED {
                self.tracking_start(doppler_hz, cn0, code_off_sec, code_offset_idx);
            } else {
                self.num_acq_fails = self.num_acq_fails.saturating_add(1);
                self.idle_start();
            }
            self.acquisition_init();
        }
    }

    fn tracking_compute_correlation(
        &mut self,
        window: &[Complex32],
    ) -> (Complex64, Complex64, Complex64, Complex64) {
        let n = self.code_sp as i32;
        let code_idx = *self.hist.code_phase_offset.back().unwrap() as i32;
        assert!(-n < code_idx && code_idx < n);

        //       [-------][-------][---------]
        // t=n   [^(.......)      ]                code_idx=0
        // t=n+1          [       ^(.......) ]     code_idx=-1

        let lo = if code_idx >= 0 {
            code_idx
        } else {
            n + code_idx
        };
        assert!(lo >= 0);
        let lo_u = lo as usize;
        let hi_u = (lo + n) as usize;

        // Mix the received code period down to baseband into a reused scratch
        // buffer (no per-call allocation; doppler_shift mixes by phase
        // recurrence). The carrier to remove is the full fi + doppler, not just
        // the Doppler: with a non-zero intermediate frequency the signal sits at
        // fi + doppler, so mixing by doppler alone leaves an fi residual that
        // destroys the prompt (fi=0 baseband recordings are unaffected).
        let (fc, phi, fs) = (self.fi + self.trk.doppler_hz, self.trk.phi, self.fs);
        self.scratch.clear();
        self.scratch.extend_from_slice(&window[lo_u..hi_u]);
        doppler_shift(&mut self.scratch, fc, phi, fs);

        // Samples per code-unit (chip; BOC sub-chip for E1) at this sample rate.
        let sp_per_chip = self.code_sec * self.fs / self.code_len as f64;
        let pos = (SP_CORR * sp_per_chip) as usize;
        let pos_noise = (NEUTRAL_CORR * sp_per_chip) as usize;

        let sig = &self.scratch;
        let code = &self.trk.prn_code;
        let len = sig.len();

        // f64 accumulators: 50k f32 products summed in f32 would cost ~3
        // significant digits of the discriminator inputs. The multiplies (the
        // SIMD-heavy part) stay f32. The code is real (±1), so sig·code is two
        // scalar multiplies (sig.re·c, sig.im·c), not a full complex product.
        #[inline]
        fn acc(a: &mut Complex64, s: Complex32, c: f32) {
            a.re += (s.re * c) as f64;
            a.im += (s.im * c) as f64;
        }
        let mut corr_prompt = Complex64::default();
        let mut corr_early = Complex64::default();
        let mut corr_late = Complex64::default();
        let mut corr_noise = Complex64::default();

        // Single fused pass: prompt (full), early/late (offset by +/-pos) and
        // noise (offset by pos_noise) accumulated together, reading sig[j]
        // and code[j] once each instead of in four separate passes.
        for j in 0..len {
            let sj = sig[j];
            let cj = code[j];
            acc(&mut corr_prompt, sj, cj);
            if j + pos < len {
                acc(&mut corr_early, sj, code[j + pos]);
                acc(&mut corr_late, sig[j + pos], cj);
            }
            if j + pos_noise < len {
                acc(&mut corr_noise, sj, code[j + pos_noise]);
            }
        }

        corr_prompt /= len as f64;
        corr_early /= (len - pos) as f64;
        corr_late /= (len - pos) as f64;
        corr_noise /= (len - pos_noise) as f64;

        (corr_prompt, corr_early, corr_late, corr_noise)
    }

    fn run_fll(&mut self) {
        if self.num_trk_periods < 2 {
            return;
        }
        let len = self.hist.corr_p.len();
        let c1 = self.hist.corr_p[len - 1];
        let c2 = self.hist.corr_p[len - 2];
        let dot = c1.re * c2.re + c1.im * c2.im;
        let cross = c1.re * c2.im - c1.im * c2.re;

        if dot == 0.0 {
            return;
        }

        let b = if self.trk_elapsed_sec() < T_FPULLIN / 2.0 {
            B_FLL_WIDE
        } else {
            B_FLL_NARROW
        };
        let err_freq = (cross / dot).atan() / 2.0 / PI;

        // First-order loop: noise bandwidth Bn = G/4, so the gain is 4·Bn.
        self.trk.doppler_hz -= b / 0.25 * err_freq;
        self.update_state_doppler_hz();
    }

    fn run_pll(&mut self, corr_prompt: Complex64) {
        if corr_prompt.re == 0.0 {
            return;
        }
        let err_phase = (corr_prompt.im / corr_prompt.re).atan() / 2.0 / PI;
        // Standard 2nd-order loop, damping ζ = 1/√2: Bn = ωn·(ζ + 1/(4ζ))/2 ≈
        // 0.53·ωn, hence ωn = B_PLL/0.53 (~18.9 rad/s) and 1.4 ≈ 2ζ. PI form:
        // 2ζωn on the error delta, ωn² integrating the error over one period.
        let w = B_PLL / 0.53;
        self.trk.doppler_hz +=
            1.4 * w * (err_phase - self.trk.err_phase) + w * w * err_phase * self.code_sec;
        self.update_state_doppler_hz();
        self.trk.err_phase = err_phase;
        self.hist.phi_error.push_back(err_phase * 2.0 * PI);
    }

    fn run_dll(&mut self, corr_early: Complex64, corr_late: Complex64) {
        // DLL update cadence in code periods (10 for L1CA's 1 ms, 2 for E1's 4 ms).
        let n = usize::max(1, (T_DLL / self.code_sec) as usize);
        self.trk.sum_corr_e += corr_early.norm();
        self.trk.sum_corr_l += corr_late.norm();
        if self.num_trk_periods.is_multiple_of(n) {
            let e = self.trk.sum_corr_e;
            let l = self.trk.sum_corr_l;
            let disc = (e - l) / (e + l);
            let err_code = disc / 2.0 * self.code_sec / self.code_len as f64;
            let t_u = self.code_sec * n as f64;
            // Loop forensics (GNSS_DLL_DEBUG=1): early/late balance, the loop
            // state and the rate trim, every ~100 ms.
            if self.num_trk_periods.is_multiple_of(10 * n)
                && std::env::var("GNSS_DLL_DEBUG").is_ok()
            {
                log::warn!(
                    "{}: DLLDBG ts={:.2} disc={:+.5} code_off_us={:+.4} trim_ns_s={:+.1} dopp={:+.1} cn0={:.1}",
                    self.sv,
                    self.ts_sec,
                    disc,
                    self.trk.code_off_sec * 1e6,
                    self.trk.code_rate_trim * 1e9,
                    self.trk.doppler_hz,
                    self.trk.cn0
                );
            }
            // Two-gear proportional path. In the linear region the velocity
            // gain (B_DLL/0.25)·disc_gain = 1/τ trims noise around the peak;
            // its slew tops out at ~±980 ns/s (L1CA, |disc| = 1), and the
            // discriminator compresses well before full deflection. A
            // *persistently* hot discriminator means the loop is not
            // filtering noise, it is behind — on the SJTU capture an
            // ~850 ns/s sample-clock vs LO skew consumed ~90% of that
            // authority: the discriminator pegged at ~0.85 (= 850/980) and
            // the remaining deficit walked the prompt off the peak in ~16 s,
            // C/N0 50 -> 30, lock lost, re-acquire, repeat. The pull-in gear
            // instead kicks out half the *estimated offset* per update
            // (τ/t_u being the deadbeat gain), converging in a few updates
            // without overshoot. Persistence (3 consecutive hot updates)
            // keeps it spike-safe: the discriminator throws isolated
            // one-update outliers (|disc| up to ~0.9 at 53 dB-Hz, synth GEO
            // bench) that the linear gain absorbs as a ~9 ns blip but a
            // deadbeat kick would turn into a ~35 ns pseudorange glitch.
            self.trk.disc_saturated_streak = if disc.abs() > 0.5 {
                self.trk.disc_saturated_streak + 1
            } else {
                0
            };
            let pull_in = self.trk.disc_saturated_streak >= 3;
            let gain = if pull_in {
                0.5 * self.tau_dll / t_u
            } else {
                B_DLL / 0.25
            };
            // Rate trim — the loop integrator. A proportional-only loop
            // chasing a steady code-rate error (front-end sample-clock vs LO
            // skew, or code Doppler the carrier aiding misses) holds a
            // permanent offset of rate·τ off the peak; the integrator learns
            // the rate instead, so the tracked phase converges onto the true
            // peak. The gain (B_DLL/4τ, well below critical damping) trades
            // windup speed for trim noise: the trim's random walk feeds
            // straight into the pseudoranges, and at critical damping it
            // costs ~3 m on the synth GEO bench vs ~1 m here (SJTU's 850 ns/s
            // still winds up in ~10 s, bridged by the pull-in gear). The
            // learning input clamps the discriminator to its linear range —
            // beyond it a saturated discriminator measures direction, not
            // rate (and a hard |disc| ≤ 0.3 learning *gate* deadlocks: the
            // kick/linear equilibrium parks right at the pull-in threshold,
            // never entering the gate, and the trim stays unlearned forever).
            // No learning during the FLL second — carrier transients there
            // are code-rate garbage it would faithfully memorize.
            let err_learn = disc.clamp(-0.3, 0.3) / 2.0 * self.code_sec / self.code_len as f64;
            if self.trk_elapsed_sec() > T_FPULLIN {
                self.trk.code_rate_trim -= 0.25 * B_DLL / self.tau_dll * err_learn * t_u;
            }
            self.trk.code_off_sec += self.trk.code_rate_trim * t_u;
            self.trk.code_off_sec -= gain * err_code * t_u;
            self.trk.sum_corr_e = 0.0;
            self.trk.sum_corr_l = 0.0;
        }
    }

    /// Tracking C/N0: prompt power over the noise-reference ("neutral")
    /// correlator power, both averaged over T_CN0, in the coherent bandwidth
    /// 1/T — so 10·log10(P_p/P_n / T) is dB-Hz. The neutral tap sits
    /// NEUTRAL_CORR chips off the peak (signal-free), making P_n the noise
    /// floor at the same integration length. The prompt's own noise is *not*
    /// subtracted (the ratio is (S+N)/N): a fraction-of-a-dB optimistic at
    /// strong signals, ~+3 dB near the lock floor — consistent, and the
    /// 35/29 dB-Hz gates are tuned with that bias in. The 0.5 update is a
    /// one-pole smoother per T_CN0 (1 s), settling in ~2-3 s.
    fn update_cn0(&mut self, corr_prompt: Complex64, corr_noise: Complex64) {
        self.trk.sum_corr_p += corr_prompt.norm_sqr();
        self.trk.sum_corr_noise += corr_noise.norm_sqr();

        if self
            .num_trk_periods
            .is_multiple_of((T_CN0 / self.code_sec) as usize)
        {
            if self.trk.sum_corr_noise > 0.0 {
                let cn0 =
                    10.0 * (self.trk.sum_corr_p / self.trk.sum_corr_noise / self.code_sec).log10();
                self.trk.cn0 += 0.5 * (cn0 - self.trk.cn0);
                self.update_state_cn0();
            }
            self.trk.sum_corr_noise = 0.0;
            self.trk.sum_corr_p = 0.0;
        }
    }
    fn get_code_and_carrier_phase(&mut self) {
        let tau = self.code_sec;
        let fc = self.fi + self.trk.doppler_hz;
        self.trk.accum_doppler_cyc += self.trk.doppler_hz * tau; // accumulated Doppler
        self.trk.code_off_sec -= self.trk.doppler_hz / self.fc * tau; // carrier-aided code offset

        // A code-period wrap shifts the transmit phase (num_tx_codes * code_sec +
        // code_off), which always tracks; num_trk_periods instead tracks
        // corr_p-buffer alignment, so it only moves together with the pop/push.
        // On the very first tracking step corr_p is still empty (the push happens
        // later in tracking_process), so leave the buffer/num_trk_periods alone.
        if self.trk.code_off_sec >= self.code_sec {
            // Positive wrap (receding SV: the received code slipped a whole
            // period later). The transmit phase num_trk_periods·τ − code_off
            // must stay continuous across the −τ jump in code_off, so
            // num_trk_periods steps back — and because it also indexes the
            // prompt history, the just-stored prompt is popped: the next
            // correlation re-correlates that same transmitted code period and
            // replaces it. (The LNAV and SBAS decoders mirror the same
            // bookkeeping on their own windows; no-op for Galileo, whose
            // symbol grid is one code period.)
            self.trk.code_off_sec -= self.code_sec;
            self.num_tx_codes += 1.0;
            if self.hist.corr_p.pop_back().is_some() {
                self.num_trk_periods -= 1;
            }
            self.nav.lnav.wrap_drop();
            self.nav.sbas.wrap_drop();
        } else if self.trk.code_off_sec < 0.0 {
            // Negative wrap (approaching SV: the received code slipped a whole
            // period earlier) — the dual case: one transmitted period falls
            // between successive correlation windows and is never correlated,
            // so the last prompt is duplicated into its slot. This keeps the
            // prompt stream aligned to *transmitted* periods, the grid the
            // 20-period nav bits divide.
            self.trk.code_off_sec += self.code_sec;
            self.num_tx_codes -= 1.0;
            if let Some(&v) = self.hist.corr_p.back() {
                self.hist.corr_p.push_back(v);
                self.num_trk_periods += 1;
            }
            self.nav.lnav.wrap_repeat();
            self.nav.sbas.wrap_repeat();
        }

        // Local-carrier phase (cycles) at the first sample of this period's
        // correlation window; tracking_compute_correlation hands it to
        // doppler_shift so the replica carrier is phase-continuous from period
        // to period — without that, the PLL's atan(Q/I) and the FLL's
        // cross/dot comparisons across periods would be meaningless. Phase is
        // used modulo one cycle, and the IF advance over whole periods is an
        // integer cycle count (every supported IF is a multiple of 1 kHz, so
        // fi·τ ∈ ℤ — the fi·τ term documents it and drops out mod 1). What
        // remains is the Doppler-accumulated phase (accum_doppler_cyc = Σ fd·τ) plus the
        // carrier phase across the window's fractional offset, code_off, at
        // the current carrier fc = fi + fd.
        let code_off = self.trk.code_off_sec * self.fs;
        self.trk.phi = self.fi * tau + self.trk.accum_doppler_cyc + fc * code_off / self.fs;
        self.update_state_phi();

        self.hist.code_phase_offset.push_back(code_off);
        self.update_state_code_idx();
    }

    fn log_periodically(&mut self) {
        let code_idx = self.hist.code_phase_offset.back().unwrap();
        if self.ts_sec - self.hist.last_log_ts > 3.0 {
            log::warn!(
                "{}: {} cn0={:.1} dopp={:5.0} code_idx={:4.0} phi={:5.2} ts_sec={:.3} code_off_sec={:+.3e}",
                self.sv,
                "TRCK".green(),
                self.trk.cn0,
                self.trk.doppler_hz,
                code_idx,
                (self.trk.phi % 1.0) * 2.0 * PI,
                self.ts_sec,
                self.trk.code_off_sec
            );
            self.hist.last_log_ts = self.ts_sec;
        }
    }

    /// Detect and correct a Costas half-symbol-rate carrier false lock.
    ///
    /// For Galileo E1-B one data symbol spans exactly one code period, so the
    /// per-symbol carrier discriminators (`atan(Q/I)` PLL, `atan(cross/dot)` FLL)
    /// are data-ambiguous beyond ±1/(4·code_sec): the loop can settle a multiple
    /// of 1/(2·code_sec) (±125 Hz) off the true frequency, holding a strong,
    /// stable lock while every symbol picks up a (−1)ⁿ flip. The code/DLL loop is
    /// immune (it tracks the true code phase), so the transmit-phase slope gives
    /// the true Doppler: `d(t_tx)/d(t_rx) = 1 + dopp/fc`. We snap the PLL Doppler
    /// onto the nearest half-rate step of it, pulling the carrier onto the true
    /// lock — which also makes the I/NAV symbol stream clean.
    ///
    /// GPS L1 C/A is immune (its PLL updates 20× per data bit, so its
    /// discriminator is not data-ambiguous at the bit rate), so this is gated to
    /// Galileo. The decoder ([`crate::galileo_inav`]) independently undoes the
    /// (−1)ⁿ flip, covering the few seconds before this correction engages.
    fn monitor_code_carrier_consistency(&mut self) {
        let tx_phase = self.trk_elapsed_sec() - self.trk.code_off_sec;
        // Establish the baseline once past carrier pull-in AND the code-loop
        // rate-trim windup: the trim transient bends the early transmit-phase
        // slope, and a baseline taken across it mis-calibrates the healthy
        // divergence — on the synth GEO bench that poisoned baseline (−103 Hz
        // vs a true ~0) got a clean 52 dB-Hz channel dropped as divergent the
        // moment the trim settled.
        if self.trk.tx_phase_ref == 0.0 {
            if self.trk_elapsed_sec() > T_FPULLIN + 4.0 {
                self.trk.tx_phase_ref_ts = self.ts_sec;
                self.trk.tx_phase_ref = tx_phase;
            }
            return;
        }
        let dt = self.ts_sec - self.trk.tx_phase_ref_ts;
        if dt < HALF_RATE_WINDOW {
            return; // need a few seconds for a precise slope
        }
        // The transmit-phase slope gives the code-implied (true) Doppler; its gap
        // from the PLL's carrier Doppler is the code-carrier divergence.
        let code_dopp = ((tx_phase - self.trk.tx_phase_ref) / dt - 1.0) * self.fc;
        let div = code_dopp - self.trk.doppler_hz;

        match self.sv.constellation {
            // Galileo I/NAV: the carrier discriminator is data-ambiguous, so a
            // healthy lock can sit a multiple of 125 Hz off; snap it back.
            Constellation::Galileo => {
                let step = 1.0 / (2.0 * self.code_sec); // 125 Hz for E1-B
                let k = (div / step).round();
                if std::env::var("GAL_DOPP_CHECK").is_ok() {
                    log::warn!(
                        "{}: DOPPCHK pll_dopp={:7.1} code_dopp={:7.1} k={:+.0}",
                        self.sv,
                        self.trk.doppler_hz,
                        code_dopp,
                        k,
                    );
                }
                if k != 0.0 {
                    self.trk.doppler_hz += k * step;
                    self.trk.err_phase = 0.0; // avoid a PLL derivative spike on the jump
                    self.update_state_doppler_hz();
                    log::warn!(
                        "{}: half-rate false lock corrected {:+.0} Hz -> {:.0} Hz at ts={:.1}",
                        self.sv,
                        k * step,
                        self.trk.doppler_hz,
                        self.ts_sec,
                    );
                }
            }
            // GPS L1 C/A (BPSK): no half-rate ambiguity, so a *persistent*
            // code-carrier divergence means the code loop has walked off the true
            // peak (a cross-correlation pull or slow code slip) while the carrier
            // holds lock — its pseudorange then ramps and silently drags the fix
            // (C/N0 can stay just above the loss gate the whole time, as G03 does
            // on the nov3 capture). The healthy divergence is a stable per-lock
            // systematic, so self-calibrate it and drop the channel to re-acquire
            // once it deviates persistently.
            Constellation::GPS => {
                const DIV_GATE_HZ: f64 = 100.0; // ~6x the healthy ±15 Hz scatter
                const DIV_FAULT_WINDOWS: u32 = 2; // ~6 s of sustained divergence
                if self.trk.divergence_ema == 0.0 {
                    self.trk.divergence_ema = div; // first window sets the baseline
                } else if (div - self.trk.divergence_ema).abs() > DIV_GATE_HZ {
                    self.trk.divergence_streak += 1;
                    if self.trk.divergence_streak >= DIV_FAULT_WINDOWS {
                        log::warn!(
                            "{}: code-carrier divergence {:+.0} Hz (healthy {:+.0}) — dropping to re-acquire at ts={:.1}",
                            self.sv,
                            div,
                            self.trk.divergence_ema,
                            self.ts_sec,
                        );
                        // The walk-off may have been steered by a bad learned
                        // code rate; relearn from scratch on the next lock.
                        self.trk.code_rate_trim = 0.0;
                        self.idle_start();
                        return;
                    }
                } else {
                    self.trk.divergence_streak = 0;
                    self.trk.divergence_ema += 0.05 * (div - self.trk.divergence_ema); // slow recal
                }
            }
            _ => {}
        }
        // Re-baseline so the next window measures the post-correction state.
        self.trk.tx_phase_ref_ts = self.ts_sec;
        self.trk.tx_phase_ref = tx_phase;
    }

    /// Elapsed tracking time (s) since lock — one code period per count.
    fn trk_elapsed_sec(&self) -> f64 {
        self.num_trk_periods as f64 * self.code_sec
    }

    fn tracking_process(&mut self, iq_vec: &[Complex32]) {
        self.get_code_and_carrier_phase();
        let (corr_prompt, corr_early, corr_late, corr_noise) =
            self.tracking_compute_correlation(iq_vec);
        self.hist.corr_p.push_back(corr_prompt);
        self.num_trk_periods += 1;
        self.num_tx_codes += 1.0;
        self.stats.trk_periods += 1;
        self.stats.trk_streak += 1;
        self.stats.max_trk_streak = self.stats.max_trk_streak.max(self.stats.trk_streak);
        self.stats.peak_cn0 = self.stats.peak_cn0.max(self.trk.cn0);

        // Integer part of the received-signal transmit-time. The continuous,
        // correctly-signed transmit phase is num_trk_periods*code_sec - code_off:
        //   - num_trk_periods advances at the *received* code rate (it gains an
        //     extra period on a code_off<0 wrap, which is when Doppler is positive
        //     i.e. the SV is approaching), and
        //   - code_off is the *replica* offset, which moves opposite to the
        //     received code phase (carrier aiding does code_off -= doppler/fc),
        //     hence the minus sign.
        // This gives d(t_tx)/d(t_rx) = 1 + doppler/fc = 1 - range_rate/c (correct).
        // The earlier num_tx_codes*code_sec + code_off form had the opposite wrap
        // sign and +code_off, yielding 1 - doppler/fc (Doppler with the wrong sign,
        // so pseudoranges moved opposite to the true range).
        self.nav.meas.trk_phase = self.trk_elapsed_sec();
        self.nav.meas.ts_sec = self.ts_sec;
        // Snapshot the fractional code phase paired with trk_phase from the same
        // period. The solver forms the transmit phase as trk_phase - code_off; the
        // absolute code_off (common cross-SV reference from acquisition) carries the
        // sub-ms range and must NOT be differenced away at the anchor.
        //
        // No Doppler-proportional correction belongs here: the code loop holds
        // no Doppler lag (carrier aiding covers code Doppler, and the rate-trim
        // integrator absorbs any residual rate — trim within ±15 ns/s across
        // gpssim's ±3 kHz spread). The 0.03 m/Hz residual slope a calibrated
        // `doppler/fc·τ` term used to null here (τ ≈ 0.157 s, long misattributed
        // to DLL group delay) was the orbit-epoch error of a t_tx anchored
        // 0.160 s early — fixed at the source (see LNAV_DECODE_LATENCY_SEC).
        // The raw slope measured fs-independent from 2.046 to 12.276 Msps: the
        // signature of an epoch error, not of anything on the sampling grid.
        self.nav.meas.code_off_sec = self.trk.code_off_sec;

        if self.trk_elapsed_sec() < T_FPULLIN {
            self.run_fll();
        } else {
            self.run_pll(corr_prompt);
        }

        self.run_dll(corr_early, corr_late);
        self.update_cn0(corr_prompt, corr_noise);

        if self.trk_elapsed_sec() >= T_NPULLIN {
            self.nav_decode();
        }

        self.hist.cn0.push_back(self.trk.cn0);
        self.hist.doppler_hz.push_back(self.trk.doppler_hz);
        self.hist.trim(if self.diagnostics {
            HISTORY_NUM
        } else {
            HISTORY_MIN
        });
        self.update_all_plots(false);
        self.monitor_code_carrier_consistency();
        self.log_periodically();
        self.nav.meas.cn0 = self.trk.cn0;

        if self.trk.cn0 < CN0_THRESHOLD_LOST {
            self.idle_start();
            return;
        }
        // A real SBAS GEO frames CRC-valid messages within seconds of locking
        // (1 msg/s, continuous); a cross-correlation phantom lock never does —
        // and phantom GEO channels correlating forever were the dominant cost
        // of the default-on SBAS block (+27% tracking periods on CTTC). Drop
        // them, with the fail counter pushed past grace so retries back off.
        if self.sv.constellation == Constellation::SBAS
            && self.trk_elapsed_sec() > SBAS_MSG_TIMEOUT_SEC
            && self.stats.subframes == self.subfr_at_lock
        {
            log::warn!(
                "{}: tracking {:.0} s without a CRC-valid message — false lock, dropping",
                self.sv,
                SBAS_MSG_TIMEOUT_SEC
            );
            self.num_acq_fails = self.num_acq_fails.max(ACQ_FAIL_GRACE + 1);
            self.idle_start();
        }
    }

    /// This channel's spreading-code period (s) — its scheduler family key.
    pub fn code_period_sec(&self) -> f64 {
        self.code_sec
    }

    pub fn process_samples(&mut self, iq_vec: &[Complex32], ts_sec: f64) {
        self.process_samples_cached(iq_vec, ts_sec, None);
    }

    /// [`process_samples`](Self::process_samples) with the receiver's shared
    /// per-block acquisition FFT cache.
    pub fn process_samples_cached(
        &mut self,
        iq_vec: &[Complex32],
        ts_sec: f64,
        cache: Option<&AcqFftCache>,
    ) {
        self.ts_sec = ts_sec;

        match self.state {
            State::Acquisition => self.acquisition_process(iq_vec, cache),
            State::Tracking => self.tracking_process(iq_vec),
            State::Idle => self.idle_process(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn doppler_bin_range_covers_and_clamps() {
        // Full grid: ±12 kHz over 75 bins (320 Hz step).
        let step = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;
        // A centred ±3 kHz window: ~19 bins, containing the centre frequency.
        let (lo, hi) = doppler_bin_range(0.0, SBAS_DOPPLER_SPREAD_HZ);
        assert!(hi - lo < DOPPLER_SPREAD_BINS / 3);
        let f = |i: usize| -DOPPLER_SPREAD_HZ + i as f64 * step;
        assert!(f(lo) <= -3000.0 && 3000.0 <= f(hi - 1));
        // An offset window keeps the requested band covered.
        let (lo, hi) = doppler_bin_range(2700.0, SBAS_DOPPLER_SPREAD_HZ);
        assert!(f(lo) <= -300.0 && 5700.0 <= f(hi - 1));
        // Extreme centres clamp to the grid instead of indexing out of it.
        let (lo, hi) = doppler_bin_range(20_000.0, SBAS_DOPPLER_SPREAD_HZ);
        assert!(lo <= hi && hi <= DOPPLER_SPREAD_BINS);
        let (lo, hi) = doppler_bin_range(-20_000.0, SBAS_DOPPLER_SPREAD_HZ);
        assert!(lo == 0 && hi <= DOPPLER_SPREAD_BINS);
    }

    /// Windowed-sinc FIR low-pass (one-sided cutoff `bw_hz`), circular — a
    /// stand-in for a receiver front-end filter.
    fn lowpass(v: &mut [f64], fs: f64, bw_hz: f64) {
        const TAPS: usize = 101;
        let fc = bw_hz / fs;
        let m = (TAPS - 1) as f64 / 2.0;
        let h: Vec<f64> = (0..TAPS)
            .map(|k| {
                let x = k as f64 - m;
                let sinc = if x == 0.0 {
                    2.0 * fc
                } else {
                    (2.0 * PI * fc * x).sin() / (PI * x)
                };
                let w = 0.54 - 0.46 * (2.0 * PI * k as f64 / (TAPS - 1) as f64).cos();
                sinc * w
            })
            .collect();
        let n = v.len();
        let orig = v.to_vec();
        for (i, out) in v.iter_mut().enumerate() {
            let mut acc = 0.0;
            for (k, hk) in h.iter().enumerate() {
                acc += orig[(i + n + k - TAPS / 2) % n] * hk;
            }
            *out = acc;
        }
    }

    /// The DLL early-late discriminator slope for `sig` at sample rate `fs`,
    /// optionally through a front-end low-pass — computed exactly as the
    /// tracking correlator does it (same ZOH replica upsampling, same
    /// integer-truncated ±pos taps, same (E−L)/(E+L)/2 normalization), with a
    /// clean noiseless signal at a small code offset. Units: normalized
    /// discriminator output per code unit (chip; BOC sub-chip for E1) — the
    /// quantity `DLL_DISC_GAIN_*` calibrates.
    fn discriminator_slope(sig: Signal, fs: f64, bw_hz: Option<f64>) -> f64 {
        let code = sig.spreading_code(1).unwrap();
        let code_len = sig.code_len();
        let code_sec = sig.code_period_sec();
        let code_sp = (fs * code_sec) as usize;
        let replica: Vec<f64> = (0..code_sp)
            .map(|i| code[i * code_len / code_sp] as f64)
            .collect();
        let sp_per_chip = code_sec * fs / code_len as f64;
        let pos = (SP_CORR * sp_per_chip) as usize;

        let disc = |delta_chips: f64| -> f64 {
            let chiprate = code_len as f64 / code_sec;
            let mut s: Vec<f64> = (0..code_sp)
                .map(|i| {
                    let t = i as f64 / fs;
                    let c = ((t * chiprate - delta_chips).rem_euclid(code_len as f64)) as usize;
                    code[c] as f64
                })
                .collect();
            if let Some(bw) = bw_hz {
                lowpass(&mut s, fs, bw);
            }
            let (mut e, mut l) = (0.0f64, 0.0f64);
            for j in 0..code_sp {
                if j + pos < code_sp {
                    e += s[j] * replica[j + pos];
                    l += s[j + pos] * replica[j];
                }
            }
            let (e, l) = (e.abs(), l.abs());
            (e - l) / (e + l) / 2.0
        };

        let h = 0.05;
        (disc(h) - disc(-h)) / (2.0 * h)
    }

    // The shape probe behind the DLL_DISC_GAIN_* constants: measures the
    // discriminator slope for every signal/front-end combination we care
    // about, so the hand-calibrated gains (BPSK 3.18, LimeSDR-BOC 0.256)
    // become explainable — and so a shape-derived gain (the planned online
    // calibration) has a verified reference implementation.
    #[test]
    #[ignore]
    fn dll_discriminator_shape_probe() {
        let cases: Vec<(&str, Signal, f64, Option<f64>)> = vec![
            ("L1CA ideal @2.046M", Signal::L1ca, 2_046_000.0, None),
            ("L1CA ideal @10M", Signal::L1ca, 10_000_000.0, None),
            (
                "L1CA @10M bw 2.5M",
                Signal::L1ca,
                10_000_000.0,
                Some(2_500_000.0),
            ),
            ("E1B ideal @4.092M", Signal::GalileoE1b, 4_092_000.0, None),
            ("E1B ideal @10M", Signal::GalileoE1b, 10_000_000.0, None),
            (
                "E1B @10M bw 4M",
                Signal::GalileoE1b,
                10_000_000.0,
                Some(4_000_000.0),
            ),
            (
                "E1B @10M bw 3M",
                Signal::GalileoE1b,
                10_000_000.0,
                Some(3_000_000.0),
            ),
            (
                "E1B @10M bw 2.5M",
                Signal::GalileoE1b,
                10_000_000.0,
                Some(2_500_000.0),
            ),
            (
                "E1B @10M bw 2M",
                Signal::GalileoE1b,
                10_000_000.0,
                Some(2_000_000.0),
            ),
        ];
        let s_ref = discriminator_slope(Signal::L1ca, 2_046_000.0, None);
        let kappa = DLL_DISC_GAIN_BPSK / s_ref;
        eprintln!("reference: L1CA ideal slope {s_ref:.3} -> kappa {kappa:.3}");
        for (name, sig, fs, bw) in cases {
            let s = discriminator_slope(sig, fs, bw);
            eprintln!(
                "{name:22}  slope {s:7.3}   implied gain {:7.3}   tau {:7.3} s",
                kappa * s,
                dll_tau(kappa * s)
            );
        }
    }
}
