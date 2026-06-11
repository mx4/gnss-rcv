use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use rustfft::FftPlanner;
use rustfft::num_complex::Complex64;
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::Mutex;

const PI: f64 = std::f64::consts::PI;

use crate::code::Signal;
use crate::navigation::Navigation;
use crate::plots::plot_channel;
use crate::state::ChannelState;
use crate::state::GnssState;
use crate::util::calc_correlation;
use crate::util::doppler_shift;
use crate::util::doppler_shifted_carrier;
use crate::util::get_max_with_idx;

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
/// error) that sets the code-loop group delay [`dll_tau`]. It depends on the
/// correlation peak vs the correlator spacing (`SP_CORR`), so it's signal-specific
/// and calibrated to null the per-signal residual Doppler slope. The diagnosis and
/// calibration experiments are written up in `docs/dll-group-delay.md`.
/// - **BPSK** (L1CA / SBAS): ~1-chip-wide triangular peak, well matched by
///   `SP_CORR` → a steep discriminator → small group delay (τ ≈ 0.157 s).
const DLL_DISC_GAIN_BPSK: f64 = 3.18;
/// - **BOC(1,1)** (Galileo E1): the ~½-chip-wide main peak is mismatched by the
///   BPSK-tuned `SP_CORR`, giving a ~12× shallower discriminator → a much larger
///   group delay (τ ≈ 1.95 s). Tightens the LimeSDR Galileo fix 3.4 km → ~110 m.
const DLL_DISC_GAIN_BOC: f64 = 0.256;

/// Code-loop group delay = 1/(loop velocity gain). `run_dll` corrects `code_off`
/// at rate `(B_DLL/0.25)·disc_gain` per unit error, so a code-Doppler ramp leaves a
/// steady lag of (rate)·τ; the tracked code phase therefore lags the true one by
/// code-Doppler·τ, a per-SV transmit-time bias ∝ Doppler we add back (see
/// `code_off_sec` capture). Derived from B_DLL so it tracks any loop retune.
fn dll_tau(disc_gain: f64) -> f64 {
    0.25 / (B_DLL * disc_gain)
}

// Acquisition Doppler search: +/-12 kHz so a large front-end LO offset (some
// captures sit several kHz off L1, e.g. the CTTC recording's SVs at +5..+10 kHz)
// still lands inside the window, on top of the +/-5 kHz of true GPS Doppler. The
// bin count keeps the step at ~320 Hz (2*12000/75) so resolution is unchanged.
const DOPPLER_SPREAD_HZ: f64 = 12000.0;
const DOPPLER_SPREAD_BINS: usize = 75;
const HISTORY_NUM: usize = 20000;
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
    prn_code: Vec<Complex64>, // upsampled
    sig_buf: Vec<Complex64>,  // reused scratch for the Doppler-mixed code period
    doppler_hz: f64,
    code_off_sec: f64,
    cn0: f64,
    adr: f64,
    phi: f64,
    err_phase: f64,
    sum_corr_e: f64,
    sum_corr_l: f64,
    sum_corr_p: f64,
    sum_corr_n: f64,
    txp_ts0: f64, // baseline rx time for the code-implied-Doppler slope (0 = unset)
    txp0: f64,    // baseline transmit phase paired with txp_ts0
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
    pub fn trim(&mut self) {
        if self.doppler_hz.len() > HISTORY_NUM {
            self.doppler_hz.pop_front();
        }
        if self.phi_error.len() > HISTORY_NUM {
            self.phi_error.pop_front();
        }
        if self.corr_p.len() > HISTORY_NUM {
            self.corr_p.pop_front();
        }
        if self.code_phase_offset.len() > HISTORY_NUM {
            self.code_phase_offset.pop_front();
        }
        if self.cn0.len() > HISTORY_NUM {
            self.cn0.pop_front();
        }
    }
}

#[derive(Default)]
pub struct Acquisition {
    prn_code_fft: Vec<Complex64>,
    sum_p: Vec<Vec<f64>>,
    // Pre-computed carrier replica for each Doppler bin. The bins are fixed and
    // PRN-independent (they depend only on fs/fi/code_sp), so the whole set is
    // built once by the receiver ([`Channel::build_carriers`]) and *shared* across
    // every channel — at 50 MHz each set is ~240 MB, so duplicating it per PRN
    // (36×) was ~8.6 GB and OOM'd. Sharing it avoids that and the sin/cos per
    // sample per bin on every acquisition step (the dominant search-time cost).
    carriers: Arc<Vec<Vec<Complex64>>>,
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

pub struct Channel {
    pub pub_state: Arc<Mutex<GnssState>>,
    pub sv: SV,
    pub stats: ChannelStats,
    plots: bool,
    fc: f64, // carrier frequency
    fs: f64, // sampling frequency
    fi: f64, // intermediate frequency

    code_sec: f64,   // code duration in sec
    code_len: usize, // prn code len: e.g. 1023
    code_sp: usize,  // samples per upsampled code: e.g. 2046 for L1CA
    tau_dll: f64,    // code-loop group delay for this signal (transmit-time lag comp)

    fft_planner: FftPlanner<f64>,
    state: State,

    pub ts_sec: f64, // current time
    pub num_trk_samples: usize,
    // Continuous count of transmitted code periods since tracking start. Unlike
    // `num_trk_samples` (whose +/-1 wraps are for correlation-buffer alignment),
    // this advances by +1 per processed code period and is corrected at code-phase
    // wraps with the sign that keeps the *transmit phase* continuous. It is the
    // source for the pseudorange transmit-time and must not be reused for buffering.
    num_tx_codes: f64,
    num_acq_samples: usize,
    num_idl_samples: usize,
    // Consecutive failed acquisitions (reset on a successful lock); drives the
    // idle backoff for PRNs that are very likely not in view.
    num_acq_fails: u32,
    // Peak C/N0 of the most recent acquisition attempt. Kept even when it is
    // below the lock threshold so a search (e.g. for weak SBAS GEOs) can tell
    // "present but weak" from "noise floor"; surfaced in the IDLE log.
    acq_cn0: f64,

    pub hist: History,
    pub nav: Navigation,
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
        self.publish(|cs| cs.cn0 = cn0);
    }

    /// Build the per-Doppler-bin carrier replicas once, to be shared (cloned
    /// `Arc`) across every channel — they depend only on `fs`/`fi`/`code_sp`, not
    /// the PRN. See [`Acquisition::carriers`].
    pub fn build_carriers(fs: f64, fi: f64, code_sp: usize) -> Arc<Vec<Vec<Complex64>>> {
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
        sig: Signal,
        sv: SV,
        fs: f64,
        fi: f64,
        plots: bool,
        pub_state: Arc<Mutex<GnssState>>,
        carriers: Arc<Vec<Vec<Complex64>>>,
    ) -> Self {
        let code_buf = sig.spreading_code(sv.prn).unwrap_or_else(|| {
            panic!(
                "no spreading code for {sig} PRN {} (Galileo E1 codes pending)",
                sv.prn
            )
        });
        let code_sec = sig.code_period_sec();
        let code_len = sig.code_len();
        let code_sp = (fs * code_sec) as usize;
        // Per-signal transmit-time lag compensation: the code (DLL) loop's group
        // delay depends on the correlation peak shape (BOC vs BPSK), see dll_tau.
        let tau_dll = dll_tau(if sig.is_boc11() {
            // The BOC discriminator gain depends on the front-end shaping of
            // the ±half-chip BOC peak: 0.256 was calibrated on the *filtered*
            // ION LimeSDR capture (docs/dll-group-delay.md), while ideal
            // unfiltered BOC (synthetic scenes) has a steep BPSK-like slope —
            // using the filtered value there leaves a Doppler-proportional
            // pseudorange bias (measured 1.5 km -> 4 m on the hermetic E1 fix
            // test). GNSS_DLL_GAIN_BOC overrides the default for such sources.
            std::env::var("GNSS_DLL_GAIN_BOC")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(DLL_DISC_GAIN_BOC)
        } else {
            DLL_DISC_GAIN_BPSK
        });
        let mut fft_planner = FftPlanner::new();

        // Resample the PRN code to the actual samples-per-code-period (code_sp =
        // fs * code_sec), so any sampling rate works. (For fs = 2.046 MHz this is
        // exactly 2 samples/chip, matching the previous hardcoded duplication.)
        let prn_code: Vec<Complex64> = (0..code_sp)
            .map(|i| {
                let chip = i * code_len / code_sp;
                Complex64::new(code_buf[chip] as f64, 0.0)
            })
            .collect();

        let mut prn_code_fft = prn_code.clone();

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
            stats: ChannelStats::default(),
            plots,
            fft_planner,
            ts_sec: 0.0,
            fc: sig.carrier_hz(),
            fs,
            fi,
            code_sec,
            code_len,
            code_sp,
            tau_dll,

            num_acq_samples: 0,
            num_idl_samples: 0,
            num_acq_fails: 0,
            acq_cn0: 0.0,
            num_trk_samples: 0,
            num_tx_codes: 0.0,

            state: State::Acquisition,
            nav: Navigation::new(sv),
            hist: History::default(),
            trk: Tracking {
                prn_code,
                ..Default::default()
            },
            acq: Acquisition {
                prn_code_fft,
                sum_p: vec![vec![0.0; code_sp]; DOPPLER_SPREAD_BINS],
                carriers,
            },
        }
    }

    fn idle_start(&mut self) {
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
        self.num_idl_samples = 0;
        self.num_trk_samples = 0;
        self.num_acq_samples = 0;
    }

    fn idle_process(&mut self) {
        self.num_idl_samples += 1;
        // Normal retry rate during the grace window; afterwards (PRN very likely
        // absent) grow the idle time linearly, capped, so the FFT search runs far
        // less often. A successful lock resets num_acq_fails, so a visible SV that
        // locks during grace is never throttled.
        let idle = if self.num_acq_fails <= ACQ_FAIL_GRACE {
            T_IDLE
        } else {
            (T_IDLE * (self.num_acq_fails - ACQ_FAIL_GRACE + 1) as f64).min(T_IDLE_MAX)
        };
        if self.num_idl_samples as f64 * self.code_sec > idle {
            self.acquisition_start();
        }
    }

    fn acquisition_init(&mut self) {
        self.acq.sum_p = vec![vec![0.0; self.code_sp]; DOPPLER_SPREAD_BINS];
        self.num_acq_samples = 0;
        self.num_idl_samples = 0;
        self.num_trk_samples = 0;
    }

    fn acquisition_start(&mut self) {
        self.acquisition_init();
        self.set_state(State::Acquisition);
    }

    fn tracking_init(&mut self) {
        self.trk.doppler_hz = 0.0;
        self.trk.cn0 = 0.0;
        self.trk.adr = 0.0;
        self.trk.code_off_sec = 0.0;
        self.trk.err_phase = 0.0;
        self.trk.sum_corr_p = 0.0;
        self.trk.sum_corr_e = 0.0;
        self.trk.sum_corr_l = 0.0;
        self.trk.sum_corr_n = 0.0;
        self.trk.txp_ts0 = 0.0;
        self.trk.txp0 = 0.0;
        self.num_trk_samples = 0;
        self.num_acq_samples = 0;
        self.num_idl_samples = 0;
        self.num_tx_codes = 0.0;
        self.nav.eph.tx_anchored = false;
        self.nav.eph.tow_trk_phase = 0.0;
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

    fn acquisition_integrate_correlation(
        &mut self,
        iq_vec_slice: &[Complex64],
        bin: usize,
    ) -> Vec<f64> {
        let mut iq_vec = iq_vec_slice.to_vec();

        assert_eq!(iq_vec.len(), self.acq.prn_code_fft.len());
        self.stats.acq_corrs += 1;

        // Apply the pre-computed carrier for this Doppler bin (was a per-sample
        // sin/cos via doppler_shift on every call).
        let carrier = &self.acq.carriers[bin];
        for (s, c) in iq_vec.iter_mut().zip(carrier.iter()) {
            *s *= *c;
        }

        let corr = calc_correlation(&mut self.fft_planner, &iq_vec, &self.acq.prn_code_fft);
        let corr_vec: Vec<_> = corr.iter().map(|v| v.norm_sqr()).collect();

        corr_vec
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
        if let Ok(mut st) = self.pub_state.lock() {
            st.histories.insert(self.sv, self.hist.clone());
        }
    }

    fn acquisition_process(&mut self, iq_vec: &[Complex64]) {
        // The feed hands us two code periods; correlate over the most recent one.
        let iq_vec_slice = &iq_vec[self.code_sp..];
        let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;

        for i in 0..DOPPLER_SPREAD_BINS {
            let c_non_coherent = self.acquisition_integrate_correlation(iq_vec_slice, i);
            assert_eq!(c_non_coherent.len(), self.code_sp);

            #[allow(clippy::needless_range_loop)]
            for j in 0..self.code_sp {
                self.acq.sum_p[i][j] += c_non_coherent[j];
            }
        }

        self.num_acq_samples += 1;

        if self.num_acq_samples as f64 * self.code_sec >= T_ACQ {
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
            for i in 0..DOPPLER_SPREAD_BINS {
                let (j_peak, v_peak) = get_max_with_idx(&self.acq.sum_p[i]);
                if v_peak > p_peak {
                    idx = i;
                    p_peak = v_peak;
                    code_offset_idx = j_peak;
                }
                p_total += self.acq.sum_p[i].iter().sum::<f64>();
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
            let p_avg = p_total / self.acq.sum_p[idx].len() as f64 / DOPPLER_SPREAD_BINS as f64;
            let cn0 = 10.0 * ((p_peak - p_avg) / p_avg / self.code_sec).log10();
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
        iq_vec2: &[Complex64],
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
        self.trk.sig_buf.clear();
        self.trk.sig_buf.extend_from_slice(&iq_vec2[lo_u..hi_u]);
        doppler_shift(&mut self.trk.sig_buf, fc, phi, fs);

        // Samples per code-unit (chip; BOC sub-chip for E1) at this sample rate.
        let sp_per_chip = self.code_sec * self.fs / self.code_len as f64;
        let pos = (SP_CORR * sp_per_chip) as usize;
        let pos_neutral = (NEUTRAL_CORR * sp_per_chip) as usize;

        let sig = &self.trk.sig_buf;
        let code = &self.trk.prn_code;
        let len = sig.len();

        let mut corr_prompt = Complex64::default();
        let mut corr_early = Complex64::default();
        let mut corr_late = Complex64::default();
        let mut corr_neutral = Complex64::default();

        // Single fused pass: prompt (full), early/late (offset by +/-pos) and
        // neutral (offset by pos_neutral) accumulated together, reading sig[j]
        // and code[j] once each instead of in four separate passes.
        for j in 0..len {
            let sj = sig[j];
            let cj = code[j];
            corr_prompt += sj * cj;
            if j + pos < len {
                corr_early += sj * code[j + pos];
                corr_late += sig[j + pos] * cj;
            }
            if j + pos_neutral < len {
                corr_neutral += sj * code[j + pos_neutral];
            }
        }

        corr_prompt /= len as f64;
        corr_early /= (len - pos) as f64;
        corr_late /= (len - pos) as f64;
        corr_neutral /= (len - pos_neutral) as f64;

        (corr_prompt, corr_early, corr_late, corr_neutral)
    }

    fn run_fll(&mut self) {
        if self.num_trk_samples < 2 {
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

        let b = if self.num_trk_samples as f64 * self.code_sec < T_FPULLIN / 2.0 {
            B_FLL_WIDE
        } else {
            B_FLL_NARROW
        };
        let err_freq = (cross / dot).atan() / 2.0 / PI;

        // First-order loop: noise bandwidth Bn = G/4, so the gain is 4·Bn.
        self.trk.doppler_hz -= b / 0.25 * err_freq;
        self.update_state_doppler_hz();
    }

    fn run_pll(&mut self, c_p: Complex64) {
        if c_p.re == 0.0 {
            return;
        }
        let err_phase = (c_p.im / c_p.re).atan() / 2.0 / PI;
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

    fn run_dll(&mut self, c_e: Complex64, c_l: Complex64) {
        // DLL update cadence in code periods (10 for L1CA's 1 ms, 2 for E1's 4 ms).
        let n = usize::max(1, (T_DLL / self.code_sec) as usize);
        self.trk.sum_corr_e += c_e.norm();
        self.trk.sum_corr_l += c_l.norm();
        if self.num_trk_samples.is_multiple_of(n) {
            let e = self.trk.sum_corr_e;
            let l = self.trk.sum_corr_l;
            let err_code = (e - l) / (e + l) / 2.0 * self.code_sec / self.code_len as f64;
            // First-order loop, gain 4·Bn — this rate is the loop velocity gain
            // whose inverse sets the DLL group delay (see dll_tau).
            self.trk.code_off_sec -= B_DLL / 0.25 * err_code * self.code_sec * n as f64;
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
    fn update_cn0(&mut self, c_p: Complex64, c_n: Complex64) {
        self.trk.sum_corr_p += c_p.norm_sqr();
        self.trk.sum_corr_n += c_n.norm_sqr();

        if self
            .num_trk_samples
            .is_multiple_of((T_CN0 / self.code_sec) as usize)
        {
            if self.trk.sum_corr_n > 0.0 {
                let cn0 =
                    10.0 * (self.trk.sum_corr_p / self.trk.sum_corr_n / self.code_sec).log10();
                self.trk.cn0 += 0.5 * (cn0 - self.trk.cn0);
                self.update_state_cn0();
            }
            self.trk.sum_corr_n = 0.0;
            self.trk.sum_corr_p = 0.0;
        }
    }
    fn get_code_and_carrier_phase(&mut self) {
        let tau = self.code_sec;
        let fc = self.fi + self.trk.doppler_hz;
        self.trk.adr += self.trk.doppler_hz * tau; // accumulated Doppler
        self.trk.code_off_sec -= self.trk.doppler_hz / self.fc * tau; // carrier-aided code offset

        // A code-period wrap shifts the transmit phase (num_tx_codes * code_sec +
        // code_off), which always tracks; num_trk_samples instead tracks
        // corr_p-buffer alignment, so it only moves together with the pop/push.
        // On the very first tracking step corr_p is still empty (the push happens
        // later in tracking_process), so leave the buffer/num_trk_samples alone.
        if self.trk.code_off_sec >= self.code_sec {
            // Positive wrap (receding SV: the received code slipped a whole
            // period later). The transmit phase num_trk_samples·τ − code_off
            // must stay continuous across the −τ jump in code_off, so
            // num_trk_samples steps back — and because it also indexes the
            // prompt history, the just-stored prompt is popped: the next
            // correlation re-correlates that same transmitted code period and
            // replaces it. (The LNAV decoder mirrors the same bookkeeping on
            // its own window; no-op for non-GPS/QZSS signals.)
            self.trk.code_off_sec -= self.code_sec;
            self.num_tx_codes += 1.0;
            if self.hist.corr_p.pop_back().is_some() {
                self.num_trk_samples -= 1;
            }
            self.nav.lnav.wrap_drop();
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
                self.num_trk_samples += 1;
            }
            self.nav.lnav.wrap_repeat();
        }

        // Local-carrier phase (cycles) at the first sample of this period's
        // correlation window; tracking_compute_correlation hands it to
        // doppler_shift so the replica carrier is phase-continuous from period
        // to period — without that, the PLL's atan(Q/I) and the FLL's
        // cross/dot comparisons across periods would be meaningless. Phase is
        // used modulo one cycle, and the IF advance over whole periods is an
        // integer cycle count (every supported IF is a multiple of 1 kHz, so
        // fi·τ ∈ ℤ — the fi·τ term documents it and drops out mod 1). What
        // remains is the Doppler-accumulated phase (adr = Σ fd·τ) plus the
        // carrier phase across the window's fractional offset, code_off, at
        // the current carrier fc = fi + fd.
        let code_off = self.trk.code_off_sec * self.fs;
        self.trk.phi = self.fi * tau + self.trk.adr + fc * code_off / self.fs;
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
    fn correct_half_rate_false_lock(&mut self) {
        if self.sv.constellation != Constellation::Galileo {
            return;
        }
        let txp = self.num_trk_samples as f64 * self.code_sec - self.trk.code_off_sec;
        // Establish the baseline once past carrier pull-in.
        if self.trk.txp0 == 0.0 {
            if self.num_trk_samples as f64 * self.code_sec > T_FPULLIN + 1.0 {
                self.trk.txp_ts0 = self.ts_sec;
                self.trk.txp0 = txp;
            }
            return;
        }
        let dt = self.ts_sec - self.trk.txp_ts0;
        if dt < HALF_RATE_WINDOW {
            return; // need a few seconds for a precise slope
        }
        // Code-implied (true) Doppler vs the PLL's, expressed in half-rate steps.
        let code_dopp = ((txp - self.trk.txp0) / dt - 1.0) * self.fc;
        let step = 1.0 / (2.0 * self.code_sec); // 125 Hz for E1-B
        let k = ((code_dopp - self.trk.doppler_hz) / step).round();
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
        // Re-baseline so the next window measures the post-correction state.
        self.trk.txp_ts0 = self.ts_sec;
        self.trk.txp0 = txp;
    }

    fn tracking_process(&mut self, iq_vec: &[Complex64]) {
        self.get_code_and_carrier_phase();
        let (c_p, c_e, c_l, c_n) = self.tracking_compute_correlation(iq_vec);
        self.hist.corr_p.push_back(c_p);
        self.num_trk_samples += 1;
        self.num_tx_codes += 1.0;
        self.stats.trk_periods += 1;
        self.stats.trk_streak += 1;
        self.stats.max_trk_streak = self.stats.max_trk_streak.max(self.stats.trk_streak);
        self.stats.peak_cn0 = self.stats.peak_cn0.max(self.trk.cn0);

        // Integer part of the received-signal transmit-time. The continuous,
        // correctly-signed transmit phase is num_trk_samples*code_sec - code_off:
        //   - num_trk_samples advances at the *received* code rate (it gains an
        //     extra period on a code_off<0 wrap, which is when Doppler is positive
        //     i.e. the SV is approaching), and
        //   - code_off is the *replica* offset, which moves opposite to the
        //     received code phase (carrier aiding does code_off -= doppler/fc),
        //     hence the minus sign.
        // This gives d(t_tx)/d(t_rx) = 1 + doppler/fc = 1 - range_rate/c (correct).
        // The earlier num_tx_codes*code_sec + code_off form had the opposite wrap
        // sign and +code_off, yielding 1 - doppler/fc (Doppler with the wrong sign,
        // so pseudoranges moved opposite to the true range).
        self.nav.eph.trk_phase = self.num_trk_samples as f64 * self.code_sec;
        // Snapshot the fractional code phase paired with trk_phase from the same
        // period. The solver forms the transmit phase as trk_phase - code_off; the
        // absolute code_off (common cross-SV reference from acquisition) carries the
        // sub-ms range and must NOT be differenced away at the anchor.
        //
        // The code (DLL) loop has a finite group delay (self.tau_dll), so the
        // tracked code phase lags the true one by code-Doppler · τ. Uncompensated
        // this is a per-SV transmit-time bias ∝ Doppler (gpssim residual slope
        // ~-0.03 m/Hz, ~170 m; far larger for E1's BOC). Add it back.
        let dll_lag = self.trk.doppler_hz / self.fc * self.tau_dll;
        self.nav.eph.code_off_sec = self.trk.code_off_sec + dll_lag;

        if self.num_trk_samples as f64 * self.code_sec < T_FPULLIN {
            self.run_fll();
        } else {
            self.run_pll(c_p);
        }

        self.run_dll(c_e, c_l);
        self.update_cn0(c_p, c_n);

        if self.num_trk_samples as f64 * self.code_sec >= T_NPULLIN {
            self.nav_decode();
        }

        self.hist.cn0.push_back(self.trk.cn0);
        self.hist.doppler_hz.push_back(self.trk.doppler_hz);
        self.hist.trim();
        self.update_all_plots(false);
        self.correct_half_rate_false_lock();
        self.log_periodically();
        self.nav.eph.cn0 = self.trk.cn0;

        if self.trk.cn0 < CN0_THRESHOLD_LOST {
            self.idle_start();
        }
    }

    pub fn process_samples(&mut self, iq_vec: &[Complex64], ts_sec: f64) {
        self.ts_sec = ts_sec;

        match self.state {
            State::Acquisition => self.acquisition_process(iq_vec),
            State::Tracking => self.tracking_process(iq_vec),
            State::Idle => self.idle_process(),
        }
    }
}
