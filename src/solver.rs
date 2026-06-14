use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AbsoluteTime, Almanac, BiasRuntime, Candidate, Carrier, ClockProfile, Config, Duration,
    EARTH_J2000, EnvironmentalBias, Ephemeris, EphemerisSource, Epoch, Frame, Method, Observation,
    Orbit, OrbitSource, Rc, SatelliteClockCorrection, Solver, SpacebornBias, UserParameters,
    UserProfile, Vector3,
};
use map_3d::{Ellipsoid, ecef2geodetic};
use std::sync::{Arc, Mutex};

use crate::{
    constants::{EARTH_ROTATION_RATE, SPEED_OF_LIGHT},
    ephemeris::{Ephemeris as RxEphemeris, Measurement},
    models::{
        compute_sv_position_ecef, elevation_azimuth, klobuchar_l1_delay_m, saastamoinen_tropo_m,
        sv_clock_correction,
    },
    sbas_corr::SbasCorrections,
    sbas_iono::SbasIonoGrid,
    state::GnssState,
    wls::wls_solve,
};
use gnss_rs::constellation::Constellation;

/// The ephemerides of the epoch being solved, shared between `PositionSolver`
/// (which writes them at the top of `compute_position`) and gnss-rtk's callback
/// objects below (which look up per-SV data during `solver.ppp()`). The
/// callbacks are constructed once with no per-call context, so this shared
/// handle is the channel between them — instance-scoped, so multiple solvers
/// (e.g. per-constellation) cannot see each other's data.
type SharedEphemerides = Arc<Mutex<Vec<RxEphemeris>>>;

struct NullEph;

impl EphemerisSource for NullEph {
    fn ephemeris_data(&self, _: Epoch, _: SV) -> Option<Ephemeris> {
        None
    }
}

struct ReceiverOrbitSource {
    ephs: SharedEphemerides,
}

impl OrbitSource for ReceiverOrbitSource {
    fn state_at(&self, epoch: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        let ephs = self.ephs.lock().unwrap();
        let eph = ephs.iter().find(|e| e.sv == sv)?;
        let (x, y, z) = compute_sv_position_ecef(eph, epoch);
        Some(Orbit::from_position(
            x / 1000.0,
            y / 1000.0,
            z / 1000.0,
            epoch,
            frame,
        ))
    }
}

struct ReceiverSpacebornBias {
    ephs: SharedEphemerides,
}

impl SpacebornBias for ReceiverSpacebornBias {
    fn clock_bias(&self, rtm: &BiasRuntime) -> SatelliteClockCorrection {
        let ephs = self.ephs.lock().unwrap();
        let Some(eph) = ephs.iter().find(|e| e.sv == rtm.sv) else {
            return SatelliteClockCorrection::default();
        };
        let corr = sv_clock_correction(eph, rtm.epoch);
        SatelliteClockCorrection::with_relativistic_correction(Duration::from_seconds(corr))
    }

    fn group_delay(&self, rtm: &BiasRuntime) -> Duration {
        let ephs = self.ephs.lock().unwrap();
        ephs.iter()
            .find(|e| e.sv == rtm.sv)
            .map(|eph| Duration::from_seconds(eph.tgd))
            .unwrap_or(Duration::ZERO)
    }

    fn mw_bias(&self, _: &BiasRuntime) -> f64 {
        0.0
    }
}

struct ReceiverEnvironmentalBias;

impl EnvironmentalBias for ReceiverEnvironmentalBias {
    fn troposphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }

    fn ionosphere_bias_m(&self, _: &BiasRuntime) -> f64 {
        0.0
    }
}

struct ReceiverTime;

impl AbsoluteTime for ReceiverTime {
    fn new_epoch(&mut self, _: Epoch) {}

    fn epoch_correction(&self, epoch: Epoch, timescale: gnss_rtk::prelude::TimeScale) -> Epoch {
        epoch.to_time_scale(timescale)
    }
}

type SolverInstance = Solver<
    NullEph,
    ReceiverOrbitSource,
    ReceiverEnvironmentalBias,
    ReceiverSpacebornBias,
    ReceiverTime,
>;

/// One satellite's fully corrected measurement for a solve epoch — the
/// signal-agnostic solver input (a multi-signal session feeds one flat list
/// regardless of which code period produced each entry).
pub(crate) struct SvMeasurement {
    pub sv: SV,
    pub cn0: f64,
    /// Corrected pseudorange (m): raw − iono − tropo + SBAS corrections.
    pub pr_m: f64,
    /// SV clock correction × c (m).
    pub clk_m: f64,
    /// SV ECEF position at transmit time, rotated into the reception-instant
    /// frame (Sagnac).
    pub svp: [f64; 3],
    pub galileo: bool,
    /// Broadcast residual-error variance prior (SBAS UDRE + slant GIVE), m².
    pub var_m2: f64,
}

pub struct PositionSolver {
    solver: SolverInstance,
    /// This solver's epoch store, shared with its callback objects (see
    /// [`SharedEphemerides`]); refreshed at the top of `compute_position`.
    ephs: SharedEphemerides,
    // Last fix, used only as the receiver position for the ionosphere pierce
    // point; the solver itself needs no a-priori (it bootstraps via Bancroft).
    last_fix_ecef: Option<Vector3<f64>>,
    pub_state: Arc<Mutex<GnssState>>,
}

fn make_config() -> Config {
    let mut cfg = Config::default().with_navigation_method(Method::SPP);
    cfg.min_sv_elev = Some(0.0);
    // gnss-rtk's default max_gdop (5.0) targets ultra-precise use and rejects
    // perfectly valid marginal-geometry fixes -- e.g. the CTTC capture, whose
    // sparse SV set solves around HDOP 4.4 (GDOP ~7-9). Relax it so a usable
    // fix from a sparse/clustered SV set isn't thrown away. Good-geometry
    // recordings (gpssim, nov3) sit well under this and are unaffected.
    cfg.solver.max_gdop = 30.0;
    // Our pseudorange t_tx is anchored to the nav-message TOW, which is the
    // satellite's OWN clock reading. So pr_m = geom + c*dT_rx - c*clock_corr,
    // and gnss-rtk must ADD clock_corr back (sv_clock_bias = true, default).
    // clock_corr is supplied via ReceiverSpacebornBias::clock_bias with the
    // relativistic term already folded in (with_relativistic_correction), so we
    // disable gnss-rtk's own relativistic clock model to avoid double-counting.
    // TGD is likewise supplied via group_delay (sv_total_group_delay = true).
    cfg.modeling.relativistic_clock_bias = false;
    // Both troposphere and ionosphere are applied directly to the pseudorange
    // (saastamoinen_tropo_m / klobuchar_l1_delay_m in compute_position) so
    // gnss-rtk's own models are disabled to avoid double-counting.
    cfg.modeling.tropospheric_bias = false;
    cfg.modeling.ionospheric_bias = false;
    cfg
}

// Build a solver with no a-priori position: it bootstraps the first epoch with
// Bancroft (closed-form) from the pseudoranges alone, then the Kalman filter
// carries the state forward. Works anywhere on Earth without a starting guess.
// `ephs` is the per-instance store its callback objects will read.
fn make_solver(
    almanac: &Almanac,
    earth_frame: Frame,
    cfg: &Config,
    ephs: &SharedEphemerides,
) -> SolverInstance {
    let eph = Rc::new(NullEph);
    let orb = Rc::new(ReceiverOrbitSource { ephs: ephs.clone() });
    let sb = Rc::new(ReceiverSpacebornBias { ephs: ephs.clone() });
    let eb = Rc::new(ReceiverEnvironmentalBias);
    let tim = ReceiverTime;
    Solver::new_survey(
        almanac.clone(),
        earth_frame,
        cfg.clone(),
        eph,
        orb,
        sb,
        eb,
        tim,
    )
}

/// Per-epoch correction inputs shared across every SV in one solve — gathered
/// once at the top of [`PositionSolver::compute_position`] and handed to
/// [`PositionSolver::build_sv_measurement`].
struct EpochContext {
    now_gpst: Epoch,
    /// Latest snapshot instant across families; each pseudorange is referenced
    /// to it to strip the family-grid offset (see `compute_position`).
    freshest: f64,
    ts_sec: f64,
    gps_sod: f64,
    iono_valid: bool,
    iono_alpha: [f64; 4],
    iono_beta: [f64; 4],
    sbas_iono: Option<SbasIonoGrid>,
    sbas_corr: Option<SbasCorrections>,
    truth_ecef: Option<Vector3<f64>>,
}

/// Mixed-pool week rebase: the LNAV week is 10 bits, pinned +2048 into the
/// modern era at decode — right for post-2019 captures, 1024 weeks late for
/// older ones (ion-lime 2017 -> "2037"). Galileo's 12-bit GST week is
/// unambiguous, so when both constellations are present, snap each GPS-family
/// SV's timeline onto Galileo's by the nearest 1024-week multiple (epochs and
/// anchor shift together, so orbit/clock evaluation and phase pairing are
/// untouched). Without this the pool spans 19.6 years and the solve is garbage.
/// No-op when the pool has no Galileo SV.
fn rebase_gps_weeks_onto_galileo(snaps: &mut [(Measurement, RxEphemeris)]) {
    let Some(gal_ref) = snaps
        .iter()
        .find(|(_, e)| e.sv.constellation == Constellation::Galileo)
        .map(|(_, e)| e.tow_gpst)
    else {
        return;
    };
    const WEEK: f64 = crate::constants::SECONDS_PER_WEEK as f64;
    for (m, e) in snaps.iter_mut() {
        if e.sv.constellation == Constellation::Galileo {
            continue;
        }
        let dw = ((e.tow_gpst - gal_ref).to_seconds() / WEEK / 1024.0).round() * 1024.0;
        if dw != 0.0 {
            let shift = Duration::from_seconds(dw * WEEK);
            log::warn!(
                "{}: rebasing GPS week by {:+.0} weeks onto the Galileo timeline                          (LNAV 10-bit week rollover)",
                e.sv,
                -dw
            );
            e.tow_gpst -= shift;
            e.toe_gpst -= shift;
            e.toc_gpst -= shift;
            m.tx_tow_gpst -= shift;
        }
    }
}

/// Each SV's transmit epoch: the message TOW anchor plus the transmit-phase time
/// elapsed since it. `trk_phase - code_off` is the continuous transmit phase
/// (channel.rs: code_off is the replica offset, opposite in sign to the received
/// code phase).
fn transmit_epochs(snaps: &[(Measurement, RxEphemeris)]) -> Vec<Epoch> {
    snaps
        .iter()
        .map(|(mm, eph)| {
            let phase = mm.trk_phase - mm.code_off_sec;
            let elapsed = if mm.tx_anchored {
                phase - mm.tow_trk_phase
            } else {
                (mm.trk_phase - mm.tow_trk_phase) - mm.code_off_sec
            };
            let tow = if mm.tx_anchored {
                mm.tx_tow_gpst
            } else {
                eph.tow_gpst
            };
            tow + Duration::from_seconds(elapsed)
        })
        .collect()
}

impl PositionSolver {
    #[allow(clippy::new_without_default)]
    pub fn new(pub_state: Arc<Mutex<GnssState>>) -> Self {
        let cfg = make_config();
        let almanac = Almanac::until_2035().expect("Almanac");
        let earth_frame = almanac.frame_from_uid(EARTH_J2000).expect("earth frame");
        let ephs: SharedEphemerides = Arc::new(Mutex::new(Vec::new()));
        let solver = make_solver(&almanac, earth_frame, &cfg, &ephs);

        Self {
            solver,
            ephs,
            last_fix_ecef: None,
            pub_state,
        }
    }

    pub fn has_fix(&self) -> bool {
        self.last_fix_ecef.is_some()
    }

    /// The last computed fix as (latitude°, longitude°, altitude m), if any.
    pub fn last_fix_geodetic(&self) -> Option<(f64, f64, f64)> {
        self.last_fix_ecef.map(|e| {
            let (lat, lon, h) = ecef2geodetic(e[0], e[1], e[2], Ellipsoid::WGS84);
            (lat.to_degrees(), lon.to_degrees(), h)
        })
    }

    /// Returns true if a position was resolved this call.
    pub fn compute_position(&mut self, ts_sec: f64, snaps: &[(Measurement, RxEphemeris)]) -> bool {
        // Snap GPS's ambiguous 10-bit week onto Galileo's timeline when both are
        // present, so a mixed pool doesn't span ~19.6 years (no-op otherwise).
        let mut snaps: Vec<(Measurement, RxEphemeris)> = snaps.to_vec();
        rebase_gps_weeks_onto_galileo(&mut snaps);
        let snaps = &snaps[..];

        // Refresh this solver's epoch store; the gnss-rtk callback objects
        // read it if the fallback path runs.
        *self.ephs.lock().unwrap() = snaps.iter().map(|(_, e)| *e).collect();

        let tx_gpst = transmit_epochs(snaps);

        // Reception epoch. The receiver has no absolute clock, so we reconstruct
        // "now" from the transmit times: the latest (highest-elevation, closest)
        // SV plus the nominal one-way signal travel time. This 70 ms is
        // load-bearing, not a free offset the receiver-clock bias absorbs — it
        // pins each pseudorange (now_gpst - t_tx) to the physical ~20 000 km
        // range, and gnss-rtk uses that magnitude as the travel time for the
        // Earth-rotation (Sagnac) correction. Zeroing it under-rotates the ECEF
        // frame and biases the fix ~22 m in longitude. The true travel time is
        // ~67-86 ms (varies with elevation); the few-ms error from a single
        // nominal is ~1 m, negligible against the receiver's accuracy.
        const NOMINAL_TRAVEL_SEC: f64 = 0.070;
        let latest_tx = *tx_gpst.iter().max().unwrap();
        let now_gpst = latest_tx + Duration::from_seconds(NOMINAL_TRAVEL_SEC);

        let (iono_valid, iono_alpha, iono_beta, sbas_iono, sbas_corr) = {
            let st = self.pub_state.lock().unwrap();
            let grid = (!st.sbas_iono.is_empty()).then(|| st.sbas_iono.clone());
            let corr = (st.sbas_corr.fast_len() + st.sbas_corr.long_len() > 0)
                .then(|| st.sbas_corr.clone());
            (st.ion_adj, st.iono_alpha, st.iono_beta, grid, corr)
        };
        let gps_sod = {
            let r = &snaps[0].1;
            (r.tow as f64 + (now_gpst - r.tow_gpst).to_seconds()).rem_euclid(86400.0)
        };

        let truth_ecef: Option<Vector3<f64>> =
            std::env::var("GNSS_TRUTH_ECEF").ok().and_then(|s| {
                let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                (v.len() == 3).then(|| Vector3::new(v[0], v[1], v[2]))
            });

        // Mixed-family staleness: each measurement's t_tx belongs to its own
        // snapshot instant, but families snapshot on different grids (C/A
        // every 1 ms, E1 every 4 ms). Referencing each pseudorange to the
        // freshest snapshot removes the family-common offset that would
        // otherwise land in the ISB state (3 ms — the grid gap — measured).
        let freshest = snaps
            .iter()
            .map(|(m, _)| m.ts_sec)
            .fold(f64::NEG_INFINITY, f64::max);

        let ctx = EpochContext {
            now_gpst,
            freshest,
            ts_sec,
            gps_sod,
            iono_valid,
            iono_alpha,
            iono_beta,
            sbas_iono,
            sbas_corr,
            truth_ecef,
        };

        log::warn!("----- now_gpst={now_gpst:?}");
        let mut meas: Vec<SvMeasurement> = Vec::with_capacity(snaps.len());
        for ((mm, eph), t_tx) in snaps.iter().zip(tx_gpst.iter()) {
            if let Some(m) = self.build_sv_measurement(mm, eph, *t_tx, &ctx) {
                meas.push(m);
            }
        }

        // The weighted+ISB WLS is the live solver (GNSS_SOLVER=rtk selects
        // gnss-rtk instead; it also remains the fallback if WLS fails).
        // Rationale + revert condition in `wls_solve`'s doc.
        if !std::env::var("GNSS_SOLVER").is_ok_and(|v| v == "rtk") {
            if self.try_wls(&meas) {
                return true;
            }
            log::warn!("WLS solve failed or rejected; trying gnss-rtk");
        }
        self.solve_rtk(&meas, now_gpst)
    }

    /// Weighted-least-squares solve (the live path). Returns true iff it
    /// published a fix; a failed solve or an off-Earth result returns false
    /// (logging the latter) so the caller falls through to gnss-rtk.
    fn try_wls(&mut self, meas: &[SvMeasurement]) -> bool {
        let x0 = self.last_fix_ecef.map_or([0.0; 3], |p| [p[0], p[1], p[2]]);
        let m: Vec<f64> = meas.iter().map(|m| m.pr_m + m.clk_m).collect();
        let svp: Vec<[f64; 3]> = meas.iter().map(|m| m.svp).collect();
        let gal: Vec<bool> = meas.iter().map(|m| m.galileo).collect();
        let var: Vec<f64> = meas.iter().map(|m| m.var_m2).collect();
        let Some(sol) = wls_solve(&m, &svp, &gal, &var, x0) else {
            return false;
        };
        let pos = Vector3::new(sol.pos[0], sol.pos[1], sol.pos[2]);
        // A 4-SV pool has no redundancy: Gauss-Newton seeded from the
        // geocentre can converge to the mirror root of the two-solution
        // GNSS ambiguity, and the self-calibrated sigmas cannot see it
        // (4 measurements, 4 unknowns, residuals ~0). A terrestrial
        // receiver sits near the geoid — reject anything else and let
        // gnss-rtk (Bancroft-initialised) arbitrate.
        let r = pos.norm();
        if !(6.25e6..6.5e6).contains(&r) {
            log::warn!(
                "WLS solution off-Earth (|pos|={:.0} km); trying gnss-rtk",
                r / 1000.0
            );
            return false;
        }
        self.last_fix_ecef = Some(pos);
        let (lat_rad, lon_rad, height_m) =
            ecef2geodetic(sol.pos[0], sol.pos[1], sol.pos[2], Ellipsoid::WGS84);
        let (lat, lon) = (lat_rad.to_degrees(), lon_rad.to_degrees());
        {
            let mut st = self.pub_state.lock().unwrap();
            st.latitude = lat;
            st.longitude = lon;
            st.height = height_m;
            st.hdop = sol.hdop;
            st.vdop = sol.vdop;
            st.fix_sv_count = meas.len();
        }
        let mut sig: Vec<String> = sol
            .sigma
            .iter()
            .map(|(c, s)| {
                let label = match c {
                    Constellation::Galileo => "gal",
                    _ => "gps",
                };
                format!("σ{label}={s:.1}m")
            })
            .collect();
        sig.sort();
        let sig = sig.join(" ");
        let isb = if sol.isb_m != 0.0 {
            format!(" isb={:+.1}ns", sol.isb_m / SPEED_OF_LIGHT * 1e9)
        } else {
            String::new()
        };
        log::warn!(
            "{}",
            format!(
                "position fix: {lat:.6},{lon:.6} h={height_m:.1}m  hdop={:.1} vdop={:.1} {}sv {sig}{isb} cdt={:.3}ms  https://maps.google.com/?ll={lat},{lon}",
                sol.hdop,
                sol.vdop,
                meas.len(),
                sol.cdt_m / SPEED_OF_LIGHT * 1e3
            )
            .green()
        );
        true
    }

    /// gnss-rtk fallback/cross-check (also the path when GNSS_SOLVER=rtk): its
    /// candidate pool is built here from the same finished measurements.
    fn solve_rtk(&mut self, meas: &[SvMeasurement], now_gpst: Epoch) -> bool {
        let params = UserParameters::new(UserProfile::Static, ClockProfile::Quartz);
        let pool: Vec<Candidate> = meas
            .iter()
            .map(|m| {
                Candidate::new(
                    m.sv,
                    now_gpst,
                    vec![Observation::pseudo_range(Carrier::L1, m.pr_m, Some(m.cn0))],
                )
            })
            .collect();
        let res = self.solver.ppp(now_gpst, params, &pool);

        match res {
            Err(err) => {
                log::warn!("Failed to get a position: {err}");
                false
            }
            Ok(pvt) => {
                let pos = Vector3::new(pvt.pos_m.0, pvt.pos_m.1, pvt.pos_m.2);
                // Same terrestrial plausibility gate as the WLS path: a
                // converged-but-garbage solution (e.g. an inconsistent mixed
                // pool) must not be published or seed the next epoch.
                let r = pos.norm();
                if !(6.25e6..6.5e6).contains(&r) {
                    log::warn!("gnss-rtk solution off-Earth (|pos|={:.3e} m); rejecting", r);
                    return false;
                }
                self.last_fix_ecef = Some(pos);

                let lat = pvt.lat_long_alt_deg_deg_m.0;
                // gnss-rtk reports longitude in [0, 360); wrap to [-180, 180] so
                // the value is usable directly (e.g. Google Maps ?ll= links).
                let lon = (pvt.lat_long_alt_deg_deg_m.1 + 180.0).rem_euclid(360.0) - 180.0;
                let height_m = pvt.lat_long_alt_deg_deg_m.2;

                // gnss-rtk 0.8.0 computes VDOP against a corrupted "up" axis: its
                // ECEF->ENU rotation (DilutionOfPrecision::q_enu) puts sin(lon)
                // where the up vector needs sin(lat), so `pvt.vdop` is wrong (it
                // even comes out below HDOP, which is physically impossible for a
                // ground receiver). GDOP (the matrix trace, rotation-invariant),
                // HDOP and TDOP are correct, and the estimated state is just
                // position+clock — verified at runtime that GDOP^2 == HDOP^2 +
                // VDOP^2 + TDOP^2 holds with the value below — so recover the true
                // VDOP from those. (A future multi-constellation solve would add an
                // inter-system-bias state and this would over-estimate VDOP.)
                // Upstream fix submitted as nav-solutions/gnss-rtk#121
                // (https://github.com/nav-solutions/gnss-rtk/pull/121); once a
                // release carrying it lands, drop this and read `pvt.vdop` directly.
                let vdop = (pvt.gdop.powi(2) - pvt.hdop.powi(2) - pvt.tdop.powi(2))
                    .max(0.0)
                    .sqrt();
                {
                    let mut st = self.pub_state.lock().unwrap();
                    st.latitude = lat;
                    st.longitude = lon;
                    st.height = height_m;
                    // Fix precision: dilution-of-precision geometry factors and
                    // the count of satellites that contributed to this solve.
                    st.hdop = pvt.hdop;
                    st.vdop = vdop;
                    st.fix_sv_count = pvt.sv.len();
                }

                log::warn!(
                    "{}",
                    format!(
                        "position fix: {lat:.6},{lon:.6} h={height_m:.1}m  hdop={:.1} vdop={vdop:.1} {}sv  https://maps.google.com/?ll={lat},{lon}",
                        pvt.hdop,
                        pvt.sv.len()
                    )
                    .green()
                );
                true
            }
        }
    }

    /// Build one SV's fully corrected measurement for the solve, or `None` if its
    /// pseudorange comes out non-finite (a NaN correction from a garbage prior
    /// fix, which would panic gnss-rtk's Duration math).
    fn build_sv_measurement(
        &self,
        mm: &Measurement,
        eph: &RxEphemeris,
        t_tx: Epoch,
        ctx: &EpochContext,
    ) -> Option<SvMeasurement> {
        let stale_sec = ctx.freshest - mm.ts_sec;
        let pseudo_range_sec = (ctx.now_gpst - t_tx).to_seconds() - stale_sec;
        let clock_corr = sv_clock_correction(eph, ctx.now_gpst);

        // This SV's broadcast residual-error variance (SBAS UDRE + slant
        // GIVE), accumulated below and fed to the WLS as its weight prior.
        let mut sv_var_m2 = 0.0;
        // Ionosphere and troposphere both need a receiver position (pierce
        // point / elevation angle). We have one once there's a previous fix;
        // before that both corrections are 0 (a few metres, dwarfed by the
        // first-fix transient anyway).
        let (iono_m, tropo_m) = if let Some(rx_ecef) = self.last_fix_ecef {
            let sat = compute_sv_position_ecef(eph, ctx.now_gpst);
            let (elev, azim) = elevation_azimuth(rx_ecef, sat);
            {
                let mut st = self.pub_state.lock().unwrap();
                if let Some(ch) = st.channels.get_mut(&eph.sv) {
                    ch.elevation_deg = elev.to_degrees();
                    ch.azimuth_deg = azim.to_degrees();
                }
            }
            let (lat, lon, h_m) =
                ecef2geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2], Ellipsoid::WGS84);
            // Ionosphere, best source first: the SBAS grid (measured, per
            // pierce point, available within seconds of tracking a GEO),
            // else Klobuchar (a climatological model whose coefficients
            // need up to 12.5 min of GPS nav data). Either applies to every
            // SV regardless of constellation — all signals here are L1
            // (standard practice for Galileo); Galileo's own NeQuick-G
            // is still just decoded inputs (eph.ai0/1/2), awaiting the
            // model port. The SBAS grid can be sparse: outside its
            // populated cells we fall back per-SV.
            let iono = if elev > 0.0 {
                ctx.sbas_iono
                    .as_ref()
                    .and_then(|g| g.delay_var_m(lat, lon, elev, azim))
                    .map(|(d, v)| {
                        sv_var_m2 += v;
                        d
                    })
                    .unwrap_or_else(|| {
                        if ctx.iono_valid {
                            klobuchar_l1_delay_m(
                                &ctx.iono_alpha,
                                &ctx.iono_beta,
                                lat,
                                lon,
                                elev,
                                azim,
                                ctx.gps_sod,
                            )
                        } else {
                            0.0
                        }
                    })
            } else {
                0.0
            };
            let tropo = saastamoinen_tropo_m(lat, h_m, elev);
            (iono, tropo)
        } else {
            (0.0, 0.0)
        };

        log::warn!(
            "{} - t_tx={t_tx:?} code_off_sec={:.7}",
            eph.sv,
            mm.code_off_sec
        );
        // SBAS differential corrections (DO-229 A.4.4.3), folded into the
        // pseudorange: PR_corrected = PR + PRC (fast, clock-error
        // dominated) + c·δclk − û·δpos (long-term; the line-of-sight
        // projection stands in for moving the modelled SV — exact to
        // |δpos|²/range, sub-mm). δpos needs a receiver position for û,
        // so like iono/tropo it waits for the first fix; PRC and δclk
        // apply from the start. Unlike iono these are per-SV, so the
        // receiver-clock state cannot absorb them.
        let mut sbas_m = 0.0;
        if let Some(c) = &ctx.sbas_corr {
            if let Some((prc, var)) = c.fast_prc_var_m(eph.sv, ctx.ts_sec) {
                sbas_m += prc;
                sv_var_m2 += var;
            }
            if let Some((dpos, dclk)) = c.long_term(eph.sv, eph.iode as u8, ctx.gps_sod) {
                sbas_m += SPEED_OF_LIGHT * dclk;
                if let Some(rx_ecef) = self.last_fix_ecef {
                    let s = compute_sv_position_ecef(eph, ctx.now_gpst);
                    let v = Vector3::new(s.0, s.1, s.2) - rx_ecef;
                    sbas_m -= v.dot(&Vector3::from(dpos)) / v.norm();
                }
            }
        }

        log::warn!(
            "{} - prng={:.2} msec tgd={:+e} clock_corr={clock_corr:+.3e} iono={iono_m:.1}m tropo={tropo_m:.1}m sbas={sbas_m:+.1}m",
            eph.sv,
            pseudo_range_sec * 1000.0,
            eph.tgd,
        );

        let pr_m = pseudo_range_sec * SPEED_OF_LIGHT - iono_m - tropo_m + sbas_m;

        // Drop a pseudorange that isn't a physically plausible Earth↔MEO range:
        // gnss-rtk's transmit-time math (rx − pr/c) *panics* ("physical non
        // sense") on NaN/inf or an absurd magnitude — e.g. a blown-up correction
        // — so guard the whole class here, not just non-finite. Real one-way
        // ranges run ~20-26 Mm (~67-87 ms); allow generous slack either side.
        if !pr_m.is_finite() || !(1.0e7..4.0e7).contains(&pr_m) {
            log::warn!(
                "{}: dropping SV, implausible pseudorange {pr_m:.3e} m",
                eph.sv
            );
            return None;
        }

        // The finished measurement: SV at transmit time rotated into the
        // reception ECEF frame (Earth turns ωe·τ while the signal travels).
        let we = EARTH_ROTATION_RATE * pseudo_range_sec;
        let (cw, sw) = (we.cos(), we.sin());
        let s = compute_sv_position_ecef(eph, t_tx);
        let m = SvMeasurement {
            sv: eph.sv,
            cn0: mm.cn0,
            pr_m,
            clk_m: clock_corr * SPEED_OF_LIGHT,
            svp: [cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1, s.2],
            galileo: eph.sv.constellation == Constellation::Galileo,
            var_m2: sv_var_m2,
        };

        if let Some(truth) = ctx.truth_ecef {
            let geom = ((m.svp[0] - truth[0]).powi(2)
                + (m.svp[1] - truth[1]).powi(2)
                + (m.svp[2] - truth[2]).powi(2))
            .sqrt();
            // t_tx is SV-time, so pr_m = geom + c*dT_rx - c*clock_corr.
            // residual = pr_m + clk_m - geom = c*dT_rx (common-mode rx clock).
            log::warn!(
                "RESID {} pr={:.2}km geom={:.2}km clk={:+.3}km resid={:+.3}km",
                eph.sv,
                pr_m / 1000.0,
                geom / 1000.0,
                m.clk_m / 1000.0,
                (pr_m + m.clk_m - geom) / 1000.0
            );
        }
        Some(m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gnss_rtk::prelude::TimeScale;

    // A realistic Galileo MEO ephemeris (near-circular, ~56° inclination).
    fn galileo_meo_eph() -> RxEphemeris {
        let mut e = RxEphemeris::new(SV::new(Constellation::Galileo, 9));
        e.a = 29_600_000.0; // Galileo semi-major axis (vs GPS ~26 560 km)
        e.ecc = 1.5e-4;
        e.i0 = 0.96;
        e.omg0 = 0.3;
        e.omg = -1.0;
        e.m0 = 0.5;
        e.toe = 61_800;
        e.week = 947;
        e.toe_gpst = Epoch::from_time_of_week(947, 61_800 * 1_000_000_000, TimeScale::GST);
        e.toc_gpst = e.toe_gpst;
        e
    }

    // Two orbit sources with separate stores must not see each other's
    // ephemerides — the property the old process-global SOLVER_EPHEMERIS
    // static broke (it limited the process to one solver and let a second
    // instance silently read the first one's data).
    #[test]
    fn orbit_sources_are_instance_scoped() {
        let eph = galileo_meo_eph();
        let sv = eph.sv;
        let t = eph.toe_gpst;

        let a = ReceiverOrbitSource {
            ephs: Arc::new(Mutex::new(vec![eph])),
        };
        let b = ReceiverOrbitSource {
            ephs: Arc::new(Mutex::new(Vec::new())),
        };

        let almanac = Almanac::until_2035().expect("Almanac");
        let frame = almanac.frame_from_uid(EARTH_J2000).expect("earth frame");

        assert!(a.state_at(t, sv, frame).is_some(), "own store is visible");
        assert!(
            b.state_at(t, sv, frame).is_none(),
            "a second solver's empty store must not see the first one's data"
        );
    }

    // The orbit math is shared with the (end-to-end validated) GPS path; this
    // guards that the Galileo branch — its µ and a GST-referenced toe — yields a
    // physically sensible MEO position rather than garbage.
    #[test]
    fn galileo_ephemeris_computes_a_meo_position() {
        let e = galileo_meo_eph();
        let (x, y, z) = compute_sv_position_ecef(&e, e.toe_gpst);
        let r = (x * x + y * y + z * z).sqrt();
        // At t = toe the geocentric radius is a(1 - e·cosE0): within a·e of a.
        assert!((r - e.a).abs() <= e.a * e.ecc + 1.0, "r={r}");
        // ~23 000 km above the WGS-84 surface — Galileo MEO altitude.
        let h = r - 6_378_137.0;
        assert!((20_000_000.0..25_000_000.0).contains(&h), "h={h}");
        // Clock correction with a zero clock model is just the relativistic term:
        // bounded (sub-microsecond) and finite.
        let dt = sv_clock_correction(&e, e.toe_gpst);
        assert!(dt.abs() < 1e-6, "dt={dt}");
    }
}
