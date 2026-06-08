use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AbsoluteTime, Almanac, BiasRuntime, Candidate, Carrier, ClockProfile, Config, Duration,
    EARTH_J2000, EnvironmentalBias, Ephemeris, EphemerisSource, Epoch, Frame, Method, Observation,
    Orbit, OrbitSource, Rc, SatelliteClockCorrection, Solver, SpacebornBias, UserParameters,
    UserProfile, Vector3,
};
use map_3d::{Ellipsoid, ecef2geodetic};
use once_cell::sync::Lazy;
use std::sync::{Arc, Mutex};

use crate::{
    constants::{EARTH_MU_GPS, EARTH_ROTATION_RATE, SPEED_OF_LIGHT},
    ephemeris::Ephemeris as RxEphemeris,
    state::GnssState,
};

const PI: f64 = std::f64::consts::PI;

fn get_eccentric_anomaly(eph: &RxEphemeris, t_k: f64) -> f64 {
    let n0 = (EARTH_MU_GPS / eph.a.powi(3)).sqrt();
    let n = n0 + eph.deln;
    let mk = eph.m0 + n * t_k;

    let mut e = mk;
    let mut e_k = 0.0;
    let mut n_iter = 0;

    while (e - e_k).abs() > 1e-14 && n_iter < 30 {
        e_k = e;
        e = e + (mk - e + eph.ecc * e.sin()) / (1.0 - eph.ecc * e.cos());
        n_iter += 1;
    }
    assert!(n_iter < 20);

    e
}

fn normalize_week_seconds(mut dt: f64) -> f64 {
    if dt > 302400.0 {
        dt -= 604800.0;
    }
    if dt < -302400.0 {
        dt += 604800.0;
    }
    dt
}

fn get_sv_clock_correction(eph: &RxEphemeris, t: Epoch) -> f64 {
    let f_rel = -2.0 * EARTH_MU_GPS.sqrt() / SPEED_OF_LIGHT.powi(2);

    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());
    let ecc_anomaly = get_eccentric_anomaly(eph, dte);
    let dtr = f_rel * eph.ecc * eph.a.sqrt() * ecc_anomaly.sin();

    let dtc = normalize_week_seconds((t - eph.toc_gpst).to_seconds());

    eph.f0 + eph.f1 * dtc + eph.f2 * dtc.powi(2) + dtr
}

fn compute_sv_position_ecef(eph: &RxEphemeris, t: Epoch) -> (f64, f64, f64) {
    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());

    log::warn!("{}: ---- now={t:?}", eph.sv);
    log::warn!("{}: ---- toe={:?} delta-t={dte} ", eph.sv, eph.toe_gpst);

    let ecc_anomaly = get_eccentric_anomaly(eph, dte);
    let v_k =
        ((1.0 - eph.ecc.powi(2)).sqrt() * ecc_anomaly.sin()).atan2(ecc_anomaly.cos() - eph.ecc);

    let phi_k = v_k + eph.omg;
    let duk = eph.cus * (2.0 * phi_k).sin() + eph.cuc * (2.0 * phi_k).cos();
    let drk = eph.crs * (2.0 * phi_k).sin() + eph.crc * (2.0 * phi_k).cos();
    let dik = eph.cis * (2.0 * phi_k).sin() + eph.cic * (2.0 * phi_k).cos();

    let uk = phi_k + duk;
    let rk = eph.a * (1.0 - eph.ecc * ecc_anomaly.cos()) + drk;
    let ik = eph.i0 + eph.i_dot * dte + dik;

    let orb_plane_x = rk * uk.cos();
    let orb_plane_y = rk * uk.sin();

    let omega =
        eph.omg0 + (eph.omg_dot - EARTH_ROTATION_RATE) * dte - EARTH_ROTATION_RATE * eph.toe as f64;

    let ecef_x = orb_plane_x * omega.cos() - orb_plane_y * ik.cos() * omega.sin();
    let ecef_y = orb_plane_x * omega.sin() + orb_plane_y * ik.cos() * omega.cos();
    let ecef_z = orb_plane_y * ik.sin();

    log::warn!(
        "{}: position: x={:8.1} y={:8.1} z={:8.1} h={:.1}",
        eph.sv,
        ecef_x / 1000.0,
        ecef_y / 1000.0,
        ecef_z / 1000.0,
        (ecef_x.powi(2) + ecef_y.powi(2) + ecef_z.powi(2)).sqrt() / 1000.0
    );
    let (lat_rad, lon_rad, h) = ecef2geodetic(ecef_x, ecef_y, ecef_z, Ellipsoid::WGS84);
    log::warn!(
        "{}: position: lat/lon: {:.6},{:.6} h={:.1}",
        eph.sv,
        lat_rad * 180.0 / PI,
        lon_rad * 180.0 / PI,
        h / 1000.0
    );
    (ecef_x, ecef_y, ecef_z)
}

fn elevation_azimuth(rx_ecef: Vector3<f64>, sat_ecef: (f64, f64, f64)) -> (f64, f64) {
    let (lat, lon, _) = ecef2geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2], Ellipsoid::WGS84);
    let (dx, dy, dz) = (
        sat_ecef.0 - rx_ecef[0],
        sat_ecef.1 - rx_ecef[1],
        sat_ecef.2 - rx_ecef[2],
    );
    let (sl, cl) = (lat.sin(), lat.cos());
    let (so, co) = (lon.sin(), lon.cos());
    let east = -so * dx + co * dy;
    let north = -sl * co * dx - sl * so * dy + cl * dz;
    let up = cl * co * dx + cl * so * dy + sl * dz;
    let range = (east * east + north * north + up * up).sqrt();
    let elev = (up / range).asin();
    let azim = east.atan2(north);
    (elev, azim)
}

#[allow(clippy::too_many_arguments)]
/// Saastamoinen troposphere slant delay (metres) using a standard atmosphere.
///
/// Computes the zenith hydrostatic delay (ZHD) from standard-atmosphere
/// pressure at the given height, adds a simplified zenith wet delay (ZWD),
/// and maps to the slant path with 1/sin(elevation). Returns 0 for elevations
/// below 5° to avoid blow-up near the horizon.
///
/// Reference: Saastamoinen (1973); standard atmosphere (ICAO).
fn saastamoinen_tropo_m(lat: f64, h_m: f64, elev: f64) -> f64 {
    if elev < 5.0_f64.to_radians() {
        return 0.0;
    }
    // Standard atmosphere: surface pressure at height h_m [hPa].
    let p_hpa = 1013.25 * (1.0 - 2.2558e-5 * h_m).powf(5.2568);
    // Saastamoinen gravity correction (latitude- and height-dependent).
    let f = 1.0 - 2.66e-3 * (2.0 * lat).cos() - 2.8e-4 * (h_m / 1000.0);
    // Zenith hydrostatic delay [m].
    let zhd = 2.2779e-3 * p_hpa / f;
    // Zenith wet delay [m]: typical mid-latitude value, decays with altitude.
    let zwd = 0.1 * (-h_m / 2000.0).exp();
    // Slant delay via simple 1/sin(elevation) mapping.
    (zhd + zwd) / elev.sin()
}

fn klobuchar_l1_delay_m(
    alpha: &[f64; 4],
    beta: &[f64; 4],
    lat: f64,
    lon: f64,
    elev: f64,
    azim: f64,
    gps_sod: f64,
) -> f64 {
    let psi = 0.0137 / (elev / PI + 0.11) - 0.022;
    let phi = (lat / PI + psi * azim.cos()).clamp(-0.416, 0.416);
    let lam = lon / PI + psi * azim.sin() / (phi * PI).cos();
    let phi = phi + 0.064 * ((lam - 1.617) * PI).cos();
    let mut tt = 43200.0 * lam + gps_sod;
    tt -= (tt / 86400.0).floor() * 86400.0;
    let f = 1.0 + 16.0 * (0.53 - elev / PI).powi(3);
    let amp = (alpha[0] + phi * (alpha[1] + phi * (alpha[2] + phi * alpha[3]))).max(0.0);
    let per = (beta[0] + phi * (beta[1] + phi * (beta[2] + phi * beta[3]))).max(72000.0);
    let x = 2.0 * PI * (tt - 50400.0) / per;
    let delay = if x.abs() < 1.57 {
        5.0e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
    } else {
        5.0e-9
    };
    SPEED_OF_LIGHT * f * delay
}

static SOLVER_EPHEMERIS: Lazy<Mutex<Vec<RxEphemeris>>> =
    Lazy::new(|| Mutex::new(Vec::<RxEphemeris>::new()));

struct NullEph;

impl EphemerisSource for NullEph {
    fn ephemeris_data(&self, _: Epoch, _: SV) -> Option<Ephemeris> {
        None
    }
}

struct ReceiverOrbitSource;

impl OrbitSource for ReceiverOrbitSource {
    fn state_at(&self, epoch: Epoch, sv: SV, frame: Frame) -> Option<Orbit> {
        let ephs = SOLVER_EPHEMERIS.lock().unwrap();
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

struct ReceiverSpacebornBias;

impl SpacebornBias for ReceiverSpacebornBias {
    fn clock_bias(&self, rtm: &BiasRuntime) -> SatelliteClockCorrection {
        let ephs = SOLVER_EPHEMERIS.lock().unwrap();
        let Some(eph) = ephs.iter().find(|e| e.sv == rtm.sv) else {
            return SatelliteClockCorrection::default();
        };
        let corr = get_sv_clock_correction(eph, rtm.epoch);
        SatelliteClockCorrection::with_relativistic_correction(Duration::from_seconds(corr))
    }

    fn group_delay(&self, rtm: &BiasRuntime) -> Duration {
        let ephs = SOLVER_EPHEMERIS.lock().unwrap();
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

pub struct PositionSolver {
    solver: SolverInstance,
    // Last fix, used only as the receiver position for the ionosphere pierce
    // point; the solver itself needs no a-priori (it bootstraps via Bancroft).
    last_fix_ecef: Option<Vector3<f64>>,
    pub_state: Arc<Mutex<GnssState>>,
}

fn make_config() -> Config {
    let mut cfg = Config::default().with_navigation_method(Method::SPP);
    cfg.min_sv_elev = Some(0.0);
    // gnss-rtk's default max_gdop (5.0) targets ultra-precise use and rejects
    // perfectly valid marginal-geometry fixes -- e.g. the CTTC capture, which
    // gnss-sdr itself solves at HDOP 4.4 (GDOP ~7-9). Relax it so a usable fix
    // from a sparse/clustered SV set isn't thrown away. Good-geometry recordings
    // (gpssim, nov3) sit well under this and are unaffected.
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
fn make_solver(almanac: &Almanac, earth_frame: Frame, cfg: &Config) -> SolverInstance {
    let eph = Rc::new(NullEph);
    let orb = Rc::new(ReceiverOrbitSource);
    let sb = Rc::new(ReceiverSpacebornBias);
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

impl PositionSolver {
    #[allow(clippy::new_without_default)]
    pub fn new(pub_state: Arc<Mutex<GnssState>>) -> Self {
        let cfg = make_config();
        let almanac = Almanac::until_2035().expect("Almanac");
        let earth_frame = almanac.frame_from_uid(EARTH_J2000).expect("earth frame");
        let solver = make_solver(&almanac, earth_frame, &cfg);

        Self {
            solver,
            last_fix_ecef: None,
            pub_state,
        }
    }

    pub fn has_fix(&self) -> bool {
        self.last_fix_ecef.is_some()
    }

    /// Returns true if a position was resolved this call.
    pub fn compute_position(&mut self, _ts_sec: f64, ephs: &[RxEphemeris]) -> bool {
        {
            let mut glob_ephs = SOLVER_EPHEMERIS.lock().unwrap();
            *glob_ephs = ephs.to_vec();
        }

        let mut pool = vec![];

        let tx_gpst: Vec<Epoch> = ephs
            .iter()
            .map(|eph| {
                // Transmit phase = trk_phase - code_off (see channel.rs: code_off is
                // the replica offset, opposite in sign to the received code phase).
                let phase = eph.trk_phase - eph.code_off_sec;
                let elapsed = if eph.tx_anchored {
                    phase - eph.tow_trk_phase
                } else {
                    (eph.trk_phase - eph.tow_trk_phase) - eph.code_off_sec
                };
                let tow = if eph.tx_anchored {
                    eph.tx_tow_gpst
                } else {
                    eph.tow_gpst
                };
                tow + Duration::from_seconds(elapsed)
            })
            .collect();

        const NOMINAL_TRAVEL_SEC: f64 = 0.070;
        let latest_tx = *tx_gpst.iter().max().unwrap();
        let now_gpst = latest_tx + Duration::from_seconds(NOMINAL_TRAVEL_SEC);

        let (iono_valid, iono_alpha, iono_beta) = {
            let st = self.pub_state.lock().unwrap();
            (st.ion_adj, st.iono_alpha, st.iono_beta)
        };
        let gps_sod = {
            let r = &ephs[0];
            (r.tow as f64 + (now_gpst - r.tow_gpst).to_seconds()).rem_euclid(86400.0)
        };

        let truth_ecef: Option<Vector3<f64>> =
            std::env::var("GNSS_TRUTH_ECEF").ok().and_then(|s| {
                let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                (v.len() == 3).then(|| Vector3::new(v[0], v[1], v[2]))
            });

        let params = UserParameters::new(UserProfile::Static, ClockProfile::Quartz);

        log::warn!("----- now_gpst={now_gpst:?}");
        for (eph, t_tx) in ephs.iter().zip(tx_gpst.iter()) {
            let pseudo_range_sec = (now_gpst - *t_tx).to_seconds();
            let clock_corr = get_sv_clock_correction(eph, now_gpst);

            // Ionosphere and troposphere both need a receiver position (pierce
            // point / elevation angle). We have one once there's a previous fix;
            // before that both corrections are 0 (a few metres, dwarfed by the
            // first-fix transient anyway).
            let (iono_m, tropo_m) = if let Some(rx_ecef) = self.last_fix_ecef {
                let sat = compute_sv_position_ecef(eph, now_gpst);
                let (elev, azim) = elevation_azimuth(rx_ecef, sat);
                let (lat, lon, h_m) =
                    ecef2geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2], Ellipsoid::WGS84);
                let iono = if iono_valid && elev > 0.0 {
                    klobuchar_l1_delay_m(&iono_alpha, &iono_beta, lat, lon, elev, azim, gps_sod)
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
                eph.code_off_sec
            );
            log::warn!(
                "{} - prng={:.2} msec tgd={:+e} clock_corr={clock_corr:+.3e} iono={iono_m:.1}m tropo={tropo_m:.1}m",
                eph.sv,
                pseudo_range_sec * 1000.0,
                eph.tgd,
            );

            let pr_m = pseudo_range_sec * SPEED_OF_LIGHT - iono_m - tropo_m;

            if let Some(truth) = truth_ecef {
                let we = EARTH_ROTATION_RATE * pseudo_range_sec;
                let (cw, sw) = (we.cos(), we.sin());
                let s = compute_sv_position_ecef(eph, *t_tx);
                let (sx, sy, sz) = (cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1, s.2);
                let geom =
                    ((sx - truth[0]).powi(2) + (sy - truth[1]).powi(2) + (sz - truth[2]).powi(2))
                        .sqrt();
                let clk_m = clock_corr * SPEED_OF_LIGHT;
                // t_tx is SV-time, so pr_m = geom + c*dT_rx - c*clock_corr.
                // residual = pr_m + clk_m - geom = c*dT_rx (common-mode rx clock).
                log::warn!(
                    "RESID {} pr={:.2}km geom={:.2}km clk={:+.3}km resid={:+.3}km",
                    eph.sv,
                    pr_m / 1000.0,
                    geom / 1000.0,
                    clk_m / 1000.0,
                    (pr_m + clk_m - geom) / 1000.0
                );
            }
            pool.push(Candidate::new(
                eph.sv,
                now_gpst,
                vec![Observation::pseudo_range(Carrier::L1, pr_m, Some(eph.cn0))],
            ));
        }

        let res = self.solver.ppp(now_gpst, params, &pool);

        match res {
            Err(err) => {
                log::warn!("Failed to get a position: {err}");
                false
            }
            Ok(pvt) => {
                let pos = Vector3::new(pvt.pos_m.0, pvt.pos_m.1, pvt.pos_m.2);
                self.last_fix_ecef = Some(pos);

                let lat = pvt.lat_long_alt_deg_deg_m.0;
                // gnss-rtk reports longitude in [0, 360); wrap to [-180, 180] so
                // the value is usable directly (e.g. Google Maps ?ll= links).
                let lon = (pvt.lat_long_alt_deg_deg_m.1 + 180.0).rem_euclid(360.0) - 180.0;
                let height = pvt.lat_long_alt_deg_deg_m.2 / 1000.0;

                self.pub_state.lock().unwrap().latitude = lat;
                self.pub_state.lock().unwrap().longitude = lon;
                self.pub_state.lock().unwrap().height = height;

                log::warn!(
                    "{}",
                    format!(
                        "position fix: {lat:.6},{lon:.6} h={height:.1}km  https://maps.google.com/?ll={lat},{lon}"
                    )
                    .green()
                );
                true
            }
        }
    }
}
