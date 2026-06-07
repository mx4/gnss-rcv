use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Carrier, Config, Duration, Epoch, InterpolationResult,
    IonosphereBias, Method, Observation, Solver, TroposphereBias, Vector3,
};
use map_3d::{Ellipsoid, ecef2geodetic};
use once_cell::sync::Lazy;
use std::sync::{Arc, Mutex};

use crate::{
    constants::{EARTH_MU_GPS, EARTH_ROTATION_RATE, SPEED_OF_LIGHT},
    ephemeris::Ephemeris,
    state::GnssState,
};

const PI: f64 = std::f64::consts::PI;

fn get_eccentric_anomaly(eph: &Ephemeris, t_k: f64) -> f64 {
    // computed mean motion
    let n0 = (EARTH_MU_GPS / eph.a.powi(3)).sqrt();
    // corrected mean motion
    let n = n0 + eph.deln;
    // mean anomaly
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

/// SV clock correction in seconds: polynomial term (af0/af1/af2) plus the
/// relativistic eccentricity correction dtr = F * e * sqrt(A) * sin(E_k),
/// with F = -2 * sqrt(mu) / c^2.
fn get_sv_clock_correction(eph: &Ephemeris, t: Epoch) -> f64 {
    let f_rel = -2.0 * EARTH_MU_GPS.sqrt() / SPEED_OF_LIGHT.powi(2);

    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());
    let ecc_anomaly = get_eccentric_anomaly(eph, dte);
    let dtr = f_rel * eph.ecc * eph.a.sqrt() * ecc_anomaly.sin();

    let dtc = normalize_week_seconds((t - eph.toc_gpst).to_seconds());

    eph.f0 + eph.f1 * dtc + eph.f2 * dtc.powi(2) + dtr
}

fn compute_sv_position_ecef(eph: &Ephemeris, t: Epoch) -> (f64, f64, f64) {
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

/// Elevation and azimuth (radians) of a satellite at `sat_ecef` as seen from the
/// receiver at `rx_ecef`, via the local East-North-Up frame.
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

/// Klobuchar ionospheric group delay on L1 in meters (ICD-GPS-200 / RTKLIB).
/// `lat`/`lon`/`elev`/`azim` are in radians; `gps_sod` is the GPS time of day [s].
#[allow(clippy::too_many_arguments)]
fn klobuchar_l1_delay_m(
    alpha: &[f64; 4],
    beta: &[f64; 4],
    lat: f64,
    lon: f64,
    elev: f64,
    azim: f64,
    gps_sod: f64,
) -> f64 {
    // earth-centered angle (semicircles)
    let psi = 0.0137 / (elev / PI + 0.11) - 0.022;
    // subionospheric latitude (semicircles), clamped
    let phi = (lat / PI + psi * azim.cos()).clamp(-0.416, 0.416);
    // subionospheric longitude (semicircles)
    let lam = lon / PI + psi * azim.sin() / (phi * PI).cos();
    // geomagnetic latitude (semicircles)
    let phi = phi + 0.064 * ((lam - 1.617) * PI).cos();
    // local time (s of day)
    let mut tt = 43200.0 * lam + gps_sod;
    tt -= (tt / 86400.0).floor() * 86400.0;
    // obliquity (slant) factor
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

fn get_tropo_iono_bias() -> (TroposphereBias, IonosphereBias) {
    let iono_bias = IonosphereBias {
        kb_model: None,
        bd_model: None,
        ng_model: None,
        stec_meas: None,
    };
    let tropo_bias = TroposphereBias {
        total: None,
        zwd_zdd: None,
    };
    (tropo_bias, iono_bias)
}

pub type I = fn(Epoch, SV, usize) -> Option<InterpolationResult>;
pub struct PositionSolver {
    solver: Solver<I>,
    cfg: Config,
    /// Linearization point of the current solver instance (gnss-rtk returns the
    /// solution as a delta relative to this). Only changes on a re-center.
    apriori_ecef: Vector3<f64>,
    /// Latest absolute fix (apriori_ecef + delta), used as the iono pierce-point
    /// reference and as the re-center target.
    last_fix_ecef: Option<Vector3<f64>>,
    recenter_count: usize,
    /// Latched once the fix is within RECENTER_DIST_M of the linearization point;
    /// after that we stop re-centering and let LSQ accumulation damp the noise.
    converged: bool,
    pub_state: Arc<Mutex<GnssState>>,
}

/// While the fix is farther than this from the solver's linearization point we
/// re-center the apriori on it, because gnss-rtk linearizes only once and never
/// re-linearizes. Below the threshold we stop and let cross-epoch LSQ
/// accumulation damp the noise.
const RECENTER_DIST_M: f64 = 30_000.0;
/// Cap on re-centers so a single noisy fix can't make the apriori wander.
const MAX_RECENTERS: usize = 10;

static SOLVER_EPHEMERIS: Lazy<Mutex<Vec<Ephemeris>>> =
    Lazy::new(|| Mutex::new(Vec::<Ephemeris>::new()));

fn sv_interp(t: Epoch, sv: SV, _size: usize) -> Option<InterpolationResult> {
    let ephs = SOLVER_EPHEMERIS.lock().unwrap();
    let eph = ephs.iter().find(|&&e| e.sv == sv).unwrap();
    let pos = compute_sv_position_ecef(eph, t);

    Some(InterpolationResult::from_apc_position(pos))
}

impl PositionSolver {
    #[allow(clippy::new_without_default)]
    pub fn new(pub_state: Arc<Mutex<GnssState>>) -> Self {
        // AprioriPosition::from_geo() feeds map_3d::geodetic2ecef(), which expects
        // latitude/longitude in radians (not degrees).
        let apriori = AprioriPosition::from_geo(Vector3::new(
            46.5_f64.to_radians(),
            6.6_f64.to_radians(),
            0.0,
        ));
        let mut cfg = Config::static_preset(Method::SPP);
        cfg.min_sv_elev = Some(0.0);
        // We add the relativistic clock correction ourselves in
        // get_sv_clock_correction(); disable the solver's own term to avoid
        // double-counting it.
        cfg.modeling.relativistic_clock_bias = false;

        let apriori_ecef = apriori.ecef();
        let solver = Solver::new(&cfg, apriori, sv_interp as I).expect("Solver issue");

        Self {
            solver,
            cfg,
            apriori_ecef,
            last_fix_ecef: None,
            recenter_count: 0,
            converged: false,
            pub_state,
        }
    }

    /// Rebuild the underlying solver so it linearizes around `apriori_ecef`.
    /// gnss-rtk performs a single linearization at the apriori and never
    /// re-linearizes, so feeding the previous fix back as the apriori is what
    /// removes the large linearization error when starting far from truth.
    fn relinearize(&mut self) {
        let apriori = AprioriPosition::from_ecef(self.apriori_ecef);
        self.solver = Solver::new(&self.cfg, apriori, sv_interp as I).expect("Solver issue");
    }

    pub fn compute_position(&mut self, _ts_sec: f64, ephs: &Vec<Ephemeris>) {
        {
            let mut glob_ephs = SOLVER_EPHEMERIS.lock().unwrap();
            *glob_ephs = ephs.clone();
        }

        /*
         * https://www.insidegnss.com/auto/IGM_janfeb12-Solutions.pdf
         *
         * sat0 is the closest. sat2 is the furthest.
         *          tow
         * sat0 -----+[....][....][....][....][....][....][...| obs   <-- reference
         *                 tow                                |
         * sat1 ------------+[....][....][....][....][....][..| obs
         *                        tow                         |
         * sat2 -------------------+[....][....][....][....][.| obs
         *
         *  sat0      []                   ~0
         *  sat1      [------]
         *  sat2      [-------------]
         */
        let mut pool = vec![];

        // Signal transmit time (in SV/GPS time) of the sample currently being
        // received, for each SV. The elapsed transmit time since the tow_gpst
        // boundary is the growth of the transmit phase (num_trk_samples advances
        // at the SV clock rate), NOT the receiver wall-clock delta.
        let tx_gpst: Vec<Epoch> = ephs
            .iter()
            .map(|eph| {
                let elapsed = (eph.trk_phase - eph.tow_trk_phase) + eph.code_off_sec;
                eph.tow_gpst + Duration::from_seconds(elapsed)
            })
            .collect();

        // All SVs are sampled at the same receiver instant. We don't know the
        // receiver clock absolutely, so we place the reception epoch a nominal
        // light-travel time after the most-recent transmit time (the closest /
        // highest SV). This makes every pseudorange physical (~67-90 ms) which
        // is what the solver's geometry expects; the residual offset is
        // absorbed by the estimated receiver clock bias.
        const NOMINAL_TRAVEL_SEC: f64 = 0.070;
        let latest_tx = *tx_gpst.iter().max().unwrap();
        let now_gpst = latest_tx + Duration::from_seconds(NOMINAL_TRAVEL_SEC);

        // Decoded Klobuchar coefficients (if subframe 4 page 18 has been seen).
        // We apply the model ourselves rather than via gnss-rtk's IonosphereBias
        // because that path expects the apriori geodetic in degrees while the
        // ECEF path uses radians, so it would mis-place the pierce point.
        let (iono_valid, iono_alpha, iono_beta) = {
            let st = self.pub_state.lock().unwrap();
            (st.ion_adj, st.iono_alpha, st.iono_beta)
        };
        // GPS time of day used by the Klobuchar local-time term.
        let gps_sod = {
            let r = &ephs[0];
            (r.tow as f64 + (now_gpst - r.tow_gpst).to_seconds()).rem_euclid(86400.0)
        };

        // Optional diagnostic: if GNSS_TRUTH_ECEF="x,y,z" is set, log per-SV
        // pseudorange residuals against that known position. After removing the
        // common (receiver clock) offset, the spread of residuals across SVs is
        // the per-SV measurement error.
        let truth_ecef: Option<Vector3<f64>> = std::env::var("GNSS_TRUTH_ECEF").ok().and_then(|s| {
            let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (v.len() == 3).then(|| Vector3::new(v[0], v[1], v[2]))
        });

        log::warn!("----- now_gpst={now_gpst:?}");
        for (eph, t_tx) in ephs.iter().zip(tx_gpst.iter()) {
            let pseudo_range_sec = (now_gpst - *t_tx).to_seconds();

            let clock_corr = get_sv_clock_correction(eph, now_gpst);

            // Klobuchar ionosphere group delay [m], subtracted from the observed
            // pseudorange (iono delays the code, lengthening the measurement).
            // Use the best position estimate available for the pierce point.
            let rx_ecef = self.last_fix_ecef.unwrap_or(self.apriori_ecef);
            let iono_m = if iono_valid {
                let sat = compute_sv_position_ecef(eph, now_gpst);
                let (elev, azim) = elevation_azimuth(rx_ecef, sat);
                let (lat, lon, _) =
                    ecef2geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2], Ellipsoid::WGS84);
                if elev > 0.0 {
                    klobuchar_l1_delay_m(&iono_alpha, &iono_beta, lat, lon, elev, azim, gps_sod)
                } else {
                    0.0
                }
            } else {
                0.0
            };

            log::warn!(
                "{} - t_tx={t_tx:?} code_off_sec={:.7}",
                eph.sv,
                eph.code_off_sec
            );
            log::warn!(
                "{} - prng={:.2} msec tgd={:+e} clock_corr={clock_corr:+.3e} iono={iono_m:.1}m",
                eph.sv,
                pseudo_range_sec * 1000.0,
                eph.tgd,
            );

            if let Some(truth) = truth_ecef {
                // Satellite position at transmit time, Sagnac-rotated by the
                // signal travel time (same convention as gnss-rtk).
                let we = EARTH_ROTATION_RATE * pseudo_range_sec;
                let (cw, sw) = (we.cos(), we.sin());
                let s = compute_sv_position_ecef(eph, *t_tx);
                let (sx, sy, sz) = (cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1, s.2);
                let geom = ((sx - truth[0]).powi(2)
                    + (sy - truth[1]).powi(2)
                    + (sz - truth[2]).powi(2))
                .sqrt();
                let pr_m = pseudo_range_sec * SPEED_OF_LIGHT - iono_m;
                let clk_m = clock_corr * SPEED_OF_LIGHT;
                log::warn!(
                    "RESID {} pr={:.2}km geom={:.2}km clk={:+.3}km resid={:+.3}km",
                    eph.sv,
                    pr_m / 1000.0,
                    geom / 1000.0,
                    clk_m / 1000.0,
                    (pr_m + clk_m - geom) / 1000.0
                );
            }

            let candidate = Candidate::new(
                eph.sv,
                now_gpst,
                Duration::from_seconds(clock_corr),
                Some(Duration::from_seconds(eph.tgd)),
                vec![Observation {
                    carrier: Carrier::L1,
                    value: pseudo_range_sec * SPEED_OF_LIGHT - iono_m,
                    snr: Some(eph.cn0),
                }],
                vec![],
                vec![],
            );

            pool.push(candidate);
        }

        let (tropo_bias, iono_bias) = get_tropo_iono_bias();

        // Persistent solver: keep the cross-epoch LSQ accumulation, which damps
        // measurement noise and is essential for a stable fix.
        let res = self.solver.resolve(now_gpst, &pool, &iono_bias, &tropo_bias);

        match res {
            Err(err) => log::warn!("Failed to get a position: {err}"),
            Ok(solution) => {
                // gnss-rtk 0.4.5 returns the position as a delta relative to the
                // apriori (it solves y = pr - rho around the apriori and never
                // adds it back), so recover the absolute ECEF here.
                let pos = self.apriori_ecef + solution.1.position;
                self.last_fix_ecef = Some(pos);

                // Adaptive apriori feedback: gnss-rtk linearizes only once at its
                // apriori and never re-linearizes, so while the fix is far from that
                // point we re-center on it (coarse Gauss-Newton) and prime the
                // rebuilt solver so the next epoch still yields a fix. Once we get
                // within the threshold we latch `converged` and stop re-centering,
                // letting cross-epoch LSQ accumulation damp the noise.
                let dist = (pos - self.apriori_ecef).norm();
                if !self.converged {
                    if dist > RECENTER_DIST_M && self.recenter_count < MAX_RECENTERS {
                        self.apriori_ecef = pos;
                        self.relinearize();
                        let _ = self.solver.resolve(now_gpst, &pool, &iono_bias, &tropo_bias);
                        self.recenter_count += 1;
                        log::warn!(
                            "re-centered apriori (#{}) at {:.0} km",
                            self.recenter_count,
                            dist / 1000.0
                        );
                    } else {
                        self.converged = true;
                    }
                }
                let (lat_rad, lon_rad, h) = ecef2geodetic(pos[0], pos[1], pos[2], Ellipsoid::WGS84);
                let lat = lat_rad * 180.0 / PI;
                let lon = lon_rad * 180.0 / PI;
                let height = h / 1000.0;

                self.pub_state.lock().unwrap().latitude = lat;
                self.pub_state.lock().unwrap().longitude = lon;
                self.pub_state.lock().unwrap().height = height;

                log::warn!(
                    "{}",
                    format!("XXX: lat/lon: {:.4},{:.4} h={:.1}", lat, lon, height).red(),
                );
            }
        }
    }
}
