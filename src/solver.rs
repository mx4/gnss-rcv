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
    apriori_ecef: Vector3<f64>,
    pub_state: Arc<Mutex<GnssState>>,
}

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
            apriori_ecef,
            pub_state,
        }
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

        log::warn!("----- now_gpst={now_gpst:?}");
        for (eph, t_tx) in ephs.iter().zip(tx_gpst.iter()) {
            let pseudo_range_sec = (now_gpst - *t_tx).to_seconds();

            let clock_corr = get_sv_clock_correction(eph, now_gpst);

            log::warn!(
                "{} - t_tx={t_tx:?} code_off_sec={:.7}",
                eph.sv,
                eph.code_off_sec
            );
            log::warn!(
                "{} - prng={:.2} msec tgd={:+e} clock_corr={clock_corr:+.3e}",
                eph.sv,
                pseudo_range_sec * 1000.0,
                eph.tgd,
            );

            let candidate = Candidate::new(
                eph.sv,
                now_gpst,
                Duration::from_seconds(clock_corr),
                Some(Duration::from_seconds(eph.tgd)),
                vec![Observation {
                    carrier: Carrier::L1,
                    value: pseudo_range_sec * SPEED_OF_LIGHT,
                    snr: Some(eph.cn0),
                }],
                vec![],
                vec![],
            );

            pool.push(candidate);
        }

        let (tropo_bias, iono_bias) = get_tropo_iono_bias();
        let res = self
            .solver
            .resolve(now_gpst, &pool, &iono_bias, &tropo_bias);

        match res {
            Err(err) => log::warn!("Failed to get a position: {err}"),
            Ok(solution) => {
                // gnss-rtk 0.4.5 returns the position as a delta relative to the
                // apriori (it solves y = pr - rho around the apriori and never
                // adds it back), so recover the absolute ECEF here.
                let pos = self.apriori_ecef + solution.1.position;
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
