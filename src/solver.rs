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
    constants::{EARTH_MU_GAL, EARTH_MU_GPS, EARTH_ROTATION_RATE, SPEED_OF_LIGHT},
    ephemeris::Ephemeris as RxEphemeris,
    state::GnssState,
};
use gnss_rs::constellation::Constellation;

const PI: f64 = std::f64::consts::PI;

/// Earth gravitational constant for this SV's constellation. GPS (WGS-84) and
/// Galileo (GTRF) publish slightly different µ; using the wrong one shifts the
/// computed SV position by ~1 mm — negligible, but free to get right.
fn earth_mu(sv: SV) -> f64 {
    match sv.constellation {
        Constellation::Galileo => EARTH_MU_GAL,
        _ => EARTH_MU_GPS,
    }
}

fn get_eccentric_anomaly(eph: &RxEphemeris, t_k: f64) -> f64 {
    let n0 = (earth_mu(eph.sv) / eph.a.powi(3)).sqrt();
    let n = n0 + eph.deln;
    let mk = eph.m0 + n * t_k;

    let mut e = mk;
    let mut e_k = 0.0;
    let mut n_iter = 0;

    // Newton's method on Kepler's equation; GNSS orbits are near-circular
    // (ecc < 0.03), so this converges in a handful of iterations. Hitting the
    // cap means it did NOT converge (corrupt ephemeris) — fail loudly below
    // rather than letting garbage flow into the SV position.
    while (e - e_k).abs() > 1e-14 && n_iter < 30 {
        e_k = e;
        e = e + (mk - e + eph.ecc * e.sin()) / (1.0 - eph.ecc * e.cos());
        n_iter += 1;
    }
    assert!(
        n_iter < 30,
        "Kepler solve did not converge: ecc={}",
        eph.ecc
    );

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

pub(crate) fn get_sv_clock_correction(eph: &RxEphemeris, t: Epoch) -> f64 {
    let f_rel = -2.0 * earth_mu(eph.sv).sqrt() / SPEED_OF_LIGHT.powi(2);

    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());
    let ecc_anomaly = get_eccentric_anomaly(eph, dte);
    let dtr = f_rel * eph.ecc * eph.a.sqrt() * ecc_anomaly.sin();

    let dtc = normalize_week_seconds((t - eph.toc_gpst).to_seconds());

    eph.f0 + eph.f1 * dtc + eph.f2 * dtc.powi(2) + dtr
}

pub(crate) fn compute_sv_position_ecef(eph: &RxEphemeris, t: Epoch) -> (f64, f64, f64) {
    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());

    log::debug!("{}: ---- now={t:?}", eph.sv);
    log::debug!("{}: ---- toe={:?} delta-t={dte} ", eph.sv, eph.toe_gpst);

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

    log::debug!(
        "{}: position: x={:8.1} y={:8.1} z={:8.1} h={:.1}",
        eph.sv,
        ecef_x / 1000.0,
        ecef_y / 1000.0,
        ecef_z / 1000.0,
        (ecef_x.powi(2) + ecef_y.powi(2) + ecef_z.powi(2)).sqrt() / 1000.0
    );
    let (lat_rad, lon_rad, h) = ecef2geodetic(ecef_x, ecef_y, ecef_z, Ellipsoid::WGS84);
    log::debug!(
        "{}: position: lat/lon: {:.6},{:.6} h={:.1}",
        eph.sv,
        lat_rad * 180.0 / PI,
        lon_rad * 180.0 / PI,
        h / 1000.0
    );
    (ecef_x, ecef_y, ecef_z)
}

pub(crate) fn elevation_azimuth(rx_ecef: Vector3<f64>, sat_ecef: (f64, f64, f64)) -> (f64, f64) {
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
    // Below 5° the 1/sin mapping blows up; a non-finite elevation (from a garbage
    // prior fix) must also bail here (a plain `<` wouldn't catch NaN).
    if elev.is_nan() || elev < 5.0_f64.to_radians() {
        return 0.0;
    }
    // Standard atmosphere: surface pressure at height h_m [hPa]. Clamp the base at
    // 0: above ~44 km it goes negative and `.powf` of a negative base is NaN,
    // which a garbage height from a bad fix would otherwise propagate into the
    // pseudorange (and crash the solver). Pressure is ~0 up there anyway → ZHD ≈ 0.
    let p_hpa = 1013.25 * (1.0 - 2.2558e-5 * h_m).max(0.0).powf(5.2568);
    // Saastamoinen gravity correction (latitude- and height-dependent).
    let f = 1.0 - 2.66e-3 * (2.0 * lat).cos() - 2.8e-4 * (h_m / 1000.0);
    // Zenith hydrostatic delay [m].
    let zhd = 2.2779e-3 * p_hpa / f;
    // Zenith wet delay [m]: typical mid-latitude value, decays with altitude.
    let zwd = 0.1 * (-h_m / 2000.0).exp();
    // Slant delay via simple 1/sin(elevation) mapping.
    (zhd + zwd) / elev.sin()
}

/// Klobuchar single-frequency ionospheric delay on L1, in metres.
///
/// The broadcast model from IS-GPS-200 20.3.3.5.2.5 (Figure 20-4) / Klobuchar
/// (1987): a thin-shell ionosphere whose vertical delay is a fixed night-time
/// bias plus a half-cosine bump peaking at 14:00 local time, with amplitude and
/// period given by cubic polynomials (in geomagnetic latitude) whose
/// coefficients are the broadcast `alpha`/`beta` (subframe 4, page 18); the
/// vertical delay is mapped to the slant path by an obliquity factor. Corrects
/// ~50% of the ionospheric error RMS.
///
/// `lat`/`lon`/`elev`/`azim` are radians, `gps_sod` GPS seconds-of-day. The ICD
/// formulates everything in *semicircles* (1 sc = π rad) — hence the `/ PI`
/// conversions — and the numeric constants below are the ICD's, verbatim.
fn klobuchar_l1_delay_m(
    alpha: &[f64; 4],
    beta: &[f64; 4],
    lat: f64,
    lon: f64,
    elev: f64,
    azim: f64,
    gps_sod: f64,
) -> f64 {
    // Earth-centred angle between receiver and ionospheric pierce point (sc).
    let psi = 0.0137 / (elev / PI + 0.11) - 0.022;
    // Pierce-point latitude (sc), clamped to ±0.416 sc (±74.9°).
    let phi = (lat / PI + psi * azim.cos()).clamp(-0.416, 0.416);
    // Pierce-point longitude (sc).
    let lam = lon / PI + psi * azim.sin() / (phi * PI).cos();
    // Geomagnetic latitude of the pierce point: (0.064, 1.617) sc is the
    // geomagnetic pole (78.3°N, 291.0°E) the alpha/beta fits are framed on.
    let phi = phi + 0.064 * ((lam - 1.617) * PI).cos();
    // Local solar time at the pierce point: 43200 s per semicircle of longitude.
    let mut tt = 43200.0 * lam + gps_sod;
    tt -= (tt / 86400.0).floor() * 86400.0; // wrap to [0, 86400)
    // Obliquity (vertical → slant) factor: 1 at zenith, ~3 at the horizon.
    let f = 1.0 + 16.0 * (0.53 - elev / PI).powi(3);
    // Amplitude (s) and period (s) of the daytime cosine, cubic in geomagnetic
    // latitude, with the ICD floors (amp ≥ 0; period ≥ 72000 s = 20 h).
    let amp = (alpha[0] + phi * (alpha[1] + phi * (alpha[2] + phi * alpha[3]))).max(0.0);
    let per = (beta[0] + phi * (beta[1] + phi * (beta[2] + phi * beta[3]))).max(72000.0);
    // Phase relative to the diurnal peak at 50400 s (14:00 local).
    let x = 2.0 * PI * (tt - 50400.0) / per;
    // Day side (|x| < 1.57 ≈ π/2): cos(x) as its 4th-order Taylor series, per
    // the ICD. Night side: only the fixed 5 ns (~1.5 m vertical) bias remains.
    let delay = if x.abs() < 1.57 {
        5.0e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
    } else {
        5.0e-9
    };
    SPEED_OF_LIGHT * f * delay
}

/// A live weighted-least-squares fix: position plus the estimated clock and
/// inter-system bias, the self-calibrated per-constellation sigmas, and
/// geometry DOPs from the unweighted normal matrix.
pub(crate) struct WlsSolution {
    pub pos: [f64; 3],
    pub cdt_m: f64,
    pub isb_m: f64,
    pub sigma: std::collections::HashMap<Constellation, f64>,
    pub hdop: f64,
    pub vdop: f64,
}

/// Weighted Gauss-Newton solve over (x, y, z, c·dt, c·ISB_gal) — the live
/// solver. It adds the two things gnss-rtk 0.8 lacks for a mixed pool:
/// per-measurement weights (its measurement sigma is literally `1.0 // TODO`)
/// and an inter-system-bias state, so Galileo's common clock offset stops
/// leaking into position. Weights are self-calibrated: solve once
/// equal-weighted, set each constellation's sigma to its residual RMS, solve
/// again — no hand-tuned constants. **Workaround status**: once a gnss-rtk
/// release accepts measurement sigmas (and a second clock state), hand the
/// weighting back upstream and let this revert to a cross-check
/// (GNSS_SOLVER=rtk selects gnss-rtk meanwhile).
///
/// `meas[i]` is the fully corrected pseudorange + c·sv_clock (m), `svp[i]`
/// the SV ECEF position at transmit time rotated into the reception frame
/// (Sagnac), `gal[i]` its constellation flag. `x0` seeds the iteration (last
/// fix, or zeros — Gauss-Newton converges from the geocentre for GNSS
/// geometry). Returns `None` for underdetermined pools or a non-finite
/// solution (caller falls back to gnss-rtk).
pub(crate) fn wls_solve(
    meas: &[f64],
    svp: &[[f64; 3]],
    gal: &[bool],
    var0: &[f64],
    x0: [f64; 3],
) -> Option<WlsSolution> {
    use std::collections::HashMap;
    let n = meas.len();
    let mixed = gal.iter().any(|&g| g) && gal.iter().any(|&g| !g);
    // The ISB state is only observable with both constellations present (and
    // one redundant measurement); single-constellation pools solve 4 states.
    let ns = if mixed && n >= 5 { 5 } else { 4 };
    if n < ns {
        return None;
    }

    // Gauss-Jordan solve of the ns×ns augmented system; also used for the
    // covariance columns. Returns None on a vanishing pivot.
    fn solve(mut m: [[f64; 6]; 5], ns: usize) -> Option<Vec<f64>> {
        for col in 0..ns {
            let p = (col..ns).max_by(|&r1, &r2| m[r1][col].abs().total_cmp(&m[r2][col].abs()))?;
            if m[p][col].abs() < 1e-12 {
                return None;
            }
            m.swap(col, p);
            let pivot = m[col];
            for (row, mrow) in m.iter_mut().enumerate().take(ns) {
                if row != col {
                    let f = mrow[col] / pivot[col];
                    for (k, pk) in pivot.iter().enumerate().skip(col) {
                        mrow[k] -= f * pk;
                    }
                }
            }
        }
        Some((0..ns).map(|j| m[j][5] / m[j][j]).collect())
    }

    let mut x = x0;
    let (mut cdt, mut isb) = (0.0f64, 0.0f64);
    let mut sigma = HashMap::<Constellation, f64>::new();
    let cons_of = |g: bool| {
        if g {
            Constellation::Galileo
        } else {
            Constellation::GPS
        }
    };

    for _pass in 0..2 {
        for _it in 0..10 {
            // Normal equations A·dx = b for H_i = [−û, 1, gal_i] and
            // r_i = meas_i − (ρ_i + c·dt + gal_i·c·ISB), weight 1/σ².
            let (mut a, mut b) = ([[0.0f64; 5]; 5], [0.0f64; 5]);
            for i in 0..n {
                let d = [svp[i][0] - x[0], svp[i][1] - x[1], svp[i][2] - x[2]];
                let rho = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                let h = [
                    -d[0] / rho,
                    -d[1] / rho,
                    -d[2] / rho,
                    1.0,
                    if gal[i] { 1.0 } else { 0.0 },
                ];
                let r = meas[i] - (rho + cdt + if gal[i] { isb } else { 0.0 });
                // Per-measurement variance: the self-calibrated constellation
                // floor plus this SV's broadcast prior (SBAS UDRE + GIVE).
                let w =
                    1.0 / (sigma.get(&cons_of(gal[i])).copied().unwrap_or(1.0).powi(2) + var0[i]);
                for j in 0..ns {
                    b[j] += w * h[j] * r;
                    for k in 0..ns {
                        a[j][k] += w * h[j] * h[k];
                    }
                }
            }
            let mut m = [[0.0f64; 6]; 5];
            for j in 0..ns {
                m[j][..5].copy_from_slice(&a[j]);
                m[j][5] = b[j];
            }
            let dx = solve(m, ns)?;
            x[0] += dx[0];
            x[1] += dx[1];
            x[2] += dx[2];
            cdt += dx[3];
            if ns == 5 {
                isb += dx[4];
            }
            if dx.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-4 {
                break;
            }
        }
        // Self-calibrate: each constellation's residual RMS becomes its σ.
        let mut acc = HashMap::<Constellation, (f64, usize)>::new();
        for i in 0..n {
            let d = [svp[i][0] - x[0], svp[i][1] - x[1], svp[i][2] - x[2]];
            let rho = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let r = meas[i] - (rho + cdt + if gal[i] { isb } else { 0.0 });
            let e = acc.entry(cons_of(gal[i])).or_insert((0.0, 0));
            e.0 += r * r;
            e.1 += 1;
        }
        for (c, (ss, cnt)) in acc {
            sigma.insert(c, (ss / cnt as f64).sqrt().max(1.0));
        }
    }
    if !(x.iter().all(|v| v.is_finite()) && cdt.is_finite() && isb.is_finite()) {
        return None;
    }

    // Geometry DOPs from the *unweighted* normal matrix at the solution (DOP
    // is a geometry factor; weighting it would conflate signal quality in).
    let mut a = [[0.0f64; 5]; 5];
    for i in 0..n {
        let d = [svp[i][0] - x[0], svp[i][1] - x[1], svp[i][2] - x[2]];
        let rho = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let h = [
            -d[0] / rho,
            -d[1] / rho,
            -d[2] / rho,
            1.0,
            if gal[i] { 1.0 } else { 0.0 },
        ];
        for j in 0..ns {
            for k in 0..ns {
                a[j][k] += h[j] * h[k];
            }
        }
    }
    // Covariance columns Q·e_j by repeated solves; only the position block is
    // needed for HDOP/VDOP.
    let mut q = [[0.0f64; 3]; 3];
    #[allow(clippy::needless_range_loop)] // j indexes both the rhs unit vector and q's column
    for j in 0..3 {
        let mut m = [[0.0f64; 6]; 5];
        for r in 0..ns {
            m[r][..5].copy_from_slice(&a[r]);
            m[r][5] = if r == j { 1.0 } else { 0.0 };
        }
        let col = solve(m, ns)?;
        for r in 0..3 {
            q[r][j] = col[r];
        }
    }
    // Rotate the position covariance to ENU at the fix for HDOP/VDOP.
    let (lat, lon, _) = ecef2geodetic(x[0], x[1], x[2], Ellipsoid::WGS84);
    let (sp, cp, sl, cl) = (lat.sin(), lat.cos(), lon.sin(), lon.cos());
    let rot = [
        [-sl, cl, 0.0],           // east
        [-sp * cl, -sp * sl, cp], // north
        [cp * cl, cp * sl, sp],   // up
    ];
    let mut qenu = [0.0f64; 3];
    for (r, rr) in rot.iter().enumerate() {
        for j in 0..3 {
            for k in 0..3 {
                qenu[r] += rr[j] * q[j][k] * rr[k];
            }
        }
    }
    Some(WlsSolution {
        pos: x,
        cdt_m: cdt,
        isb_m: isb,
        sigma,
        hdop: (qenu[0] + qenu[1]).max(0.0).sqrt(),
        vdop: qenu[2].max(0.0).sqrt(),
    })
}

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
        let corr = get_sv_clock_correction(eph, rtm.epoch);
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
            (lat * 180.0 / PI, lon * 180.0 / PI, h)
        })
    }

    /// Returns true if a position was resolved this call.
    pub fn compute_position(&mut self, ts_sec: f64, ephs: &[RxEphemeris]) -> bool {
        // Refresh this solver's epoch store; the gnss-rtk callback objects
        // read it if the fallback path runs.
        *self.ephs.lock().unwrap() = ephs.to_vec();

        let mut meas: Vec<SvMeasurement> = Vec::with_capacity(ephs.len());

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

            // This SV's broadcast residual-error variance (SBAS UDRE + slant
            // GIVE), accumulated below and fed to the WLS as its weight prior.
            let mut sv_var_m2 = 0.0;
            // Ionosphere and troposphere both need a receiver position (pierce
            // point / elevation angle). We have one once there's a previous fix;
            // before that both corrections are 0 (a few metres, dwarfed by the
            // first-fix transient anyway).
            let (iono_m, tropo_m) = if let Some(rx_ecef) = self.last_fix_ecef {
                let sat = compute_sv_position_ecef(eph, now_gpst);
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
                    sbas_iono
                        .as_ref()
                        .and_then(|g| g.delay_var_m(lat, lon, elev, azim))
                        .map(|(d, v)| {
                            sv_var_m2 += v;
                            d
                        })
                        .unwrap_or_else(|| {
                            if iono_valid {
                                klobuchar_l1_delay_m(
                                    &iono_alpha,
                                    &iono_beta,
                                    lat,
                                    lon,
                                    elev,
                                    azim,
                                    gps_sod,
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
                eph.code_off_sec
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
            if let Some(c) = &sbas_corr {
                if let Some((prc, var)) = c.fast_prc_var_m(eph.sv, ts_sec) {
                    sbas_m += prc;
                    sv_var_m2 += var;
                }
                if let Some((dpos, dclk)) = c.long_term(eph.sv, eph.iode as u8, gps_sod) {
                    sbas_m += SPEED_OF_LIGHT * dclk;
                    if let Some(rx_ecef) = self.last_fix_ecef {
                        let s = compute_sv_position_ecef(eph, now_gpst);
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

            // A non-finite pseudorange (e.g. a NaN correction from a garbage prior
            // fix) would panic gnss-rtk's Duration math — drop the SV instead.
            if !pr_m.is_finite() {
                log::warn!("{}: dropping SV, non-finite pseudorange", eph.sv);
                continue;
            }

            // The finished measurement: SV at transmit time rotated into the
            // reception ECEF frame (Earth turns ωe·τ while the signal travels).
            let we = EARTH_ROTATION_RATE * pseudo_range_sec;
            let (cw, sw) = (we.cos(), we.sin());
            let s = compute_sv_position_ecef(eph, *t_tx);
            let m = SvMeasurement {
                sv: eph.sv,
                cn0: eph.cn0,
                pr_m,
                clk_m: clock_corr * SPEED_OF_LIGHT,
                svp: [cw * s.0 + sw * s.1, -sw * s.0 + cw * s.1, s.2],
                galileo: eph.sv.constellation == Constellation::Galileo,
                var_m2: sv_var_m2,
            };

            if let Some(truth) = truth_ecef {
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
            meas.push(m);
        }

        // The weighted+ISB WLS is the live solver (GNSS_SOLVER=rtk selects
        // gnss-rtk instead; it also remains the fallback if WLS fails).
        // Rationale + revert condition in `wls_solve`'s doc.
        if !std::env::var("GNSS_SOLVER").is_ok_and(|v| v == "rtk") {
            let x0 = self.last_fix_ecef.map_or([0.0; 3], |p| [p[0], p[1], p[2]]);
            let m: Vec<f64> = meas.iter().map(|m| m.pr_m + m.clk_m).collect();
            let svp: Vec<[f64; 3]> = meas.iter().map(|m| m.svp).collect();
            let gal: Vec<bool> = meas.iter().map(|m| m.galileo).collect();
            let var: Vec<f64> = meas.iter().map(|m| m.var_m2).collect();
            if let Some(sol) = wls_solve(&m, &svp, &gal, &var, x0) {
                let pos = Vector3::new(sol.pos[0], sol.pos[1], sol.pos[2]);
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
                return true;
            }
            log::warn!("WLS solve failed (underdetermined/singular); trying gnss-rtk");
        }

        // gnss-rtk fallback/cross-check: its pool is built only here, from
        // the same finished measurements.
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

    #[test]
    fn earth_mu_is_constellation_specific() {
        assert_eq!(earth_mu(SV::new(Constellation::Galileo, 1)), EARTH_MU_GAL);
        assert_eq!(earth_mu(SV::new(Constellation::GPS, 1)), EARTH_MU_GPS);
        assert_eq!(earth_mu(SV::new(Constellation::QZSS, 1)), EARTH_MU_GPS);
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
        let dt = get_sv_clock_correction(&e, e.toe_gpst);
        assert!(dt.abs() < 1e-6, "dt={dt}");
    }
}
