//! Broadcast-model evaluation: satellite orbits (Kepler) and clocks from
//! the decoded ephemeris, receiver-relative geometry, and the atmospheric
//! delay models. Pure functions of their inputs — no receiver state.

use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{Epoch, Vector3};
use map_3d::{Ellipsoid, ecef2aer, ecef2geodetic};

use crate::constants::{
    EARTH_MU_GAL, EARTH_MU_GPS, EARTH_ROTATION_RATE, HALF_WEEK_SECONDS, SECONDS_PER_WEEK,
    SPEED_OF_LIGHT,
};
use crate::ephemeris::Ephemeris as RxEphemeris;

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

fn eccentric_anomaly(eph: &RxEphemeris, t_k: f64) -> f64 {
    let n0 = (earth_mu(eph.sv) / eph.a.powi(3)).sqrt();
    let n = n0 + eph.deln;
    let mk = eph.m0 + n * t_k;

    let mut ek = mk;
    let mut ek_prev = 0.0;
    let mut n_iter = 0;

    // Newton's method on Kepler's equation; GNSS orbits are near-circular
    // (ecc < 0.03), so this converges in a handful of iterations. Hitting the
    // cap means it did NOT converge (corrupt ephemeris) — fail loudly below
    // rather than letting garbage flow into the SV position.
    while (ek - ek_prev).abs() > 1e-14 && n_iter < 30 {
        ek_prev = ek;
        ek = ek + (mk - ek + eph.ecc * ek.sin()) / (1.0 - eph.ecc * ek.cos());
        n_iter += 1;
    }
    assert!(
        n_iter < 30,
        "Kepler solve did not converge: ecc={}",
        eph.ecc
    );

    ek
}

fn normalize_week_seconds(mut dt: f64) -> f64 {
    let week = SECONDS_PER_WEEK as f64;
    if dt > HALF_WEEK_SECONDS {
        dt -= week;
    }
    if dt < -HALF_WEEK_SECONDS {
        dt += week;
    }
    dt
}

pub(crate) fn sv_clock_correction(eph: &RxEphemeris, t: Epoch) -> f64 {
    let f_rel = -2.0 * earth_mu(eph.sv).sqrt() / SPEED_OF_LIGHT.powi(2);

    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());
    let ecc_anomaly = eccentric_anomaly(eph, dte);
    let dtr = f_rel * eph.ecc * eph.a.sqrt() * ecc_anomaly.sin();

    let dtc = normalize_week_seconds((t - eph.toc_gpst).to_seconds());

    eph.f0 + eph.f1 * dtc + eph.f2 * dtc.powi(2) + dtr
}

pub(crate) fn compute_sv_position_ecef(eph: &RxEphemeris, t: Epoch) -> (f64, f64, f64) {
    let dte = normalize_week_seconds((t - eph.toe_gpst).to_seconds());

    log::debug!("{}: ---- now={t:?}", eph.sv);
    log::debug!("{}: ---- toe={:?} delta-t={dte} ", eph.sv, eph.toe_gpst);

    let ecc_anomaly = eccentric_anomaly(eph, dte);
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
    let (lat, lon, h) = ecef2geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2], Ellipsoid::WGS84);
    let (az, elev, _range) = ecef2aer(
        sat_ecef.0,
        sat_ecef.1,
        sat_ecef.2,
        lat,
        lon,
        h,
        Ellipsoid::WGS84,
    );
    // ecef2aer reports azimuth in [0, 2π); fold to (−π, π] — the convention the
    // rest of the receiver assumes (synth's azimuth-sector binning, the signed
    // sky-plot azimuth_deg).
    let azim = if az > PI { az - 2.0 * PI } else { az };
    (elev, azim)
}

/// Saastamoinen troposphere slant delay (metres) using a standard atmosphere.
///
/// Computes the zenith hydrostatic delay (ZHD) from standard-atmosphere
/// pressure at the given height, adds a simplified zenith wet delay (ZWD),
/// and maps to the slant path with 1/sin(elevation). Returns 0 for elevations
/// below 5° to avoid blow-up near the horizon.
///
/// Reference: Saastamoinen (1973); standard atmosphere (ICAO).
pub(crate) fn saastamoinen_tropo_m(lat: f64, h_m: f64, elev: f64) -> f64 {
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
pub(crate) fn klobuchar_l1_delay_m(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_mu_is_constellation_specific() {
        assert_eq!(earth_mu(SV::new(Constellation::Galileo, 1)), EARTH_MU_GAL);
        assert_eq!(earth_mu(SV::new(Constellation::GPS, 1)), EARTH_MU_GPS);
        assert_eq!(earth_mu(SV::new(Constellation::QZSS, 1)), EARTH_MU_GPS);
    }
}
