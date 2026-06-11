//! The live weighted-least-squares solver with an inter-system-bias state —
//! the workaround for gnss-rtk 0.8's equal weighting and single clock state.
//! See [`wls_solve`] for the revert condition once upstream ships weights.

use gnss_rs::constellation::Constellation;
use map_3d::{Ellipsoid, ecef2geodetic};

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
