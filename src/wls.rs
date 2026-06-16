//! The live weighted-least-squares solver with an inter-system-bias state and
//! RAIM fault detection & exclusion — the workaround for gnss-rtk 0.8's equal
//! weighting and single clock state. See [`wls_solve`] for the revert
//! condition once upstream ships weights.

use gnss_rs::constellation::Constellation;
use map_3d::{Ellipsoid, ecef2geodetic};

/// A live weighted-least-squares fix: position plus the estimated clock and
/// inter-system bias, the self-calibrated per-constellation sigmas, geometry
/// DOPs from the unweighted normal matrix, and any RAIM-excluded outliers.
pub(crate) struct WlsSolution {
    pub pos: [f64; 3],
    pub cdt_m: f64,
    pub isb_m: f64,
    pub sigma: std::collections::HashMap<Constellation, f64>,
    pub hdop: f64,
    pub vdop: f64,
    /// Indices (into the caller's measurement arrays) that RAIM dropped as
    /// outliers before this solution was computed; empty for a clean pool.
    pub excluded: Vec<usize>,
}

/// A-priori 1σ pseudorange measurement noise (m) for the RAIM integrity test.
/// Deliberately a *fixed nominal* value, not the self-calibrated weighting σ:
/// a gross outlier inflates the self-calibrated σ (a 220 km blunder drove
/// σ_GPS to ~68 km on the ION HackRF set), which would then mask the very fault
/// we are testing for. The test compares each residual against the noise a
/// healthy single-frequency fix *should* show.
const RAIM_SIGMA_M: f64 = 10.0;

/// w-test gate: exclude the SV whose normalized residual exceeds this many
/// nominal σ. 6σ ⇒ a per-SV false-alarm rate ~2e-9, so a healthy SV is
/// essentially never dropped, while a km-scale blunder trips it by orders of
/// magnitude. (The denominator omits the residual-sensitivity factor √sᵢᵢ ≤ 1,
/// which only makes the test more conservative — never more trigger-happy.)
const RAIM_GATE: f64 = 6.0;

/// State count for a pool: position+clock (4), plus the Galileo inter-system
/// bias (5) when both constellations are present with a redundant measurement.
/// The ISB state is only observable with both constellations and one spare.
fn state_count(active: &[usize], gal: &[bool]) -> usize {
    let mixed = active.iter().any(|&i| gal[i]) && active.iter().any(|&i| !gal[i]);
    if mixed && active.len() >= 5 { 5 } else { 4 }
}

/// Weighted Gauss-Newton solve over (x, y, z, c·dt, c·ISB_gal) with RAIM
/// fault detection & exclusion — the live solver. It adds the things gnss-rtk
/// 0.8 lacks for a mixed pool: per-measurement weights (its measurement sigma
/// is literally `1.0 // TODO`), an inter-system-bias state so Galileo's common
/// clock offset stops leaking into position, and outlier rejection. Weights are
/// self-calibrated: solve once equal-weighted, set each constellation's sigma to
/// its residual RMS, solve again — no hand-tuned constants. **Workaround
/// status**: once a gnss-rtk release accepts measurement sigmas (and a second
/// clock state), hand the weighting back upstream and let this revert to a
/// cross-check (GNSS_SOLVER=rtk selects gnss-rtk meanwhile).
///
/// RAIM: solve over the full pool, then — while the pool still has redundancy
/// (more measurements than states) — drop the satellite whose residual is the
/// largest multiple of [`RAIM_SIGMA_M`] when that exceeds [`RAIM_GATE`], and
/// re-solve. A single gross outlier (cross-correlation false lock, bad
/// ephemeris) is otherwise least-squares-smeared across the fix: on the ION
/// HackRF set one such SV (a ~220 km pseudorange blunder) pushed the fix 63 km
/// horizontally and 37 km up. Excluded indices land in [`WlsSolution::excluded`].
///
/// `meas[i]` is the fully corrected pseudorange + c·sv_clock (m), `svp[i]`
/// the SV ECEF position at transmit time rotated into the reception frame
/// (Sagnac), `gal[i]` its constellation flag, `var0[i]` its broadcast
/// residual-error variance prior (m²). `x0` seeds the iteration (last fix, or
/// zeros — Gauss-Newton converges from the geocentre for GNSS geometry).
/// Returns `None` for underdetermined pools or a non-finite solution (caller
/// falls back to gnss-rtk).
pub(crate) fn wls_solve(
    meas: &[f64],
    svp: &[[f64; 3]],
    gal: &[bool],
    var0: &[f64],
    x0: [f64; 3],
) -> Option<WlsSolution> {
    let n = meas.len();
    let mut active: Vec<usize> = (0..n).collect();
    let mut excluded: Vec<usize> = Vec::new();
    loop {
        let (mut sol, resid) = solve_once(&active, meas, svp, gal, var0, x0)?;
        let ns = state_count(&active, gal);
        // With no redundancy there is nothing to test the residuals against;
        // accept the solution as-is.
        if active.len() <= ns {
            sol.excluded = excluded;
            return Some(sol);
        }
        // Worst normalized residual (a-priori σ plus this SV's broadcast prior).
        let mut worst = (0usize, 0.0f64);
        for (k, &r) in resid.iter().enumerate() {
            let s2 = RAIM_SIGMA_M.powi(2) + var0[active[k]];
            let w = r.abs() / s2.sqrt();
            if w > worst.1 {
                worst = (k, w);
            }
        }
        if worst.1 > RAIM_GATE {
            excluded.push(active[worst.0]);
            active.remove(worst.0);
            continue;
        }
        sol.excluded = excluded;
        return Some(sol);
    }
}

/// One weighted Gauss-Newton solve over the `active` subset of the pool. Returns
/// the fix plus the per-SV post-fit residuals (m, in `active` order) the RAIM
/// loop tests. Indices in `active` address the full `meas`/`svp`/`gal`/`var0`
/// arrays. Weights are self-calibrated; see [`wls_solve`].
fn solve_once(
    active: &[usize],
    meas: &[f64],
    svp: &[[f64; 3]],
    gal: &[bool],
    var0: &[f64],
    x0: [f64; 3],
) -> Option<(WlsSolution, Vec<f64>)> {
    use std::collections::HashMap;
    let na = active.len();
    let ns = state_count(active, gal);
    if na < ns {
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
            for &i in active {
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
        for &i in active {
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

    // Post-fit residuals (in `active` order) for the RAIM integrity test.
    let mut resid = Vec::with_capacity(na);
    for &i in active {
        let d = [svp[i][0] - x[0], svp[i][1] - x[1], svp[i][2] - x[2]];
        let rho = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        resid.push(meas[i] - (rho + cdt + if gal[i] { isb } else { 0.0 }));
    }

    // Geometry DOPs from the *unweighted* normal matrix at the solution (DOP
    // is a geometry factor; weighting it would conflate signal quality in).
    let mut a = [[0.0f64; 5]; 5];
    for &i in active {
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
    Some((
        WlsSolution {
            pos: x,
            cdt_m: cdt,
            isb_m: isb,
            sigma,
            hdop: (qenu[0] + qenu[1]).max(0.0).sqrt(),
            vdop: qenu[2].max(0.0).sqrt(),
            excluded: Vec::new(),
        },
        resid,
    ))
}

/// Post-fit range-rate residual (m/s) above which a TDCP measurement is treated
/// as a cycle slip / bad SV and excluded. Carrier-phase rate noise is cm/s once
/// the receiver velocity is in the model, so 2 m/s is hugely conservative —
/// it only catches gross slips, never a fast-but-clean receiver (its motion is
/// absorbed by the velocity state, not the residual).
const TDCP_SLIP_GATE_MPS: f64 = 2.0;

/// A time-differenced carrier-phase (TDCP) velocity solution: receiver velocity
/// in ECEF (m/s), the receiver clock drift (m/s), the SV count used, and how
/// many SVs were dropped as cycle-slip outliers.
pub(crate) struct TdcpSolution {
    pub vel_ecef: [f64; 3],
    pub clock_drift_mps: f64,
    pub n_sv: usize,
    pub excluded: usize,
}

/// 4×4 Gauss-Jordan with partial pivoting (velocity + clock-drift state); `None`
/// on a vanishing pivot.
fn solve4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Option<[f64; 4]> {
    for col in 0..4 {
        let p = (col..4).max_by(|&r1, &r2| a[r1][col].abs().total_cmp(&a[r2][col].abs()))?;
        if a[p][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, p);
        b.swap(col, p);
        let (pivot, bp) = (a[col], b[col]);
        for row in 0..4 {
            if row != col {
                let f = a[row][col] / pivot[col];
                for (k, pk) in pivot.iter().enumerate().skip(col) {
                    a[row][k] -= f * pk;
                }
                b[row] -= f * bp;
            }
        }
    }
    Some([
        b[0] / a[0][0],
        b[1] / a[1][1],
        b[2] / a[2][2],
        b[3] / a[3][3],
    ])
}

/// Solve for receiver velocity from per-SV range-rate observations. The caller
/// has already differenced each SV's carrier phase across two epochs and removed
/// the predicted SV-motion range change and SV-clock drift, leaving
///   rate_i = −ê_i · v_rx + c·ḋt_rx + noise,   ê_i = (svp_i − x)/|svp_i − x|.
/// Same geometry rows as the position solve at [`solve_once`], but the observable
/// is a rate and the unknowns are velocity + clock drift; the integer carrier
/// ambiguity cancelled in the caller's difference, so there is nothing to fix.
/// Weighted by linear C/N0; a surviving cycle slip is rejected RAIM-style (drop
/// the worst post-fit residual while the pool stays redundant). `None` for fewer
/// than 4 usable SVs or a singular geometry.
pub(crate) fn tdcp_solve(
    rate: &[f64],
    svp: &[[f64; 3]],
    x: [f64; 3],
    cn0: &[f64],
) -> Option<TdcpSolution> {
    let mut active: Vec<usize> = (0..rate.len()).collect();
    let mut excluded = 0usize;
    loop {
        if active.len() < 4 {
            return None;
        }
        // Weighted normal equations over [vx, vy, vz, c·ḋt_rx].
        let (mut a, mut b) = ([[0.0f64; 4]; 4], [0.0f64; 4]);
        let mut rows: Vec<[f64; 4]> = Vec::with_capacity(active.len());
        for &i in &active {
            let d = [svp[i][0] - x[0], svp[i][1] - x[1], svp[i][2] - x[2]];
            let rho = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let h = [-d[0] / rho, -d[1] / rho, -d[2] / rho, 1.0];
            let w = 10f64.powf(cn0[i].max(0.0) / 10.0);
            for j in 0..4 {
                b[j] += w * h[j] * rate[i];
                for k in 0..4 {
                    a[j][k] += w * h[j] * h[k];
                }
            }
            rows.push(h);
        }
        let s = solve4(a, b)?;
        let resid: Vec<f64> = active
            .iter()
            .zip(&rows)
            .map(|(&i, h)| rate[i] - (h[0] * s[0] + h[1] * s[1] + h[2] * s[2] + h[3] * s[3]))
            .collect();
        // RAIM: drop a gross cycle-slip outlier while redundancy remains.
        if active.len() > 4 {
            let worst = resid
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                .map(|(k, r)| (k, r.abs()))
                .unwrap();
            if worst.1 > TDCP_SLIP_GATE_MPS {
                active.remove(worst.0);
                excluded += 1;
                continue;
            }
        }
        if !(s.iter().all(|v| v.is_finite())) {
            return None;
        }
        return Some(TdcpSolution {
            vel_ecef: [s[0], s[1], s[2]],
            clock_drift_mps: s[3],
            n_sv: active.len(),
            excluded,
        });
    }
}
