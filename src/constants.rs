//! Scale constants for decoding broadcast navigation parameters.

/// 2^-n as an exact f64 (n < 64). The navigation-message LSB scales are all
/// negative powers of two — exact in f64 at any magnitude — so we compute them
/// rather than transcribe 16-digit decimals (error-prone, and trips
/// `clippy::excessive_precision`). Const-folded, so zero runtime cost.
const fn p2(n: u32) -> f64 {
    1.0 / (1u64 << n) as f64
}

/// L1/E1 carrier frequency (Hz) — the one band this receiver works in (GPS
/// L1 C/A, Galileo E1, SBAS L1 and QZSS L1 all share it).
pub const L1_HZ: f64 = 1_575_420_000.0;

/// Seconds in a GPS/GST week — the modulus for time-of-week arithmetic.
pub const SECONDS_PER_WEEK: u32 = 7 * 24 * 60 * 60; // 604_800
/// Half a week — the wrap threshold when folding a signed week-seconds delta
/// back into ±½ week (see `models::normalize_week_seconds`).
pub const HALF_WEEK_SECONDS: f64 = SECONDS_PER_WEEK as f64 / 2.0;

pub const P2_2: f64 = p2(2); // Galileo iono ai0
pub const P2_5: f64 = p2(5);
pub const P2_8: f64 = p2(8); // Galileo iono ai1
pub const P2_11: f64 = p2(11);
pub const P2_15: f64 = p2(15); // Galileo iono ai2
pub const P2_19: f64 = p2(19);
pub const P2_20: f64 = p2(20);
pub const P2_21: f64 = p2(21);
pub const P2_23: f64 = p2(23);
pub const P2_24: f64 = p2(24);
pub const P2_27: f64 = p2(27);
pub const P2_29: f64 = p2(29);
pub const P2_30: f64 = p2(30);
pub const P2_31: f64 = p2(31);
pub const P2_32: f64 = p2(32); // Galileo BGD
pub const P2_33: f64 = p2(33);
pub const P2_34: f64 = p2(34); // Galileo af0
pub const P2_35: f64 = p2(35); // Galileo GGTO A0G
pub const P2_38: f64 = p2(38);
pub const P2_39: f64 = p2(39); // SBAS long-term daf1
pub const P2_43: f64 = p2(43);
pub const P2_46: f64 = p2(46); // Galileo af1
pub const P2_50: f64 = p2(50);
pub const P2_51: f64 = p2(51); // Galileo GGTO A1G
pub const P2_55: f64 = p2(55);
pub const P2_59: f64 = p2(59); // Galileo af2

/// Semicircles → radians for broadcast orbit/clock parameters. Deliberately the
/// *truncated* π mandated by IS-GPS-200 (and the Galileo OS SIS ICD) — NOT
/// `std::f64::consts::PI`. The ground segment fits the ephemeris using exactly
/// this value, so the receiver must use it too, or it diverges from the broadcast
/// convention (~7e-15 rad). Shared by GPS and Galileo. Do not "increase its
/// precision". The true π is still used for carrier/FFT DSP elsewhere.
#[allow(clippy::approx_constant)] // intentional ICD value, not an approximation of PI
pub const SC2RAD: f64 = 3.141_592_653_589_8;

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;
/// WGS-84 µ mandated for the broadcast-ephemeris user algorithm (IS-GPS-200
/// Table 20-IV). Deliberately the *original* WGS-84 value, NOT the refined
/// 3.986004418e14: the control segment fits the broadcast orbit with this µ, so
/// the receiver must match it (same convention trap as `SC2RAD`).
pub const EARTH_MU_GPS: f64 = 3.986005e14;
pub const EARTH_MU_GAL: f64 = 3.986004418e14; // GTRF µ (Galileo OS SIS ICD)
pub const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5; // shared GPS/Galileo Ωe
