pub const P2_5: f64 = 0.03125; /* 2^-5 */
pub const P2_11: f64 = 4.882_812_5e-4; /* 2^-11 */
pub const P2_19: f64 = 1.907_348_632_812_5e-6; /* 2^-19 */
pub const P2_20: f64 = 9.536_743_164_062_5e-7; /* 2^-20 */
pub const P2_21: f64 = 4.768_371_582_031_25e-7; /* 2^-21 */
pub const P2_23: f64 = 1.192_092_895_507_81e-7; /* 2^-23 */
pub const P2_24: f64 = 5.960_464_477_539_063e-8; /* 2^-24 */
pub const P2_27: f64 = 7.450_580_596_923_828e-9; /* 2^-27 */
pub const P2_29: f64 = 1.862_645_149_230_957e-9; /* 2^-29 */
pub const P2_30: f64 = 9.313_225_746_154_785e-10; /* 2^-30 */
pub const P2_31: f64 = 4.656_612_873_077_393e-10; /* 2^-31 */
pub const P2_32: f64 = 2.328_306_436_538_696e-10; /* 2^-32 (Galileo BGD) */
pub const P2_33: f64 = 1.164_153_218_269_348e-10; /* 2^-33 */
pub const P2_34: f64 = 5.820_766_091_346_741e-11; /* 2^-34 (Galileo af0) */
pub const P2_38: f64 = 3.637_978_807_091_71e-12; /* 2^-38 */
pub const P2_43: f64 = 1.136_868_377_216_16e-13; /* 2^-43 */
pub const P2_46: f64 = 1.421_085_471_520_2e-14; /* 2^-46 (Galileo af1) */
pub const P2_50: f64 = 8.881_784_197_001_252e-16; /* 2^-50 */
pub const P2_55: f64 = 2.775_557_561_562_891e-17; /* 2^-55 */
pub const P2_59: f64 = 1.734_723_475_976_807e-18; /* 2^-59 (Galileo af2) */

/// Semicircles → radians for broadcast orbit/clock parameters. Deliberately the
/// *truncated* π mandated by IS-GPS-200 (and the Galileo OS SIS ICD) — NOT
/// `std::f64::consts::PI`. The ground segment fits the ephemeris using exactly
/// this value, so the receiver must use it too, or it diverges from the broadcast
/// convention (~7e-15 rad). Shared by GPS and Galileo (= gnss-sdr `GNSS_PI`,
/// RTKLIB `SC2RAD`). Do not "increase its precision". The true π is still used
/// for carrier/FFT DSP elsewhere.
#[allow(clippy::approx_constant)] // intentional ICD value, not an approximation of PI
pub const SC2RAD: f64 = 3.141_592_653_589_8;

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;
pub const EARTH_MU_GPS: f64 = 3.9860058e14; // earth gravitational constant
pub const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;
