use crate::{
    bits::{getbitu, getbitu2},
    constants::{P2_11, P2_19, P2_20, P2_21, P2_23, P2_38, SC2RAD},
};

#[derive(Default, Clone, Debug)]
pub struct Almanac {
    pub sat: u32,    /* satellite number */
    pub svh: u32,    /* sv health (0:ok) */
    pub svconf: u32, /* as and sv config */
    //toa: Epoch,
    /* SV orbit parameters */
    pub a: f64,
    pub e: f64,
    pub omg0: f64,
    pub omg: f64,
    pub m0: f64,
    pub omg_dot: f64,
    pub week: u32, /* GPS/QZS: gps week, GAL: galileo week */
    pub toas: u32, /* Toa (s) in week */
    pub f0: f64,   /* SV clock parameters (af0,af1) */
    pub f1: f64,
}

impl Almanac {
    pub fn nav_decode_alm(&mut self, buf: &[u8], svid: u32) {
        assert!(svid > 0 && svid <= 32);
        self.sat = svid;
        self.e = getbitu(buf, 68, 16) as f64 * P2_21;
        self.toas = getbitu(buf, 90, 8) * 4096;
        let _delta_i = getbitu(buf, 98, 16) as f64 * P2_19 * SC2RAD;

        self.omg_dot = getbitu(buf, 120, 16) as f64 * P2_38 * SC2RAD;
        self.svh = getbitu(buf, 136, 8);
        let sqrt_a = getbitu(buf, 150, 24) as f64 * P2_11;
        self.a = sqrt_a * sqrt_a;
        self.omg0 = getbitu(buf, 180, 24) as f64 * P2_23 * SC2RAD;
        self.omg = getbitu(buf, 210, 24) as f64 * P2_23 * SC2RAD;
        self.m0 = getbitu(buf, 240, 24) as f64 * P2_23 * SC2RAD;
        self.f0 = getbitu2(buf, 270, 8, 289, 3) as f64 * P2_20;
        self.f1 = getbitu(buf, 278, 11) as f64 * P2_38;
    }
}
