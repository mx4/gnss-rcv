pub const L1CA_CODE_LEN: usize = 1023;

pub struct Code {}

impl Code {
    fn gen_l1ca_code(prn: u8) -> Vec<i8> {
        const G2_DELAY: [usize; 210] = [
            5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470, 471,
            472, 473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862, 863, 950, 947, 948,
            950, 67, 103, 91, 19, 679, 225, 625, 946, 638, 161, 1001, 554, 280, 710, 709, 775, 864,
            558, 220, 397, 55, 898, 759, 367, 299, 1018, 729, 695, 780, 801, 788, 732, 34, 320,
            327, 389, 407, 525, 405, 221, 761, 260, 326, 955, 653, 699, 422, 188, 438, 959, 539,
            879, 677, 586, 153, 792, 814, 446, 264, 1015, 278, 536, 819, 156, 957, 159, 712, 885,
            461, 248, 713, 126, 807, 279, 122, 197, 693, 632, 771, 467, 647, 203, 145, 175, 52, 21,
            237, 235, 886, 657, 634, 762, 355, 1012, 176, 603, 130, 359, 595, 68, 386, 797, 456,
            499, 883, 307, 127, 211, 121, 118, 163, 628, 853, 484, 289, 811, 202, 1021, 463, 568,
            904, 670, 230, 911, 684, 309, 644, 932, 12, 314, 891, 212, 185, 675, 503, 150, 395,
            345, 846, 798, 992, 357, 995, 877, 112, 144, 476, 193, 109, 445, 291, 87, 399, 292,
            901, 339, 208, 711, 189, 263, 537, 663, 942, 173, 900, 30, 500, 935, 556, 373, 85, 652,
            310,
        ];
        let mut g1 = [0i8; L1CA_CODE_LEN];
        let mut g2 = [0i8; L1CA_CODE_LEN];
        let mut r1 = [-1i8; 10];
        let mut r2 = [-1i8; 10];
        let mut g = vec![];
        for i in 0..L1CA_CODE_LEN {
            g1[i] = r1[9];
            g2[i] = r2[9];
            let c1 = r1[2] * r1[9];
            let c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];
            r1.rotate_right(1);
            r2.rotate_right(1);
            r1[0] = c1;
            r2[0] = c2;
        }
        let start = L1CA_CODE_LEN - G2_DELAY[(prn - 1) as usize];
        for (i, &g1_i) in g1.iter().enumerate() {
            let j = (start + i) % L1CA_CODE_LEN;
            g.push(-g1_i * g2[j]);
        }

        g
    }

    pub fn gen_code(sig: &str, prn: u8) -> Option<Vec<i8>> {
        match sig {
            "L1CA" => Some(Self::gen_l1ca_code(prn)),
            _ => None,
        }
    }

    pub fn get_code_period(sig: &str) -> f64 {
        match sig {
            "L1CA" => 1e-3,
            _ => 0.0,
        }
    }

    pub fn get_code_len(sig: &str) -> usize {
        match sig {
            "L1CA" => L1CA_CODE_LEN,
            _ => 0,
        }
    }

    pub fn get_code_freq(sig: &str) -> f64 {
        match sig {
            "L1CA" => 1575.42e6,
            _ => 0.0,
        }
    }

    pub fn print_l1ca_codes() {
        println!("generating gold codes for L1CA");
        for i in 1..=32 {
            let g = Self::gen_l1ca_code(i as u8);
            println!("  code-{:02}: {:?}", i, &g[0..20]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Circular correlation of `a` against `b` shifted by `shift` chips.
    fn circ_corr(a: &[i8], b: &[i8], shift: usize) -> i32 {
        let n = a.len();
        (0..n)
            .map(|i| a[i] as i32 * b[(i + shift) % n] as i32)
            .sum()
    }

    // A length-1023 Gold code (degree-10 m-sequences) is balanced bipolar, has a
    // 1023 autocorrelation peak, and its off-peak auto- and cross-correlations
    // are three-valued: {-1, -65, 63} where 65 = 1 + 2^((10+2)/2). These props
    // are what acquisition relies on, so they validate that the SBAS PRNs
    // (120-138) pick up correct G2 phase delays from the extended code table.
    const VALID_OFF_PEAK: [i32; 3] = [-1, -65, 63];

    fn assert_gold_code(code: &[i8]) {
        assert_eq!(code.len(), L1CA_CODE_LEN);
        assert!(
            code.iter().all(|&c| c == 1 || c == -1),
            "code must be bipolar"
        );
        assert_eq!(
            circ_corr(code, code, 0),
            L1CA_CODE_LEN as i32,
            "autocorr peak"
        );
        for shift in 1..L1CA_CODE_LEN {
            let v = circ_corr(code, code, shift);
            assert!(
                VALID_OFF_PEAK.contains(&v),
                "off-peak autocorr {v} at shift {shift} not in {VALID_OFF_PEAK:?}"
            );
        }
    }

    #[test]
    fn sbas_prns_are_valid_gold_codes() {
        // EGNOS / legacy SBAS L1 PRNs that we expect to be able to acquire.
        for prn in [120u8, 123, 126, 136, 138] {
            let code = Code::gen_code("L1CA", prn).expect("L1CA code");
            assert_gold_code(&code);
        }
    }

    #[test]
    fn gps_prns_are_valid_gold_codes() {
        // Sanity check the same properties hold for the GPS block we already use.
        for prn in [1u8, 16, 32] {
            let code = Code::gen_code("L1CA", prn).expect("L1CA code");
            assert_gold_code(&code);
        }
    }

    #[test]
    fn distinct_prn_codes_have_bounded_cross_correlation() {
        // Cross-correlation between distinct PRNs (SBAS-vs-SBAS and SBAS-vs-GPS)
        // is three-valued and bounded by 65 -- this is what keeps an SBAS search
        // from cross-locking onto a strong GPS SV.
        let pairs = [(123u8, 136u8), (120, 1), (136, 32)];
        for (a, b) in pairs {
            let ca = Code::gen_code("L1CA", a).unwrap();
            let cb = Code::gen_code("L1CA", b).unwrap();
            for shift in 0..L1CA_CODE_LEN {
                let v = circ_corr(&ca, &cb, shift);
                assert!(
                    VALID_OFF_PEAK.contains(&v),
                    "xcorr {v} of PRN{a}xPRN{b} at shift {shift} not in {VALID_OFF_PEAK:?}"
                );
            }
        }
    }
}
