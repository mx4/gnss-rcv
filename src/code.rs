pub const L1CA_CODE_LEN: usize = 1023;

pub struct Code {}

impl Code {
    fn gen_l1ca_code(prn: u8) -> Vec<i8> {
        const G2_DELAY: [usize; 210] = [
            005, 006, 007, 008, 017, 018, 139, 140, 141, 251, /*   1- 10 */
            252, 254, 255, 256, 257, 258, 469, 470, 471, 472, /*  11- 20 */
            473, 474, 509, 512, 513, 514, 515, 516, 859, 860, /*  21- 30 */
            861, 862, 863, 950, 947, 948, 950, 067, 103, 091, /*  31- 40 */
            019, 679, 225, 625, 946, 638, 161, 1001, 554, 280, /*  41- 50 */
            710, 709, 775, 864, 558, 220, 397, 055, 898, 759, /*  51- 60 */
            367, 299, 1018, 729, 695, 780, 801, 788, 732, 34, /*  61- 70 */
            320, 327, 389, 407, 525, 405, 221, 761, 260, 326, /*  71- 80 */
            955, 653, 699, 422, 188, 438, 959, 539, 879, 677, /*  81- 90 */
            586, 153, 792, 814, 446, 264, 1015, 278, 536, 819, /*  91-100 */
            156, 957, 159, 712, 885, 461, 248, 713, 126, 807, /* 101-110 */
            279, 122, 197, 693, 632, 771, 467, 647, 203, 145, /* 111-120 */
            175, 052, 021, 237, 235, 886, 657, 634, 762, 355, /* 121-130 */
            1012, 176, 603, 130, 359, 595, 68, 386, 797, 456, /* 131-140 */
            499, 883, 307, 127, 211, 121, 118, 163, 628, 853, /* 141-150 */
            484, 289, 811, 202, 1021, 463, 568, 904, 670, 230, /* 151-160 */
            911, 684, 309, 644, 932, 012, 314, 891, 212, 185, /* 161-170 */
            675, 503, 150, 395, 345, 846, 798, 992, 357, 995, /* 171-180 */
            877, 112, 144, 476, 193, 109, 445, 291, 87, 399, /* 181-190 */
            292, 901, 339, 208, 711, 189, 263, 537, 663, 942, /* 191-200 */
            173, 900, 030, 500, 935, 556, 373, 085, 652, 310, /* 201-210 */
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
        let mut j = L1CA_CODE_LEN - G2_DELAY[(prn - 1) as usize];
        for i in 0..L1CA_CODE_LEN {
            let v = -g1[i] * g2[j % L1CA_CODE_LEN];
            g.push(v);
            j += 1;
        }

        g
    }

    pub fn gen_code(sig: &str, prn: u8) -> Option<Vec<i8>> {
        match sig {
            "L1CA" => return Some(Self::gen_l1ca_code(prn)),
            _ => return None,
        }
    }

    pub fn get_code_period(sig: &str) -> f64 {
        match sig {
            "L1CA" => 1e-3,
            _ => 0.0,
        }
    }

    pub fn get_code_len(sig: &str) -> f64 {
        match sig {
            "L1CA" => L1CA_CODE_LEN as f64,
            _ => 0.0,
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
