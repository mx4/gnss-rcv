use colored::Colorize;
use rustfft::num_complex::Complex64;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::Instant;

const BUFFER_SIZE: usize = 128 * 1024;

pub struct IQRecording {
    pub file_path: PathBuf,
    pub sample_rate: usize,
    pub iq_vec: Vec<Complex64>,
    pub format_int16: bool,
}

//https://wiki.gnuradio.org/index.php?title=File_Sink#Handling_File_Sink_data

impl IQRecording {
    pub fn new(file_path: PathBuf, sample_rate: usize, format_int16: bool) -> Self {
        Self {
            file_path,
            sample_rate,
            iq_vec: vec![],
            format_int16,
        }
    }
    pub fn read_iq_file(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let file = File::open(self.file_path.clone())?;
        let mut reader = BufReader::with_capacity(BUFFER_SIZE, &file);
        let mut n: u64 = 0;
        let ts = Instant::now();

        loop {
            let buf = reader.fill_buf()?;
            let len = buf.len();

            if len == 0 {
                break;
            }

            if self.format_int16 {
                for off in (0..len).step_by(4) {
                    let i = i16::from_le_bytes([buf[off + 0], buf[off + 1]]);
                    let q = i16::from_le_bytes([buf[off + 2], buf[off + 3]]);
                    self.iq_vec.push(Complex64 {
                        re: i as f64 / std::i16::MAX as f64,
                        im: q as f64 / std::i16::MAX as f64,
                    });
                }
            } else {
                for off in (0..len).step_by(8) {
                    let i = f32::from_le_bytes([
                        buf[off + 0],
                        buf[off + 1],
                        buf[off + 2],
                        buf[off + 3],
                    ]);
                    let q = f32::from_le_bytes([
                        buf[off + 4],
                        buf[off + 5],
                        buf[off + 6],
                        buf[off + 7],
                    ]);
                    assert!(-1.0 <= i && i <= 1.0);
                    assert!(-1.0 <= q && q <= 1.0);
                    self.iq_vec.push(Complex64 {
                        re: i as f64,
                        im: q as f64,
                    });
                }
            }
            n += 1;
            if n == 1000 {
                break;
            }
            reader.consume(len);
        }

        println!(
            "num_samples: {} -- {:.1} msec",
            format!("{}", self.iq_vec.len()).yellow(),
            self.iq_vec.len() as f64 * 1000.0 / self.sample_rate as f64,
        );
        let elapsed_msec = ts.elapsed().as_millis();
        let bw = n as f64 * BUFFER_SIZE as f64 * 1000.0 / 1024.0 / 1024.0 / elapsed_msec as f64;
        println!(
            "read_from_file: {} msec -- bandwidth: {:.1} MB/sec -- num_read_ops={}",
            elapsed_msec, bw, n
        );

        Ok(())
    }

    pub fn get_msec_sample(&self, off_msec: usize, num_msec: usize) -> Vec<Complex64> {
        let num_samples_per_msec = self.sample_rate / 1000;
        let num_samples = num_msec * num_samples_per_msec;
        let lo = off_msec * num_samples_per_msec;
        let hi = off_msec * num_samples_per_msec + num_samples;
        println!(
            "get_msec_sample: off_msec={} duration={} msec num_samples={} lo={} hi={}",
            off_msec, num_msec, num_samples, lo, hi
        );
        assert!(hi <= self.iq_vec.len());
        let w: Vec<_> = self.iq_vec[lo..hi].iter().cloned().collect();
        w
    }
}
