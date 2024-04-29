use bytesize::ByteSize;
use colored::Colorize;
use rustfft::num_complex::Complex64;
use std::error::Error;
use std::fmt;
use std::fs::File;
use std::io::Seek;
use std::io::SeekFrom;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Instant;

pub enum IQFileType {
    TypePairFloat32,
    TypePairInt16,
    TypePairInt8,
    TypeOneInt8,
}


#[derive(Default, Clone)]
pub struct IQSample {
    pub iq_vec: Vec<Complex64>,
    off_msec: usize,
}

impl FromStr for IQFileType {
    type Err = Box<dyn Error>;
    fn from_str(input: &str) -> Result<IQFileType, Self::Err> {
        match input {
            "2xf32" => Ok(IQFileType::TypePairFloat32),
            "2xi16" => Ok(IQFileType::TypePairInt16),
            "2xi8" => Ok(IQFileType::TypePairInt8),
            "i8" => Ok(IQFileType::TypeOneInt8),
            _ => Err(format!("Failed to parse {}", input).into()),
        }
    }
}

impl fmt::Display for IQFileType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            IQFileType::TypePairFloat32 => write!(f, "2xf32"),
            IQFileType::TypePairInt16 => write!(f, "2xi16"),
            IQFileType::TypePairInt8 => write!(f, "2xi8"),
            IQFileType::TypeOneInt8 => write!(f, "i8"),
        }
    }
}

pub struct IQRecording {
    pub file_path: PathBuf,
    pub sample_rate: usize,
    pub file_type: IQFileType,
}

impl IQRecording {
    pub fn new(file_path: PathBuf, sample_rate: usize, file_type: IQFileType) -> Self {
        let file_size = file_path.metadata().unwrap().len();
        let sample_size = Self::get_sample_size_bytes(&file_type) as f64;
        let recording_duration_sec = file_size as f64 / sample_rate as f64 / sample_size;

        println!(
            "file: {} -- {} {} duration: {:.1} secs",
            file_path.display().to_string().green(),
            file_type,
            ByteSize::b(file_size).to_string_as(false).bold(),
            recording_duration_sec
        );
        Self {
            file_path,
            sample_rate,
            file_type,
        }
    }

    fn get_sample_size_bytes(file_type: &IQFileType) -> usize {
        match file_type {
            IQFileType::TypePairInt8 => 2 * 1,
            IQFileType::TypeOneInt8 => 1,
            IQFileType::TypePairInt16 => 2 * 2,
            IQFileType::TypePairFloat32 => 2 * 4,
        }
    }
    pub fn read_iq_file(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<IQSample, Box<dyn std::error::Error>> {
        let file = File::open(self.file_path.clone())?;
        let sample_size = Self::get_sample_size_bytes(&self.file_type);
        let buf_size = sample_size * num_samples;
        let mut reader = BufReader::with_capacity(buf_size, &file);
        let mut n: usize = 0;
        let ts = Instant::now();
        let mut iq_vec = vec![];

        let off_file = off_samples as i64 * sample_size as i64;

        log::debug!(
            "read_iq_file: off_samples={} num_samples={}",
            off_samples,
            num_samples
        );

        let _ = reader.seek(SeekFrom::Current(off_file)).unwrap();

        loop {
            let buf = reader.fill_buf()?;
            let len = buf.len();

            if len == 0 {
                break;
            }

            match self.file_type {
                IQFileType::TypePairInt8 => {
                    for off in (0..len).step_by(2) {
                        iq_vec.push(Complex64 {
                            re: buf[off + 0] as i8 as f64 / std::i8::MAX as f64,
                            im: buf[off + 1] as i8 as f64 / std::i8::MAX as f64,
                        });
                        n += 1;
                        if n >= num_samples {
                            break;
                        }
                    }
                }
                IQFileType::TypeOneInt8 => {
                    for off in 0..len {
                        iq_vec.push(Complex64 {
                            re: buf[off] as i8 as f64 / std::i8::MAX as f64,
                            im: 0.0,
                        });
                        n += 1;
                        if n >= num_samples {
                            break;
                        }
                    }
                }
                IQFileType::TypePairInt16 => {
                    for off in (0..len).step_by(4) {
                        let i = i16::from_le_bytes([buf[off + 0], buf[off + 1]]);
                        let q = i16::from_le_bytes([buf[off + 2], buf[off + 3]]);
                        iq_vec.push(Complex64 {
                            re: i as f64 / std::i16::MAX as f64,
                            im: q as f64 / std::i16::MAX as f64,
                        });
                        n += 1;
                        if n >= num_samples {
                            break;
                        }
                    }
                }
                IQFileType::TypePairFloat32 => {
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
                        iq_vec.push(Complex64 {
                            re: i as f64,
                            im: q as f64,
                        });
                        n += 1;
                        if n >= num_samples {
                            break;
                        }
                    }
                }
            }
            if n >= num_samples {
                break;
            }
            reader.consume(len);
        }
        if n < num_samples {
            return Err("end of file".into());
        }
        assert_eq!(n, num_samples);

        log::debug!(
            "num_samples: {} -- {:.1} msec",
            format!("{}", iq_vec.len()).yellow(),
            iq_vec.len() as f64 * 1000.0 / self.sample_rate as f64,
        );
        let bw = n as f64 * buf_size as f64 / 1024.0 / 1024.0 / ts.elapsed().as_secs_f64();
        log::debug!(
            "read_from_file: {} msec -- bandwidth: {:.1} MB/sec -- num_read_ops={}",
            ts.elapsed().as_millis(),
            bw,
            n
        );

        Ok(IQSample{ iq_vec, off_msec: off_samples * 1000 / self.sample_rate })
    }
}
