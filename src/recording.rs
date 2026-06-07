use bytesize::ByteSize;
use colored::Colorize;
use rustfft::num_complex::Complex64;
use std::error::Error;
use std::fmt;
use std::fs::File;
use std::io::BufReader;
use std::io::Read;
use std::io::Seek;
use std::io::SeekFrom;
use std::path::Path;
use std::path::PathBuf;
use std::str::FromStr;

use crate::receiver::IQReader;

#[derive(Clone)]
pub enum IQFileType {
    TypePairFloat32,
    TypePairInt16,
    TypeRtlSdrFile,
    TypeOneInt8,
}

impl FromStr for IQFileType {
    type Err = Box<dyn Error>;
    fn from_str(input: &str) -> Result<IQFileType, Self::Err> {
        match input {
            "2xf32" => Ok(IQFileType::TypePairFloat32),
            "2xi16" => Ok(IQFileType::TypePairInt16),
            "rtlsdr-file" => Ok(IQFileType::TypeRtlSdrFile),
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
            IQFileType::TypeRtlSdrFile => write!(f, "rtlsdr-file"),
            IQFileType::TypeOneInt8 => write!(f, "i8"),
        }
    }
}

// 1 MiB read buffer amortizes the underlying read() syscalls over many code
// periods (each fetch is only a few KiB).
const READ_BUF_CAPACITY: usize = 1 << 20;

pub struct IQRecording {
    file_path: PathBuf,
    file_type: IQFileType,
    // The file handle is opened once and read sequentially. Re-opening and
    // seeking from the start on every 1 ms fetch (as before) cost one open()+
    // seek() syscall per code period (~hundreds of thousands over a long file).
    reader: Option<BufReader<File>>,
    // Current read position, in samples, so we only seek when a fetch is not
    // contiguous with the previous one (e.g. the initial --off-msec offset).
    pos_samples: usize,
}

impl IQReader for IQRecording {
    fn get_iq_data(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        let sample_size = Self::get_sample_size_bytes(&self.file_type);

        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX; // force an initial seek
        }
        let reader = self.reader.as_mut().unwrap();

        // Reads are normally sequential; only seek when the requested offset is
        // not where we already are (initial offset or any non-contiguous access).
        if off_samples != self.pos_samples {
            reader.seek(SeekFrom::Start((off_samples * sample_size) as u64))?;
            self.pos_samples = off_samples;
        }

        let mut bytes = vec![0u8; sample_size * num_samples];
        if reader.read_exact(&mut bytes).is_err() {
            return Err("end of file".into());
        }
        self.pos_samples += num_samples;

        let mut iq_vec = Vec::with_capacity(num_samples);
        match self.file_type {
            IQFileType::TypeRtlSdrFile => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    iq_vec.push(Complex64 {
                        re: (bytes[off] as f64 - 127.3) / 128.0,
                        im: (bytes[off + 1] as f64 - 127.3) / 128.0,
                    });
                }
            }
            IQFileType::TypeOneInt8 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    iq_vec.push(Complex64 {
                        re: bytes[off] as i8 as f64 / i8::MAX as f64,
                        im: 0.0,
                    });
                }
            }
            IQFileType::TypePairInt16 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    let i = i16::from_le_bytes([bytes[off], bytes[off + 1]]);
                    let q = i16::from_le_bytes([bytes[off + 2], bytes[off + 3]]);
                    iq_vec.push(Complex64 {
                        re: i as f64 / i16::MAX as f64,
                        im: q as f64 / i16::MAX as f64,
                    });
                }
            }
            IQFileType::TypePairFloat32 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    let i = f32::from_le_bytes([
                        bytes[off],
                        bytes[off + 1],
                        bytes[off + 2],
                        bytes[off + 3],
                    ]);
                    let q = f32::from_le_bytes([
                        bytes[off + 4],
                        bytes[off + 5],
                        bytes[off + 6],
                        bytes[off + 7],
                    ]);
                    assert!((-1.0..=1.0).contains(&i));
                    assert!((-1.0..=1.0).contains(&q));
                    iq_vec.push(Complex64 {
                        re: i as f64,
                        im: q as f64,
                    });
                }
            }
        }

        Ok(iq_vec)
    }
}

impl IQRecording {
    pub fn new(file_path: &Path, fs: f64, file_type: &IQFileType) -> Self {
        let file_size = file_path.metadata().unwrap().len();
        let sample_size = Self::get_sample_size_bytes(file_type) as f64;
        let recording_duration_sec = file_size as f64 / fs / sample_size;

        println!(
            "file: {} -- {file_type} {} duration: {:.1} secs",
            file_path.display().to_string().green(),
            ByteSize::b(file_size).to_string().bold(),
            recording_duration_sec
        );
        Self {
            file_path: file_path.to_path_buf(),
            file_type: file_type.clone(),
            reader: None,
            pos_samples: 0,
        }
    }

    fn get_sample_size_bytes(file_type: &IQFileType) -> usize {
        match file_type {
            IQFileType::TypeRtlSdrFile => 2,
            IQFileType::TypeOneInt8 => 1,
            IQFileType::TypePairInt16 => 2 * 2,
            IQFileType::TypePairFloat32 => 2 * 4,
        }
    }
}
