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
    // interleaved signed int8 I/Q (complex), e.g. HackRF / many SDRs.
    TypePairInt8,
    TypeRtlSdrFile,
    TypeOneInt8,
    // signed 4-bit real samples, 2 packed per byte (high nibble first), I-only,
    // e.g. the SX3 front-end (ION SJTU L1E1 capture).
    TypeOne4Bit,
    // 1-bit hard-limited real samples, 8 packed per byte (MSB first). I-only,
    // e.g. jks.com's gps.samples.1bit.I.fs5456.if4092.bin.
    TypeOneBit,
}

impl FromStr for IQFileType {
    type Err = Box<dyn Error>;
    fn from_str(input: &str) -> Result<IQFileType, Self::Err> {
        match input {
            "2xf32" => Ok(IQFileType::TypePairFloat32),
            "2xi16" => Ok(IQFileType::TypePairInt16),
            "2xi8" => Ok(IQFileType::TypePairInt8),
            "rtlsdr-file" => Ok(IQFileType::TypeRtlSdrFile),
            "i8" => Ok(IQFileType::TypeOneInt8),
            "4bit" => Ok(IQFileType::TypeOne4Bit),
            "1bit" => Ok(IQFileType::TypeOneBit),
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
            IQFileType::TypeRtlSdrFile => write!(f, "rtlsdr-file"),
            IQFileType::TypeOneInt8 => write!(f, "i8"),
            IQFileType::TypeOne4Bit => write!(f, "4bit"),
            IQFileType::TypeOneBit => write!(f, "1bit"),
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
        // 1-bit samples are bit-packed (8 per byte), so they don't fit the
        // integer-bytes-per-sample math below; handle them on a dedicated path.
        if let IQFileType::TypeOneBit = self.file_type {
            return self.get_iq_data_1bit(off_samples, num_samples);
        }
        if let IQFileType::TypeOne4Bit = self.file_type {
            return self.get_iq_data_4bit(off_samples, num_samples);
        }
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
            IQFileType::TypePairInt8 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    iq_vec.push(Complex64 {
                        re: bytes[off] as i8 as f64 / i8::MAX as f64,
                        im: bytes[off + 1] as i8 as f64 / i8::MAX as f64,
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
            // Returned early above via get_iq_data_1bit / get_iq_data_4bit.
            IQFileType::TypeOneBit => unreachable!(),
            IQFileType::TypeOne4Bit => unreachable!(),
        }

        Ok(iq_vec)
    }
}

impl IQRecording {
    pub fn new(file_path: &Path, fs: f64, file_type: &IQFileType) -> Result<Self, Box<dyn Error>> {
        let file_size = file_path
            .metadata()
            .map_err(|e| format!("{}: {e}", file_path.display()))?
            .len();
        let recording_duration_sec = match file_type {
            // sub-byte packings: 8 / 2 samples per byte respectively.
            IQFileType::TypeOneBit => (file_size * 8) as f64 / fs,
            IQFileType::TypeOne4Bit => (file_size * 2) as f64 / fs,
            _ => file_size as f64 / fs / Self::get_sample_size_bytes(file_type) as f64,
        };

        // Diagnostic banner -> stderr (log), so stdout stays clean for `--json -`.
        log::warn!(
            "file: {} -- {file_type} {} duration: {:.1} secs",
            file_path.display().to_string().green(),
            ByteSize::b(file_size).to_string().bold(),
            recording_duration_sec
        );
        Ok(Self {
            file_path: file_path.to_path_buf(),
            file_type: file_type.clone(),
            reader: None,
            pos_samples: 0,
        })
    }

    fn get_sample_size_bytes(file_type: &IQFileType) -> usize {
        match file_type {
            IQFileType::TypeRtlSdrFile => 2,
            IQFileType::TypeOneInt8 => 1,
            IQFileType::TypePairInt8 => 2,
            IQFileType::TypePairInt16 => 2 * 2,
            IQFileType::TypePairFloat32 => 2 * 4,
            // Sub-byte; not expressible here -- handled on their own read paths.
            IQFileType::TypeOneBit => unreachable!("1-bit uses get_iq_data_1bit"),
            IQFileType::TypeOne4Bit => unreachable!("4-bit uses get_iq_data_4bit"),
        }
    }

    // Read bit-packed 1-bit real samples (8 per byte, MSB first). Each bit is a
    // hard-limited sample: 1 -> +1.0, 0 -> -1.0; the quadrature part is 0.
    // Reads are byte-aligned: period_sp = PERIOD_RCV * fs is a multiple of 8 for
    // any fs that is a multiple of 8000 (e.g. the 5.456 MHz this format uses), so
    // off_samples and num_samples are always multiples of 8.
    fn get_iq_data_1bit(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        assert!(
            off_samples.is_multiple_of(8) && num_samples.is_multiple_of(8),
            "1-bit reads must be 8-sample aligned (use an fs that is a multiple of 8000)"
        );

        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX; // force an initial seek
        }
        let reader = self.reader.as_mut().unwrap();

        if off_samples != self.pos_samples {
            reader.seek(SeekFrom::Start((off_samples / 8) as u64))?;
            self.pos_samples = off_samples;
        }

        let mut bytes = vec![0u8; num_samples / 8];
        if reader.read_exact(&mut bytes).is_err() {
            return Err("end of file".into());
        }
        self.pos_samples += num_samples;

        let mut iq_vec = Vec::with_capacity(num_samples);
        for byte in bytes {
            for bit in (0..8).rev() {
                let re = if (byte >> bit) & 1 == 1 { 1.0 } else { -1.0 };
                iq_vec.push(Complex64 { re, im: 0.0 });
            }
        }
        Ok(iq_vec)
    }

    // Read signed 4-bit real samples, 2 per byte (high nibble first), I-only.
    // Each nibble is two's-complement [-8, 7]. Reads are byte-aligned: num/off
    // are even (period_sp = PERIOD_RCV * fs is even for fs a multiple of 2000).
    fn get_iq_data_4bit(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        assert!(
            off_samples.is_multiple_of(2) && num_samples.is_multiple_of(2),
            "4-bit reads must be 2-sample aligned (use an fs that is a multiple of 2000)"
        );

        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX; // force an initial seek
        }
        let reader = self.reader.as_mut().unwrap();

        if off_samples != self.pos_samples {
            reader.seek(SeekFrom::Start((off_samples / 2) as u64))?;
            self.pos_samples = off_samples;
        }

        let mut bytes = vec![0u8; num_samples / 2];
        if reader.read_exact(&mut bytes).is_err() {
            return Err("end of file".into());
        }
        self.pos_samples += num_samples;

        // sign-extend a 4-bit nibble (0..15) to [-8, 7], then normalize by 8.
        let nib = |n: u8| -> f64 {
            let v = if n >= 8 { n as i32 - 16 } else { n as i32 };
            v as f64 / 8.0
        };
        let mut iq_vec = Vec::with_capacity(num_samples);
        for byte in bytes {
            iq_vec.push(Complex64 {
                re: nib(byte >> 4),
                im: 0.0,
            });
            iq_vec.push(Complex64 {
                re: nib(byte & 0x0f),
                im: 0.0,
            });
        }
        Ok(iq_vec)
    }
}
