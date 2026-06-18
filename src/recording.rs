use bytesize::ByteSize;
use colored::Colorize;
use rustfft::FftPlanner;
use rustfft::num_complex::Complex32;
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

// === PocketSDR FE4CH RAW16 (CH0 = GPS L1/E1) front-end constants ===
// The recording's .tag fixes these: F_S = 16 MHz, CH0 F_LO = 1568 MHz, so the
// L1/E1 band sits at +7.42 MHz (1575.42 - 1568). The reader downconverts CH0 to
// baseband and decimates to the configured output rate so the receiver runs at a
// sane low rate and never sees the +7.42 MHz IF (which sits at fs/2 = 8 MHz, where
// the wide-IF channel's >8 MHz content has aliased to negative frequencies as a
// self-image of comparable power — left in place it slips the L1 carrier loop and
// the nav data never decodes). The brick-wall keeps only the positive signal lobe.
const PSDR_NATIVE_FS: f64 = 16.0e6;
const PSDR_L1_IF: f64 = 7.42e6;
// Positive-frequency passband to keep (Hz). The C/A main lobe is 7.42 ± 1.023 MHz;
// everything above Nyquist (8 MHz) has aliased to negative frequencies, so keep
// [PSDR_BAND_LO, Nyquist] and zero all negative bins to discard the image.
const PSDR_BAND_LO: f64 = 6.0e6;
// Overlap-save guard (native samples each side of a fetch). The band mask has a
// raised-cosine lower edge so its impulse response decays well within this guard.
const PSDR_GUARD: usize = 1024;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IQFileType {
    TypePairFloat32,
    TypePairInt16,
    // interleaved signed int16 I/Q, *big-endian* (most SDRs are little-endian;
    // some instruments aren't, e.g. the Tampere TUNI Galileo E1 capture).
    TypePairInt16Be,
    // interleaved signed int8 I/Q (complex), e.g. HackRF / many SDRs.
    TypePairInt8,
    TypeRtlSdrFile,
    TypeOneInt8,
    // signed 4-bit real samples, 2 packed per byte (high nibble first), I-only,
    // e.g. the SX3 front-end (ION SJTU L1E1 capture).
    TypeOne4Bit,
    // signed 2-bit real samples, 4 packed per byte (most-significant pair first),
    // two's-complement [-2, 1], I-only, e.g. the IFEN SX3 dual-RF L1 capture
    // (ION IFEN_Bands-L1.stream: 20.48 MHz, IF 5.5 MHz).
    TypeOne2Bit,
    // 1-bit hard-limited real samples, 8 packed per byte (MSB first). I-only,
    // e.g. jks.com's gps.samples.1bit.I.fs5456.if4092.bin.
    TypeOneBit,
    // PocketSDR FE 4CH RAW16: 4 channels × (2-bit I + 2-bit Q) packed into each
    // 16-bit word (little-endian). Channel 0 lives in bits[3:0] of the low byte:
    // I = bits[1:0], Q = bits[3:2]. Sign+magnitude {00,01,10,11}→{+1,+3,-1,-3}/3.
    // The reader image-rejects, downconverts CH0's +7.42 MHz L1/E1 IF to baseband
    // and decimates to the configured output rate (see PSDR_* and the read path),
    // so the standard FE4CH L1 config uses `--fs 4000000 --fi 0`.
    TypePocketSdrRaw16,
    // PocketSDR FE 4CH RAW16, channel 2 (the L5/E5a band). On captures that carry
    // E5a (e.g. the eindhoven FE4CH, .tag F_LO = …,1176.45,…) channel 2's LO sits
    // at the band centre, so it is already at zero IF — a plain nibble decode at
    // the native 16 MHz, no image rejection/downconversion (unlike CH0). Run it as
    // `-t pocketsdr-raw16-ch2 --fs 16000000 --fi 0 --sig E5A`.
    TypePocketSdrRaw16Ch2,
}

impl FromStr for IQFileType {
    type Err = Box<dyn Error>;
    fn from_str(input: &str) -> Result<IQFileType, Self::Err> {
        match input {
            "2xf32" => Ok(IQFileType::TypePairFloat32),
            "2xi16" => Ok(IQFileType::TypePairInt16),
            "2xi16-be" => Ok(IQFileType::TypePairInt16Be),
            "2xi8" => Ok(IQFileType::TypePairInt8),
            "rtlsdr-file" => Ok(IQFileType::TypeRtlSdrFile),
            "i8" => Ok(IQFileType::TypeOneInt8),
            "4bit" => Ok(IQFileType::TypeOne4Bit),
            "2bit" => Ok(IQFileType::TypeOne2Bit),
            "1bit" => Ok(IQFileType::TypeOneBit),
            "pocketsdr-raw16" => Ok(IQFileType::TypePocketSdrRaw16),
            "pocketsdr-raw16-ch2" => Ok(IQFileType::TypePocketSdrRaw16Ch2),
            _ => Err(format!("Failed to parse {}", input).into()),
        }
    }
}

impl fmt::Display for IQFileType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            IQFileType::TypePairFloat32 => write!(f, "2xf32"),
            IQFileType::TypePairInt16 => write!(f, "2xi16"),
            IQFileType::TypePairInt16Be => write!(f, "2xi16-be"),
            IQFileType::TypePairInt8 => write!(f, "2xi8"),
            IQFileType::TypeRtlSdrFile => write!(f, "rtlsdr-file"),
            IQFileType::TypeOneInt8 => write!(f, "i8"),
            IQFileType::TypeOne4Bit => write!(f, "4bit"),
            IQFileType::TypeOne2Bit => write!(f, "2bit"),
            IQFileType::TypeOneBit => write!(f, "1bit"),
            IQFileType::TypePocketSdrRaw16 => write!(f, "pocketsdr-raw16"),
            IQFileType::TypePocketSdrRaw16Ch2 => write!(f, "pocketsdr-raw16-ch2"),
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
    // Total recording length in seconds (file size / sample rate), for the UI
    // progress bar.
    total_sec: f64,
    // PocketSDR RAW16 only: decimation factor (native 16 MHz / output rate) and
    // the FFT planner used by the image-reject downconvert. 1 / unused otherwise.
    psdr_dec: usize,
    fft_planner: FftPlanner<f32>,
}

impl IQReader for IQRecording {
    fn duration_sec(&self) -> Option<f64> {
        Some(self.total_sec)
    }

    fn read_iq_block(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        // 1-bit samples are bit-packed (8 per byte), so they don't fit the
        // integer-bytes-per-sample math below; handle them on a dedicated path.
        if let IQFileType::TypeOneBit = self.file_type {
            return self.get_iq_data_1bit(off_samples, num_samples);
        }
        if let IQFileType::TypeOne4Bit = self.file_type {
            return self.get_iq_data_4bit(off_samples, num_samples);
        }
        if let IQFileType::TypeOne2Bit = self.file_type {
            return self.get_iq_data_2bit(off_samples, num_samples);
        }
        if let IQFileType::TypePocketSdrRaw16 = self.file_type {
            return self.get_iq_data_pocketsdr_raw16(off_samples, num_samples);
        }
        if let IQFileType::TypePocketSdrRaw16Ch2 = self.file_type {
            return self.get_iq_data_pocketsdr_raw16_ch2(off_samples, num_samples);
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
                    iq_vec.push(Complex32 {
                        re: (bytes[off] as f32 - 127.3) / 128.0,
                        im: (bytes[off + 1] as f32 - 127.3) / 128.0,
                    });
                }
            }
            IQFileType::TypeOneInt8 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    iq_vec.push(Complex32 {
                        re: bytes[off] as i8 as f32 / i8::MAX as f32,
                        im: 0.0,
                    });
                }
            }
            IQFileType::TypePairInt8 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    iq_vec.push(Complex32 {
                        re: bytes[off] as i8 as f32 / i8::MAX as f32,
                        im: bytes[off + 1] as i8 as f32 / i8::MAX as f32,
                    });
                }
            }
            IQFileType::TypePairInt16 => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    let i = i16::from_le_bytes([bytes[off], bytes[off + 1]]);
                    let q = i16::from_le_bytes([bytes[off + 2], bytes[off + 3]]);
                    iq_vec.push(Complex32 {
                        re: i as f32 / i16::MAX as f32,
                        im: q as f32 / i16::MAX as f32,
                    });
                }
            }
            IQFileType::TypePairInt16Be => {
                for off in (0..bytes.len()).step_by(sample_size) {
                    let i = i16::from_be_bytes([bytes[off], bytes[off + 1]]);
                    let q = i16::from_be_bytes([bytes[off + 2], bytes[off + 3]]);
                    iq_vec.push(Complex32 {
                        re: i as f32 / i16::MAX as f32,
                        im: q as f32 / i16::MAX as f32,
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
                    iq_vec.push(Complex32 {
                        re: i as f32,
                        im: q as f32,
                    });
                }
            }
            // Returned early above via get_iq_data_1bit / _2bit / _4bit / _pocketsdr_raw16[_ch2].
            IQFileType::TypeOneBit => unreachable!(),
            IQFileType::TypeOne2Bit => unreachable!(),
            IQFileType::TypeOne4Bit => unreachable!(),
            IQFileType::TypePocketSdrRaw16 => unreachable!(),
            IQFileType::TypePocketSdrRaw16Ch2 => unreachable!(),
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
            // sub-byte packings: 8 / 4 / 2 samples per byte respectively.
            IQFileType::TypeOneBit => (file_size * 8) as f64 / fs,
            IQFileType::TypeOne2Bit => (file_size * 4) as f64 / fs,
            IQFileType::TypeOne4Bit => (file_size * 2) as f64 / fs,
            // RAW16: one 16-bit word per native sample at the fixed 16 MHz rate
            // (CH0 decimates to `fs`; CH2 runs at the native rate).
            IQFileType::TypePocketSdrRaw16 | IQFileType::TypePocketSdrRaw16Ch2 => {
                file_size as f64 / 2.0 / PSDR_NATIVE_FS
            }
            _ => file_size as f64 / fs / Self::get_sample_size_bytes(file_type) as f64,
        };
        // RAW16 is downconverted+decimated to the requested output rate `fs`.
        let psdr_dec = if let IQFileType::TypePocketSdrRaw16 = file_type {
            ((PSDR_NATIVE_FS / fs).round() as usize).max(1)
        } else {
            1
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
            total_sec: recording_duration_sec,
            psdr_dec,
            fft_planner: FftPlanner::new(),
        })
    }

    fn get_sample_size_bytes(file_type: &IQFileType) -> usize {
        match file_type {
            IQFileType::TypeRtlSdrFile => 2,
            IQFileType::TypeOneInt8 => 1,
            IQFileType::TypePairInt8 => 2,
            IQFileType::TypePairInt16 | IQFileType::TypePairInt16Be => 2 * 2,
            IQFileType::TypePairFloat32 => 2 * 4,
            // Not expressible here -- handled on their own read paths.
            IQFileType::TypeOneBit => unreachable!("1-bit uses get_iq_data_1bit"),
            IQFileType::TypeOne2Bit => unreachable!("2-bit uses get_iq_data_2bit"),
            IQFileType::TypeOne4Bit => unreachable!("4-bit uses get_iq_data_4bit"),
            IQFileType::TypePocketSdrRaw16 => {
                unreachable!("RAW16 uses get_iq_data_pocketsdr_raw16")
            }
            IQFileType::TypePocketSdrRaw16Ch2 => {
                unreachable!("RAW16 CH2 uses get_iq_data_pocketsdr_raw16_ch2")
            }
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
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
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
                iq_vec.push(Complex32 { re, im: 0.0 });
            }
        }
        Ok(iq_vec)
    }

    // Read signed 2-bit real samples, 4 per byte (least-significant pair first),
    // I-only. Each pair is two's-complement [-2, 1], normalized by 2. Reads are
    // byte-aligned: num/off are multiples of 4 (period_sp = PERIOD_RCV * fs is a
    // multiple of 4 for fs a multiple of 4000, e.g. the IFEN SX3's 20.48 MHz).
    //
    // NB: the pair order is LSB-first -- the earliest sample is in bits 1:0, not
    // 7:6. This was confirmed against a live acquire of the IFEN SX3 capture: with
    // the wrong (MSB-first) order the C/A code still correlates, but the carrier
    // phase is scrambled (96.7 deg/sample at fi 5.5 MHz / fs 20.48 MHz when 4
    // adjacent samples are time-reversed), capping every SV's C/N0 at ~35 dB-Hz
    // and blocking nav-bit sync. LSB-first restores 44-49 dB-Hz on the strong SVs
    // (matching the GN3S front-end's view of the same Munich capture) and decode.
    fn get_iq_data_2bit(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        assert!(
            off_samples.is_multiple_of(4) && num_samples.is_multiple_of(4),
            "2-bit reads must be 4-sample aligned (use an fs that is a multiple of 4000)"
        );

        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX; // force an initial seek
        }
        let reader = self.reader.as_mut().unwrap();

        if off_samples != self.pos_samples {
            reader.seek(SeekFrom::Start((off_samples / 4) as u64))?;
            self.pos_samples = off_samples;
        }

        let mut bytes = vec![0u8; num_samples / 4];
        if reader.read_exact(&mut bytes).is_err() {
            return Err("end of file".into());
        }
        self.pos_samples += num_samples;

        // sign-extend a 2-bit pair (0..3) to [-2, 1], then normalize by 2.
        let pair = |n: u8| -> f32 {
            let v = if n >= 2 { n as i32 - 4 } else { n as i32 };
            v as f32 / 2.0
        };
        let mut iq_vec = Vec::with_capacity(num_samples);
        for byte in bytes {
            for shift in [0, 2, 4, 6] {
                iq_vec.push(Complex32 {
                    re: pair((byte >> shift) & 0x03),
                    im: 0.0,
                });
            }
        }
        Ok(iq_vec)
    }

    // Read PocketSDR FE 4CH RAW16 channel-0 (L1/E1) as complex baseband, image-
    // rejected and decimated to the configured output rate.
    //
    // Each 16-bit word (2 bytes, little-endian) packs 4 channels, one per nibble,
    // each 2-bit I + 2-bit Q (RAW16, IQ=2, BITS=2 per the recording's .tag). CH0
    // lives in bits[3:0]: I = bits[1:0], Q = bits[3:2]; the other nibbles are the
    // L2/L5/L6-band channels and are skipped. Sign+magnitude {00,01,10,11} →
    // {+1,+3,-1,-3}/3. Q is conjugated: with PocketSDR's convention the GPS L1 C/A
    // lands at -7.42 MHz, so conjugating moves it to +7.42 MHz.
    //
    // `off_samples`/`num_samples` are at the *output* rate (16 MHz / `psdr_dec`).
    // We read the matching native window (plus an overlap-save guard), FFT it, keep
    // only the positive L1 lobe [PSDR_BAND_LO, Nyquist] and zero all negative bins
    // — a brick wall at Nyquist that discards the >8 MHz content that aliased to
    // negative frequencies as a self-image — then mix +7.42 MHz down to DC and take
    // every `psdr_dec`-th sample. Without the image rejection the carrier loop slips
    // and the nav data never decodes; see the PSDR_* constants above.
    fn get_iq_data_pocketsdr_raw16(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        const BYTES_PER_SAMPLE: usize = 2; // one 16-bit word per native sample
        let dec = self.psdr_dec;

        // Native window: the requested output span plus a guard each side. `pad`
        // counts samples clamped off the front of the file (zero-filled).
        let in_center = off_samples * dec; // native index of output sample 0
        let total = num_samples * dec + 2 * PSDR_GUARD; // logical native span
        let in_first = in_center as i64 - PSDR_GUARD as i64;
        let in_start = in_first.max(0) as usize;
        let pad = (in_start as i64 - in_first) as usize;
        let to_read = total - pad;

        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX;
        }
        let reader = self.reader.as_mut().unwrap();
        reader.seek(SeekFrom::Start((in_start * BYTES_PER_SAMPLE) as u64))?;

        let mut bytes = vec![0u8; BYTES_PER_SAMPLE * to_read];
        let mut filled = 0;
        while filled < bytes.len() {
            match reader.read(&mut bytes[filled..]) {
                Ok(0) => break,
                Ok(n) => filled += n,
                Err(e) => return Err(Box::new(e)),
            }
        }
        let got = filled / BYTES_PER_SAMPLE;
        // The middle (the actual output region) must be fully present; a trailing
        // guard running past EOF is fine (zero-filled).
        if pad + got < PSDR_GUARD + num_samples * dec {
            return Err("end of file".into());
        }
        self.pos_samples = usize::MAX; // reads are not contiguous on this path

        // Decode CH0 (conjugated) into a zero-padded FFT buffer.
        const VAL: [f32; 4] = [1.0 / 3.0, 1.0, -1.0 / 3.0, -1.0];
        let p = total.next_power_of_two();
        let mut buf = vec![Complex32::new(0.0, 0.0); p];
        for k in 0..got {
            let lo = bytes[k * BYTES_PER_SAMPLE]; // CH0 = low nibble
            buf[pad + k] = Complex32 {
                re: VAL[(lo & 0x03) as usize],
                im: -VAL[((lo >> 2) & 0x03) as usize],
            };
        }

        // Keep the positive L1 lobe [PSDR_BAND_LO, Nyquist]; zero everything else,
        // including all negative-frequency bins (the aliased image). Raised-cosine
        // lower edge so the impulse response stays inside PSDR_GUARD.
        let fwd = self.fft_planner.plan_fft_forward(p);
        let inv = self.fft_planner.plan_fft_inverse(p);
        fwd.process(&mut buf);
        let lo_bin = (PSDR_BAND_LO / PSDR_NATIVE_FS * p as f64).round() as usize;
        let nyq = p / 2;
        let edge = ((0.4e6 / PSDR_NATIVE_FS) * p as f64).round().max(1.0) as usize;
        for (b, v) in buf.iter_mut().enumerate() {
            let w = if b < lo_bin || b > nyq {
                0.0
            } else if b < lo_bin + edge {
                let x = (b - lo_bin) as f32 / edge as f32;
                0.5 - 0.5 * (std::f32::consts::PI * x).cos()
            } else {
                1.0
            };
            *v *= w;
        }
        inv.process(&mut buf);
        let norm = 1.0 / p as f32;

        // Mix +7.42 MHz -> DC (phase keyed to the absolute native index, so it is
        // continuous across fetches) and take every dec-th sample.
        let w0 = -2.0 * std::f64::consts::PI * PSDR_L1_IF / PSDR_NATIVE_FS;
        let mut out = Vec::with_capacity(num_samples);
        for j in 0..num_samples {
            let local = PSDR_GUARD + j * dec; // this output sample's slot in buf
            let ph = w0 * (in_center + j * dec) as f64;
            let mix = Complex32::new(ph.cos() as f32, ph.sin() as f32);
            out.push(buf[local] * norm * mix);
        }
        Ok(out)
    }

    // Read PocketSDR FE 4CH RAW16 channel 2 (the L5/E5a band) as complex baseband
    // at the native 16 MHz. Each 16-bit word (2 bytes, little-endian) packs four
    // channels one per nibble; CH2 is the *high* byte's low nibble: I = bits[9:8],
    // Q = bits[11:10]. Captures carrying E5a tune CH2's LO to the band centre
    // (1176.45 MHz), so it is already at zero IF — no image rejection or
    // downconversion (unlike CH0), just the sign+magnitude nibble decode
    // {00,01,10,11}→{+1,+3,-1,-3}/3. Q is conjugated to match PocketSDR's spectral
    // convention. Contiguous (no decimation): run with `--fs 16000000 --fi 0`.
    fn get_iq_data_pocketsdr_raw16_ch2(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
        const BYTES_PER_SAMPLE: usize = 2; // one 16-bit word per native sample
        if self.reader.is_none() {
            let file = File::open(&self.file_path)?;
            self.reader = Some(BufReader::with_capacity(READ_BUF_CAPACITY, file));
            self.pos_samples = usize::MAX; // force an initial seek
        }
        let reader = self.reader.as_mut().unwrap();
        if off_samples != self.pos_samples {
            reader.seek(SeekFrom::Start((off_samples * BYTES_PER_SAMPLE) as u64))?;
            self.pos_samples = off_samples;
        }
        let mut bytes = vec![0u8; num_samples * BYTES_PER_SAMPLE];
        if reader.read_exact(&mut bytes).is_err() {
            return Err("end of file".into());
        }
        self.pos_samples += num_samples;

        const VAL: [f32; 4] = [1.0 / 3.0, 1.0, -1.0 / 3.0, -1.0];
        let iq_vec = bytes
            .chunks_exact(2)
            .map(|w| {
                let hi = w[1]; // CH2/CH3 byte; CH2 = its low nibble
                Complex32 {
                    re: VAL[(hi & 0x03) as usize],
                    im: -VAL[((hi >> 2) & 0x03) as usize], // conjugate (PocketSDR convention)
                }
            })
            .collect();
        Ok(iq_vec)
    }

    // Read signed 4-bit real samples, 2 per byte (high nibble first), I-only.
    // Each nibble is two's-complement [-8, 7]. Reads are byte-aligned: num/off
    // are even (period_sp = PERIOD_RCV * fs is even for fs a multiple of 2000).
    fn get_iq_data_4bit(
        &mut self,
        off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex32>, Box<dyn std::error::Error>> {
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
        let nib = |n: u8| -> f32 {
            let v = if n >= 8 { n as i32 - 16 } else { n as i32 };
            v as f32 / 8.0
        };
        let mut iq_vec = Vec::with_capacity(num_samples);
        for byte in bytes {
            iq_vec.push(Complex32 {
                re: nib(byte >> 4),
                im: 0.0,
            });
            iq_vec.push(Complex32 {
                re: nib(byte & 0x0f),
                im: 0.0,
            });
        }
        Ok(iq_vec)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustfft::num_complex::Complex32;

    // Encode a complex sample into a PocketSDR RAW16 word: CH0 in the low nibble
    // (I = bits[1:0], Q = bits[3:2]); the reader decodes im = -VAL[Q], so the Q
    // field carries -q. Sign+magnitude levels {±1/3, ±1}, threshold at 0.66.
    fn enc(v: f32) -> u16 {
        match (v >= 0.0, v.abs() > 0.66) {
            (true, false) => 0,  // +1/3
            (true, true) => 1,   // +1
            (false, false) => 2, // -1/3
            (false, true) => 3,  // -1
        }
    }
    fn raw16_tone(path: &Path, f_hz: f64, n: usize) {
        let mut bytes = Vec::with_capacity(n * 2);
        for k in 0..n {
            let th = 2.0 * std::f64::consts::PI * f_hz * k as f64 / PSDR_NATIVE_FS;
            let (i, q) = (th.cos() as f32, th.sin() as f32); // re + j im = exp(j2pi f t)
            let word = enc(i) | (enc(-q) << 2); // Q field holds -q (im = -VAL[Q])
            bytes.extend_from_slice(&word.to_le_bytes());
        }
        std::fs::write(path, &bytes).unwrap();
    }
    fn block_power(path: &Path) -> (f64, f64) {
        // Returns (DC power, total power) of a decoded output block. The +7.42 MHz
        // signal mixes to DC; a rejected image leaves little of either.
        let mut rec = IQRecording::new(path, 4.0e6, &IQFileType::TypePocketSdrRaw16).unwrap();
        let out = rec.read_iq_block(2000, 8192).unwrap();
        let n = out.len() as f32;
        let mean: Complex32 = out.iter().sum::<Complex32>() / n;
        let total: f64 = out.iter().map(|c| c.norm_sqr() as f64).sum::<f64>() / n as f64;
        (mean.norm_sqr() as f64, total)
    }

    #[test]
    fn raw16_decimates_signal_and_rejects_image() {
        let dir = std::env::temp_dir();
        let sig = dir.join(format!("psdr_sig_{}.bin", std::process::id()));
        let img = dir.join(format!("psdr_img_{}.bin", std::process::id()));
        // +7.42 MHz: the L1 IF -> mixes to DC, survives the positive-lobe bandpass.
        raw16_tone(&sig, PSDR_L1_IF, 80_000);
        // -7.42 MHz: a negative-frequency image -> the brick wall zeros it.
        raw16_tone(&img, -PSDR_L1_IF, 80_000);

        let (sig_dc, sig_tot) = block_power(&sig);
        let (_img_dc, img_tot) = block_power(&img);
        let _ = std::fs::remove_file(&sig);
        let _ = std::fs::remove_file(&img);

        // The +7.42 MHz tone lands at DC: nearly all its power is in the mean.
        assert!(
            sig_dc / sig_tot > 0.8,
            "signal not at DC: dc/tot={}",
            sig_dc / sig_tot
        );
        // The image is suppressed at least ~15 dB below the signal.
        assert!(
            img_tot < sig_tot / 30.0,
            "image not rejected: {img_tot} vs {sig_tot}"
        );
    }

    #[test]
    fn raw16_duration_uses_native_rate() {
        let dir = std::env::temp_dir();
        let f = dir.join(format!("psdr_dur_{}.bin", std::process::id()));
        raw16_tone(&f, PSDR_L1_IF, 16_000); // 16000 words = 1 ms at 16 MHz native
        let rec = IQRecording::new(&f, 4.0e6, &IQFileType::TypePocketSdrRaw16).unwrap();
        let _ = std::fs::remove_file(&f);
        // Duration is file-size/native-rate (1 ms), independent of the 4 MHz output.
        assert!((rec.duration_sec().unwrap() - 1.0e-3).abs() < 1e-9);
    }
}
