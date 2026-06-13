use chrono::Local;
use clap::Parser;
use colored::Colorize;
use coredump::register_panic_handler;
use log::LevelFilter;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use gnss_rcv::code::Signal;
use gnss_rcv::plots::{plot_generate_html, plot_remove_old_graph};
use gnss_rcv::receiver::{Receiver, ReceiverConfig};
use gnss_rcv::recording::IQFileType;
use gnss_rcv::state::GnssState;

#[derive(Parser)]
#[command(name = "gnss-rcv", about = "gnss-rcv: GNSS receiver")]
struct Options {
    #[arg(
        short = 'f',
        long,
        default_value = "resources/nov_3_time_18_48_st_ives"
    )]
    file: PathBuf,
    #[arg(short = 's', long, help = "host for rtl-sdr-tcp", default_value = "")]
    hostname: String,
    #[arg(
        long,
        help = "signal family/families: L1CA, E1B, E1C, or a comma list (L1CA,E1B).                 Default: every family the bandwidth admits (L1CA always; +E1B at fs >= 4.092 MHz)"
    )]
    sig: Option<String>,
    #[arg(short = 'd', long, help = "use rtl-sdr device")]
    use_device: bool,
    #[arg(short = 'l', long, help = "path to log file", default_value = "")]
    log_file: PathBuf,
    #[arg(
        short = 't',
        long,
        help = "type of IQ file",
        default_value = "2xf32",
        value_parser = parse_iq_file_type
    )]
    iq_file_type: IQFileType,
    #[arg(
        long,
        help = "sampling frequency, e.g. 10M / 2.046M / 2046000",
        value_parser = parse_freq,
        default_value = "2046000"
    )]
    fs: f64,
    #[arg(
        long,
        help = "intermediate frequency, e.g. 420K / 0",
        value_parser = parse_freq,
        default_value = "0"
    )]
    fi: f64,
    #[arg(long, help = "offset in file", default_value = "0")]
    off_msec: usize,
    #[arg(long, help = "duration of sample", default_value = "0")]
    num_msec: usize,
    #[arg(
        long,
        help = "satellites to search: prefixed (G3,E11,J193) or bare PRN (3,11; constellation inferred from --sig)",
        default_value = ""
    )]
    sats: String,
    #[arg(
        long,
        help = "search the SBAS L1 block (PRN 120-138) — on by default; kept for compatibility"
    )]
    sbas: bool,
    #[arg(long, help = "disable the SBAS L1 search")]
    no_sbas: bool,
    #[arg(long, help = "also search the QZSS L1 block (PRN 193-202)")]
    qzss: bool,
    #[arg(short = 'u', long, help = "use ui")]
    use_ui: bool,
    #[arg(short = 'p', long, help = "write diagnostic plots to plots/")]
    plots: bool,
    #[arg(
        short = 'x',
        long,
        help = "stop as soon as the first position fix is computed"
    )]
    exit_on_fix: bool,
    #[arg(
        long,
        help = "write an end-of-run JSON summary to <path> ('-' = stdout)"
    )]
    json: Option<PathBuf>,
}

fn init_logging(log_file: &PathBuf) {
    if !log_file.as_os_str().is_empty() {
        println!("using log file: {}", log_file.display());
        let target = Box::new(File::create(log_file).expect("log file err"));
        env_logger::Builder::new()
            .target(env_logger::Target::Pipe(target))
            .filter(None, LevelFilter::Debug)
            .format(|buf, record| {
                writeln!(
                    buf,
                    "{} {} {}",
                    Local::now().format("%Y-%m-%dT%H:%M:%S%.3fZ"),
                    record.level(),
                    record.args()
                )
            })
            .init();
    } else {
        // Default to warn so a plain run still shows tracking status and the
        // position fix; RUST_LOG overrides it (e.g. RUST_LOG=info for detail).
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
            .format_target(false)
            .format_module_path(false)
            .format_timestamp_millis()
            .init();
    }
}

fn init_ctrl_c(exit_req: Arc<AtomicBool>) {
    register_panic_handler().unwrap();
    ctrlc::set_handler(move || {
        exit_req.store(true, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");
}

/// Parse a frequency that may carry a K/M/G suffix (case-insensitive):
/// "10M" -> 10_000_000, "420K" -> 420_000, "2.046M", or a plain "2046000".
fn parse_freq(s: &str) -> Result<f64, String> {
    let s = s.trim();
    let (num, mult) = match s.chars().last() {
        Some('k' | 'K') => (&s[..s.len() - 1], 1e3),
        Some('m' | 'M') => (&s[..s.len() - 1], 1e6),
        Some('g' | 'G') => (&s[..s.len() - 1], 1e9),
        _ => (s, 1.0),
    };
    num.trim()
        .parse::<f64>()
        .map(|v| v * mult)
        .map_err(|_| format!("invalid frequency '{s}' (use e.g. 10M, 420K, 2046000)"))
}

/// clap value parser for `--iq-file-type`: defers to `IQFileType`'s `FromStr`
/// but flattens its `Box<dyn Error>` (not `Send + Sync`) into a `String`, the
/// error type clap's derive requires.
fn parse_iq_file_type(s: &str) -> Result<IQFileType, String> {
    s.parse::<IQFileType>().map_err(|e| e.to_string())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opt = Options::parse();
    let exit_req = Arc::new(AtomicBool::new(false));

    init_logging(&opt.log_file);
    init_ctrl_c(exit_req.clone());
    if opt.plots {
        plot_remove_old_graph();
        plot_generate_html();
    }

    log::warn!(
        "gnss-rcv: sampling: {} fi: {} off_msec={} num_msec={}",
        format!("{:.1} KHz", opt.fs / 1000.0).bold(),
        format!("{:.1} KHz", opt.fi / 1000.0).bold(),
        opt.off_msec,
        opt.num_msec,
    );
    let families: Vec<Signal> = match &opt.sig {
        Some(list) => list
            .split(',')
            .map(|t| {
                t.trim()
                    .parse::<Signal>()
                    .unwrap_or_else(|_| panic!("invalid signal: {t}"))
            })
            .collect(),
        // All-signals default, gated by what the bandwidth can carry: the
        // C/A family always fits; E1 BOC(1,1) needs its ±2.046 MHz main
        // lobes (fs ≥ 4.092 Msps).
        None => {
            let mut f = vec![Signal::L1ca];
            if opt.fs >= 4_092_000.0 {
                f.push(Signal::GalileoE1b);
            }
            f
        }
    };
    let sig = families[0];
    log::warn!(
        "gnss-rcv: signal families {:?} (L1 band, {:.1} MHz)",
        families.iter().map(|f| f.to_string()).collect::<Vec<_>>(),
        sig.carrier_hz() / 1_000_000.0
    );

    // SBAS is on by default (--no-sbas disables; bare --sbas is accepted for
    // compatibility and wins over --no-sbas). The E1 path drops the GEOs
    // regardless (L1 C/A signals).
    let sbas = opt.sbas || !opt.no_sbas;

    if opt.use_ui {
        gnss_rcv::egui_main(opt.plots, sbas, opt.qzss);
        return Ok(());
    }

    let config = ReceiverConfig {
        use_device: opt.use_device,
        hostname: opt.hostname.clone(),
        file: opt.file.clone(),
        iq_file_type: opt.iq_file_type.clone(),
        fs: opt.fs,
        fi: opt.fi,
        off_msec: opt.off_msec,
        sig,
        families,
        sats: opt.sats.clone(),
        sbas,
        qzss: opt.qzss,
        plots: opt.plots,
        // Headless CLI: diagnostics history only pays off when --plots writes it.
        diagnostics: opt.plots,
        exit_on_fix: opt.exit_on_fix,
        json: opt.json.clone(),
    };
    let mut receiver = Receiver::new(
        &config,
        exit_req.clone(),
        Arc::new(Mutex::new(GnssState::new())),
    )?;

    receiver.run_loop(opt.num_msec);

    exit_req.store(true, Ordering::SeqCst);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::parse_freq;

    #[test]
    fn parse_freq_handles_si_suffixes() {
        assert_eq!(parse_freq("10M").unwrap(), 10_000_000.0);
        assert_eq!(parse_freq("420K").unwrap(), 420_000.0);
        assert_eq!(parse_freq("2046000").unwrap(), 2_046_000.0);
        assert_eq!(parse_freq("1g").unwrap(), 1e9);
        assert_eq!(parse_freq("0").unwrap(), 0.0);
        // fractional suffix: f64 rounding, so compare approximately.
        assert!((parse_freq("2.046M").unwrap() - 2_046_000.0).abs() < 1.0);
        assert!(parse_freq("10X").is_err());
        assert!(parse_freq("abc").is_err());
    }
}
