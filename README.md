[![Linux](https://github.com/mx4/gnss-rcv/actions/workflows/linux.yml/badge.svg)](https://github.com/mx4/gnss-rcv/actions/workflows/linux.yml)
[![MacOS](https://github.com/mx4/gnss-rcv/actions/workflows/macos.yml/badge.svg)](https://github.com/mx4/gnss-rcv/actions/workflows/macos.yml)

# gnss-rcv: GNSS receiver for :artificial_satellite: GPS L1 signal in Rust
This app takes as input:
- an SDR IQ recording (several sample formats, sample rates and IFs), or
- an rtl-sdr device

It performs signal acquisition, tracking and ephemeris decoding, and finally computes a position fix.

## Diagnostic output
As the gnss receiver processes the IQ data it periodically updates a web page (index.html + pics) that helps explain the inner state of the decoder. Cf plots/index.html.

![diagnostic output](./assets/iq-output.png)

## User Interface
The UI interface can be started with the command line option -u.
![diagnostic output](./assets/gnss-rcv-ui.png)

## Run with an IQ recording
```
$ RUST_LOG=info cargo run --release -- -f path/to/recording.bin
```
With no `-f`, it runs against the default development recording (2xf32, 2.046 MHz, zero IF).

### Supported IQ formats (`-t`)
| `-t`          | sample layout                                       |
|---------------|-----------------------------------------------------|
| `2xf32`       | interleaved float32 I/Q (default)                   |
| `2xi16`       | interleaved int16 I/Q (e.g. `gps-sdr-sim -b 16`)    |
| `i8`          | single int8, real-only                              |
| `rtlsdr-file` | interleaved uint8 I/Q (an `rtl_sdr` capture)        |
| `1bit`        | 8 hard-limited 1-bit real samples packed per byte   |

### Sample rate & intermediate frequency
The PRN code is resampled to the actual rate, so any sampling frequency works. Set
the rate with `--fs` and the intermediate frequency with `--fi` (both in Hz, default
2.046 MHz / 0 Hz):
```
# 1-bit real recording sampled at 5.456 MHz (IF 4.092 MHz aliases to 1.364 MHz):
$ cargo run --release -- -f resources/gps.samples.1bit.I.fs5456.if4092.bin \
    -t 1bit --fs 5456000 --fi 1364000
```

### Other useful options
- `--num-msec N` / `--off-msec N`: process only N ms, or start N ms into the file.
- `--sats 1,11,30`: restrict acquisition to a subset of PRNs.
- `-p` / `--plots`: write per-SV diagnostic PNGs to `plots/` (off by default).
- `-u`: open the UI; `-l <file>`: also write logs to a file.

## Download an existing IQ recording with GPS L1 signal

Use the helper script to fetch the downloadable recordings into `resources/`:
```
$ ./resources/fetch.sh          # list what's available
$ ./resources/fetch.sh nov3     # the main dev recording (2xf32, 12.7 GiB)
$ ./resources/fetch.sh all      # everything
```

The one I used for most of the development:
https://github.com/codyd51/gypsum/releases/download/1.0/nov_3_time_18_48_st_ives.zip
.. unzip and move the file under resources/. Use "-t 2xf32".

A few online SDR recordings at 1575,42 MHz are available online:
- https://jeremyclark.ca/wp/telecom/rtl-sdr-for-satellite-gps/
- https://s-taka.org/en/gnss-sdr-with-rtl-tcp/
- https://destevez.net/2022/03/timing-sdr-recordings-with-gps/

The info required to download/generate samples data: [README.md](./resources/README.md)

## Simulate a GPS L1 SDR recording
Cf [GPS-SDR-SIM](https://github.com/osqzss/gps-sdr-sim)
```
 ./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
```
This generates an IQ recording w/ 2 int16 per I and Q sample.
You can use this using the cmd-line option "-t 2xi16".

## RTLSDR

## Dependencies
You need to install librtlsdr:
```
$ sudo apt install librtlsdr-dev
```
or
```
$ brew install librtlsdr
```

### Use rtlsdr dongle w/ L1 antenna as input
If you have an rtlsdr dongle with a GPS L1 antenna you can try to run the receiver directly off of the IQ sampled by the device:
```
$ RUST_LOG=warn cargo run --release -- -d
```
WIP: I haven't been able to identify satellites by using rtlsdr directly with my h/w setup. Not sure it's due to a bug or my setup.

### Use rtl_tcp
If you have a device w/ an rtlsdr dongle, you can use rtl_tcp on that host to stream the IQ data to a gnss-rcv instance running on a different host.
Run rtl_tcp on host w/ rtlsdr device:
```
$ rtl_tcp -a
```
and connect to it w/ gnss-rcv:
```
$ RUST_LOG=warn cargo run --release -- -s <hostname>
```
gnss-rcv will automatically configure the sampling rate, center frequency, etc.
WIP: same caveat

### Record from rtl-sdr to file
You can use your rtlsdr device to capture a set of IQ samples that can then be fed to gnss-rcv.

- you need to activate bias-t and power the gps/lna antenna:
```
$ rtl_biast -d 0 -b 1
```
- command to sample L1 at 2046KHz for 10 sec:
```
$ rtl_sdr -f 1575420000 -s 2046000 -n 20460000 output.bin
```
WIP: same caveat

## Resources:
- [RTL-SDR](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)
- [Software Defined GPS](https://www.ocf.berkeley.edu/~marsy/resources/gnss/A%20Software-Defined%20GPS%20and%20Galileo%20Receiver.pdf)
- [GPS-SDR-SIM](https://github.com/osqzss/gps-sdr-sim)
- [Python GPS software: Gypsum](https://github.com/codyd51/gypsum)
- [SWIFT-NAV](https://github.com/swift-nav/libswiftnav)
- [Raw GPS Signal](http://www.jks.com/gps/gps.html)
- [PocketSDR](https://github.com/tomojitakasu/PocketSDR/)

## General info about GNSS
- [GPS Spec: IS-GPS-200N.pdf](https://www.gps.gov/technical/icwg/IS-GPS-200N.pdf)
- [GPS visualisation](https://ciechanow.ski/gps/)
- [GPS signal](https://www.e-education.psu.edu/geog862/node/1407)

## Contributions
Any code contribution is welcome!

## TODO
- use anise for Ephemerides and Almanac
- use anise to compute SV position from keplerian elements
- test + fix rtlsdr support
- support: SBAS, Galileo, QZSS, Beidu.
- use received Almanac to decide which satellite are in view
