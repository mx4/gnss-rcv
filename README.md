[![Linux](https://github.com/mx4/gnss-rcv/actions/workflows/linux.yml/badge.svg)](https://github.com/mx4/gnss-rcv/actions/workflows/linux.yml)
[![MacOS](https://github.com/mx4/gnss-rcv/actions/workflows/macos.yml/badge.svg)](https://github.com/mx4/gnss-rcv/actions/workflows/macos.yml)

# gnss-rcv: GNSS receiver for :artificial_satellite: GPS L1 signal in Rust
This app takes as input:
- an SDR IQ recording
- or an rtl-sdr device

It performs signal acquisition, tracking and ephemeris decoding. Finally it gets a position fix.

## Diagnostic output
As the gnss receiver processes the IQ data it periodically updates a web page (index.html + pics) that helps explain the inner state of the decoder. Cf plots/index.html.

![diagnostic output](./assets/iq-output.png)

## User Interface
The UI interface can be started with the command line option -u.
![diagnostic output](./assets/gnss-rcv-ui.png)

## Run with IQ recording of L1 signal sampled at 2046MHz
```
$ RUST_LOG=info cargo run --release -- -f path/to/recording.bin
```
Note that the app supports multiple IQ file formats: i8, 2xf16, 2xf32, etc. This can be specified via the cmd-line option -t.

## Download an existing IQ recording with GPS L1 signal

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
You can use this using the cmd-line option "-t 2xf16".

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
$ RUST_LOG=warn cargo run --release -- -h <hostname>
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
- handle different sampling frequencies
- use received Almanac to decide which satellite are in view
