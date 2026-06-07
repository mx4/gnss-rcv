```
-rw-r--r-- 1 pi 12699331696 Nov  5 23:15 nov_3_time_18_48_st_ives
-rw-r--r-- 1 pi   490221600 Apr 25 22:33 gpssim.bin
-rw-r--r-- 1 pi         716 Apr 25 22:33 gpssim.txt
-rw-r--r-- 1 pi   240000000 Apr 25 15:55 GPS-L1-2022-03-27.sigmf-data
-rw-r--r-- 1 pi    16368000 Mar  3  2014 gioveAandB_short.bin
```

The downloadable recordings can be fetched with [fetch.sh](./fetch.sh):
```
./resources/fetch.sh            # list what's available / already present
./resources/fetch.sh jks-1bit   # fetch one (or several) by name
./resources/fetch.sh all        # fetch everything (incl. the 12.7 GiB nov3)
```

## nov_3_time_18_48_st_ives
https://github.com/codyd51/gypsum/releases
file-type: 2xf32
Captured in the UK in Nov 2023.
Download/unzip it with `./resources/fetch.sh nov3`.

## Generating a recording with gps-sdr-sim ([gen_gpssim.sh](./gen_gpssim.sh))

`gen_gpssim.sh` produces a recording end-to-end from a date + location: it
derives the day-of-year, downloads the matching broadcast ephemeris (brdc) for
that day, runs gps-sdr-sim, and writes `gpssim_gen_2xi16` plus a
`gpssim_gen.meta` (truth lat/lon + visible PRNs) that the integration test reads.
```
./resources/gen_gpssim.sh                                   # default: Geneva Jet d'Eau, 2026/04/28
./resources/gen_gpssim.sh 2023/06/15,12:00:00 48.8566,2.3522,35 60
```
- Ephemeris is fetched from **ESA GSSC** over FTP, which (unlike NASA CDDIS)
  serves brdc without an Earthdata login:
  `ftp://gssc.esa.int/gnss/data/daily/<YYYY>/<DDD>/brdc<DDD>0.<YY>n.gz`.
- Needs `curl`, `gunzip`, and a built gps-sdr-sim binary (`$GPS_SDR_SIM`, or
  `~/git/gps-sdr-sim/gps-sdr-sim`, or on `PATH`). Pick a real past date — recent
  days may not have a brdc posted yet.
- Drives the `generates_and_solves_gpssim` test in [../tests/gpssim.rs](../tests/gpssim.rs).

## gpssim.bin (manual gps-sdr-sim example)
cf https://github.com/osqzss/gps-sdr-sim
result of (Geneva, Jet d'Eau -- the fixture's default scenario):
 ./gps-sdr-sim -b 16 -d 45 -t 2026/04/28,17:00:00 -l 46.2075,6.1557,375 -e brdc1180.26n -s 2046000
file-type: 2xi16
A worked example of invoking gps-sdr-sim by hand (the steps `gen_gpssim.sh`
automates). The `gpssim_2xi16` fixture the fixture-based tests read is produced
by `gen_gpssim.sh` (Geneva default); generate it with `./resources/gen_gpssim.sh`.

```
./gps-sdr-sim -b 16 -d 45 -t 2026/04/28,17:00:00 -l 46.2075,6.1557,375 -e brdc1180.26n -s 2046000
Using static location mode.
xyz =   4396463.3,    474169.7,   4581510.0
llh =   46.207500,    6.155700,       375.0
Start time = 2026/04/28,17:00:00 (2416:234000)
Duration = 45.0 [sec]
01  134.4  40.9  21981064.0   9.1
02  140.1  12.4  24692095.6  16.8
03   54.8  63.7  20555397.7   6.6
04  180.0  69.4  20591518.5   6.6
06  309.5  30.7  22779445.7  10.5
09  212.4  38.2  22161552.4   9.9
17  244.7  39.5  22311933.7   9.5
19  283.8  37.9  21943836.7   9.5
28   41.5  15.8  24129434.2  12.9
31   69.2  26.3  22829955.5  11.1
Done!
Process time = 1.5 [sec]
```

## GPS-L1-2022-03-27.sigmf-data
source: https://zenodo.org/records/6394603
SigMF: complex int16 (ci16), 4 MHz, centered on L1 (1575.42 MHz, so fi=0).
Acquires and tracks GPS SVs (e.g. G31 @ ~40 dB-Hz):
```
RUST_LOG=info cargo run --release -- -f resources/GPS-L1-2022-03-27.sigmf-data -t 2xi16 --fs 4000000
```
Only ~15s long, so it is too short to decode an ephemeris / get a position fix.

## gioveAandB_short.bin
http://gfix.dk/matlab-gnss-sdr-book/gnss-signal-records/
Real int8 samples (one signed byte per sample) at 16367600 Hz, IF=4130400 Hz:
```
cargo run --release -- -f resources/gioveAandB_short.bin -t i8 --fs 16367600 --fi 4130400
```

## gps.samples.1bit.I.fs5456.if4092.bin
http://www.jks.com/gps/gps.html
1-bit hard-limited real samples, 8 packed per byte (MSB first), ~81.8s.
fs=5.456 MHz, IF=4.092 MHz. Use the new "-t 1bit" type.

Note: IF 4.092 MHz is above Nyquist (fs/2 = 2.728 MHz), so the real signal
aliases to fs - IF = 1.364 MHz with an inverted spectrum. Pass that aliased IF
(--fi 1364000), not 4.092 MHz -- using 4092000 selects the mirrored sideband and
the nav data never bit-syncs.
```
cargo run --release -- -f resources/gps.samples.1bit.I.fs5456.if4092.bin -t 1bit --fs 5456000 --fi 1364000
```
Acquires and tracks ~7 SVs (G01/G09/G21/G25/G29/G30/G31) and reaches bit/frame
sync, but at a marginal ~30 dB-Hz (near the 29 dB-Hz drop threshold) sync drops
repeatedly, so it does not hold long enough to decode an ephemeris / get a fix.
