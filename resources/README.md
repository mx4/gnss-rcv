```
-rw-r--r-- 1 pi 12699331696 Nov  5 23:15 nov_3_time_18_48_st_ives
-rw-r--r-- 1 pi   490221600 Apr 25 22:33 gpssim.bin
-rw-r--r-- 1 pi         716 Apr 25 22:33 gpssim.txt
-rw-r--r-- 1 pi   240000000 Apr 25 15:55 GPS-L1-2022-03-27.sigmf-data
-rw-r--r-- 1 pi    16368000 Mar  3  2014 gioveAandB_short.bin
```

## nov_3_time_18_48_st_ives
https://github.com/codyd51/gypsum/releases
file-type: 2xf32
Captured in the UK in Nov 2023.
You can use this script to download/unzip the file [get_iq_samples.sh](./resources/get_iq_samples.sh).

## gpssim.bin
cf https://github.com/osqzss/gps-sdr-sim
result of:
 ./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
file-type: 2xi16

```
./gps-sdr-sim -b 16 -d 60 -t 2022/01/01,01:02:03 -l 35.681298,139.766247,10.0 -e brdc0010.22n -s 2046000
Using static location mode.
xyz =  -3959617.5,   3350136.6,   3699531.5
llh =   35.681298,  139.766247,        10.0
Start time = 2022/01/01,01:02:03 (2190:522123)
Duration = 60.0 [sec]
05  146.8  12.9  24517023.8   9.6
10  315.8  31.4  22789584.0   4.9
12  157.5  30.8  22679311.2   6.1
13   79.9  19.7  23736998.7   7.6
15   77.0  50.6  21203005.0   4.1
18  230.4  24.7  23191867.9   6.2
23  298.9  65.8  20585659.2   3.3
24  356.1  79.4  19958939.4   3.2
25  186.1   9.4  24769194.9  10.1
28   42.8  14.7  24631379.7   7.9
32  285.5   2.0  25712738.2   6.4
Time into run = 60.0
Done!
Process time = 7.5 [sec]
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
