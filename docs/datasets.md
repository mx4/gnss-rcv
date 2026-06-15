# GNSS IQ recording datasets

Public raw-IQ recordings evaluated for gnss-rcv, so we don't re-search. The
auto-downloadable ones live in [`resources/fetch.py`](../resources/fetch.py)
(`./resources/fetch.py` to list, `fetch.py <name>` to get one); this file is the
wider survey, including manual-download and crossed-off sources.

We need: **raw IQ** (not RINEX/SBF), the **L1/E1 band** (1575.42 MHz), enough
**bandwidth** for E1 BOC (≳4 MHz), and ≳60 s for a fix. For **OSNMA** we also need
a recording from **2023 or later** (OSNMA went operational Jan 2023; everything we
already had is 2013–2021 and carries no OSNMA bits).

## Recommended — Galileo E1 / OSNMA (to fetch)

| Dataset | Bands | Format | OSNMA | Notes |
|---|---|---|---|---|
| **FGI OSNMA** (FGI-JSDR) | GPS L1 C/A + **Galileo E1** | real **i8**, **26 MHz**, **IF 6.39 MHz** (confirmed) | **✅ yes (2023)** | `fgi-osnma` in fetch.py. ~35 GB / 6 files, CC BY 4.0, **manual download** (Fairdata package flow). **✅ verified: Galileo-only fix at 60.182, 24.829 (Otaniemi/Espoo, Finland)** in ~48 s with `-t i8 --fs 26000000 --fi 6390000 --sig E1B` — cross-validates the BOC-τ on independent real data. Files: clean `OSNMAspoofingdatasets/Scenario1:Clean opensky/OSNMA_cleandata_opensky_460s.dat` (471 s; quote the path — it has a space), jammertest `…/Scenario2…/OSNMA_jammertest2023_17.1.6_740s.dat` (740 s). Both **GPS and Galileo fix** here, to ~30 m of each other (recorded 2023-10-31). (GPS needed the gentler LNAV sync recovery — before that it stalled at 3 ephemerides.) **OSNMA (automatic on E1B): ✅ the 169-byte DSM-PKR public key verifies against the GSC 2023 Merkle root** — proving the real-signal OSNMA decode is byte-perfect — but this clean capture's PKR-dominated window carries no complete DSM-KROOT, so it stops short of full nav-data auth. Run with `--sats 4,9,21,31,34,36` (2.8× real-time). The **Scenario-2 jammertest** (Andøya, authentic fix at 69.275, 15.970; `--sats 3,5,15,24,25,31`) *does* broadcast DSM-KROOT (NB=8) but loses **block 4** to interference (receives 0,1,2,3,5,6,7) — also one block short of auth. Full write-up: [docs/osnma.md](osnma.md). [Etsin](https://etsin.fairdata.fi/dataset/09dc5c1b-933d-4efd-aa66-be2c07fab3b3) · [FGI-JSDR](https://www.maanmittauslaitos.fi/en/research/research/gnss-specialists/fgi-gnss-jamming-and-spoofing-dataset-repository-fgi-jsdr) |
| **FGI-SpoofRepo** (FGI-JSDR) | GPS L1 C/A + **Galileo E1** + GPS L5 + Galileo E5a | raw IQ + spoofing types | no | Multi-band (also useful for a future L5/E5a). [Etsin](https://etsin.fairdata.fi/dataset/367379a8-7d78-4b08-91f0-8027ce7a621b). Repo characterized in [GPS Solutions 2024](https://link.springer.com/article/10.1007/s10291-024-01719-2). |

## For OSNMA crypto testing (not IQ)

- **EUSPA OSNMA test vectors** (Receiver Guidelines v1.3, Jan 2024) — a CSV
  **hex I/NAV stream** per satellite. Not IQ, but the ideal way to validate the
  `galileo-osnma` wiring directly (feed the hex I/NAV + OSNMA bits, no
  acquire/track). From the [GSC OSNMA products](https://www.gsc-europa.eu/gsc-products/OSNMA).
- **Trust anchor** for OSNMA: the **Merkle tree root** / ECDSA public key, also
  from the GSC ([MT](https://www.gsc-europa.eu/gsc-products/OSNMA/MT) /
  [PKI](https://www.gsc-europa.eu/gsc-products/OSNMA/PKI)); **must match the
  recording's epoch.** The Merkle tree was **renewed 2024-01-15**, so the GSC's
  current files don't authenticate a pre-2024 capture. The **2023 anchor** (built
  into [`osnma.rs`](../src/osnma.rs), used for the FGI recording) is Merkle root
  `0E63F552…0148B8` + PKID-1 P-256 key `0374A925…F0F6DB0`. See
  [docs/osnma.md](osnma.md) for the epoch timeline and the renewal keys.

## Already in fetch.py (have / used)

| Dataset | Galileo? | Status |
|---|---|---|
| ION LimeSDR (NL, 10 MHz, 60 s) | yes | **Galileo-only fix ~110 m**; our main E1 recording |
| ION SJTU (Shanghai, L1+E1, 25 MHz, 60 s) | yes | **GPS fix** at 31.025, 121.439 (Shanghai, TTFF ~26 s); Galileo E1 still too sparse for a Galileo-only fix |
| ION RTL-SDR / HackRF / BladeRF (NL) | some | RTL-SDR & HackRF **fix** (Netherlands ~52.18, 4.49 — HackRF now nav-decodes); BladeRF ~13 s, too short |
| PocketSDR (Tokyo, Dec 2021) | yes | ~30 s; GPS+QZSS; too short for an E1 ephemeris |
| CTTC (Spain, 2013) | **IOV** | GPS fix + **EGNOS SBAS** S120/S126; also `--sig E1B` decodes **3 Galileo IOV ephemerides** (E11/E12/E20, GST week 710) — the 2013 4-satellite constellation, too sparse for a Galileo-only fix but real dual-constellation data |
| **tuni2025** (Tampere, 2025) | **yes** | TUNI clear-sky, 50 MHz, **int16 big-endian** (`2xi16-be`; dataset's "32-bit float" label is wrong). Carries both: **Galileo fix** (8 E1B SVs) + **GPS fix** (16 L1CA SVs) at 61.450, 23.856. **✅ first live full OSNMA nav-data authentication** — DSM-KROOT (NB=8) → KROOT verified vs the built-in 2024 PKID-1 key → TESLA chain → 7 SVs authenticated. Prime combined-GPS+Galileo candidate. |
| **texbat-clean** / TEXBAT (UT Austin) | no (GPS-only spoof testbed) | The canonical public **GPS L1 spoofing** battery (Humphreys et al.). Real GPS L1 C/A, **complex baseband int16, 25 Msps** (`-t 2xi16 --fs 25000000`, zero-IF; `scott.m` reads from byte 0, no header). We fetch **`cleanStatic80.bin`** (~80 s, 7.5 GiB) — the spoof-free static reference, which **fixes** (Austin, TX). The spoofed/evil-waveform sets **ds1–ds8** (each ~44 GiB, first 100 s spoof-free then the spoofer ramps in) sit in the [same datastore](https://rnl-data.ae.utexas.edu/datastore/texbat/) — a robustness probe nothing else here offers (watch a tracking loop get dragged off), complementing the Galileo-side OSNMA anti-spoofing on tuni2025/fgi. Full clean sets `cleanStatic.bin`/`cleanDynamic.bin` are ~44/41 GiB. [TEXBAT](https://radionavlab.ae.utexas.edu/texbat/). |
| **ion-gn3s** / SiGe (**North America**, 2013) | no (geometry) | Real **8-bit IF** at 16.368 MHz, IF 4.092 MHz, ~120 s — **runs on the existing `i8` reader** (`-t i8 --fs 16368000 --fi 4092000 --invert-spectrum`). The GN3S front-end is only ~2 MHz wide, so effectively GPS L1 C/A only (PRN 1,7,8,9,11,17,28) despite the `.sdrx` "GPS + Galileo" label. **Inverted IF spectrum** (high-side LO, like the SX3): `--invert-spectrum` flips the carrier→code aiding sign so higher-Doppler SVs stop slipping — deterministically lifting subframes 103→138 and cutting parity errors 11→4 (per-recording, exactly as SJTU is *not* inverted and the flag breaks it). **Not Munich:** SiGe GN3S Sampler v3 (CU GNSS Lab), collected 2013-05-23 — the decoded sky is North-American (G09's sub-satellite point is over Baja, ~15° *below* Munich's horizon). Even with the inversion fixed, only ~4 weak SVs complete ephemerides, so the fix is exactly-determined/high-GDOP (GDOP≈24) and mostly rejected; a pass lands in North America (~36.4,-81.2), **not** the (wrong) "Munich" gate. The week also decodes as 2765/2033 (the +2048 LNAV pin on a pre-2019 capture) — common-mode and cancels for a GPS-only fix. [`SiGe_Bands-L1.dat`](https://sdr.ion.org/SiGe/SiGe_Bands-L1.dat). |
| **ion-ifen** / IFEN SX3 (Munich, 2016) | **yes** | GPS L1 + Galileo E1 (E11/E12/E19/E20), 10 MHz BW. Real **2-bit packed IF** (4 samples/byte, two's-complement), 20.48 MHz, IF 5.5 MHz, ~18 min (5.7 GiB). Uses the **`2bit` reader** in [`recording.rs`](../src/recording.rs). The four samples per byte are **LSB-first** (earliest in bits 1:0): the reader first shipped MSB-first (mirroring the 1-/4-bit readers) and decoded **nothing** — the wrong order time-reverses each 4-sample group, scrambling the carrier phase (96.7°/sample at this IF/fs) and capping every SV at ~35 dB-Hz. LSB-first restores 44–49 dB-Hz on the strong SVs (in line with the strong-SV C/N0 the GN3S front-end reaches on a comparable GPS capture, up to 51 — though gn3s is a *different* recording, North America 2013, not this Munich one). The SX3 front-end also **inverts the IF spectrum** (high-side LO): without `--invert-spectrum` the carrier-aided code loop fights the DLL and slips the Costas loop, so nav parity fails even at 50 dB-Hz and no fix forms. With it, GPS fixes at Munich (48.078, 11.638), TTFF ~25 s. `-t 2bit --fs 20480000 --fi 5500000 --invert-spectrum`. [`IFEN_Bands-L1.stream`](https://sdr.ion.org/IFEN/IFEN_Bands-L1.stream). |

## Evaluated — not usable for us

- **LuGRE** (lunar surface, Firefly Blue Ghost M1, 2025) — GPS/Galileo L1+L5 raw
  IQ recorded *on the Moon* ([Zenodo 16411687](https://zenodo.org/records/16411687),
  CC BY 4.0). Genuinely novel, but **not worth the plumbing**: the `.bin` is a
  Qascom QN400-SPACE block format (per-block 62-byte header) that must be run
  through [`qascom_to_sigmf.py`](https://github.com/daniestevez/lugre) (needs
  `hifitime`/`numpy`/`sigmf`) to get a flat SigMF `ci8`, and **every clip is
  ≤ 2.5 s** — acquisition-only, far too short for a fix. Not in fetch.py.
- **ION OhioTRIGR** (Athens, OH) — real GPS **L1/L2 + Galileo E5a/E5b**, 56.32 MHz
  1-bit. Its L1 is GPS-only (no E1), so nothing new for the L1/E1 path today, but
  it's the best **real L5/E5a** candidate for when that roadmap item lands.
- **ION NTLab** ([`.bin`](https://sdr.ion.org/NTLab/NTLab_Bands_GPS_GLONASS_L12.bin),
  NT1065, Minsk 2017, 7.3 GiB ≈ 138 s) — GPS + **GLONASS** L1/L2, 53 Msps. **Not**
  a plain stream and **not** readable by our `2bit` reader: every byte multiplexes
  *four* bands, 2 bits each — `[7:6]` GPS-L1 (IF −14.58 MHz), `[5:4]` GLONASS-L1,
  `[3:2]` GLONASS-L2, `[1:0]` GPS-L2 — in **sign-magnitude** (`encoding=MS`). Would
  need a dedicated deinterleaving reader, and since we don't decode GLONASS it would
  only yield GPS-L1 — i.e. nothing over `ion-gn3s`. Skipped.
- **FGI-GSRx "Raw GPS L1C I/Q-data"** ([Etsin](https://etsin.fairdata.fi/dataset/63f8b776-680b-4c98-ace7-d5e443f2b1c5))
  — **Skydel-simulated** (like Flexiband below): GPS **L1C** + L1 C/A + Galileo
  E1 OS + **BDS B1C**, `2xi8` 25 MHz, 119 s, CC BY 4.0. A clean modernized-signal
  testbed if an L1C/B1C decoder is ever added; not real-sky.
- **IEEE DataPort "GNSS Recordings for Galileo OSNMA"** — Septentrio **SBF**
  (processed PVT), *not* raw IQ. [link](https://ieee-dataport.org/documents/gnss-recordings-galileo-osnma-evaluation)
- **Fraunhofer Flexiband** (L1/E1, 18 MHz) — Spirent-**simulated**, access-restricted.
- **DemoGRAPE / SANAE IV (Antarctica) scintillation SDR** — scientifically the
  richest polar raw-IF source: INGV + PoliTo NavSAS + EC-JRC "4tuNe" bit-grabber at
  SANAE IV (and Brazil's EACF), continuous since Jan 2016, raw IF in **L1/E1 + L2 +
  L5/E5a** at 5/30 MHz, 7+ yr incl. storm/scintillation events ([GPS Solutions
  2018](https://link.springer.com/article/10.1007/s10291-018-0761-7), [Annals of
  Geophysics 2024, ag-9016](https://www.annalsofgeophysics.eu/index.php/annals/article/view/9016)).
  Would be a great polar-geometry + scintillation stress test, **but not openly
  downloadable** — no public repo, access on-request via NavSAS/INGV. For *raw IF*
  that is openly hosted there's only the off-target **CYGNSS L1 Raw-IF** (NASA
  PO.DAAC) — spaceborne GNSS-reflectometry (±38°, not polar), not a ground fix.
- A 2024 highway E1/E6 set — only **20 ms snapshots**, too short for a fix.
- **galileo-sdr-sim** — generates a trackable E1 signal but its **I/NAV time
  fields (t0e/t0c/GST week) are non-conformant**, so no ephemeris/fix (see the
  build under `~/git/galileo-sdr-sim`; reason we built the hermetic `synth.rs`
  encoder instead).
