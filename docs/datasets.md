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
| **FGI OSNMA** (FGI-JSDR) | GPS L1 C/A + **Galileo E1** | real **i8**, 26 MHz, BW 4.2 MHz, LO ≈1569.03 MHz (→ L1 at IF ≈6.39 MHz) | **✅ yes (2023)** | `fgi-osnma` in fetch.py. ~35 GB / 6 files, CC BY 4.0, **manual download** (Fairdata package flow). Clean open-sky **and** Jammertest-2023 spoofing scenarios. The clean one is for a normal E1 fix; OSNMA-bearing for the auth work. Run: `-t i8 --fs 26000000 --fi 6390000 --sig E1B` (confirm the exact IF/filenames from the dataset README). Processed by the Finns' own SDR receiver FGI-GSRx, so it's software-receiver-friendly. [Etsin](https://etsin.fairdata.fi/dataset/09dc5c1b-933d-4efd-aa66-be2c07fab3b3) · [FGI-JSDR](https://www.maanmittauslaitos.fi/en/research/research/gnss-specialists/fgi-gnss-jamming-and-spoofing-dataset-repository-fgi-jsdr) |
| **FGI-SpoofRepo** (FGI-JSDR) | GPS L1 C/A + **Galileo E1** + GPS L5 + Galileo E5a | raw IQ + spoofing types | no | Multi-band (also useful for a future L5/E5a). [Etsin](https://etsin.fairdata.fi/dataset/367379a8-7d78-4b08-91f0-8027ce7a621b). Repo characterized in [GPS Solutions 2024](https://link.springer.com/article/10.1007/s10291-024-01719-2). |

## For OSNMA crypto testing (not IQ)

- **EUSPA OSNMA test vectors** (Receiver Guidelines v1.3, Jan 2024) — a CSV
  **hex I/NAV stream** per satellite. Not IQ, but the ideal way to validate the
  `galileo-osnma` wiring directly (feed the hex I/NAV + OSNMA bits, no
  acquire/track). From the [GSC OSNMA products](https://www.gsc-europa.eu/gsc-products/OSNMA).
- **Trust anchor** for OSNMA: the **Merkle tree root** / ECDSA public key, also
  from the GSC ([MT](https://www.gsc-europa.eu/gsc-products/OSNMA/MT) /
  [PKI](https://www.gsc-europa.eu/gsc-products/OSNMA/PKI)); must match the
  recording's date (PKID).

## Already in fetch.py (have / used)

| Dataset | Galileo? | Status |
|---|---|---|
| ION LimeSDR (NL, 10 MHz, 60 s) | yes | **Galileo-only fix ~110 m**; our main E1 recording |
| ION SJTU (Shanghai, L1+E1, 25 MHz, 60 s) | yes | tracks 4 E1 SVs but decode too sparse for a fix |
| ION BladeRF / HackRF / RTL-SDR | some | too short / narrow / don't nav-decode |
| PocketSDR (Tokyo, Dec 2021) | yes | ~30 s; GPS+QZSS; too short for an E1 ephemeris |
| CTTC (Spain, 2013) | no (GPS) | GPS fix + **EGNOS SBAS** S120/S126 |

## Evaluated — not usable for us

- **IEEE DataPort "GNSS Recordings for Galileo OSNMA"** — Septentrio **SBF**
  (processed PVT), *not* raw IQ. [link](https://ieee-dataport.org/documents/gnss-recordings-galileo-osnma-evaluation)
- **Fraunhofer Flexiband** (L1/E1, 18 MHz) — Spirent-**simulated**, access-restricted.
- A 2024 highway E1/E6 set — only **20 ms snapshots**, too short for a fix.
- **galileo-sdr-sim** — generates a trackable E1 signal but its **I/NAV time
  fields (t0e/t0c/GST week) are non-conformant**, so no ephemeris/fix (see the
  build under `~/git/galileo-sdr-sim`; reason we built the hermetic `synth.rs`
  encoder instead).
