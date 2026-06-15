#!/usr/bin/env python3
"""Generate a gps-sdr-sim IQ recording end-to-end for the gnss-rcv fix test.

Pick a date+time and a location, download the matching broadcast ephemeris
(brdc) for that day, run gps-sdr-sim, and write the recording plus a .meta file
(truth location + visible PRNs) that tests/gpssim.rs consumes.

    ./resources/gen_gpssim.py [date,time] [lat,lon,alt] [duration_sec]

Defaults to a known-good scenario (Geneva, Jet d'Eau, 2026/04/28) that is
verified to reach a position fix. Override via the args.

Needs curl and a built gps-sdr-sim binary (set $GPS_SDR_SIM, else it looks in
~/git/gps-sdr-sim/gps-sdr-sim and on PATH). Exits non-zero -- so the test skips
cleanly -- when a tool or the network is unavailable.

Ephemeris source: ESA GSSC, which serves brdc over FTP without authentication
(unlike CDDIS, which requires an Earthdata login).
"""

import gzip
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

FS = "2046000"

OUT = "gpssim_gen_2xi16"
META = "gpssim_gen.meta"
LOG_FILE = "gpssim_gen.txt"


def die(msg: str, code: int) -> "None":
    print(f"gen_gpssim: {msg}", file=sys.stderr)
    raise SystemExit(code)


def resolve_sim() -> str:
    """Resolve the gps-sdr-sim binary: $GPS_SDR_SIM, then ~/git, then PATH."""
    sim = os.environ.get("GPS_SDR_SIM", "")
    if not sim:
        home_sim = Path.home() / "git" / "gps-sdr-sim" / "gps-sdr-sim"
        if os.access(home_sim, os.X_OK):
            sim = str(home_sim)
        else:
            sim = shutil.which("gps-sdr-sim") or ""
    if not sim or not os.access(sim, os.X_OK):
        die("gps-sdr-sim not found (set $GPS_SDR_SIM or build it in ~/git/gps-sdr-sim)", 2)
    return sim


def download_ephemeris(nav: str, year: str, doy: str) -> None:
    """Fetch + decompress the day's brdc into `nav` (skip if already present)."""
    if Path(nav).is_file():
        return
    url = f"ftp://gssc.esa.int/gnss/data/daily/{year}/{doy}/{nav}.gz"
    print(f"gen_gpssim: downloading {url}")
    part = f"{nav}.gz.part"
    rc = subprocess.run(
        ["curl", "-sS", "--fail", "-m", "60", "-o", part, url]
    ).returncode
    if rc != 0:
        Path(part).unlink(missing_ok=True)
        die("ephemeris download failed (offline, or no brdc for that day yet)", 3)
    try:
        with gzip.open(part, "rb") as src, open(nav, "wb") as dst:
            shutil.copyfileobj(src, dst)
    except OSError:
        Path(part).unlink(missing_ok=True)
        Path(nav).unlink(missing_ok=True)
        die("gunzip failed", 3)
    Path(part).unlink(missing_ok=True)


def main(argv: "list[str]") -> int:
    datetime_arg = argv[1] if len(argv) > 1 else "2026/04/28,17:00:00"  # Geneva Jet d'Eau, a
    location = argv[2] if len(argv) > 2 else "46.2075,6.1557,375"       # verified known-good
    duration = argv[3] if len(argv) > 3 else "45"                       # scenario (see README)

    scenario = f"{datetime_arg}|{location}|{duration}|{FS}"

    # Resolve the gps-sdr-sim binary to an absolute path before we cd elsewhere.
    sim = resolve_sim()
    if shutil.which("curl") is None:
        die("curl not found", 2)

    os.chdir(Path(__file__).resolve().parent)

    # Reuse an existing recording if it already matches this exact scenario.
    if Path(OUT).is_file() and Path(META).is_file():
        if f"scenario={scenario}" in Path(META).read_text().splitlines():
            print(f"gen_gpssim: {OUT} already matches scenario, skipping generation")
            return 0

    # date -> day-of-year + 2-digit year.
    datepart = datetime_arg.split(",", 1)[0]  # e.g. 2022/01/01
    try:
        dt = datetime.strptime(datepart, "%Y/%m/%d")
    except ValueError:
        die(f"cannot parse date '{datepart}' (want YYYY/MM/DD)", 2)
    doy = dt.strftime("%j")
    year = dt.strftime("%Y")
    yy = year[2:4]
    nav = f"brdc{doy}0.{yy}n"

    # Download the broadcast ephemeris for that day from ESA (auth-free FTP).
    download_ephemeris(nav, year, doy)

    # Run gps-sdr-sim. Capture stdout both to a log file and for PRN parsing.
    print(
        f"gen_gpssim: gps-sdr-sim {duration} s @ {FS} Hz, loc {location}, "
        f"t {datetime_arg} -> resources/{OUT}"
    )
    proc = subprocess.run(
        [sim, "-e", nav, "-l", location, "-t", datetime_arg, "-d", duration,
         "-b", "16", "-s", FS, "-o", OUT],
        capture_output=True,
        text=True,
    )
    log = proc.stdout + proc.stderr
    # gps-sdr-sim's stdout includes a noisy per-0.1s progress counter; keep the
    # full capture in the log file but don't echo it.
    Path(LOG_FILE).write_text(log if log.endswith("\n") else log + "\n")
    if proc.returncode != 0 or not (Path(OUT).is_file() and Path(OUT).stat().st_size > 0):
        die(f"gps-sdr-sim failed (rc={proc.returncode})", 4)

    # Write the meta file the test reads: truth location and the visible PRNs.
    # gps-sdr-sim prints one "PRN az el range iono" row per simulated channel.
    parts = location.split(",")
    lat, lon = parts[0], parts[1]
    sats = ",".join(
        f[0] for line in log.splitlines()
        if len(f := line.split()) == 5 and f[0].isdigit()
    )
    Path(META).write_text(
        f"scenario={scenario}\nlat={lat}\nlon={lon}\nsats={sats}\n"
    )

    print(f"gen_gpssim: done -> resources/{OUT} (truth {lat},{lon}; sats {sats})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
