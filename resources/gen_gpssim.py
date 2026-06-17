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
import math
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

# Dynamic (moving-receiver) variant — distinct names so it never clobbers the
# static recording the gpssim.rs fix test consumes.
OUT_DYN = "gpssim_dyn_2xi16"
META_DYN = "gpssim_dyn.meta"
MOTION = "gpssim_dyn_motion.csv"
LOG_DYN = "gpssim_dyn.txt"

# gps-sdr-sim samples the -u motion file at a fixed 10 Hz (it ignores the time
# column for stepping), so trajectory points must be spaced 0.1 s apart.
MOTION_HZ = 10


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h: float) -> "tuple[float, float, float]":
    """WGS-84 geodetic (deg, deg, m) -> ECEF (m)."""
    a, f = 6378137.0, 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sp, cp, sl, cl = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    n = a / math.sqrt(1.0 - e2 * sp * sp)
    return ((n + h) * cp * cl, (n + h) * cp * sl, (n * (1.0 - e2) + h) * sp)


def enu_to_ecef_vel(ve: float, vn: float, vu: float, lat_deg: float, lon_deg: float):
    """ENU velocity (m/s) -> ECEF velocity (m/s) at lat/lon (the transpose of
    the receiver's ECEF->ENU rotation, so truth matches the solver's output)."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sp, cp, sl, cl = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return (
        -sl * ve - sp * cl * vn + cp * cl * vu,
        cl * ve - sp * sl * vn + cp * sl * vu,
        cp * vn + sp * vu,
    )


def write_motion_file(path: str, lat: float, lon: float, alt: float, vel_enu, dur: float) -> None:
    """Write a constant-velocity ECEF trajectory (10 Hz) for gps-sdr-sim's -u."""
    x0, y0, z0 = geodetic_to_ecef(lat, lon, alt)
    vx, vy, vz = enu_to_ecef_vel(vel_enu[0], vel_enu[1], vel_enu[2], lat, lon)
    n = int(round(dur * MOTION_HZ))
    with open(path, "w") as fh:
        for i in range(n + 1):
            t = i / MOTION_HZ
            fh.write(f"{t:.1f},{x0 + vx * t:.4f},{y0 + vy * t:.4f},{z0 + vz * t:.4f}\n")


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
    # Optional dynamic mode: --vel E,N,U (m/s, ENU). Pop it before positionals.
    vel_enu = None
    for i, a in enumerate(argv):
        if a == "--vel" and i + 1 < len(argv):
            vel_enu = [float(v) for v in argv[i + 1].split(",")]
            argv = argv[:i] + argv[i + 2 :]
            break
        if a.startswith("--vel="):
            vel_enu = [float(v) for v in a.split("=", 1)[1].split(",")]
            argv = argv[:i] + argv[i + 1 :]
            break
    if vel_enu is not None and len(vel_enu) != 3:
        die("--vel wants E,N,U in m/s, e.g. --vel 10,5,0", 2)

    datetime_arg = argv[1] if len(argv) > 1 else "2026/04/28,17:00:00"  # Geneva Jet d'Eau, a
    location = argv[2] if len(argv) > 2 else "46.2075,6.1557,375"       # verified known-good
    # Dynamic mode is capped at 300 s by gps-sdr-sim; 60 s gives a fix + a long
    # velocity arc. Static keeps the 45 s default the fix test expects.
    duration = argv[3] if len(argv) > 3 else ("60" if vel_enu else "45")

    dynamic = vel_enu is not None
    out = OUT_DYN if dynamic else OUT
    meta = META_DYN if dynamic else META
    log_file = LOG_DYN if dynamic else LOG_FILE
    vel_tag = f"|vel={','.join(str(v) for v in vel_enu)}" if dynamic else ""
    scenario = f"{datetime_arg}|{location}|{duration}|{FS}{vel_tag}"

    # Resolve the gps-sdr-sim binary to an absolute path before we cd elsewhere.
    sim = resolve_sim()
    if shutil.which("curl") is None:
        die("curl not found", 2)

    os.chdir(Path(__file__).resolve().parent)

    # Reuse an existing recording if it already matches this exact scenario.
    if Path(out).is_file() and Path(meta).is_file():
        if f"scenario={scenario}" in Path(meta).read_text().splitlines():
            print(f"gen_gpssim: {out} already matches scenario, skipping generation")
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

    # Static: -l location. Dynamic: write a constant-velocity ECEF trajectory
    # and fly it with -u (the receiver's truth is then a velocity, not a point).
    parts = location.split(",")
    lat, lon = parts[0], parts[1]
    if dynamic:
        alt = float(parts[2]) if len(parts) > 2 else 0.0
        write_motion_file(MOTION, float(lat), float(lon), alt, vel_enu, float(duration))
        mode_args = ["-u", MOTION]
        where = f"vel {vel_enu} m/s ENU from {location}"
    else:
        mode_args = ["-l", location]
        where = f"loc {location}"

    # Run gps-sdr-sim. Capture stdout both to a log file and for PRN parsing.
    print(
        f"gen_gpssim: gps-sdr-sim {duration} s @ {FS} Hz, {where}, "
        f"t {datetime_arg} -> resources/{out}"
    )
    proc = subprocess.run(
        [sim, "-e", nav, *mode_args, "-t", datetime_arg, "-d", duration,
         "-b", "16", "-s", FS, "-o", out],
        capture_output=True,
        text=True,
    )
    log = proc.stdout + proc.stderr
    # gps-sdr-sim's stdout includes a noisy per-0.1s progress counter; keep the
    # full capture in the log file but don't echo it.
    Path(log_file).write_text(log if log.endswith("\n") else log + "\n")
    if proc.returncode != 0 or not (Path(out).is_file() and Path(out).stat().st_size > 0):
        die(f"gps-sdr-sim failed (rc={proc.returncode})", 4)

    # Write the meta file the test reads: truth location and the visible PRNs.
    # gps-sdr-sim prints one "PRN az el range iono" row per simulated channel.
    sats = ",".join(
        f[0] for line in log.splitlines()
        if len(f := line.split()) == 5 and f[0].isdigit()
    )
    vel_line = f"vel_enu={','.join(str(v) for v in vel_enu)}\n" if dynamic else ""
    Path(meta).write_text(
        f"scenario={scenario}\nlat={lat}\nlon={lon}\n{vel_line}sats={sats}\n"
    )

    print(f"gen_gpssim: done -> resources/{out} (truth {lat},{lon}{'; ' + where if dynamic else ''}; sats {sats})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
