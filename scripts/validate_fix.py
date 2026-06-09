#!/usr/bin/env python3
"""Validate DSP / positioning changes against known recordings via the --json summary.

Two checks, each skipping cleanly when its recording is absent:

  - GPS fix (gps-sdr-sim fixture): position within the ~2 km gate vs known truth,
    plus the per-SV RESID spread (the geometry-error diagnostic).
  - Galileo E1-B I/NAV decode + fix (wideband L1 captures): Galileo SVs track and
    decode CRC-valid I/NAV words; a recording long enough to complete >=4 ephemerides
    also asserts a real Galileo-only position fix vs the site's known truth (shorter
    recordings assert only the decode chain).

Returns non-zero if any *present* recording's check fails, so it doubles as a CI /
pre-commit assertion. Usage: ./scripts/validate_fix.py
"""

import json
import math
import os
import re
import subprocess
import sys

GATE_KM = 2.0  # same as the computes_position_fix_gpssim test (0.02° ~= 2 km)


def run_json(args, env_extra=None):
    """Run the receiver with `--json -` and return (summary_dict_or_None, stderr)."""
    env = dict(os.environ, RUST_LOG="warn", **(env_extra or {}))
    proc = subprocess.run(
        ["./target/release/gnss-rcv", *args, "--json", "-"],
        capture_output=True,
        text=True,
        env=env,
    )
    try:
        return json.loads(proc.stdout), proc.stderr
    except json.JSONDecodeError:
        return None, proc.stderr


def print_funnel(summary):
    f = summary.get("funnel", {})
    print(
        "funnel: searched {searched} -> acquired {acquired} -> tracked {tracked} "
        "-> ephemeris {ephemeris} -> used-in-fix {used_in_fix}".format(**f)
    )


# --- GPS L1 C/A: a real position fix vs known truth -------------------------

GPS_FIXTURE = "resources/gpssim_2xi16"
GPS_SATS = "1,2,3,4,6,9,17,19,28,31"
GPS_TRUTH_ECEF = "4396463.3,474169.7,4581510.0"  # Geneva, Jet d'Eau
GPS_TRUTH_LAT, GPS_TRUTH_LON = 46.2075, 6.1557


def validate_gps_fix():
    """-> 'PASS' / 'FAIL' / 'SKIP'."""
    print("=== GPS L1 C/A fix (gpssim) ===")
    if not os.path.isfile(GPS_FIXTURE):
        print(f"SKIP: {GPS_FIXTURE} absent -- generate with ./resources/gen_gpssim.sh")
        return "SKIP"

    summary, stderr = run_json(
        ["-f", GPS_FIXTURE, "-t", "2xi16", "--sats", GPS_SATS, "-x"],
        {"GNSS_TRUTH_ECEF": GPS_TRUTH_ECEF},
    )
    if summary is None:
        print("FAIL: could not parse --json output")
        return "FAIL"
    print_funnel(summary)

    # Per-SV residuals of the final fix attempt (resid ~= common rx clock bias;
    # the spread across SVs is the geometry error).
    resid_lines = [ln for ln in stderr.splitlines() if "RESID" in ln][-len(GPS_SATS.split(",")):]
    resids = [
        float(m.group(1))
        for ln in resid_lines
        if (m := re.search(r"resid=([-+]?[0-9.]+)km", ln))
    ]
    if resids:
        lo, hi = min(resids), max(resids)
        print(f"residual spread: {hi - lo:.3f} km (min {lo:.3f}, max {hi:.3f}, n={len(resids)})")

    fix = summary.get("fix")
    if not fix:
        print("FAIL: NO FIX (check the funnel / RUST_LOG=info for detail)")
        return "FAIL"
    lat, lon = fix["lat"], fix["lon"]
    # ~111 km/deg lat; lon scaled by cos(lat).
    km = math.hypot(
        (lat - GPS_TRUTH_LAT) * 111.0,
        (lon - GPS_TRUTH_LON) * 111.0 * math.cos(math.radians(GPS_TRUTH_LAT)),
    )
    print(f"fix: {lat:.6f}, {lon:.6f}  alt {fix['alt_m']:.1f} m  ({fix['n_sv']} SVs)  error ~{km:.1f} km")
    ok = km <= GATE_KM
    print(f"RESULT: {'PASS' if ok else 'FAIL'} (gate {GATE_KM:.1f} km)")
    return "PASS" if ok else "FAIL"


# --- Galileo E1-B: acquire -> track -> decode I/NAV -> position fix -----------

# Wideband L1 recordings carrying decodable Galileo E1-B I/NAV. Every present one
# is checked: (file, fmt_args, num_msec, fix_expectation_or_None).
#
# fix_expectation = dict(lat, lon, gate_km, alt_band_m, min_eph), checked against
# the JSON `fix` object when the recording is long enough to complete >=min_eph
# ephemerides. The LimeSDR truth (52.177, 4.488, Netherlands) is the same antenna
# site as that capture's GPS/RTL-SDR fixes; the Galileo-only fix lands ~4 km off,
# within the open per-SV ~0.5 ms pseudorange bias, so the gate is looser than the
# GPS 2 km. Recordings too short to complete an ephemeris (PocketSDR ~30 s) pass
# on the decode chain alone.
GAL_CANDIDATES = [
    ("resources/L1_20211226_082212_12MHz_I.bin",
     ["-t", "i8", "--fs", "12M", "--fi", "3M"], 20000, None),  # PocketSDR (~30 s: decode only)
    ("resources/ION_LimeSDR_Bands-L1.2xi16",
     ["-t", "2xi16", "--fs", "10M", "--fi", "420K"], 60000,
     {"lat": 52.177, "lon": 4.488, "gate_km": 6.0, "alt_band_m": (-500.0, 2000.0), "min_eph": 4}),  # ION LimeSDR
]
GAL_MIN_TRACKED = 2
GAL_MIN_WORDS = 3  # CRC-valid I/NAV words across all SVs


def _check_galileo_recording(path, fmt_args, num_msec, fix_exp):
    """Run one Galileo recording; -> True on pass. Always checks the decode chain;
    also asserts a position fix when `fix_exp` is given."""
    print(f"\nusing {path} ({num_msec // 1000}s)")
    summary, _ = run_json(["-f", path, *fmt_args, "--sig", "E1B", "--num-msec", str(num_msec)])
    if summary is None:
        print("  FAIL: could not parse --json output")
        return False
    print_funnel(summary)

    tracked = summary.get("funnel", {}).get("tracked", 0)
    # I/NAV words are counted in the per-SV "subframes" field.
    words = sum(s.get("subframes", 0) for s in summary.get("sats", []))
    decoders = [s["sv"] for s in summary.get("sats", []) if s.get("subframes", 0) > 0]
    print(f"  tracked {tracked} SVs; {words} CRC-valid I/NAV words from {', '.join(decoders) or 'none'}")
    ok = tracked >= GAL_MIN_TRACKED and words >= GAL_MIN_WORDS
    if not ok:
        print(f"  decode FAIL (need >= {GAL_MIN_TRACKED} tracked & >= {GAL_MIN_WORDS} words)")

    if fix_exp is not None:
        eph = summary.get("funnel", {}).get("ephemeris", 0)
        fix = summary.get("fix")
        if eph < fix_exp["min_eph"]:
            print(f"  fix FAIL: {eph} ephemerides (need >= {fix_exp['min_eph']})")
            ok = False
        elif not fix:
            print("  fix FAIL: NO FIX (check the funnel / RUST_LOG=info for detail)")
            ok = False
        else:
            lat, lon, alt = fix["lat"], fix["lon"], fix["alt_m"]
            km = math.hypot(
                (lat - fix_exp["lat"]) * 111.0,
                (lon - fix_exp["lon"]) * 111.0 * math.cos(math.radians(fix_exp["lat"])),
            )
            lo, hi = fix_exp["alt_band_m"]
            print(f"  fix: {lat:.6f}, {lon:.6f}  alt {alt:.1f} m  ({fix['n_sv']} SVs)  "
                  f"error ~{km:.1f} km (gate {fix_exp['gate_km']:.1f})")
            if km > fix_exp["gate_km"]:
                print(f"  fix FAIL: {km:.1f} km > gate {fix_exp['gate_km']:.1f} km")
                ok = False
            if not (lo <= alt <= hi):
                print(f"  fix FAIL: altitude {alt:.1f} m outside [{lo:.0f}, {hi:.0f}] m")
                ok = False

    print(f"  RESULT: {'PASS' if ok else 'FAIL'}")
    return ok


def validate_galileo_decode():
    """-> 'PASS' / 'FAIL' / 'SKIP'."""
    print("\n=== Galileo E1-B I/NAV decode + fix ===")
    present = [c for c in GAL_CANDIDATES if os.path.isfile(c[0])]
    if not present:
        print("SKIP: no Galileo recording present -- fetch one with ./resources/fetch.py pocketsdr")
        return "SKIP"
    return "PASS" if all(_check_galileo_recording(*c) for c in present) else "FAIL"


def main() -> int:
    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True)
    if root.returncode != 0:
        print("error: not inside a git repository", file=sys.stderr)
        return 1
    os.chdir(root.stdout.strip())

    print("building (release)...")
    if subprocess.run(["cargo", "build", "--release"]).returncode != 0:
        print("RESULT: build failed", file=sys.stderr)
        return 1

    results = {
        "GPS fix": validate_gps_fix(),
        "Galileo I/NAV": validate_galileo_decode(),
    }

    print("\n=== summary ===")
    for name, status in results.items():
        print(f"  {name:14} {status}")
    # Fail only if a present recording's check failed; all-skip is still exit 0.
    return 1 if "FAIL" in results.values() else 0


if __name__ == "__main__":
    sys.exit(main())
