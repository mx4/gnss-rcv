#!/usr/bin/env python3
"""Validate a positioning / DSP change against the known-truth gps-sdr-sim fixture.

Builds release, runs the receiver to the first fix, and reads its machine-readable
`--json` summary (fix + funnel) from stdout. The per-SV RESID diagnostic (enabled
by GNSS_TRUTH_ECEF) stays a stderr log line, parsed for the residual spread.

Prints the funnel, the residual spread (across SVs), and the fix error vs truth,
and returns non-zero if the fix is missing or worse than the ~2 km gate — so it
doubles as an assertion in CI / pre-commit.

Skips cleanly (exit 0) when the fixture is absent, so it is always safe to run.

Usage: ./scripts/validate_fix.py
"""

import json
import math
import os
import re
import subprocess
import sys

FIXTURE = "resources/gpssim_2xi16"
SATS = "1,2,3,4,6,9,17,19,28,31"
# Geneva, Jet d'Eau -- the fixture's scenario (resources/README.md).
TRUTH_ECEF = "4396463.3,474169.7,4581510.0"
TRUTH_LAT = 46.2075
TRUTH_LON = 6.1557
# Same gate as the computes_position_fix_gpssim test (0.02° ~= 2 km).
GATE_KM = 2.0


def main() -> int:
    root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
    )
    if root.returncode != 0:
        print("error: not inside a git repository", file=sys.stderr)
        return 1
    os.chdir(root.stdout.strip())

    if not os.path.isfile(FIXTURE):
        print(f"skip: {FIXTURE} not present -- generate it with ./resources/gen_gpssim.sh")
        return 0

    print("building (release)...")
    if subprocess.run(["cargo", "build", "--release"]).returncode != 0:
        print("RESULT: build failed", file=sys.stderr)
        return 1

    print("running to first fix...")
    env = dict(os.environ, GNSS_TRUTH_ECEF=TRUTH_ECEF, RUST_LOG="warn")
    proc = subprocess.run(
        [
            "./target/release/gnss-rcv",
            "-f", FIXTURE,
            "-t", "2xi16",
            "--sats", SATS,
            "-x",
            "--json", "-",  # fix + funnel as JSON on stdout; logs (RESID) on stderr
        ],
        capture_output=True,
        text=True,
        env=env,
    )

    # stdout is the pure JSON run summary; stderr carries the logs.
    try:
        summary = json.loads(proc.stdout)
    except json.JSONDecodeError:
        print("RESULT: could not parse --json output", file=sys.stderr)
        sys.stderr.write(proc.stdout)
        return 1

    funnel = summary.get("funnel", {})
    print(
        "funnel: searched {searched} -> acquired {acquired} -> tracked {tracked} "
        "-> ephemeris {ephemeris} -> used-in-fix {used_in_fix}".format(**funnel)
    )

    # Per-SV residuals of the final fix attempt (resid ~= common rx clock bias;
    # the spread across SVs is the geometry error). Keep the last batch, one/SV.
    n_sats = len(SATS.split(","))
    resid_lines = [ln for ln in proc.stderr.splitlines() if "RESID" in ln][-n_sats:]
    if resid_lines:
        print("--- residuals (final attempt) ---")
        for ln in resid_lines:
            print(ln)
        resids = [float(m.group(1)) for ln in resid_lines
                  if (m := re.search(r"resid=([-+]?[0-9.]+)km", ln))]
        if resids:
            lo, hi = min(resids), max(resids)
            print(
                f"residual spread: {hi - lo:.3f} km "
                f"(min {lo:.3f}, max {hi:.3f}, n={len(resids)})"
            )

    fix = summary.get("fix")
    if not fix:
        print("RESULT: NO FIX (check the funnel above / RUST_LOG=info for detail)")
        return 1

    lat, lon = fix["lat"], fix["lon"]
    print("--- fix ---")
    print(f"position: {lat:.6f}, {lon:.6f}  alt {fix['alt_m']:.1f} m  ({fix['n_sv']} SVs)")
    dlat, dlon = lat - TRUTH_LAT, lon - TRUTH_LON
    # ~111 km/deg lat; lon scaled by cos(lat).
    km = math.hypot(dlat * 111.0, dlon * 111.0 * math.cos(math.radians(TRUTH_LAT)))
    print(f"fix error vs truth: dlat={dlat:.4f} dlon={dlon:.4f}  (~{km:.1f} km)")

    ok = km <= GATE_KM
    print(f"RESULT: {'PASS' if ok else 'FAIL'} (gate {GATE_KM:.1f} km)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
