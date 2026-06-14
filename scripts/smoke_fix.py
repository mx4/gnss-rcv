#!/usr/bin/env python3
"""Quick position-fix smoke test across every manifest recording.

For each recording present under resources/, seek past the cold-start warmup and
run a short window with --exit-on-fix, then report whether a fix was solved and
how long it took. A fast "do they all still decode?" regression sweep, driven by
the same manifest (and canonical per-file flags) the receiver and fetch.py use --
complementary to the truth-gated, decode-depth checks in validate_fix.py.

The window is the dominant cost (a GPS fix needs ~3 subframes ~= 18-30 s decoded),
so seeking 5 s in only trims acquisition jitter; the 30 s window is what must be
long enough. Recordings shorter than that, or known not to fix (E1B-auth only,
acquire-but-no-decode), are listed in EXPECT so the sweep doesn't cry wolf.

Usage:
  ./scripts/smoke_fix.py                  # every present recording
  ./scripts/smoke_fix.py ifen gn3s        # only names containing these substrings
  ./scripts/smoke_fix.py -j 4             # run 4 recordings in parallel
  ./scripts/smoke_fix.py --window 45 --off 5
"""

import argparse
import json
import math
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

# Known site truth (lat, lon) for recordings whose location is documented, so a
# fix at the wrong place (cross-correlation lock, bad ephemeris) is caught rather
# than counted as success. Coarse GATE -- a continent/city check, not precision.
# Unlisted recordings only assert that *a* fix was solved.
GATE_KM = 60.0
SITES = {
    "nov3": (52.33, -0.07),       # St Ives, Cambridgeshire UK (not Cornwall!)
    "cttc": (41.2748, 1.9876),    # CTTC, Spain
    "ion-lime": (52.177, 4.488),  # ION, Netherlands
    "ion-rtlsdr": (52.177, 4.488),
    "ion-hackrf": (52.177, 4.488),
    "ion-bladerf": (52.177, 4.488),
    # ion-gn3s is NOT Munich: it's a CU GNSS Lab SiGe Sampler v3 capture from
    # North America (2013-05-23), mislabeled "Munich" upstream. Its true antenna
    # coords are undocumented and a 4-SV fix is exactly-determined, so there is
    # no point to gate against -- left out of SITES (asserts only that *a* fix is
    # solved, when the weak geometry lets one through).
    "ion-ifen": (48.14, 11.58),   # ION, Munich (IFEN SX3, 2016)
    "sjtu": (31.20, 121.50),      # SJTU, Shanghai
    "tuni2025": (61.45, 23.857),  # Tampere, Finland
    "texbat-clean": (30.29, -97.74),  # TEXBAT, Austin TX
}


def km_from(name, fix):
    truth = SITES.get(name)
    if not truth or not fix:
        return None
    return math.hypot((fix["lat"] - truth[0]) * 111.0,
                      (fix["lon"] - truth[1]) * 111.0 * math.cos(math.radians(truth[0])))

# bytes per (real or complex) sample, by -t type -- to estimate duration so the
# window never overruns the file and short recordings are flagged not failed.
BYTES_PER_SAMPLE = {
    "2xf32": 8.0, "2xi16": 4.0, "2xi16-be": 4.0, "2xi8": 2.0, "i8": 1.0,
    "rtlsdr-file": 2.0, "4bit": 0.5, "2bit": 0.25, "1bit": 0.125,
}

# Recordings that legitimately do NOT yield a GPS/Galileo position fix in a short
# window -- so a "no fix" is the expected result, not a regression.
#   decode:   acquire/track/decode is the goal; no fix expected (auth-only or weak).
#   short:    recording is too short to complete >=4 ephemerides in any window.
#   weakgeom: completes ephemerides but only ~4 weak SVs -> exactly-determined,
#             high-GDOP fix that is mostly rejected; a pass is fine, a miss is not a regression.
EXPECT = {
    "fgi-osnma": "decode",     # Galileo OSNMA auth focus; not a positioning capture
    "pocketsdr": "decode",     # ~30 s PocketSDR -- decode demo, below the ephemeris floor
    "ion-bladerf": "short",    # ~13 s -- below the ~25 s ephemeris floor
    "zenodo-sigmf": "short",   # short 4 MHz capture; window hits EOF before a fix
    "jks-1bit": "decode",      # 1-bit hard-limited demo; tracks but does not complete ephemerides
    "ion-gn3s": "weakgeom",    # North-America capture (not Munich); only ~4 weak SVs
                               # complete -> exactly-determined, high-GDOP, mostly rejected
                               # (a pass lands in N. America, ~36.4,-81.2). Not a bad fix.
}


def parse_fs(flags):
    toks = flags.split()
    for i, t in enumerate(toks):
        if t == "--fs" and i + 1 < len(toks):
            v = toks[i + 1]
            mult = {"M": 1e6, "K": 1e3, "k": 1e3, "m": 1e6, "G": 1e9}.get(v[-1], 1.0)
            return float(v[:-1] if mult != 1.0 else v) * mult
    return None


def file_type(flags):
    toks = flags.split()
    for i, t in enumerate(toks):
        if t == "-t" and i + 1 < len(toks):
            return toks[i + 1]
    return None


def duration_sec(path, flags):
    fs, ft = parse_fs(flags), file_type(flags)
    bps = BYTES_PER_SAMPLE.get(ft)
    if not fs or not bps:
        return None
    return os.path.getsize(path) / (fs * bps)


def run_one(entry, off_s, window_s):
    name, flags = entry["name"], entry["flags"]
    path = os.path.join("resources", entry["dest"])
    dur = duration_sec(path, flags)
    off = off_s if (dur is None or dur > off_s + 10) else 0
    win = window_s
    if dur is not None:
        win = min(window_s, max(0.0, dur - off - 0.5))
    expect = EXPECT.get(name, "fix")

    args = ["./target/release/gnss-rcv", "-f", path, *flags.split(),
            "--off-msec", str(int(off * 1000)), "--num-msec", str(int(win * 1000)),
            "-x", "--json", "-"]
    proc = subprocess.run(args, capture_output=True, text=True,
                          env=dict(os.environ, RUST_LOG="error"))
    try:
        s = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return name, expect, dict(error=proc.stderr.strip().splitlines()[-1:] or "no JSON")

    f, fn, st = s.get("fix"), s.get("funnel", {}), s.get("stats", {})
    return name, expect, dict(
        fix=f, win=win, off=off,
        tracked=fn.get("tracked", 0), eph=fn.get("ephemeris", 0),
        ttff=st.get("ttff_sec"),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("filters", nargs="*", help="only names containing these substrings")
    ap.add_argument("-j", "--jobs", type=int, default=1, help="parallel recordings")
    ap.add_argument("--off", type=float, default=5.0, help="seek offset (s)")
    # -x exits at the first fix, so this is only a ceiling: fast recordings stop
    # at their TTFF and pay nothing for the headroom slow ones need.
    ap.add_argument("--window", type=float, default=50.0, help="run window ceiling (s)")
    ap.add_argument("--no-build", action="store_true")
    args = ap.parse_args()

    root = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                          capture_output=True, text=True).stdout.strip()
    os.chdir(root)
    if not args.no_build:
        print("building (release)...")
        if subprocess.run(["cargo", "build", "--release"]).returncode != 0:
            return 1

    manifest = json.load(open("resources/manifest.json"))
    todo = [e for e in manifest
            if e.get("flags") and os.path.isfile(os.path.join("resources", e["dest"]))
            and (not args.filters or any(g in e["name"] for g in args.filters))]
    if not todo:
        print("no matching present recordings")
        return 0

    print(f"smoke-testing {len(todo)} recordings (off={args.off}s window<={args.window}s, "
          f"jobs={args.jobs})\n")
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        results = list(ex.map(lambda e: run_one(e, args.off, args.window), todo))

    print(f"{'recording':14} {'result':9} {'ttff':>6}  detail")
    print("-" * 72)
    regressions = 0
    for name, expect, r in sorted(results, key=lambda x: x[0]):
        if r.get("error"):
            status, detail, bad = "ERROR", str(r["error"]), expect == "fix"
        elif r["fix"]:
            f = r["fix"]
            km = km_from(name, f)
            loc = f"{f['lat']:.5f},{f['lon']:.5f} {f['n_sv']}sv"
            if km is not None and km > GATE_KM:
                status, bad = "bad-fix", expect == "fix"
                detail = f"{loc}  {km:.0f}km OFF truth"
            else:
                status, bad = "FIX", False
                detail = loc + (f"  ({km:.0f}km)" if km is not None else "") + f"  (win {r['win']:.0f}s)"
        else:
            status = "no-fix"
            detail = f"tracked {r['tracked']} eph {r['eph']}  (win {r['win']:.0f}s)"
            bad = expect == "fix"
        if expect != "fix" and status != "FIX":
            status = f"{status}*"  # expected non-fix
        ttff = f"{r['ttff']:.1f}s" if r.get("ttff") else "-"
        flag = " <== REGRESSION" if bad else ""
        print(f"{name:14} {status:9} {ttff:>6}  {detail}{flag}")
        regressions += bad

    print("-" * 72)
    print(f"{'* expected non-fix (decode/short)' :40} regressions: {regressions}")
    return 1 if regressions else 0


if __name__ == "__main__":
    sys.exit(main())
