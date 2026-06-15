#!/usr/bin/env python3
"""Download the IQ sample recordings used by gnss-rcv into this directory.

    ./resources/fetch.py                 # list what's available / already present
    ./resources/fetch.py jks-1bit        # fetch one (or several) by name
    ./resources/fetch.py qzss            # fetch every recording carrying QZSS
    ./resources/fetch.py all             # fetch everything (incl. the 12.7 GiB one)
    ./resources/fetch.py --tag galileo   # just *list* the recordings carrying Galileo

A positional argument is a recording name, a constellation tag (gps/qzss/galileo),
or "all". Re-running skips files already present; interrupted downloads resume
(curl -C -). Needs `curl` (plus `unzip`/`tar` for the archived recordings).
"""

import json
import os
import re
import sys
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

HERE = Path(__file__).resolve().parent

# Constellation tags. Every recording is at L1 (1575.42 MHz) so all carry GPS.
#   gps     - GPS L1 C/A (decoded today).
#   qzss    - QZSS L1 C/A present and verified to acquire (J-prefixed PRNs).
#   galileo - wide enough (>= ~8 MHz) to contain Galileo E1's BOC(1,1) sidelobes,
#             so E1 is in the samples (decoding it is still on the roadmap).
TAGS = ("gps", "qzss", "galileo")
TAG_COLOR = {"gps": "green", "qzss": "magenta", "galileo": "cyan"}


@dataclass
class Rec:
    name: str
    url: str
    dest: str  # final filename under resources/ (may contain a sub-path)
    size: int  # expected bytes; 0 = just check existence
    archive: str  # raw | zip | tgz
    flags: str  # gnss-rcv flags for the run command
    note: str  # short human note (location / status)
    tags: Tuple[str, ...] = field(default_factory=lambda: ("gps",))


# fmt: off
# Ordered by interest, newest-marquee first: this list order is the order the UI
# file-picker shows (write_manifest preserves it). The three leads are the local
# dev recording and the two Galileo/OSNMA showcases; the rest descend roughly by
# capability — fixes (richest first), then rich tracking, then short/non-decoding.
RECORDINGS: List[Rec] = [
    Rec("nov3",
        "https://github.com/codyd51/gypsum/releases/download/1.0/nov_3_time_18_48_st_ives.zip",
        "nov_3_time_18_48_st_ives", 12699331696, "zip",
        "-t 2xf32 --fs 2046000",
        "main dev recording, 12.7 GiB; fixes (St Ives, Cambs UK)", ("gps",)),
    Rec("tuni2025",
        "https://zenodo.org/records/15470143/files/clearsky_signal_C-1.bin?download=1",
        "clearsky_signal_C-1.bin", 0, "raw",
        "-t 2xi16-be --fs 50000000 --sig E1B",
        "Tampere Univ. clear-sky 2025, 50 MHz zero-IF, ~150s. The dataset labels it 32-bit float but it is int16 big-endian (2xi16-be). Carries BOTH constellations: 8 Galileo E1B SVs fix at 61.4500, 23.8563, and 16 GPS L1CA SVs (--sig L1CA) fix at 61.4501, 23.8566 (both Tampere, ~few m) -- prime candidate for a combined GPS+Galileo fix.",
        ("gps", "galileo")),
    Rec("fgi-osnma",
        "https://etsin.fairdata.fi/dataset/09dc5c1b-933d-4efd-aa66-be2c07fab3b3",
        "OSNMAspoofingdatasets/Scenario1:Clean opensky/OSNMA_cleandata_opensky_460s.dat", 0, "manual",
        "-t i8 --fs 26000000 --fi 6390000 --sig E1B",
        "FGI OSNMA (Finland 2023), L1/E1 i8 26 MHz fi 6.39 MHz; Galileo fixes (Otaniemi); clean 460s + jammertest + OSNMA bits; manual, see docs/datasets.md",
        ("gps", "galileo")),
    Rec("ion-lime",
        "https://sdr.ion.org/LimeSDR/LimeSDR_Bands-L1.int16",
        "ION_LimeSDR_Bands-L1.2xi16", 2459077760, "raw",
        "-t 2xi16 --fs 10000000 --fi 420000",
        "ION LimeSDR L1, 10 MHz IF 420 kHz, 60s; fixes (Netherlands)", ("gps", "galileo")),
    Rec("cttc",
        "https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz",
        "2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat", 0, "tgz",
        "-t 2xi16 --fs 4000000",
        "gnss-sdr CTTC Spain 2013, ~1.1 GiB; GPS fix + EGNOS; also 3 Galileo IOV E1B ephemerides (E11/E12/E20)", ("gps", "galileo")),
    Rec("texbat-clean",
        "https://rnl-data.ae.utexas.edu/datastore/texbat/cleanStatic80.bin",
        "TEXBAT_cleanStatic80.2xi16", 8016392192, "raw",
        "-t 2xi16 --fs 25000000",
        "TEXBAT clean-static (UT Austin), GPS L1 complex baseband int16 25 MHz, ~80s (7.5 GiB), no header (scott.m reads from byte 0); spoof-free reference -> fixes (Austin TX). The spoofed/evil-waveform sets ds1-ds8 live in the same datastore (each ~44 GiB, first 100s spoof-free) for robustness tests; see docs/datasets.md", ("gps",)),
    Rec("ion-rtlsdr",
        "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8",
        "ION_RTLSDR_Bands-L1.rtlsdr", 245760000, "raw",
        "-t rtlsdr-file --fs 2048000",
        "ION rooftop RTL-SDR L1, 60s; fixes (Netherlands)", ("gps",)),
    Rec("ion-gn3s",
        "https://sdr.ion.org/SiGe/SiGe_Bands-L1.dat",
        "ION_GN3S_Bands-L1.i8", 1963982848, "raw",
        "-t i8 --fs 16368000 --fi 4092000 --invert-spectrum",
        "ION GN3S (SiGe) L1, real 8-bit IF 4.092 MHz, ~120s (North America 2013, CU GNSS Lab -- not Munich); GPS L1 C/A, inverted IF spectrum (--invert-spectrum). Weak/high-GDOP fix lands in eastern N. America (~37-39N, 82W)", ("gps",)),
    Rec("sjtu",
        "https://sdr.ion.org/SJTU/SJTU_Bands-L1E1.dat",
        "SJTU_Bands-L1E1.dat", 749899776, "raw",
        "-t 4bit --fs 25000000 --fi 6250000",
        "ION SJTU Shanghai L1+E1, SX3 4-bit, 60s; tracks 15-29 SVs; GPS fix at 31.025,121.439 (TTFF ~26s)", ("gps", "galileo")),
    Rec("ion-ifen",
        "https://sdr.ion.org/IFEN/IFEN_Bands-L1.stream",
        "ION_IFEN_Bands-L1.2bit", 5725209600, "raw",
        "-t 2bit --fs 20480000 --fi 5500000 --invert-spectrum",
        "ION IFEN SX3 dual-RF L1, real 2-bit IF 5.5 MHz, 10 MHz BW, ~18 min (Munich 2016, 5.7 GiB); GPS L1 + 4 Galileo E1 (E11/E12/E19/E20). 2-bit reader is LSB-first + inverted IF spectrum (--invert-spectrum); GPS fix at Munich 48.078,11.638 (TTFF ~25s)", ("gps", "galileo")),
    Rec("pocketsdr",
        "http://gpspp.sakura.ne.jp/pocketsdr/L1L6_20211226_082212_12MHz.zip",
        "L1_20211226_082212_12MHz_I.bin", 364904448, "zip",
        "-t i8 --fs 12000000 --fi 3000000 --qzss",
        "PocketSDR Tokyo L1, ~30s; tracks GPS + QZSS J194/195/199", ("gps", "qzss", "galileo")),
    Rec("zenodo-sigmf",
        "https://zenodo.org/records/6394603/files/GPS-L1-2022-03-27.sigmf-data?download=1",
        "GPS-L1-2022-03-27.sigmf-data", 240000000, "raw",
        "-t 2xi16 --fs 4000000",
        "SigMF ci16, 4 MHz, ~15s; GPS + E1B both track (5 SVs each), too short for a fix", ("gps", "galileo")),
    Rec("ion-bladerf",
        "https://sdr.ion.org/BladeRF/BladeRF_Bands-L1.int16",
        "ION_BladeRF_Bands-L1.2xi16", 527335424, "raw",
        "-t 2xi16 --fs 10000000",
        "ION BladeRF L1, 10 MHz, ~13s; tracks 13 SVs, too short for a fix", ("gps", "galileo")),
    Rec("jks-1bit",
        "http://www.jks.com/gps/gps.samples.1bit.I.fs5456.if4092.bin",
        "gps.samples.1bit.I.fs5456.if4092.bin", 55791616, "raw",
        "-t 1bit --fs 5456000 --fi 1364000",
        "jks.com 1-bit real, ~82s; tracks ~7 SVs at a marginal ~30 dB-Hz", ("gps",)),
    Rec("ion-hackrf",
        "https://sdr.ion.org/HackRF/HackRF_Bands-L1.int8",
        "ION_HackRF_Bands-L1.2xi8", 1200000000, "raw",
        "-t 2xi8 --fs 10000000 --fi 420000",
        "ION HackRF L1, 60s; acquires 11 SVs; GPS fix at 52.150,4.497 (Netherlands, TTFF ~34s)", ("gps", "galileo")),
    Rec("pocketsdr-eindhoven",
        "https://rnav.info.hiroshima-cu.ac.jp/gnss/pocketsdr/20250605eindhoven/20250605-073318fe4.bin",
        "pocketsdr-eindhoven.bin", 0, "raw",
        "-t pocketsdr-raw16 --fs 4000000 --fi 0",
        "PocketSDR FE4CH Eindhoven NL Jun 2025, GPS L1CA + Galileo E1B + SBAS, ~300s; "
        "RAW16 4-ch 2-bit IQ at 16 MHz, CH0 F_LO=1568 MHz (L1/E1 at +7.42 MHz). The "
        "reader image-rejects (the wide IF channel aliases a self-image across the 8 MHz "
        "Nyquist), downconverts and decimates to 4 MHz baseband; pass --fs 4000000 --fi 0. "
        "Fixes in Eindhoven (cn0 40-48, TTFF ~33s)",
        ("gps", "galileo")),
    Rec("pocketsdr-bonn",
        "https://rnav.info.hiroshima-cu.ac.jp/gnss/pocketsdr/20241107bonn/L1al.bin.gz",
        "pocketsdr-bonn.bin", 0, "gz",
        "-t 2xi8 --fs 24000000",
        "PocketSDR Bonn DE Nov 2024, GPS L1CA, INT8X2 24 MHz zero-IF, ~130s; "
        "F_LO=1575.42 MHz. Galileo E1B on separate E1.bin.gz from same session",
        ("gps",)),
    Rec("pocketsdr-hiroshima",
        "https://rnav.info.hiroshima-cu.ac.jp/gnss/pocketsdr/20241113hiroshima-cu/L1a.bin.gz",
        "pocketsdr-hiroshima.bin", 0, "gz",
        "-t 2xi8 --fs 24000000 --qzss",
        "PocketSDR Hiroshima JP Nov 2024, GPS L1CA + Michibiki QZSS, INT8X2 24 MHz zero-IF, ~30s; "
        "F_LO=1575.42 MHz",
        ("gps", "qzss")),
]
# fmt: on

BY_NAME = {r.name: r for r in RECORDINGS}

# ----------------------------------------------------------------------------- color

USE_COLOR = (
    sys.stdout.isatty()
    and os.environ.get("NO_COLOR") is None
    and os.environ.get("TERM") != "dumb"
)
_CODES = {
    "reset": "\033[0m", "bold": "\033[1m", "dim": "\033[2m",
    "red": "\033[31m", "green": "\033[32m", "yellow": "\033[33m",
    "blue": "\033[34m", "magenta": "\033[35m", "cyan": "\033[36m",
    # deliberately no bright-black (90): Solarized Dark renders it == background.
}
_ANSI_RE = re.compile(r"\033\[[0-9;]*m")


def paint(text: str, *styles: str) -> str:
    if not USE_COLOR or not styles:
        return text
    return "".join(_CODES[s] for s in styles) + text + _CODES["reset"]


def vlen(s: str) -> int:
    """Visible length (ANSI escapes don't take screen columns)."""
    return len(_ANSI_RE.sub("", s))


def pad(s: str, width: int) -> str:
    return s + " " * max(0, width - vlen(s))


def rpad(s: str, width: int) -> str:  # right-align
    return " " * max(0, width - vlen(s)) + s


# ----------------------------------------------------------------------------- helpers


def human(n: int) -> str:
    if n <= 0:
        return "?"
    units = ["B", "KiB", "MiB", "GiB", "TiB"]
    f = float(n)
    i = 0
    while f >= 1024 and i < len(units) - 1:
        f /= 1024
        i += 1
    return "%.0f %s" % (f, units[i])


def dest_path(rec: Rec) -> Path:
    return HERE / rec.dest


def filesize(p: Path) -> int:
    try:
        return p.stat().st_size
    except OSError:
        return 0


def is_present(rec: Rec) -> bool:
    p = dest_path(rec)
    if not p.is_file():
        return False
    return rec.size == 0 or filesize(p) == rec.size


def runcmd(rec: Rec) -> str:
    # Just the arguments (prefix them with `cargo run --release --`).
    return "-f resources/%s %s" % (rec.dest, rec.flags)


def tag_chips(rec: Rec) -> str:
    return " ".join(paint(t, TAG_COLOR[t]) for t in rec.tags)


def tags_plain(rec: Rec) -> str:
    return " ".join(rec.tags)


# ----------------------------------------------------------------------------- listing


def hr(width: int = 78, ch: str = "─") -> str:
    # NB: never bright-black (\033[90m) -- Solarized maps it to base03, which IS
    # the background on the dark theme, so such text renders invisible. Use the
    # `dim` attribute (faint, but never background-equal) for de-emphasis.
    return paint(ch * width, "dim")


def print_list(recs: List[Rec], tag_filter=None) -> None:
    title = "gnss-rcv · IQ sample recordings"
    sub = "./resources/fetch.py [name | tag | all]"
    legend = "tags: " + " ".join(paint(t, TAG_COLOR[t]) for t in TAGS)
    print()
    print("  " + paint(title, "bold"))
    print("  " + paint(sub, "dim") + "    " + legend)
    if tag_filter:
        print("  " + paint("filtered to tag: ", "dim") + paint(tag_filter, TAG_COLOR[tag_filter]))
    print(hr())

    name_w = max((len(r.name) for r in recs), default=4)
    tags_w = max((vlen(tags_plain(r)) for r in recs), default=4)
    size_w = max((len(human(r.size)) for r in recs), default=5)

    present = 0
    disk = 0
    for r in recs:
        here = is_present(r)
        if here:
            present += 1
            disk += filesize(dest_path(r))
            icon = paint("✓", "green", "bold")
        else:
            icon = paint("·", "dim")
        line = "  %s  %s  %s  %s  %s" % (
            icon,
            pad(paint(r.name, "bold") if here else r.name, name_w),
            pad(tag_chips(r), tags_w),
            rpad(human(r.size), size_w),
            r.dest,  # default fg: a filename must stay readable on any theme
        )
        print(line)
        print("       " + paint("↳ " + r.note, "dim"))
        print("       " + paint(runcmd(r), "cyan"))

    print(hr())
    summary = "%d recordings · %s present (%s on disk) · %d missing" % (
        len(recs), paint(str(present), "green"), human(disk), len(recs) - present,
    )
    print("  " + summary)
    print("  " + paint("note: gioveAandB_short.bin (gfix.dk) blocks scripted downloads — fetch by hand", "dim"))
    print()


# ----------------------------------------------------------------------------- fetching


def _run(cmd: List[str]) -> bool:
    try:
        subprocess.run(cmd, cwd=str(HERE), check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(paint("    command failed: %s (%s)" % (" ".join(cmd), e), "red"), file=sys.stderr)
        return False


def fetch_one(rec: Rec) -> bool:
    if is_present(rec):
        print("%s %s" % (paint("✓", "green"), paint(rec.name + ": already present, run with:", "bold")))
        print("  " + paint(runcmd(rec), "cyan"))
        return True

    if rec.archive == "manual":
        print("%s %s" % (paint("✋", "yellow"), paint(rec.name + ": not scriptable — download by hand", "bold")))
        print("  from: " + paint(rec.url, "cyan"))
        print("  save under resources/ (see the dataset README for the exact file names),")
        print("  then run with: " + paint(runcmd(rec), "cyan"))
        return True

    print("%s %s -> resources/%s" % (paint("↓", "yellow"), paint(rec.name + ": downloading", "bold"), rec.dest))
    curl_base = ["curl", "-L", "--fail", "--progress-bar", "-C", "-", "-o"]

    if rec.archive == "zip":
        tmp = rec.name + ".zip"
        ok = _run(curl_base + [tmp, rec.url]) and _run(["unzip", "-o", tmp])
        if (HERE / tmp).exists():
            (HERE / tmp).unlink()
    elif rec.archive == "tgz":
        # tar.gz may expand into a sub-directory; download flat, let tar lay it out.
        tmp = rec.name + ".tgz"
        ok = _run(curl_base + [tmp, rec.url]) and _run(["tar", "xzf", tmp])
        if (HERE / tmp).exists():
            (HERE / tmp).unlink()
    elif rec.archive == "gz":
        # Plain gzip (not tar): download as <dest>.gz, gunzip in-place → <dest>.
        dst_name = Path(rec.dest).name          # e.g. "pocketsdr-bonn.bin"
        gz_tmp   = dst_name + ".gz"
        ok = _run(curl_base + [gz_tmp, rec.url])
        if ok:
            ok = _run(["gunzip", "-f", gz_tmp]) # removes gz_tmp, creates dst_name
        gz_path = HERE / gz_tmp
        if gz_path.exists():
            gz_path.unlink()
    else:  # raw
        part = rec.dest + ".part"
        ok = _run(curl_base + [part, rec.url])
        if ok:
            shutil.move(str(HERE / part), str(dest_path(rec)))

    if not ok or not dest_path(rec).is_file():
        print(paint("%s: '%s' missing after fetch" % (rec.name, rec.dest), "red"), file=sys.stderr)
        return False
    print("%s %s" % (paint("✓", "green"), paint(rec.name + ": done, run with:", "bold")))
    print("  " + paint(runcmd(rec), "cyan"))
    return True


# ----------------------------------------------------------------------------- cli


def resolve(target: str) -> List[Rec]:
    """A target is a recording name, a constellation tag, or 'all'."""
    if target == "all":
        return list(RECORDINGS)
    if target in TAGS:
        return [r for r in RECORDINGS if target in r.tags]
    if target in BY_NAME:
        return [BY_NAME[target]]
    print(paint("unknown target: %s" % target, "red"), file=sys.stderr)
    print("  names: " + ", ".join(BY_NAME), file=sys.stderr)
    print("  tags:  " + ", ".join(TAGS) + ", all", file=sys.stderr)
    return []


def write_manifest() -> None:
    """Emit resources/manifest.json — the recordings list the desktop UI reads to
    populate its file picker and auto-fill -t/--fs/--fi/--sig. Regenerated on
    every run so it stays in sync with RECORDINGS (the single source of truth)."""
    manifest = [
        {"name": r.name, "url": r.url, "dest": r.dest, "size": r.size, "archive": r.archive,
         "flags": r.flags, "note": r.note, "tags": list(r.tags)}
        for r in RECORDINGS
    ]
    (HERE / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main(argv: List[str]) -> int:
    write_manifest()  # keep the UI's recording list in sync with RECORDINGS
    args = list(argv)
    tag_filter = None
    if "--tag" in args:
        i = args.index("--tag")
        if i + 1 < len(args):
            tag_filter = args[i + 1]
            del args[i : i + 2]
            if tag_filter not in TAGS:
                print(paint("unknown tag: %s (have %s)" % (tag_filter, ", ".join(TAGS)), "red"), file=sys.stderr)
                return 1

    if not args or args[0] in ("list", "help", "-h", "--help"):
        recs = [r for r in RECORDINGS if tag_filter in r.tags] if tag_filter else RECORDINGS
        print_list(recs, tag_filter)
        return 0

    # Fetch: expand each target, preserving order, de-duplicated.
    seen = set()
    queue: List[Rec] = []
    rc = 0
    for target in args:
        recs = resolve(target)
        if not recs:
            rc = 1
            continue
        for r in recs:
            if r.name not in seen:
                seen.add(r.name)
                queue.append(r)
    for r in queue:
        if not fetch_one(r):
            rc = 1
    return rc


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)
