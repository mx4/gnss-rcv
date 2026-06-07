#!/usr/bin/env bash
#
# Generate a gps-sdr-sim IQ recording end-to-end for the gnss-rcv fix test:
# pick a date+time and a location, download the matching broadcast ephemeris
# (brdc) for that day, run gps-sdr-sim, and write the recording plus a .meta
# file (truth location + visible PRNs) that tests/gpssim.rs consumes.
#
#   ./resources/gen_gpssim.sh [date,time] [lat,lon,alt] [duration_sec]
#
# Defaults to a known-good scenario (Geneva, Jet d'Eau, 2026/04/28) that is
# verified to reach a position fix. Override via the args.
#
# Needs curl, gunzip and a built gps-sdr-sim binary (set $GPS_SDR_SIM, else it
# looks in ~/git/gps-sdr-sim/gps-sdr-sim and on PATH). Exits non-zero -- so the
# test skips cleanly -- when a tool or the network is unavailable.
#
# Ephemeris source: ESA GSSC, which serves brdc over FTP without authentication
# (unlike CDDIS, which requires an Earthdata login).

set -uo pipefail

DATETIME="${1:-2026/04/28,17:00:00}"            # Geneva Jet d'Eau, a verified
LOCATION="${2:-46.2075,6.1557,375}"             # known-good scenario (see README)
DURATION="${3:-45}"
FS=2046000

OUT="gpssim_gen_2xi16"
META="gpssim_gen.meta"
LOG_FILE="gpssim_gen.txt"
SCENARIO="$DATETIME|$LOCATION|$DURATION|$FS"

# Resolve the gps-sdr-sim binary to an absolute path before we cd elsewhere.
SIM="${GPS_SDR_SIM:-}"
if [ -z "$SIM" ]; then
  if [ -x "$HOME/git/gps-sdr-sim/gps-sdr-sim" ]; then
    SIM="$HOME/git/gps-sdr-sim/gps-sdr-sim"
  else
    SIM="$(command -v gps-sdr-sim || true)"
  fi
fi
if [ -z "$SIM" ] || [ ! -x "$SIM" ]; then
  echo "gen_gpssim: gps-sdr-sim not found (set \$GPS_SDR_SIM or build it in ~/git/gps-sdr-sim)" >&2
  exit 2
fi
command -v curl   >/dev/null || { echo "gen_gpssim: curl not found" >&2;   exit 2; }
command -v gunzip >/dev/null || { echo "gen_gpssim: gunzip not found" >&2; exit 2; }

cd "$(dirname "$0")" || exit 1

# Reuse an existing recording if it already matches this exact scenario.
if [ -f "$OUT" ] && [ -f "$META" ] && grep -qxF "scenario=$SCENARIO" "$META"; then
  echo "gen_gpssim: $OUT already matches scenario, skipping generation"
  exit 0
fi

# date -> day-of-year + 2-digit year (portable: try BSD/macOS, then GNU).
DATEPART="${DATETIME%%,*}" # e.g. 2022/01/01
if DOY=$(date -j -f "%Y/%m/%d" "$DATEPART" "+%j" 2>/dev/null); then
  YEAR=$(date -j -f "%Y/%m/%d" "$DATEPART" "+%Y")
elif DOY=$(date -d "$DATEPART" "+%j" 2>/dev/null); then
  YEAR=$(date -d "$DATEPART" "+%Y")
else
  echo "gen_gpssim: cannot parse date '$DATEPART' (want YYYY/MM/DD)" >&2
  exit 2
fi
YY="${YEAR:2:2}"
NAV="brdc${DOY}0.${YY}n"

# Download the broadcast ephemeris for that day from ESA (auth-free FTP).
if [ ! -f "$NAV" ]; then
  URL="ftp://gssc.esa.int/gnss/data/daily/${YEAR}/${DOY}/${NAV}.gz"
  echo "gen_gpssim: downloading $URL"
  if ! curl -sS --fail -m 60 -o "$NAV.gz.part" "$URL"; then
    echo "gen_gpssim: ephemeris download failed (offline, or no brdc for that day yet)" >&2
    rm -f "$NAV.gz.part"
    exit 3
  fi
  mv "$NAV.gz.part" "$NAV.gz"
  gunzip -f "$NAV.gz" || { echo "gen_gpssim: gunzip failed" >&2; exit 3; }
fi

# Run gps-sdr-sim. Capture stdout both to a log file and for PRN parsing.
echo "gen_gpssim: gps-sdr-sim $DURATION s @ $FS Hz, loc $LOCATION, t $DATETIME -> resources/$OUT"
LOG=$("$SIM" -e "$NAV" -l "$LOCATION" -t "$DATETIME" -d "$DURATION" \
             -b 16 -s "$FS" -o "$OUT" 2>&1)
rc=$?
# gps-sdr-sim's stdout includes a noisy per-0.1s progress counter; keep the full
# capture in the log file but don't echo it.
printf '%s\n' "$LOG" > "$LOG_FILE"
if [ $rc -ne 0 ] || [ ! -s "$OUT" ]; then
  echo "gen_gpssim: gps-sdr-sim failed (rc=$rc)" >&2
  exit 4
fi

# Write the meta file the test reads: truth location and the visible PRNs.
# gps-sdr-sim prints one "PRN az el range iono" row per simulated channel.
LAT="${LOCATION%%,*}"
LON=$(printf '%s\n' "$LOCATION" | cut -d, -f2)
SATS=$(printf '%s\n' "$LOG" | awk 'NF==5 && $1 ~ /^[0-9]+$/ {printf "%s%s", sep, $1; sep=","}')
{
  echo "scenario=$SCENARIO"
  echo "lat=$LAT"
  echo "lon=$LON"
  echo "sats=$SATS"
} > "$META"

echo "gen_gpssim: done -> resources/$OUT (truth $LAT,$LON; sats $SATS)"
