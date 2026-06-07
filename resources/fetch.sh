#!/usr/bin/env bash
#
# Download the IQ sample recordings used by gnss-rcv into this directory.
#
#   ./resources/fetch.sh                # list what's available / already present
#   ./resources/fetch.sh jks-1bit       # fetch one (or several) by name
#   ./resources/fetch.sh all            # fetch everything (incl. the 12.7 GiB one)
#
# Re-running skips files that are already the expected size; interrupted
# downloads resume. Needs `curl` (and `unzip` for the gypsum recording).

set -uo pipefail
cd "$(dirname "$0")" || exit 1

# name | url | dest filename | expected size (bytes) | zipped? (0/1) | run hint
RESOURCES=(
  "jks-1bit|http://www.jks.com/gps/gps.samples.1bit.I.fs5456.if4092.bin|gps.samples.1bit.I.fs5456.if4092.bin|55791616|0|-t 1bit --fs 5456000 --fi 1364000"
  "zenodo-sigmf|https://zenodo.org/records/6394603/files/GPS-L1-2022-03-27.sigmf-data?download=1|GPS-L1-2022-03-27.sigmf-data|240000000|0|-t 2xi16 --fs 4000000  (~15s, too short for a fix)"
  "nov3|https://github.com/codyd51/gypsum/releases/download/1.0/nov_3_time_18_48_st_ives.zip|nov_3_time_18_48_st_ives|12699331696|1|-t 2xf32  (main dev recording)"
)

filesize() { # portable stat: macOS uses -f%z, GNU uses -c%s
  stat -f%z "$1" 2>/dev/null || stat -c%s "$1" 2>/dev/null
}

human() { # bytes -> human-readable
  awk -v b="$1" 'BEGIN{u[0]="B";u[1]="KiB";u[2]="MiB";u[3]="GiB";
    while(b>=1024&&i<3){b/=1024;i++} printf "%.0f %s",b,u[i]}'
}

list() {
  echo "Downloadable IQ recordings (./resources/fetch.sh <name>... | all):"
  echo
  for entry in "${RESOURCES[@]}"; do
    IFS='|' read -r name url dest size zip run <<<"$entry"
    if [ -f "$dest" ] && [ "$(filesize "$dest")" = "$size" ]; then mark="have"; else mark=" -- "; fi
    printf "  [%s] %-12s %9s  %s\n" "$mark" "$name" "$(human "$size")" "$dest"
    printf "            run: %s\n" "$run"
  done
  echo
  echo "  note: gioveAandB_short.bin (gfix.dk) blocks scripted downloads -- fetch it by hand."
}

fetch_one() {
  local want="$1" name url dest size zip run
  for entry in "${RESOURCES[@]}"; do
    IFS='|' read -r name url dest size zip run <<<"$entry"
    [ "$name" = "$want" ] || continue

    if [ -f "$dest" ] && [ "$(filesize "$dest")" = "$size" ]; then
      echo "$name: already present ($(human "$size")), skipping"
      return 0
    fi
    echo "$name: downloading $(human "$size") -> resources/$dest"

    if [ "$zip" = "1" ]; then
      curl -L --fail --progress-bar -C - -o "$dest.zip" "$url" \
        || { echo "$name: download failed" >&2; return 1; }
      unzip -o "$dest.zip" || { echo "$name: unzip failed" >&2; return 1; }
      rm -f "$dest.zip"
    else
      curl -L --fail --progress-bar -C - -o "$dest.part" "$url" \
        || { echo "$name: download failed" >&2; return 1; }
      mv "$dest.part" "$dest"
    fi
    echo "$name: done"
    return 0
  done
  echo "unknown resource: $want" >&2
  return 1
}

if [ $# -eq 0 ] || [ "$1" = "list" ] || [ "$1" = "help" ] || [ "$1" = "-h" ]; then
  list
  exit 0
fi

names=("$@")
if [ "$1" = "all" ]; then
  names=()
  for entry in "${RESOURCES[@]}"; do
    IFS='|' read -r name _ <<<"$entry"
    names+=("$name")
  done
fi

rc=0
for name in "${names[@]}"; do
  fetch_one "$name" || rc=1
done
exit $rc
