#!/usr/bin/env bash
#
# Download the IQ sample recordings used by gnss-rcv into this directory.
#
#   ./resources/fetch.sh                # list what's available / already present
#   ./resources/fetch.sh jks-1bit       # fetch one (or several) by name
#   ./resources/fetch.sh all            # fetch everything (incl. the 12.7 GiB one)
#
# Re-running skips files that are already present; interrupted downloads resume.
# Needs `curl` (plus `unzip`/`tar` for the archived recordings).

set -uo pipefail
cd "$(dirname "$0")" || exit 1

# name | url | dest filename | expected size in bytes (0 = don't size-check, just
#   check the file exists) | archive: raw|zip|tgz | run hint
RESOURCES=(
  "jks-1bit|http://www.jks.com/gps/gps.samples.1bit.I.fs5456.if4092.bin|gps.samples.1bit.I.fs5456.if4092.bin|55791616|raw|-t 1bit --fs 5456000 --fi 1364000"
  "zenodo-sigmf|https://zenodo.org/records/6394603/files/GPS-L1-2022-03-27.sigmf-data?download=1|GPS-L1-2022-03-27.sigmf-data|240000000|raw|-t 2xi16 --fs 4000000  (~15s, too short for a fix)"
  "cttc|https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz|2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat|0|tgz|-t 2xi16 --fs 4000000  (CTTC Spain 2013; ~1.1 GiB download)"
  "ion-rtlsdr|https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8|ION_RTLSDR_Bands-L1.rtlsdr|245760000|raw|-t rtlsdr-file --fs 2048000  (ION rooftop RTL-SDR L1, 60s; fixes)"
  "ion-bladerf|https://sdr.ion.org/BladeRF/BladeRF_Bands-L1.int16|ION_BladeRF_Bands-L1.2xi16|527335424|raw|-t 2xi16 --fs 10000000  (ION BladeRF L1, 10 MHz, ~13s; tracks 13 SVs, too short for a fix)"
  "ion-lime|https://sdr.ion.org/LimeSDR/LimeSDR_Bands-L1.int16|ION_LimeSDR_Bands-L1.2xi16|2459077760|raw|-t 2xi16 --fs 10000000 --fi 420000  (ION LimeSDR L1, 10 MHz IF 420 kHz, 60s; fixes)"
  "ion-hackrf|https://sdr.ion.org/HackRF/HackRF_Bands-L1.int8|ION_HackRF_Bands-L1.2xi8|1200000000|raw|-t 2xi8 --fs 10000000 --fi 420000  (ION HackRF L1; acquires 11 SVs but doesn't nav-decode -- open issue)"
  "nov3|https://github.com/codyd51/gypsum/releases/download/1.0/nov_3_time_18_48_st_ives.zip|nov_3_time_18_48_st_ives|12699331696|zip|-t 2xf32  (main dev recording)"
)

filesize() { # portable stat: macOS uses -f%z, GNU uses -c%s
  stat -f%z "$1" 2>/dev/null || stat -c%s "$1" 2>/dev/null
}

human() { # bytes -> human-readable ("?" when size is unknown/0)
  [ "$1" = "0" ] && { printf "?"; return; }
  awk -v b="$1" 'BEGIN{u[0]="B";u[1]="KiB";u[2]="MiB";u[3]="GiB";
    while(b>=1024&&i<3){b/=1024;i++} printf "%.0f %s",b,u[i]}'
}

is_present() { # dest size -> 0 if present (size 0 means existence-only)
  [ -f "$1" ] || return 1
  [ "$2" = "0" ] || [ "$(filesize "$1")" = "$2" ]
}

# dest, run-hint -> print the ready-to-run command (the run-hint is "<flags>
# (note)"; drop the parenthetical note and prepend the actual file path).
runcmd() {
  local flags
  flags="$(printf '%s' "$2" | sed -E 's/\(.*//; s/[[:space:]]+$//')"
  printf "  cargo run --release -- -f resources/%s %s\n" "$1" "$flags"
}

list() {
  echo "Downloadable IQ recordings (./resources/fetch.sh <name>... | all):"
  echo
  for entry in "${RESOURCES[@]}"; do
    IFS='|' read -r name url dest size zip run <<<"$entry"
    if is_present "$dest" "$size"; then mark="have"; else mark=" -- "; fi
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

    if is_present "$dest" "$size"; then
      echo "$name: already present, run with:"
      runcmd "$dest" "$run"
      return 0
    fi
    echo "$name: downloading -> resources/$dest"

    case "$zip" in
      zip)
        curl -L --fail --progress-bar -C - -o "$name.zip" "$url" \
          || { echo "$name: download failed" >&2; return 1; }
        unzip -o "$name.zip" || { echo "$name: unzip failed" >&2; return 1; }
        rm -f "$name.zip"
        ;;
      tgz)
        # tar.gz may expand into a subdirectory, so download to a flat temp name
        # ($dest can contain a slash) and let tar create the directory tree.
        curl -L --fail --progress-bar -C - -o "$name.tgz" "$url" \
          || { echo "$name: download failed" >&2; return 1; }
        tar xzf "$name.tgz" || { echo "$name: untar failed" >&2; return 1; }
        rm -f "$name.tgz"
        ;;
      *)
        curl -L --fail --progress-bar -C - -o "$dest.part" "$url" \
          || { echo "$name: download failed" >&2; return 1; }
        mv "$dest.part" "$dest"
        ;;
    esac
    [ -f "$dest" ] || { echo "$name: '$dest' missing after extract" >&2; return 1; }
    echo "$name: done, run with:"
    runcmd "$dest" "$run"
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
