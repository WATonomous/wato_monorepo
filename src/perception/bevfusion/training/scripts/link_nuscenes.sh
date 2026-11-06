#!/usr/bin/env bash
# Usage: scripts/link_nuscenes.sh mini|full DEST
# Builds a writable nuScenes root at DEST from the read-only copy on wato-drive2: raw data is
# symlinked, the map expansion is unpacked, and prepare_data.sh then writes its outputs into DEST.
# Run on the host (login node), not in the container. job.sbatch must mount /mnt/wato-drive2 at
# the same path, or the links are broken inside the container.
#   NUSC_SRC, MAP_ZIP: override the source copy and the map expansion zip
set -euo pipefail
SPLIT="${1:?mini or full}"; DEST="${2:?destination directory}"
case "$SPLIT" in
  # Mini's metadata folder is named v1.0-trainval on the drive but holds the 10 mini scenes.
  mini) SRC="${NUSC_SRC:-/mnt/wato-drive2/nuscenes-1.0-mini}"
        META=("v1.0-mini:v1.0-trainval") ;;
  # create_data.py --version v1.0 processes trainval and then test, so both are linked.
  full) SRC="${NUSC_SRC:-/mnt/wato-drive2/wato-datasets/nuscenes/full-dataset}"
        META=("v1.0-trainval:v1.0-trainval" "v1.0-test:v1.0-test") ;;
  *) echo "unknown split: $SPLIT" >&2; exit 1 ;;
esac
MAP_ZIP="${MAP_ZIP:-$(ls /mnt/wato-drive2/wato-datasets/nuscenes/map-expansion/nuScenes-map-expansion-v1.3.zip* | head -1)}"

mkdir -p "$DEST/maps"
for d in samples sweeps; do ln -sfn "$SRC/$d" "$DEST/$d"; done
for m in "${META[@]}"; do ln -sfn "$SRC/${m#*:}" "$DEST/${m%%:*}"; done
for png in "$SRC"/maps/*.png; do ln -sfn "$png" "$DEST/maps/"; done
# LoadBEVSegmentation in the base nuScenes pipeline opens a NuScenesMap per city, even for detection.
if [[ ! -d "$DEST/maps/expansion" ]]; then
  python3 -m zipfile -e "$MAP_ZIP" "$DEST/maps/"
fi
ls -l "$DEST" "$DEST/maps/expansion"
