#!/usr/bin/env bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Usage: scripts/link_nuscenes.sh mini|full DEST
# Builds a writable nuScenes root at DEST by linking raw data and unpacking maps.
# Run on the host, not in the container; prepare_data.sh later writes outputs into DEST.
# Paths come from params.env. The container must mount $STORAGE at the same path.
# NUSC_SRC and MAP_ZIP override the dataset source and map expansion zip.
set -euo pipefail
# shellcheck source=../params.env
source "$(dirname "$0")/../params.env"
SPLIT="${1:?mini or full}"; DEST="${2:?destination directory}"
case "$SPLIT" in
  # Mini's metadata folder is named v1.0-trainval in the source copy but holds the 10 mini scenes.
  mini) SRC="${NUSC_SRC:-$NUSC_MINI_SRC}"
        META=("v1.0-mini:v1.0-trainval") ;;
  # create_data.py --version v1.0 processes trainval and then test, so both are linked.
  full) SRC="${NUSC_SRC:-$NUSC_FULL_SRC}"
        META=("v1.0-trainval:v1.0-trainval" "v1.0-test:v1.0-test") ;;
  *) echo "unknown split: $SPLIT" >&2; exit 1 ;;
esac
zips=("$NUSC_MAP_DIR"/nuScenes-map-expansion-v1.3.zip*)
MAP_ZIP="${MAP_ZIP:-${zips[0]}}"
[[ -f "$MAP_ZIP" ]] || { echo "map expansion zip not found: $MAP_ZIP (set NUSC_MAP_DIR or MAP_ZIP)" >&2; exit 1; }

mkdir -p "$DEST/maps"
for d in samples sweeps; do ln -sfn "$SRC/$d" "$DEST/$d"; done
for m in "${META[@]}"; do ln -sfn "$SRC/${m#*:}" "$DEST/${m%%:*}"; done
for png in "$SRC"/maps/*.png; do ln -sfn "$png" "$DEST/maps/"; done
# LoadBEVSegmentation in the base nuScenes pipeline opens a NuScenesMap per city, even for detection.
if [[ ! -d "$DEST/maps/expansion" ]]; then
  python3 -m zipfile -e "$MAP_ZIP" "$DEST/maps/"
fi
ls -l "$DEST" "$DEST/maps/expansion"
