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
set -euo pipefail

TOOL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPSTREAM_REV="4b707e067bfdb8db04223b87aa2da290c5cba607"
UPSTREAM="https://github.com/tier4/autoware_lanelet2_to_opendrive.git#$UPSTREAM_REV"
IMAGE="watod/lanelet_to_xodr:autoware-${UPSTREAM_REV:0:12}"

usage() {
  cat <<'EOF'
Usage: osm_to_xodr.sh INPUT.osm [OUTPUT.xodr] [--force] map=CONFIG [KEY=VALUE ...]

Builds upstream's pinned Dockerfile (convert target) and invokes its convert CLI.
Default output: maps/xodr/INPUT.xodr, relative to the monorepo root.
Pass upstream Hydra map settings explicitly; target=carla is selected for you.
--force permits overwriting an existing output. Requires Docker only.

Example with an explicit geographic reference (does not center the map):
  osm_to_xodr.sh map.osm map=example_latlon \
    map.lat_lon.latitude=45.5017 map.lat_lon.longitude=-73.5673

To center the exported map, use map=example_mgrs_offset and set map.mgrs_grid,
map.offset.x, map.offset.y, and map.offset.z for the desired reference point.
See the README and upstream usage guide for the coordinate settings.
EOF
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
  usage
  exit 0
fi

INPUT="$1"
shift
[[ -f "$INPUT" ]] || { echo "Error: input file does not exist: $INPUT" >&2; exit 1; }
INPUT_DIR="$(cd "$(dirname "$INPUT")" && pwd)"
INPUT_NAME="$(basename "$INPUT")"
OUTPUT="$TOOL_DIR/../../../maps/xodr/${INPUT_NAME%.*}.xodr"
if [[ $# -gt 0 && "$1" != -* && "$1" != *=* ]]; then
  OUTPUT="$1"
  shift
fi

FORCE=false
MAP_SELECTED=false
OVERRIDES=()
for argument in "$@"; do
  case "$argument" in
    --force) FORCE=true ;;
    map=*) MAP_SELECTED=true; OVERRIDES+=("$argument") ;;
    *=*) OVERRIDES+=("$argument") ;;
    *) echo "Error: expected upstream KEY=VALUE argument, got: $argument" >&2; exit 2 ;;
  esac
done
if [[ "$MAP_SELECTED" == false ]]; then
  echo "Error: specify map=CONFIG and its coordinate settings; see --help" >&2
  exit 2
fi
if [[ -e "$OUTPUT" && "$FORCE" == false ]]; then
  echo "Error: $OUTPUT already exists; use --force to overwrite" >&2
  exit 1
fi
command -v docker >/dev/null || { echo "Error: Docker is required" >&2; exit 1; }
mkdir -p "$(dirname "$OUTPUT")"
OUTPUT_DIR="$(cd "$(dirname "$OUTPUT")" && pwd)"
OUTPUT_NAME="$(basename "$OUTPUT")"
if [[ "$INPUT_DIR/$INPUT_NAME" == "$OUTPUT_DIR/$OUTPUT_NAME" ]]; then
  echo "Error: input and output must be different files" >&2
  exit 1
fi

# Rootless container root maps to the daemon owner's host user.
DOCKER_USER=(--user "$(id -u):$(id -g)")
if [[ "$(docker info --format '{{json .SecurityOptions}}')" == *'name=rootless'* ]]; then
  DOCKER_USER=(--user 0:0)
fi

# Quote the output path for Hydra's override grammar, including spaces/commas.
OUTPUT_VALUE="${OUTPUT_NAME//\\/\\\\}"
OUTPUT_VALUE="${OUTPUT_VALUE//\"/\\\"}"
docker build --platform linux/amd64 --target convert --tag "$IMAGE" "$UPSTREAM"
exec docker run --rm --platform linux/amd64 "${DOCKER_USER[@]}" \
  --network none --workdir /tmp --env MPLCONFIGDIR=/tmp/matplotlib \
  --volume "$INPUT_DIR/$INPUT_NAME:/input/map.osm:ro" \
  --volume "$OUTPUT_DIR:/output" \
  "$IMAGE" "${OVERRIDES[@]}" target=carla \
  input_map_path=/input/map.osm "output_map_path=\"/output/$OUTPUT_VALUE\""
