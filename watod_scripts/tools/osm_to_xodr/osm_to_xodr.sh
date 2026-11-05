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
CONFIG_TEMPLATE="$TOOL_DIR/config.sh"
CONFIG_FILE="$TOOL_DIR/config.local.sh"

usage() {
  cat <<'EOF'
Usage: osm_to_xodr.sh INPUT.osm [OUTPUT.xodr] [--force] [map=CONFIG] [KEY=VALUE ...]

Builds upstream's pinned Dockerfile (convert target) and invokes its convert CLI.
Default output: maps/xodr/INPUT.xodr, relative to the monorepo root.
Coordinate settings and other defaults are loaded from config.local.sh.
Command-line Hydra settings override config.local.sh; target=carla is selected
for you. --force permits overwriting an existing output. Requires Docker only.

Example with an explicit geographic reference (does not center the map):
  osm_to_xodr.sh map.osm map=example_latlon \
    map.lat_lon.latitude=45.5017 map.lat_lon.longitude=-73.5673

To center the exported map, use map=example_mgrs_offset and set map.mgrs_grid,
map.offset.x, map.offset.y, and map.offset.z for the desired reference point.
See the README and upstream usage guide for the coordinate settings.
EOF
}

[[ -f "$CONFIG_TEMPLATE" ]] || {
  echo "Error: config template does not exist: $CONFIG_TEMPLATE" >&2
  exit 1
}
# shellcheck source=config.sh
source "$CONFIG_TEMPLATE"
[[ -f "$CONFIG_FILE" ]] || {
  echo "Error: local config does not exist: $CONFIG_FILE" >&2
  echo "Create it with: cp $CONFIG_TEMPLATE $CONFIG_FILE" >&2
  exit 1
}
# shellcheck disable=SC1090  # config.local.sh is intentionally untracked.
source "$CONFIG_FILE"

MAP_CONFIG="${MAP_CONFIG:-}"
LATITUDE="${LATITUDE:-}"
LONGITUDE="${LONGITUDE:-}"
ALTITUDE="${ALTITUDE:-0}"
MGRS_GRID="${MGRS_GRID:-}"
OFFSET_X="${OFFSET_X:-}"
OFFSET_Y="${OFFSET_Y:-}"
OFFSET_Z="${OFFSET_Z:-0}"
FORCE="${FORCE:-false}"
declare -p EXTRA_OPTIONS &>/dev/null || EXTRA_OPTIONS=()
if [[ "$(declare -p EXTRA_OPTIONS 2>/dev/null)" != "declare -a"* ]]; then
  echo "Error: EXTRA_OPTIONS in $CONFIG_FILE must be a Bash array" >&2
  exit 2
fi
if [[ "$FORCE" != true && "$FORCE" != false ]]; then
  echo "Error: FORCE in $CONFIG_FILE must be true or false" >&2
  exit 2
fi

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

CLI_FORCE=false
CLI_MAP=""
CLI_OVERRIDES=()
declare -A CLI_KEYS=()
for argument in "$@"; do
  case "$argument" in
    --force) CLI_FORCE=true ;;
    map=*) CLI_MAP="${argument#map=}"; CLI_KEYS[map]=true; CLI_OVERRIDES+=("$argument") ;;
    *=*) CLI_KEYS["${argument%%=*}"]=true; CLI_OVERRIDES+=("$argument") ;;
    *) echo "Error: expected upstream KEY=VALUE argument, got: $argument" >&2; exit 2 ;;
  esac
done
if [[ "$CLI_FORCE" == true ]]; then
  FORCE=true
fi

EFFECTIVE_MAP="${CLI_MAP:-$MAP_CONFIG}"
if [[ -z "$EFFECTIVE_MAP" ]]; then
  echo "Error: set MAP_CONFIG in $CONFIG_FILE or specify map=CONFIG; see --help" >&2
  exit 2
fi

CONFIG_OVERRIDES=("map=$EFFECTIVE_MAP")
case "$EFFECTIVE_MAP" in
  example_latlon)
    [[ -n "$LATITUDE" && -n "$LONGITUDE" ]] || {
      echo "Error: set LATITUDE and LONGITUDE in $CONFIG_FILE" >&2
      exit 2
    }
    CONFIG_OVERRIDES+=(
      "map.lat_lon.latitude=$LATITUDE"
      "map.lat_lon.longitude=$LONGITUDE"
      "map.lat_lon.altitude=$ALTITUDE"
    )
    ;;
  example_mgrs_offset)
    [[ -n "$MGRS_GRID" && -n "$OFFSET_X" && -n "$OFFSET_Y" ]] || {
      echo "Error: set MGRS_GRID, OFFSET_X, and OFFSET_Y in $CONFIG_FILE" >&2
      exit 2
    }
    CONFIG_OVERRIDES+=(
      "map.mgrs_grid=$MGRS_GRID"
      "map.offset.x=$OFFSET_X"
      "map.offset.y=$OFFSET_Y"
      "map.offset.z=$OFFSET_Z"
    )
    ;;
esac
CONFIG_OVERRIDES+=("${EXTRA_OPTIONS[@]}")

# Command-line values win over values with the same Hydra key in config.sh.
OVERRIDES=()
for argument in "${CONFIG_OVERRIDES[@]}"; do
  key="${argument%%=*}"
  [[ -v "CLI_KEYS[$key]" ]] || OVERRIDES+=("$argument")
done
OVERRIDES+=("${CLI_OVERRIDES[@]}")

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
