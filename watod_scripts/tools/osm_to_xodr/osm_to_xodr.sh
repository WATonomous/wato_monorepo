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
IMAGE="watod/osm_to_xodr:carla-0.9.16"

usage() {
  cat <<'EOF'
Usage: osm_to_xodr.sh INPUT.osm [OUTPUT.xodr] [conversion options]

Builds the Docker image and converts an OSM XML export to OpenDRIVE.
Default output: watod_scripts/tools/osm_to_xodr/output/INPUT.xodr

Options:
  --force                               Overwrite existing output
  --osm-way-types TYPE [TYPE ...]        Select OSM highway types
  --offset-x METERS --offset-y METERS    Enable coordinate offsets
  --center-map / --no-center-map        Toggle centering at the map origin
  --default-lane-width METERS            Set width for lanes without OSM widths
  --elevation-layer-height METERS        Set height between OSM layers
  --proj-string STRING                  Set the PROJ projection
  --generate-traffic-lights / --no-generate-traffic-lights
                                         Toggle OSM traffic-light generation
  --all-junctions-with-traffic-lights    Generate lights at every junction
  --traffic-light-excluded-way-types TYPE [TYPE ...]

Requires Docker. Uses linux/amd64, including on Apple Silicon via emulation.
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
if [[ ! -f "$INPUT" ]]; then
  echo "Error: input file does not exist: $INPUT" >&2
  exit 1
fi
INPUT_DIR="$(cd "$(dirname "$INPUT")" && pwd)"
INPUT_NAME="$(basename "$INPUT")"
OUTPUT="$TOOL_DIR/output/${INPUT_NAME%.*}.xodr"
if [[ $# -gt 0 && "$1" != -* ]]; then
  OUTPUT="$1"
  shift
fi
if ! command -v docker >/dev/null 2>&1; then
  echo "Error: Docker is required to run osm_to_xodr" >&2
  exit 1
fi
mkdir -p "$(dirname "$OUTPUT")"
OUTPUT_DIR="$(cd "$(dirname "$OUTPUT")" && pwd)"
OUTPUT_NAME="$(basename "$OUTPUT")"
if [[ "$INPUT_DIR/$INPUT_NAME" == "$OUTPUT_DIR/$OUTPUT_NAME" ]]; then
  echo "Error: input and output must be different files" >&2
  exit 1
fi

docker build --platform linux/amd64 --tag "$IMAGE" "$TOOL_DIR"
exec docker run --rm --platform linux/amd64 \
  --user "$(id -u):$(id -g)" \
  --network none \
  --volume "$INPUT_DIR/$INPUT_NAME:/input/map.osm:ro" \
  --volume "$OUTPUT_DIR:/output" \
  "$IMAGE" /input/map.osm "/output/$OUTPUT_NAME" "$@"
