#!/usr/bin/env bash
# Prepare a Lanelet2 test track without a CARLA server or ROS installation.
set -euo pipefail
task_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 2 ]]; then
  echo "Usage: watod-map.sh INPUT.osm OUTPUT_DIR --origin-lat LAT --origin-lon LON [options]" >&2
  exit 2
fi
task_input="$1"
task_output="$2"
shift 2
[[ -f "$task_input" ]] || { echo "Missing map: $task_input" >&2; exit 1; }
task_input_dir="$(cd "$(dirname "$task_input")" && pwd)"
task_input_name="$(basename "$task_input")"
mkdir -p "$task_output"
task_output_dir="$(cd "$task_output" && pwd)"
docker build --tag watod/lanelet-track:1 --file "$task_repo/watod_scripts/tools/lanelet_to_xodr/Dockerfile" "$task_repo"
exec docker run --rm --network none --user "$(id -u):$(id -g)" \
  --volume "$task_input_dir/$task_input_name:/input/map.osm:ro" \
  --volume "$task_output_dir:/output" \
  watod/lanelet-track:1 /input/map.osm /output "$@"
