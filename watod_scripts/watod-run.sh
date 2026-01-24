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

# Available recording profiles
PROFILES_DIR="${MONO_DIR}/src/infrastructure/bag_recorder/config"

# Get list of available profiles (strip .yaml extension)
get_profiles() {
  if [[ -d "$PROFILES_DIR" ]]; then
    find "$PROFILES_DIR" -name "*.yaml" -type f -exec basename {} .yaml \; | sort
  fi
}

# Print usage
usage() {
  cat <<EOF
Usage: watod run <profile> [options]

Launch preconfigured recording profiles as ROS2 nodes.

Available profiles:
EOF
  # List profiles with descriptions
  for profile in $(get_profiles); do
    desc=$(grep -m1 "^# Recording profile:" "$PROFILES_DIR/$profile.yaml" 2>/dev/null | sed 's/# Recording profile: //' || echo "$profile")
    printf "  %-16s %s\n" "$profile" "$desc"
  done
  cat <<EOF

Options:
  -o, --output NAME    Output bag name (default: <profile>_YYYYMMDD_HHMMSS)
  -l, --list           List available profiles
  -h, --help           Show this help

Examples:
  watod run all_sensors              # Record all sensors with rectified images
  watod run camera_only              # Record only cameras (no LiDAR)
  watod run lidar_only               # Record only LiDARs (no cameras)
  watod run all_sensors -o my_bag    # Record with custom output name
EOF
}

# Handle --list flag
if [[ $# -eq 0 ]] || [[ "$1" == "-l" ]] || [[ "$1" == "--list" ]]; then
  echo "Available recording profiles:"
  for profile in $(get_profiles); do
    desc=$(grep -A1 "^# Recording profile:" "$PROFILES_DIR/$profile.yaml" 2>/dev/null | tail -1 | sed 's/^# //' || echo "")
    printf "  %-16s %s\n" "$profile" "$desc"
  done
  exit 0
fi

# Handle help
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
  usage
  exit 0
fi

# Parse arguments
PROFILE="$1"
shift

OUTPUT_NAME=""
while [[ $# -gt 0 ]]; do
  case $1 in
    -o|--output)
      OUTPUT_NAME="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

# Validate profile exists
if [[ ! -f "$PROFILES_DIR/$PROFILE.yaml" ]]; then
  echo "Error: Profile '$PROFILE' not found."
  echo "Available profiles: $(get_profiles | tr '\n' ' ')"
  exit 1
fi

# Generate output name if not specified
if [[ -z "$OUTPUT_NAME" ]]; then
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  OUTPUT_NAME="${PROFILE}_${TIMESTAMP}"
fi

OUTPUT_PATH="$BAG_DIRECTORY/$OUTPUT_NAME"

# Ensure bags directory exists
mkdir -p "$BAG_DIRECTORY"

echo "Recording profile: $PROFILE"
echo "Output: $OUTPUT_PATH"
echo ""

# Cleanup function
cleanup_recorder() {
  echo ""
  echo "Stopping recording..."
  docker stop "${COMPOSE_PROJECT_NAME}-bag_recorder_node" 2>/dev/null || true
  echo "Recording stopped. Bag saved to: $OUTPUT_PATH"
  exit 0
}

trap cleanup_recorder SIGINT SIGTERM

# Check if infrastructure is running
infra_container="${COMPOSE_PROJECT_NAME}-infrastructure_bringup-1"
if ! docker ps --format '{{.Names}}' | grep -q "^${infra_container}$"; then
  echo "Error: Infrastructure container '${infra_container}' is not running."
  echo "Please start watod services first with './watod up -d'"
  exit 1
fi

# Run the recorder node via ros2 launch
docker run --rm -t \
  --ulimit memlock=-1 \
  --ipc host \
  --network host \
  --name "${COMPOSE_PROJECT_NAME}-bag_recorder_node" \
  -v "$BAG_DIRECTORY:/bags" \
  -v "$MONO_DIR/src/infrastructure/bag_recorder:/opt/ros_ws/src/bag_recorder:ro" \
  -w /bags \
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -e "ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI}" \
  -e "ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI}" \
  "$INFRASTRUCTURE_IMAGE:$TAG" \
  ros2 launch bag_recorder record.launch.yaml \
    profile:="$PROFILE" \
    bag_path:="/bags/$OUTPUT_NAME" &

wait $!
