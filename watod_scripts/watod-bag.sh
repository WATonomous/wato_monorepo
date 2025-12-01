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

# Ensure bags directory exists
mkdir -p "$BAG_DIRECTORY"

# Handle custom ls command
if [[ $# -gt 0 && "$1" == "ls" ]]; then
  echo "Bags directory: $BAG_DIRECTORY"
  echo ""

  if [[ ! -d "$BAG_DIRECTORY" ]]; then
    echo "Directory does not exist"
    exit 0
  fi

  # Find all metadata.yaml files which indicate bag directories
  mapfile -t bag_files < <(find "$BAG_DIRECTORY" -name "metadata.yaml" -type f 2>/dev/null | sort)

  if [[ ${#bag_files[@]} -eq 0 ]]; then
    echo "No bag files found"
    exit 0
  fi

  echo "Available bags:"
  for metadata in "${bag_files[@]}"; do
    # Get the directory containing metadata.yaml (this is the bag)
    bag_dir=$(dirname "$metadata")
    # Get the relative path from BAG_DIRECTORY
    rel_path=${bag_dir#"$BAG_DIRECTORY"/}

    # Get size and modification time
    size=$(du -sh "$bag_dir" 2>/dev/null | cut -f1)
    mtime=$(stat -c %y "$bag_dir" 2>/dev/null | cut -d'.' -f1)

    echo "  $rel_path"
    echo "    Size: $size, Modified: $mtime"
  done

  exit 0
fi

# Check if this is a record command without output specified
# If so, add default output with timestamp
declare -a ros2_bag_args=("$@")

# Handle absolute paths for play command
# If an absolute path is provided, we need to mount its parent directory
# and adjust the path inside the container
if [[ $# -gt 0 && "$1" == "play" ]]; then
  # Find the bag_path argument (first non-flag argument after "play")
  bag_path=""
  for ((i=2; i<=$#; i++)); do
    arg="${!i}"
    if [[ "$arg" != -* ]]; then
      bag_path="$arg"
      break
    fi
  done
  
  # If bag_path is absolute and outside BAG_DIRECTORY, mount parent directory
  if [[ -n "$bag_path" && "$bag_path" == /* ]]; then
    if [[ "$bag_path" != "$BAG_DIRECTORY"/* ]]; then
      # Absolute path outside BAG_DIRECTORY - mount parent directory
      bag_parent=$(dirname "$bag_path")
      bag_name=$(basename "$bag_path")
      # Replace the absolute path with relative path inside container
      for i in "${!ros2_bag_args[@]}"; do
        if [[ "${ros2_bag_args[$i]}" == "$bag_path" ]]; then
          ros2_bag_args[$i]="/bags/$bag_name"
          # Mount the parent directory instead of BAG_DIRECTORY
          BAG_DIRECTORY="$bag_parent"
          break
        fi
      done
    else
      # Absolute path inside BAG_DIRECTORY - convert to relative
      rel_path=${bag_path#"$BAG_DIRECTORY"/}
      for i in "${!ros2_bag_args[@]}"; do
        if [[ "${ros2_bag_args[$i]}" == "$bag_path" ]]; then
          ros2_bag_args[$i]="/bags/$rel_path"
          break
        fi
      done
    fi
  fi
fi

if [[ $# -gt 0 && "$1" == "record" ]]; then
  # Check if -o or --output is specified
  has_output=false
  has_storage=false
  has_max_bag_size=false
  for arg in "${@}"; do
    if [[ "$arg" == "-o" || "$arg" == "--output" ]]; then
      has_output=true
    fi
    if [[ "$arg" == "-s" || "$arg" == "--storage" ]]; then
      has_storage=true
    fi
    if [[ "$arg" == "-b" || "$arg" == "--max-bag-size" ]]; then
      has_max_bag_size=true
    fi
  done

  # Build the record command with storage format
  if [[ "$has_output" == "false" ]]; then
    # No output specified, add default output with timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)
    bag_name="recording_${timestamp}"
    ros2_bag_args=("record" "-o" "$bag_name" "${@:2}")
    echo "Recording to: $BAG_DIRECTORY/$bag_name"
  fi

  # Always ensure mcap storage format
  if [[ "$has_storage" == "false" ]]; then
    ros2_bag_args+=("--storage" "mcap")
    echo "Using mcap storage format"
  fi

  # Add max bag size limit (20GB in bytes) if not already specified
  if [[ "$has_max_bag_size" == "false" ]]; then
    echo "splitting bags by 20GB"
    ros2_bag_args+=("--max-bag-size" "21474836480")
  fi
fi

# Run ros2 bag command in container with bags directory mounted
cleanup_bag() {
  echo ""
  echo "Stopping recording..."
  docker stop "${COMPOSE_PROJECT_NAME}-bag_recorder" || true
  echo "Recording stopped"
  exit 0
}

trap cleanup_bag SIGINT SIGTERM

# Get the roudi container name for volume sharing
roudi_container="${COMPOSE_PROJECT_NAME}-roudi-1"

# Check if roudi is running
if ! docker ps --format '{{.Names}}' | grep -q "^${roudi_container}$"; then
  echo "Error: RouDi container '${roudi_container}' is not running."
  echo "Please start watod services first with './watod up -d'"
  exit 1
fi

# Use CycloneDDS for bag operations to avoid RouDi connection issues.
# Still join RouDi's IPC namespace so Iceoryx clients won't segfault if RMW gets overridden.
docker run --rm -t \
  --ipc "container:${roudi_container}" \
  --network host \
  --name "${COMPOSE_PROJECT_NAME}-bag_recorder" \
  --volumes-from "${roudi_container}" \
  -v "$BAG_DIRECTORY:/bags" \
  -w /bags \
  -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  -e "CYCLONEDDS_URI=${CYCLONEDDS_URI}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  "$INFRASTRUCTURE_IMAGE:$TAG" \
  ros2 bag "${ros2_bag_args[@]}" &

wait $!
