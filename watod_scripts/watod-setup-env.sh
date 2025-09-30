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
set -e

# ------------------------------------------------------------------------------------
# watod-setup-env.sh
# -------------------
# Generates a .env file for docker‑compose based on project & host settings.
#
# Usage (source so variables propagate):
#   . ./watod-setup-env.sh                   # normal local run
#   . ./watod-setup-env.sh --is-ci           # CI run (forces MODULES_DIR layout & skips UID/GID)
#
# Flags
#   --is-ci        Explicitly signal that we are running inside a CI agent.  *No autodetection*
#                  · Forces MODULES_DIR="$MONO_DIR/modules"
#                  · Omits SETUID / SETGID entries from the generated .env
#
# To override any variable, create a sibling script named `watod-config.sh` and define
# the desired env‑vars, e.g.  COMPOSE_PROJECT_NAME=<name>
# ------------------------------------------------------------------------------------

################################  Flag parsing  ######################################
IS_CI=false
for arg in "$@"; do
  case "$arg" in
    --is-ci) IS_CI=true ;;
  esac
done

################################  Safety checks  #####################################
if [ -f /.dockerenv ]; then
  echo "Please run this in the host machine (not in the Docker container)" >&2
  exit 1
fi

################################  Git branch  ########################################
if command -v git >/dev/null 2>&1; then
  BRANCH=${BRANCH:-$(git branch --show-current)}
else
  echo 'Error: git is not installed.' >&2
fi

################################  Overrides hook  ####################################
# shellcheck source=./watod-config.sh
if [ -f "$(dirname "$0")/watod-config.sh" ]; then
  # shellcheck disable=SC1091
  . "$(dirname "$0")/watod-config.sh"
fi

################################  Paths & Layout  ####################################
# Force MODULES_DIR and MONO_DIR layout when running in CI
if $IS_CI; then
  git_root=$(git rev-parse --show-toplevel 2>/dev/null)
  MONO_DIR="$git_root"
  MODULES_DIR="$MONO_DIR/modules"
  echo "[setup-env] CI mode: forcing MODULES_DIR → $MODULES_DIR"
fi

################################  Configuration  #####################################

COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-watod_$USER}

# Tag for docker images – convert slashes to dashes
TAG=$(echo "${TAG:-$BRANCH}" | tr '/' '-')

# Registry
REGISTRY_URL=${REGISTRY_URL:-"ghcr.io/watonomous/wato_monorepo"}
REGISTRY="${REGISTRY_URL%%/*}"
REPOSITORY="${REGISTRY_URL##*/}"

################################  Image names  #######################################
# NOTE: ALL IMAGE NAMES MUST BE IN THE FORMAT <COMPOSE_FILE>_<SERVICE>

# Infrastructure
INFRASTRUCTURE_IMAGE=${INFRASTRUCTURE_IMAGE:-"$REGISTRY_URL/infrastructure/infrastructure"}
FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES=${FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES:-268435456}

# Perception
PERCEPTION_IMAGE=${PERCEPTION_IMAGE:-"$REGISTRY_URL/perception/perception"}
PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE=${PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/radar_object_detection"}
PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE=${PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/lidar_object_detection"}
PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE=${PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/camera_object_detection"}
PERCEPTION_LANE_DETECTION_IMAGE=${PERCEPTION_LANE_DETECTION_IMAGE:-"$REGISTRY_URL/perception/lane_detection"}
PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE=${PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE:-"$REGISTRY_URL/perception/semantic_segmentation"}
PERCEPTION_TRACKING_IMAGE=${PERCEPTION_TRACKING_IMAGE:-"$REGISTRY_URL/perception/tracking"}
PERCEPTION_BBOX_2D_3D_IMAGE=${PERCEPTION_BBOX_2D_3D_IMAGE:-"$REGISTRY_URL/perception/bbox_2d_3d"}
PERCEPTION_PATCHWORK_IMAGE=${PERCEPTION_PATCHWORK_IMAGE:-"$REGISTRY_URL/perception/patchwork"}
PERCEPTION_DEPTH_ESTIMATION_IMAGE=${PERCEPTION_DEPTH_ESTIMATION_IMAGE:-"$REGISTRY_URL/perception/depth_estimation"}

# World Modeling
WORLD_MODELING_IMAGE=${WORLD_MODELING_IMAGE:-"$REGISTRY_URL/world_modeling/world_modeling"}


# Action
ACTION_IMAGE=${ACTION_IMAGE:-"$REGISTRY_URL/action/action"}

# Simulation
SIMULATION_CARLA_IMAGE=${SIMULATION_CARLA_IMAGE:-"$REGISTRY_URL/simulation/carla_sim"}
SIMULATION_CARLA_ROS_BRIDGE_IMAGE=${SIMULATION_CARLA_ROS_BRIDGE_IMAGE:-"$REGISTRY_URL/simulation/carla_ros_bridge"}
SIMULATION_CARLAVIZ_IMAGE=${SIMULATION_CARLAVIZ_IMAGE:-"$REGISTRY_URL/simulation/carla_viz"}
SIMULATION_CARLA_NOTEBOOKS_IMAGE=${SIMULATION_CARLA_NOTEBOOKS_IMAGE:-"$REGISTRY_URL/simulation/carla_notebooks"}
SIMULATION_CARLA_SAMPLE_NODE_IMAGE=${SIMULATION_CARLA_SAMPLE_NODE_IMAGE:-"$REGISTRY_URL/simulation/carla_sample_node"}

# Interfacing
INTERFACING_IMAGE=${INTERFACING_IMAGE:-"$REGISTRY_URL/interfacing/interfacing"}

################################  UID / GID  #########################################
SETUID=$(id -u)
SETGID=$(id -g)

################################  Ports  #############################################
BASE_PORT=${BASE_PORT:-$((SETUID*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT+1))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((BASE_PORT+2))}
CARLAVIZ_PORT=${CARLAVIZ_PORT:-$((BASE_PORT+3))}
CARLAVIZ_PORT_2=${CARLAVIZ_PORT_2:-$((BASE_PORT+4))}
CARLA_NOTEBOOKS_PORT=${CARLA_NOTEBOOKS_PORT:-$((BASE_PORT+5))}

################################  Write .env  ########################################

mkdir -p "$MODULES_DIR"
ENV_FILE="$MODULES_DIR/.env"
echo "# Auto-generated by ${BASH_SOURCE[0]}. Edit at own risk." > "$ENV_FILE"

# Helper for clean appends
append() { echo "$1=$2" >> "$ENV_FILE"; }

# General paths
append "MODULES_DIR" "$MODULES_DIR"
append "MONO_DIR" "$MONO_DIR"

append "COMPOSE_DOCKER_CLI_BUILD" "1"
append "COMPOSE_PROJECT_NAME" "$COMPOSE_PROJECT_NAME"

append "TAG" "$TAG"

if ! $IS_CI; then
  append "SETUID" "$SETUID"
  append "SETGID" "$SETGID"
fi

append "BASE_PORT" "$BASE_PORT"
append "GUI_TOOLS_VNC_PORT" "$GUI_TOOLS_VNC_PORT"
append "FOXGLOVE_BRIDGE_PORT" "$FOXGLOVE_BRIDGE_PORT"
append "CARLAVIZ_PORT" "$CARLAVIZ_PORT"
append "CARLAVIZ_PORT_2" "$CARLAVIZ_PORT_2"
append "CARLA_NOTEBOOKS_PORT" "$CARLA_NOTEBOOKS_PORT"

append "REGISTRY" "$REGISTRY"
append "REPOSITORY" "$REPOSITORY"

# Image variables
append "INFRASTRUCTURE_IMAGE" "$INFRASTRUCTURE_IMAGE"
append "FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES" "$FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES"

append "PERCEPTION_IMAGE" "$PERCEPTION_IMAGE"
append "PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE" "$PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE"
append "PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE" "$PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE"
append "PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE" "$PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE"
append "PERCEPTION_LANE_DETECTION_IMAGE" "$PERCEPTION_LANE_DETECTION_IMAGE"
append "PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE" "$PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE"
append "PERCEPTION_TRACKING_IMAGE" "$PERCEPTION_TRACKING_IMAGE"
append "PERCEPTION_PATCHWORK_IMAGE" "$PERCEPTION_PATCHWORK_IMAGE"
append "PERCEPTION_DEPTH_ESTIMATION_IMAGE" "$PERCEPTION_DEPTH_ESTIMATION_IMAGE"

append "WORLD_MODELING_IMAGE" "$WORLD_MODELING_IMAGE"

append "ACTION_IMAGE" "$ACTION_IMAGE"

append "SIMULATION_CARLA_IMAGE" "$SIMULATION_CARLA_IMAGE"
append "SIMULATION_CARLA_ROS_BRIDGE_IMAGE" "$SIMULATION_CARLA_ROS_BRIDGE_IMAGE"
append "SIMULATION_CARLAVIZ_IMAGE" "$SIMULATION_CARLAVIZ_IMAGE"
append "SIMULATION_CARLA_NOTEBOOKS_IMAGE" "$SIMULATION_CARLA_NOTEBOOKS_IMAGE"
append "SIMULATION_CARLA_SAMPLE_NODE_IMAGE" "$SIMULATION_CARLA_SAMPLE_NODE_IMAGE"

append "INTERFACING_IMAGE" "$INTERFACING_IMAGE"

echo "[setup-env] .env generated at $ENV_FILE"
