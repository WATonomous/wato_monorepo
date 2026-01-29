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
# Auto-detect CI environment if not explicitly set
if [[ -n ${CI:-} || -n ${GITHUB_ACTIONS:-} ]]; then
  IS_CI=true
else
  IS_CI=false
fi

# Allow explicit override via command-line flag
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
# In CI, git is in detached HEAD state, so use CI environment variables
if $IS_CI && [[ -n ${SOURCE_BRANCH:-} ]]; then
  BRANCH=${BRANCH:-$SOURCE_BRANCH}
  echo "[setup-env] CI mode: using SOURCE_BRANCH → $BRANCH"
elif command -v git >/dev/null 2>&1; then
  BRANCH=${BRANCH:-$(git branch --show-current)}
  if [[ -z $BRANCH ]]; then
    echo 'Error: git branch is empty (detached HEAD?). Set BRANCH or SOURCE_BRANCH environment variable.' >&2
    exit 1
  fi
else
  echo 'Error: git is not installed.' >&2
  exit 1
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

# Bags directory
BAG_DIRECTORY=${BAG_DIRECTORY:-"$MONO_DIR/bags"}

# ROS 2 Middleware configuration
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-"rmw_zenoh_cpp"}

# Zenoh configuration URIs (paths inside Docker containers)
ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI:-"/opt/watonomous/rmw_zenoh_router_config.json5"}
ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI:-"/opt/watonomous/rmw_zenoh_session_config.json5"}

# Docker socket path (needed for log viewer)
# DOCKER_HOST should be automaticall set in any WATCloud SLURM session
DOCKER_HOST=${DOCKER_HOST:-unix:///var/run/docker.sock}
DOCKER_SOCKET_PATH=${DOCKER_HOST#unix://} # strip the "unix://" prefix to get path

################################  Image names  #######################################
# NOTE: ALL IMAGE NAMES MUST BE IN THE FORMAT <COMPOSE_FILE>_<SERVICE>

# Infrastructure
INFRASTRUCTURE_IMAGE=${INFRASTRUCTURE_IMAGE:-"$REGISTRY_URL/infrastructure/infrastructure"}
: "${FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES:=67108864}"

# Perception
PERCEPTION_IMAGE=${PERCEPTION_IMAGE:-"$REGISTRY_URL/perception/perception"}
PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE=${PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/camera_object_detection"}
PERCEPTION_DEPTH_ESTIMATION_IMAGE=${PERCEPTION_DEPTH_ESTIMATION_IMAGE:-"$REGISTRY_URL/perception/depth_estimation"}

# World Modeling
WORLD_MODELING_IMAGE=${WORLD_MODELING_IMAGE:-"$REGISTRY_URL/world_modeling/world_modeling"}

# Action
ACTION_IMAGE=${ACTION_IMAGE:-"$REGISTRY_URL/action/action"}

# Simulation
SIMULATION_IMAGE=${SIMULATION_IMAGE:-"$REGISTRY_URL/simulation/carla_sim"}

# Interfacing
INTERFACING_IMAGE=${INTERFACING_IMAGE:-"$REGISTRY_URL/interfacing/interfacing"}

################################  UID / GID  #########################################
SETUID=$(id -u)
SETGID=$(id -g)
USERNAME=${USERNAME:-$USER}

################################  Ports  #############################################
BASE_PORT=${BASE_PORT:-$((SETUID*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT+1))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((BASE_PORT+2))}
LOG_VIEWER__PORT=${LOG_VIEWER__PORT:-$((BASE_PORT+6))}
PYGAME_HUD_PORT=${PYGAME_HUD_PORT:-$((BASE_PORT+7))}
CARLA_PORT=${CARLA_PORT:-$((BASE_PORT+8))}

################################  Simulation  ########################################
CARLA_RENDER_MODE=${CARLA_RENDER_MODE:-"no_gpu"}
# Enable pygame HUD when running without GPU (provides web-based visualization fallback)
if [[ "$CARLA_RENDER_MODE" == "no_gpu" ]]; then
  PYGAME_HUD_ENABLED=${PYGAME_HUD_ENABLED:-"true"}
else
  PYGAME_HUD_ENABLED=${PYGAME_HUD_ENABLED:-"false"}
fi

############################  ROS DOMAIN ID  #########################################
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((SETUID % 230))}

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
  append "USERNAME" "$USERNAME"
fi

append "BASE_PORT" "$BASE_PORT"
append "GUI_TOOLS_VNC_PORT" "$GUI_TOOLS_VNC_PORT"
append "FOXGLOVE_BRIDGE_PORT" "$FOXGLOVE_BRIDGE_PORT"
append "LOG_VIEWER__PORT" "$LOG_VIEWER__PORT"
append "PYGAME_HUD_PORT" "$PYGAME_HUD_PORT"
append "CARLA_PORT" "$CARLA_PORT"

# Simulation
append "CARLA_RENDER_MODE" "$CARLA_RENDER_MODE"
append "PYGAME_HUD_ENABLED" "$PYGAME_HUD_ENABLED"

append "REGISTRY" "$REGISTRY"
append "REPOSITORY" "$REPOSITORY"

# Image variables
append "INFRASTRUCTURE_IMAGE" "$INFRASTRUCTURE_IMAGE"
append "FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES" "$FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES"

append "PERCEPTION_IMAGE" "$PERCEPTION_IMAGE"
append "PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE" "$PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE"
append "PERCEPTION_DEPTH_ESTIMATION_IMAGE" "$PERCEPTION_DEPTH_ESTIMATION_IMAGE"

append "WORLD_MODELING_IMAGE" "$WORLD_MODELING_IMAGE"

append "ACTION_IMAGE" "$ACTION_IMAGE"

append "SIMULATION_IMAGE" "$SIMULATION_IMAGE"

append "INTERFACING_IMAGE" "$INTERFACING_IMAGE"

# ROS 2 Middleware
append "RMW_IMPLEMENTATION" "$RMW_IMPLEMENTATION"

# Zenoh configuration
append "ZENOH_ROUTER_CONFIG_URI" "$ZENOH_ROUTER_CONFIG_URI"
append "ZENOH_SESSION_CONFIG_URI" "$ZENOH_SESSION_CONFIG_URI"

append "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"

# Docker socket (needed for log viewer)
append "DOCKER_SOCKET_PATH" "$DOCKER_SOCKET_PATH"

echo "[setup-env] .env generated at $ENV_FILE"
