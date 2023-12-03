#!/bin/bash

# This script generates a .env file to be used with docker-compose
# To override any of the variables in this script, create watod-config.sh 
#   in the same directory and populate it with variables
#   e.g. `COMPOSE_PROJECT_NAME=<NAME>`.

if [ -f /.dockerenv ]; then
	echo "Please run this in the host machine (not in the Docker container)"
	exit 1
fi

MONO_DIR="$(dirname "$(realpath "$0")")"
# moves us one level out to the root monorepo directory
MONO_DIR=${MONO_DIR%/*}

MODULES_DIR="$MONO_DIR/modules"

# Allow for local overrides of any of the below parameters
if [ -f "$MONO_DIR/watod-config.sh" ]; then
	source "$MONO_DIR/watod-config.sh"
fi

if ! [ -x "$(command -v git)" ]; then
    echo 'Error: git is not installed.' >&2
else
    BRANCH=${BRANCH:-$(git branch --show-current)}
fi

## ----------------------- Configuration (Subject to Override) ----------------------------

COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-watod_$USER}

# Tag to use. Images as formatted as <IMAGE_NAME>:<TARGET_STAGE>-<TAG> with forward slashes replaced
# with dashes
TAG=$(echo ${TAG:-$BRANCH} | tr / -)
# replace / with -
TAG=${TAG/\//-}

# List of active modules to run, defined in docker-compose.yaml.
# Possible values:
#   - infrastructure     	:   starts visualization tools (foxglove and/or vnc)
#	- perception			:	starts perception nodes
#	- world_modeling		:	starts world modeling nodes
#	- action				:	starts action nodes
#	- simulation			:	starts simulation
#   - samples             	:   starts sample ROS2 pubsub nodes
ACTIVE_MODULES=${ACTIVE_MODULES:-""}

# List of modules to IGNORE when using the --all flag
PROFILE_BLACKLIST=${PROFILE_BLACKLIST:-"production"}

# Docker Registry to pull/push images
# REGISTRY_URL=${REGISTRY_URL:-"ghcr.io/watonomous/wato_monorepo"}

# REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')
# REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')

## --------------------------- Images -------------------------------
# NOTE: ALL IMAGE NAMES MUCH BE IN THE FORMAT OF <COMPOSE_FILE>_<SERVICE>

# ROS2 C++ Samples
SAMPLES_CPP_AGGREGATOR_IMAGE=${SAMPLES_CPP_AGGREGATOR_IMAGE:-"$REGISTRY_URL/samples_cpp_aggregator"}
SAMPLES_CPP_PRODUCER_IMAGE=${SAMPLES_CPP_PRODUCER_IMAGE:-"$REGISTRY_URL/samples_cpp_producer"}
SAMPLES_CPP_TRANSFORMER_IMAGE=${SAMPLES_CPP_TRANSFORMER_IMAGE:-"$REGISTRY_URL/samples_cpp_transformer"}

# ROS2 Python Samples
SAMPLES_PYTHON_AGGREGATOR_IMAGE=${SAMPLES_PYTHON_AGGREGATOR_IMAGE:-"$REGISTRY_URL/samples_py_aggregator"}
SAMPLES_PYTHON_PRODUCER_IMAGE=${SAMPLES_PYTHON_PRODUCER_IMAGE:-"$REGISTRY_URL/samples_py_producer"}
SAMPLES_PYTHON_TRANSFORMER_IMAGE=${SAMPLES_PYTHON_TRANSFORMER_IMAGE:-"$REGISTRY_URL/samples_py_transformer"}

# Infrastructure
INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE=${INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE:-"$REGISTRY_URL/infrastructure_vis_tools_vnc"}
INFRASTRUCTURE_DATA_STREAM_IMAGE=${DATA_STREAM_IMAGE:-"$REGISTRY_URL/infrastructure_data_stream"}
INFRASTRUCTURE_FOXGLOVE_IMAGE=${DATA_STREAM_IMAGE:-"$REGISTRY_URL/infrastructure_foxglove"}

# Perception
RADAR_OBJECT_DETECTION_IMAGE=${RADAR_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/radar_object_detection"}

## -------------------------- User ID -----------------------------

FIXUID=$(id -u) 
FIXGID=$(id -g) 

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((BASE_PORT++))}

## -------------------- Export Environment Variables -------------------------

# General
echo "# Auto-generated by ${BASH_SOURCE[0]}. Edit at own risk." > "$MODULES_DIR/.env"

echo "MODULES_DIR=$MODULES_DIR" >> "$MODULES_DIR/.env"
echo "MONO_DIR=$MONO_DIR" >> "$MODULES_DIR/.env"

echo "ACTIVE_MODULES=\"$ACTIVE_MODULES\"" >> "$MODULES_DIR/.env"
echo "PROFILE_BLACKLIST=\"$PROFILE_BLACKLIST\"" >> "$MODULES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$MODULES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$MODULES_DIR/.env"

echo "ROS_IP=$ROS_IP" >> "$MODULES_DIR/.env"
echo "ROS_HOSTNAME=$ROS_HOSTNAME" >> "$MODULES_DIR/.env"

echo "TAG=$TAG" >> "$MODULES_DIR/.env"
echo "TARGET_STAGE=$TARGET_STAGE" >> "$MODULES_DIR/.env"

echo "FIXUID=$FIXUID" >> "$MODULES_DIR/.env"
echo "FIXGID=$FIXGID" >> "$MODULES_DIR/.env"

echo "BASE_PORT=$BASE_PORT" >> "$MODULES_DIR/.env"
echo "GUI_TOOLS_VNC_PORT=$GUI_TOOLS_VNC_PORT" >> "$MODULES_DIR/.env"
echo "FOXGLOVE_BRIDGE_PORT=$FOXGLOVE_BRIDGE_PORT" >> "$MODULES_DIR/.env"

echo "REGISTRY=$REGISTRY" >> "$MODULES_DIR/.env"
echo "REGISTRY=$REPOSITORY" >> "$MODULES_DIR/.env"

# ROS2 C++ Samples
echo "SAMPLES_CPP_AGGREGATOR_IMAGE=$SAMPLES_CPP_AGGREGATOR_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_CPP_PRODUCER_IMAGE=$SAMPLES_CPP_PRODUCER_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_CPP_TRANSFORMER_IMAGE=$SAMPLES_CPP_TRANSFORMER_IMAGE" >> "$MODULES_DIR/.env"

# ROS2 Python Samples
echo "SAMPLES_PYTHON_AGGREGATOR_IMAGE=$SAMPLES_PYTHON_AGGREGATOR_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_PYTHON_PRODUCER_IMAGE=$SAMPLES_PYTHON_PRODUCER_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_PYTHON_TRANSFORMER_IMAGE=$SAMPLES_PYTHON_TRANSFORMER_IMAGE" >> "$MODULES_DIR/.env"

# Infrastructure
echo "INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE=$INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE" >> "$MODULES_DIR/.env"
echo "INFRASTRUCTURE_DATA_STREAM_IMAGE=$INFRASTRUCTURE_DATA_STREAM_IMAGE" >> "$MODULES_DIR/.env"
echo "INFRASTRUCTURE_FOXGLOVE_IMAGE=$INFRASTRUCTURE_FOXGLOVE_IMAGE" >> "$MODULES_DIR/.env"

# Perception
echo "RADAR_OBJECT_DETECTION_IMAGE=$RADAR_OBJECT_DETECTION_IMAGE" >> "$MODULES_DIR/.env"

# World Modeling
# Control
# Simulation
