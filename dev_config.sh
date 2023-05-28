#!/bin/bash

# This script generates a .env file to be used with docker-compose
# To override any of the variables in this script, create dev_config.local.sh 
#   in the same directory and populate it with variables
#   e.g. `COMPOSE_PROJECT_NAME=<NAME>`.

if [ -f /.dockerenv ]; then
	echo "Please run this in the host machine (not in the Docker container)"
	exit 1
fi

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PROFILES_DIR="$SCRIPT_DIR/profiles"

# Allow for local overrides of any of the below parameters
if [ -f "$SCRIPT_DIR/dev_config.local.sh" ]; then
	source "$SCRIPT_DIR/dev_config.local.sh"
fi

if ! [ -x "$(command -v git)" ]; then
    echo 'Error: git is not installed.' >&2
else
    BRANCH=${BRANCH:-$(git branch --show-current)}
fi
# replace / with -
BRANCH=${BRANCH/\//-}

## ----------------------- Configuration ----------------------------
COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-watod_$USER}

# Docker build stage to build or run. Typically our dockerfiles have two main stages: 
#   repo (minimal) and debug (containing debug tools, eg code-server)
TARGET_STAGE=${TARGET_STAGE:-"debug"}

# Tag to use. Images as formatted as <IMAGE_NAME>:<TARGET_STAGE>-<TAG> with forward slashes replaced with dashes
TAG=$(echo ${TAG:-$BRANCH} | tr / -)

# ROS hostname + URI for the production profile
# ROS_IP is the IP that nodes will publish as (client's hostname)
ROS_HOSTNAME=${ROS_HOSTNAME:-"localhost"}
ROS_IP=${ROS_IP:-"127.0.0.1"}

## ----------------- Profile Configuration --------------------

# List of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - vis_tools     		  :   starts visualization tools (vnc and foxglove)
#   - production    		  :   configs for all containers required in production
#   - samples             :   starts sample ROS2 pubsub nodes
ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}

# List of profiles to IGNORE when using the --all flag
PROFILE_BLACKLIST=${PROFILE_BLACKLIST:-"production"}

## --------------------------- Images -------------------------------

# ROS2 C++ Samples
SAMPLES_CPP_AGGREGATOR_IMAGE=${SAMPLES_CPP_AGGREGATOR_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_aggregator"}
SAMPLES_CPP_PRODUCER_IMAGE=${SAMPLES_CPP_PRODUCER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_producer"}
SAMPLES_CPP_TRANSFORMER_IMAGE=${SAMPLES_CPP_TRANSFORMER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_transformer"}

# Infrastructure
INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE=${INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/infrastructure_vis_tools_vnc"}
INFRASTRUCTURE_DATA_STREAM_IMAGE=${DATA_STREAM_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/infrastructure_data_stream"}

# Simulation
CARLA_SERVER_IMAGE=${CARLA_SERVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/carla_server_image"}
CARLA_ROS2_BRIDGE_IMAGE=${CARLA_ROS2_BRIDGE_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/carla_ros2_bridge_image"}
SIMULATION_CAMERA_COMPRESSION_IMAGE=${SIMULATION_CAMERA_COMPRESSION_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/simulation_camera_compression"}
INFRASTRUCTURE_FOXGLOVE_IMAGE=${DATA_STREAM_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/infrastructure_foxglove"}
## -------------------------- User ID -----------------------------

FIXUID=$(id -u) 
FIXGID=$(id -g) 

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}

## -------------------- Environment Variables -------------------------

echo "# Auto-generated by ${BASH_SOURCE[0]}. Please do not edit." > "$PROFILES_DIR/.env"
echo "ACTIVE_PROFILES=\"$ACTIVE_PROFILES\"" > "$PROFILES_DIR/.env"
echo "PROFILE_BLACKLIST=\"$PROFILE_BLACKLIST\"" >> "$PROFILES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$PROFILES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$PROFILES_DIR/.env"

echo "ROS_IP=$ROS_IP" >> "$PROFILES_DIR/.env"
echo "ROS_HOSTNAME=$ROS_HOSTNAME" >> "$PROFILES_DIR/.env"

echo "INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE=$INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE" >> "$PROFILES_DIR/.env"

echo "SAMPLES_CPP_AGGREGATOR_IMAGE=$SAMPLES_CPP_AGGREGATOR_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_CPP_PRODUCER_IMAGE=$SAMPLES_CPP_PRODUCER_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_CPP_TRANSFORMER_IMAGE=$SAMPLES_CPP_TRANSFORMER_IMAGE" >> "$PROFILES_DIR/.env"
echo "INFRASTRUCTURE_DATA_STREAM_IMAGE=$INFRASTRUCTURE_DATA_STREAM_IMAGE" >> "$PROFILES_DIR/.env"
echo "INFRASTRUCTURE_FOXGLOVE_IMAGE=$INFRASTRUCTURE_FOXGLOVE_IMAGE" >> "$PROFILES_DIR/.env"

echo "CARLA_SERVER_IMAGE=$CARLA_SERVER_IMAGE" >> "$PROFILES_DIR/.env"
echo "CARLA_ROS2_BRIDGE_IMAGE=$CARLA_ROS2_BRIDGE_IMAGE" >> "$PROFILES_DIR/.env"
echo "SIMULATION_CAMERA_COMPRESSION_IMAGE=$SIMULATION_CAMERA_COMPRESSION_IMAGE" >> "$PROFILES_DIR/.env"

echo "TAG=$TAG" >> "$PROFILES_DIR/.env"
echo "TARGET_STAGE=$TARGET_STAGE" >> "$PROFILES_DIR/.env"

echo "FIXUID=$FIXUID" >> "$PROFILES_DIR/.env"
echo "FIXGID=$FIXGID" >> "$PROFILES_DIR/.env"

echo "BASE_PORT=$BASE_PORT" >> "$PROFILES_DIR/.env"
echo "GUI_TOOLS_VNC_PORT=$GUI_TOOLS_VNC_PORT" >> "$PROFILES_DIR/.env"
cat $PROFILES_DIR/.env
