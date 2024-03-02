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

if [ ! -z $MODULES_DIR_EXP ]; then
	MODULES_DIR="$MODULES_DIR_EXP"
else
	MODULES_DIR="$MONO_DIR/modules"
fi

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

# Tag to use. Images as formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced
# with dashes
TAG=$(echo ${TAG:-$BRANCH} | tr / -)
# replace / with -
TAG=${TAG/\//-}

# Happens during CI, we use the TAG from CI
if [ ! -z $MODULES_DIR_EXP ]; then
	TAG="build_$TAG"
fi

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
MODULE_BLACKLIST=${MODULE_BLACKLIST:-"production"}

# Docker Registry to pull/push images
REGISTRY_URL=${REGISTRY_URL:-"ghcr.io/watonomous/wato_monorepo"}

REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')
REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')

## --------------------------- Images -------------------------------
# NOTE: ALL IMAGE NAMES MUCH BE IN THE FORMAT OF <COMPOSE_FILE>_<SERVICE>

# ROS2 C++ Samples
SAMPLES_CPP_AGGREGATOR_IMAGE=${SAMPLES_CPP_AGGREGATOR_IMAGE:-"$REGISTRY_URL/samples/cpp_aggregator"}
SAMPLES_CPP_PRODUCER_IMAGE=${SAMPLES_CPP_PRODUCER_IMAGE:-"$REGISTRY_URL/samples/cpp_producer"}
SAMPLES_CPP_TRANSFORMER_IMAGE=${SAMPLES_CPP_TRANSFORMER_IMAGE:-"$REGISTRY_URL/samples/cpp_transformer"}

# ROS2 Python Samples
SAMPLES_PYTHON_AGGREGATOR_IMAGE=${SAMPLES_PYTHON_AGGREGATOR_IMAGE:-"$REGISTRY_URL/samples/py_aggregator"}
SAMPLES_PYTHON_PRODUCER_IMAGE=${SAMPLES_PYTHON_PRODUCER_IMAGE:-"$REGISTRY_URL/samples/py_producer"}
SAMPLES_PYTHON_TRANSFORMER_IMAGE=${SAMPLES_PYTHON_TRANSFORMER_IMAGE:-"$REGISTRY_URL/samples/py_transformer"}

# Infrastructure
INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE=${INFRASTRUCTURE_VIS_TOOLS_VNC_IMAGE:-"$REGISTRY_URL/infrastructure/vis_tools_vnc"}
INFRASTRUCTURE_DATA_STREAM_IMAGE=${DATA_STREAM_IMAGE:-"$REGISTRY_URL/infrastructure/data_stream"}
INFRASTRUCTURE_FOXGLOVE_IMAGE=${DATA_STREAM_IMAGE:-"$REGISTRY_URL/infrastructure/foxglove"}

# Perception
PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE=${PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/radar_object_detection"}
PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE=${PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/lidar_object_detection"}
PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE=${PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/camera_object_detection"}
PERCEPTION_TRAFFIC_LIGHT_DETECTION_IMAGE=${PERCEPTION_TRAFFIC_LIGHT_DETECTION_IMAGE:-"$REGISTRY_URL/perception/traffic_light_detection"}
PERCEPTION_LANE_DETECTION_IMAGE=${PERCEPTION_LANE_DETECTION_IMAGE:-"$REGISTRY_URL/perception/lane_detection"}
PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE=${PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE:-"$REGISTRY_URL/perception/semantic_segmentation"}
PERCEPTION_TRACKING_IMAGE=${PERCEPTION_TRACKING_IMAGE:-"$REGISTRY_URL/perception/tracking"}

# World Modeling
WORLD_MODELING_HD_MAP_IMAGE=${WORLD_MODELING_HD_MAP_IMAGE:-"$REGISTRY_URL/world_modeling/hd_map"}
WORLD_MODELING_OCCUPANCY_IMAGE=${WORLD_MODELING_OCCUPANCY_IMAGE:-"$REGISTRY_URL/world_modeling/occupancy"}
WORLD_MODELING_OCCUPANCY_SEGMENTATION_IMAGE=${WORLD_MODELING_OCCUPANCY_SEGMENTATION_IMAGE:-"$REGISTRY_URL/world_modeling/occupancy_segmentation"}
WORLD_MODELING_MOTION_FORECASTING_IMAGE=${WORLD_MODELING_MOTION_FORECASTING_IMAGE:-"$REGISTRY_URL/world_modeling/motion_forecasting"}
WORLD_MODELING_LOCALIZATION_IMAGE=${WORLD_MODELING_LOCALIZATION_IMAGE:-"$REGISTRY_URL/world_modeling/localization"}

# Action
ACTION_GLOBAL_PLANNING_IMAGE=${ACTION_GLOBAL_PLANNING_IMAGE:-"$REGISTRY_URL/action/global_planning"}
ACTION_BEHAVIOUR_PLANNING_IMAGE=${ACTION_BEHAVIOUR_PLANNING_IMAGE:-"$REGISTRY_URL/action/behaviour_planning"}
ACTION_LOCAL_PLANNING_IMAGE=${ACTION_LOCAL_PLANNING_IMAGE:-"$REGISTRY_URL/action/local_planning"}
ACTION_MPC_IMAGE=${ACTION_MPC_IMAGE:-"$REGISTRY_URL/action/model_predictive_control"}

# Simulation
SIMULATION_CARLA_IMAGE=${SIMULATION_CARLA_IMAGE:-"$REGISTRY_URL/simulation/carla_sim"}

# Interfacing
INTERFACING_CAN_IMAGE=${INTERFACING_CAN_IMAGE:-"$REGISTRY_URL/interfacing/can_interfacing"}
INTERFACING_SENSOR_IMAGE=${INTERFACING_SENSOR_IMAGE:-"$REGISTRY_URL/interfacing/sensor_interfacing"}

## -------------------------- User ID -----------------------------

SETUID=$(id -u) 
SETGID=$(id -g) 

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((GUI_TOOLS_VNC_PORT++))}

## -------------------- Export Environment Variables -------------------------

# General
echo "$MODULES_DIR"
echo "# Auto-generated by ${BASH_SOURCE[0]}. Edit at own risk." > "$MODULES_DIR/.env"

echo "MODULES_DIR=$MODULES_DIR" >> "$MODULES_DIR/.env"
echo "MONO_DIR=$MONO_DIR" >> "$MODULES_DIR/.env"

echo "ACTIVE_MODULES=\"$ACTIVE_MODULES\"" >> "$MODULES_DIR/.env"
echo "MODULE_BLACKLIST=\"$MODULE_BLACKLIST\"" >> "$MODULES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$MODULES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$MODULES_DIR/.env"

echo "ROS_IP=$ROS_IP" >> "$MODULES_DIR/.env"
echo "ROS_HOSTNAME=$ROS_HOSTNAME" >> "$MODULES_DIR/.env"

echo "TAG=$TAG" >> "$MODULES_DIR/.env"

echo "SETUID=$SETUID" >> "$MODULES_DIR/.env"
echo "SETGID=$SETGID" >> "$MODULES_DIR/.env"

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
echo "PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE=$PERCEPTION_RADAR_OBJECT_DETECTION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE=$PERCEPTION_LIDAR_OBJECT_DETECTION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE=$PERCEPTION_CAMERA_OBJECT_DETECTION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_TRAFFIC_LIGHT_DETECTION_IMAGE=$PERCEPTION_TRAFFIC_LIGHT_DETECTION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_LANE_DETECTION_IMAGE=$PERCEPTION_LANE_DETECTION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE=$PERCEPTION_SEMANTIC_SEGMENTATION_IMAGE" >> "$MODULES_DIR/.env"
echo "PERCEPTION_TRACKING_IMAGE=$PERCEPTION_TRACKING_IMAGE" >> "$MODULES_DIR/.env"

# World Modeling
echo "WORLD_MODELING_HD_MAP_IMAGE=$WORLD_MODELING_HD_MAP_IMAGE" >> "$MODULES_DIR/.env"
echo "WORLD_MODELING_OCCUPANCY_IMAGE=$WORLD_MODELING_OCCUPANCY_IMAGE" >> "$MODULES_DIR/.env"
echo "WORLD_MODELING_OCCUPANCY_SEGMENTATION_IMAGE=$WORLD_MODELING_OCCUPANCY_SEGMENTATION_IMAGE" >> "$MODULES_DIR/.env"
echo "WORLD_MODELING_MOTION_FORECASTING_IMAGE=$WORLD_MODELING_MOTION_FORECASTING_IMAGE" >> "$MODULES_DIR/.env"
echo "WORLD_MODELING_LOCALIZATION_IMAGE=$WORLD_MODELING_LOCALIZATION_IMAGE" >> "$MODULES_DIR/.env"

# Action
echo "ACTION_GLOBAL_PLANNING_IMAGE=$ACTION_GLOBAL_PLANNING_IMAGE" >> "$MODULES_DIR/.env"
echo "ACTION_BEHAVIOUR_PLANNING_IMAGE=$ACTION_BEHAVIOUR_PLANNING_IMAGE" >> "$MODULES_DIR/.env"
echo "ACTION_LOCAL_PLANNING_IMAGE=$ACTION_LOCAL_PLANNING_IMAGE" >> "$MODULES_DIR/.env"
echo "ACTION_MPC_IMAGE=$ACTION_MPC_IMAGE" >> "$MODULES_DIR/.env"

# Simulation
echo "SIMULATION_CARLA_IMAGE=$SIMULATION_CARLA_IMAGE" >> "$MODULES_DIR/.env"

# Interfacing
echo "INTERFACING_CAN_IMAGE=$INTERFACING_CAN_IMAGE" >> "$MODULES_DIR/.env"
echo "INTERFACING_SENSOR_IMAGE=$INTERFACING_SENSOR_IMAGE" >> "$MODULES_DIR/.env"
