#!/bin/bash
ACTIVE_PROFILES="python_samples"

# This script generates a .env file to be used with docker-compose
# To override any of the variables in this script, create dev_config.local.sh 
#   in the same directory and populate it with variables
#   e.g. `COMPOSE_PROJECT_NAME=<NAME>`.

if [ -f /.dockerenv ]; then
	echo "Please run this in the host machine (not in the Docker container)"
	exit 1
fi

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

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

# Additional tag to cache from
CACHE_FROM_TAG=${CACHE_FROM_TAG:-$([[ $TAG != "develop" ]] && echo $TAG || echo "__develop_ignore_tag_cache__")}

# Which versions of the carla to use. This variable is used to ensure that the carla server, carla python API that is built, and carla ros bridge version that is cloned are all compatible
CARLA_VERSION=${CARLA_VERSION:-"0.9.10.1"}
CARLA_QUALITY=${CARLA_QUALITY:-"Low"}

# ROS hostname + URI for the production profile
# ROS_IP is the IP that nodes will publish as (client's hostname)
ROS_HOSTNAME=${ROS_HOSTNAME:-"localhost"}
ROS_IP=${ROS_IP:-"127.0.0.1"}

# REMOVE============================================================
# URI is the IP at which the rosmaster is being hosted
ROS_MASTER_URI=${ROS_MASTER_URI:-"https://localhost:11311"}

## ----------------- Profile Configuration --------------------

# List of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - carla         		:   if enabled, starts the carla simulation tools
#   - tools         		:   starts any debug and visualization tools
#   - matlab        		:   starts the matlab container
#   - perception    		:   starts the all perception modules
#   - path_planning 		:   starts the all path planning modules
#   - hd_maps_processing 	: 	starts container for creating/processing hd maps
#   - production    		:   configs for all containers required in production
# 	- sensors				: 	starts all sensor drivers
#   - can_interface    		:   starts the ROS drivers for CAN

ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}

# List of profiles to IGNORE when using the --all flag
PROFILE_BLACKLIST=${PROFILE_BLACKLIST:-"production"}

## --------------------------- Images -------------------------------

# GUI Tools default configs
GUI_TOOLS_IMAGE=${GUI_TOOLS_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/gui_tools:$TARGET_STAGE"}

# Image that runs the 3rd party and custom CARLA ros bridges
CARLA_ROS_BRIDGE_IMAGE=${CARLA_ROS_BRIDGE_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/carla_ros_bridge:$TARGET_STAGE"}
# Same as official carla server image but with WATOs custom maps imported
CARLA_SERVER_IMAGE=${CARLA_SERVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/carla_server"}

# Env Model default configs
ENV_MODEL_IMAGE=${ENV_MODEL_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/pp_env_model:$TARGET_STAGE"}
# Global mapping default configs
GLOBAL_MAPPING_IMAGE=${GLOBAL_MAPPING_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/pp_global_mapping:$TARGET_STAGE"}
# Occupancy default configs
OCCUPANCY_IMAGE=${OCCUPANCY_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/occupancy:$TARGET_STAGE"}
# Local Planning
LOCAL_PLANNING_IMAGE=${LOCAL_PLANNING_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/local_planning:$TARGET_STAGE"}
# Ego Localization
EGO_LOCALIZATION_IMAGE=${EGO_LOCALIZATION_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/ego_localization:$TARGET_STAGE"}
# Lidar Odometry
LIDAR_ODOMETRY_IMAGE=${LIDAR_ODOMETRY_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/lidar_odometry:$TARGET_STAGE"}
# HD Maps
HD_MAPS_PROCESSING_IMAGE=${HD_MAPS_PROCESSING_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/hd_maps_processing:$TARGET_STAGE"}

# Object Tracker
OBJECT_TRACKING_IMAGE=${OBJECT_TRACKING_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/object_tracking:$TARGET_STAGE"}
# Perception
PERCEPTION_IMAGE=${PERCEPTION_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/perception:$TARGET_STAGE"}
# Object Detection
CAMERA_DETECTION_IMAGE=${CAMERA_DETECTION_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/camera_detection:$TARGET_STAGE"}
ACTION_IMAGE=${ACTION_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/action_classification"}

# Camera Driver
CAMERA_DRIVER_IMAGE=${CAMERA_DRIVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/camera_driver"}
# Bluelight Driver
WATO_BLUELIGHT_IMAGE=${WATO_BLUELIGHT_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/bluelight_driver"}
# Lidar Driver
LIDAR_DRIVER_IMAGE=${LIDAR_DRIVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/lidar_driver"}
# Radar Driver
RADAR_DRIVER_IMAGE=${RADAR_DRIVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/radar_driver"}
# GNSS/IMU Driver
GNSS_IMU_DRIVER_IMAGE=${GNSS_IMU_DRIVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/gnss_imu_driver"}
# CAN Interface
CAN_INTERFACE_IMAGE=${CAN_INTERFACE_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/can_interface"}

#ROS2 Python Samples
SAMPLES_PYTHON_AGGREGATOR_IMAGE=${SAMPLES_PYTHON_AGGREGATOR_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_aggregator"}
SAMPLES_PYTHON_PRODUCER_IMAGE=${SAMPLES_PYTHON_PRODUCER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_producer"}
SAMPLES_PYTHON_TRANSFORMER_IMAGE=${SAMPLES_PYTHON_TRANSFORMER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_transformer"}


## -------------------------- User ID -----------------------------

FIXUID=$(id -u) 
FIXGID=$(id -g) 

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}
MATLAB_VNC_PORT=${MATLAB_VNC_PORT:-$((BASE_PORT++))}

## -------------------- Environment Variables -------------------------

echo "# Auto-generated by ${BASH_SOURCE[0]}. Please do not edit." > "$SCRIPT_DIR/.env"
echo "ACTIVE_PROFILES=\"$ACTIVE_PROFILES\"" > "$SCRIPT_DIR/.env"
echo "PROFILE_BLACKLIST=\"$PROFILE_BLACKLIST\"" >> "$SCRIPT_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$SCRIPT_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$SCRIPT_DIR/.env"

echo "CARLA_VERSION=$CARLA_VERSION" >> "$SCRIPT_DIR/.env"
echo "CARLA_QUALITY=$CARLA_QUALITY" >> "$SCRIPT_DIR/.env"
echo "CODE_SERVER_PASS=$CODE_SERVER_PASS" >> "$SCRIPT_DIR/.env"

echo "ROS_IP=$ROS_IP" >> "$SCRIPT_DIR/.env"
echo "ROS_HOSTNAME=$ROS_HOSTNAME" >> "$SCRIPT_DIR/.env"
echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> "$SCRIPT_DIR/.env"

echo "ENV_MODEL_IMAGE=$ENV_MODEL_IMAGE" >> "$SCRIPT_DIR/.env"
echo "GLOBAL_MAPPING_IMAGE=$GLOBAL_MAPPING_IMAGE" >> "$SCRIPT_DIR/.env"
echo "OCCUPANCY_IMAGE=$OCCUPANCY_IMAGE" >> "$SCRIPT_DIR/.env"

echo "GUI_TOOLS_IMAGE=$GUI_TOOLS_IMAGE" >> "$SCRIPT_DIR/.env"
echo "CARLA_ROS_BRIDGE_IMAGE=$CARLA_ROS_BRIDGE_IMAGE" >> "$SCRIPT_DIR/.env"
echo "CARLA_SERVER_IMAGE=$CARLA_SERVER_IMAGE" >> "$SCRIPT_DIR/.env"
echo "OBJECT_TRACKING_IMAGE=$OBJECT_TRACKING_IMAGE" >> "$SCRIPT_DIR/.env"
echo "PERCEPTION_IMAGE=$PERCEPTION_IMAGE" >> "$SCRIPT_DIR/.env"
echo "CAMERA_DETECTION_IMAGE=$CAMERA_DETECTION_IMAGE" >> "$SCRIPT_DIR/.env"
echo "ACTION_IMAGE=$ACTION_IMAGE" >> "$SCRIPT_DIR/.env"
echo "CAMERA_DRIVER_IMAGE=$CAMERA_DRIVER_IMAGE" >> "$SCRIPT_DIR/.env"
echo "LOCAL_PLANNING_IMAGE=$LOCAL_PLANNING_IMAGE" >> "$SCRIPT_DIR/.env"
echo "WATO_BLUELIGHT_IMAGE=$WATO_BLUELIGHT_IMAGE" >> "$SCRIPT_DIR/.env"
echo "LIDAR_DRIVER_IMAGE=$LIDAR_DRIVER_IMAGE" >> "$SCRIPT_DIR/.env"
echo "RADAR_DRIVER_IMAGE=$RADAR_DRIVER_IMAGE" >> "$SCRIPT_DIR/.env"
echo "EGO_LOCALIZATION_IMAGE=$EGO_LOCALIZATION_IMAGE" >> "$SCRIPT_DIR/.env"
echo "LIDAR_ODOMETRY_IMAGE=$LIDAR_ODOMETRY_IMAGE" >> "$SCRIPT_DIR/.env"
echo "GNSS_IMU_DRIVER_IMAGE=$GNSS_IMU_DRIVER_IMAGE" >> "$SCRIPT_DIR/.env"
echo "CAN_INTERFACE_IMAGE=$CAN_INTERFACE_IMAGE" >> "$SCRIPT_DIR/.env"
echo "HD_MAPS_PROCESSING_IMAGE=$HD_MAPS_PROCESSING_IMAGE" >> "$SCRIPT_DIR/.env"

echo "TAG=$TAG" >> "$SCRIPT_DIR/.env"
echo "TARGET_STAGE=$TARGET_STAGE" >> "$SCRIPT_DIR/.env"
echo "CACHE_FROM_TAG=$CACHE_FROM_TAG" >> "$SCRIPT_DIR/.env"

echo "FIXUID=$FIXUID" >> "$SCRIPT_DIR/.env"
echo "FIXGID=$FIXGID" >> "$SCRIPT_DIR/.env"

echo "BASE_PORT=$BASE_PORT" >> "$SCRIPT_DIR/.env"
echo "GUI_TOOLS_VNC_PORT=$GUI_TOOLS_VNC_PORT" >> "$SCRIPT_DIR/.env"
echo "MATLAB_VNC_PORT=$MATLAB_VNC_PORT" >> "$SCRIPT_DIR/.env"
cat .env
