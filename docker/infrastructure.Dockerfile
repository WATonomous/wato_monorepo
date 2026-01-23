ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/simulation/carla_ros_bridge/carla_msgs carla_msgs
COPY src/wato_test wato_test
COPY src/infrastructure/infrastructure_bringup infrastructure_bringup
COPY src/infrastructure/vision_msgs_markers vision_msgs_markers
COPY src/world_modeling/lanelet_msgs lanelet_msgs
COPY src/world_modeling/prediction_msgs prediction_msgs
COPY src/interfacing/eve_description eve_description

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies (non-rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*
