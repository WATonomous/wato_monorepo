ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Behaviour Tree ROS2
RUN git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git --branch humble behaviortree_ros2
WORKDIR ${AMENT_WS}/src/behaviortree_ros2
RUN git checkout 6c6aa078ee7bc52fec98984bed4964556abf5beb
WORKDIR ${AMENT_WS}/src

# Copy in source code needed for world modeling build
COPY src/world_modeling world_modeling
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/infrastructure/lanelet_markers lanelet_markers
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Download maps (ADD fetches GitHub API to bust cache when repo updates)
ADD https://api.github.com/repos/WATonomous/map_data/git/refs/heads/master /tmp/map_version.json
ENV MAPS_DIR="${WATONOMOUS_INSTALL}/maps/"
RUN apt-get update && \
    git clone https://github.com/WATonomous/map_data.git --depth 1 "${MAPS_DIR}" && \
    chmod -R 755 "${MAPS_DIR}" && \
    rm -rf /var/lib/apt/lists/*
