ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in CARLA messages (and its contribution text, test requirement)
RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Copy in source code
COPY src/wato_msgs wato_msgs
COPY src/infrastructure/infrastructure_deps infrastructure_deps
COPY src/interfacing/eve_description eve_description

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies (non-rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*
