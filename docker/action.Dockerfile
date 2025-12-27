ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Clone in Carla-specific items
RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0 carla_msgs
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/carla_msgs/CONTRIBUTING.md

# Copy in source code
COPY src/action action
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# No module-specific dependencies for action
