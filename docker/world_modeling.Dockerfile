# syntax=docker/dockerfile:1.4

ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/world_modeling world_modeling

COPY src/wato_msgs/common_msgs common_msgs
COPY src/wato_msgs/interfacing_msgs interfacing_msgs
COPY src/wato_msgs/world_modeling_msgs world_modeling_msgs
COPY src/wato_test wato_test

RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0 carla_msgs

# Update CONTRIBUTING.md to pass copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/carla_msgs/CONTRIBUTING.md

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Download maps
ENV MAPS_DIR="${AMENT_WS}/etc/maps/"
RUN apt-get update && \
    git clone https://github.com/WATonomous/map_data.git --depth 1 "${MAPS_DIR}" && \
    chmod -R 755 "${MAPS_DIR}" && \
    rm -rf /var/lib/apt/lists/*
