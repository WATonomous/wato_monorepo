ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
# This stage gathers all source code and identifies all dependencies in one go.
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# --- Custom WATO ROS Packages ---
COPY src/world_modeling world_modeling

COPY src/wato_msgs/common_msgs common_msgs
COPY src/wato_msgs/interfacing_msgs interfacing_msgs
COPY src/wato_msgs/world_modeling_msgs world_modeling_msgs
COPY src/wato_test wato_test

RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0 carla_msgs

# Update CONTRIBUTING.md to pass copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/carla_msgs/CONTRIBUTING.md

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list && \
    rm -rf /var/lib/apt/lists/*

################################ Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Download maps and install apt deps
ENV MAPS_DIR="${AMENT_WS}/etc/maps/"
RUN apt-get update && \
    git clone https://github.com/WATonomous/map_data.git --depth 1 "${MAPS_DIR}" && \
    chmod -R 755 "${MAPS_DIR}" && \
    rm -rf /var/lib/apt/lists/*

# Install all rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update \
    && xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
