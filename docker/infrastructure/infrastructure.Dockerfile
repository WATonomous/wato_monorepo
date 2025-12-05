ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/wato_msgs wato_msgs
COPY src/infrastructure/infrastructure_deps infrastructure_deps
COPY src/interfacing/eve_description eve_description

# Copy in CARLA messages (and its contribution text, test requirement)
RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# Install Rosdep requirements (including all ROS dependencies from infrastructure_deps/package.xml)
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# RMW Configurations
COPY docker/dds_config.xml ${WATONOMOUS_INSTALL}/dds_config.xml
COPY docker/iox_config.toml ${WATONOMOUS_INSTALL}/iox_config.toml

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
