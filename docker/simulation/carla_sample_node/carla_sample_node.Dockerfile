ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/simulation/carla_sample_node carla_sample_node
COPY src/wato_msgs/simulation/embedded_msgs embedded_msgs
COPY src/wato_msgs/simulation/path_planning_msgs path_planning_msgs

# Carla specific messages

RUN git clone --depth 1 https://github.com/ros-drivers/ackermann_msgs.git --branch 2.0.2 \
    && git clone --depth 1 https://github.com/ros-perception/image_common.git --branch 3.1.8 \
    && git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# TODO (wato) pipefail is failing here
RUN set +o pipefail && apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install Rosdep requirements
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
    rm -rf /root/* /root/.ros /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
