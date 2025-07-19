ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

# every RUN with pipes uses bash + pipefail (DL4006)
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/world_modeling/hd_map                     hd_map
COPY src/wato_msgs/common_msgs                     common_msgs
COPY src/wato_msgs/world_modeling_msgs             world_modeling_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | grep 'apt-get install' \
      | awk '{print $3}' \
      | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Download maps and set permissions
ENV MAPS_DIR="${AMENT_WS}/etc/maps"
RUN apt-get update -qq && \
    git clone --depth 1 https://github.com/WATonomous/map_data.git "${MAPS_DIR}" && \
    chmod -R 755 "${MAPS_DIR}" && \
    rm -rf /var/lib/apt/lists/*

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch.
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
