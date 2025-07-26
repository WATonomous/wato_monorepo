ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# --- Custom WATO ROS Packages ---
COPY src/world_modeling world_modeling

# COPY src/world_modeling/hd_map hd_map
# COPY src/world_modeling/localization localization
# COPY src/world_modeling/world_modeling_bringup world_modeling_bringup

COPY src/wato_msgs/common_msgs common_msgs
COPY src/wato_msgs/world_modeling_msgs world_modeling_msgs

RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0 carla_msgs

# Update CONTRIBUTING.md to pass copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/carla_msgs/CONTRIBUTING.md

# SHELL for pipefail handling
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Generate dependency list
RUN apt-get -qq update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list && \
    rm -rf /var/lib/apt/lists/*

################################ Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

ENV MAPS_DIR="${AMENT_WS}/etc/maps/"

# Download maps and install apt deps
RUN apt-get update && \
    git clone https://github.com/WATonomous/map_data.git --depth 1 "${MAPS_DIR}" && \
    chmod -R 755 "${MAPS_DIR}" && \
    rm -rf /var/lib/apt/lists/*

# Install all rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    apt-fast install -qq -y --no-install-recommends "$(cat /tmp/colcon_install_list)" && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build

WORKDIR ${AMENT_WS}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Production ################################
FROM build as deploy

SHELL ["/bin/bash", "-c"]

# Cleanup
RUN chown -R "$USER:$USER" "${AMENT_WS}" && \
    rm -rf src/*

USER ${USER}
