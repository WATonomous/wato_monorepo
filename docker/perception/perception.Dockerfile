ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
COPY src/perception/perception_bringup perception_bringup
COPY src/perception/patchwork patchwork
COPY src/perception/tracking_2d tracking_2d
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test


# Update CONTRIBUTING.md to pass ament_copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps across the copied sources
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths . --ignore-src -r -s > /tmp/rosdep_output && \
    (grep 'apt-get install' /tmp/rosdep_output || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list && \
    (grep 'pip3 install' /tmp/rosdep_output || true) \
        | sed 's/.*pip3 install //' \
        | sort  > /tmp/colcon_pip_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install rosdep requirements collected from source stage
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
COPY --from=source /tmp/colcon_pip_install_list /tmp/colcon_pip_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    if [ -s /tmp/colcon_pip_install_list ]; then \
        xargs -a /tmp/colcon_pip_install_list pip3 install --no-cache-dir; \
    fi

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src
# Ensure source files have correct ownership
RUN chown -R ${USER}:${USER} "${AMENT_WS}/src"

# Ensure bash with pipefail for RUN commands with pipelines
SHELL ["/bin/bash", "-o", "pipefail", "-c"]


# Dependency Cleanup
WORKDIR ${AMENT_WS}
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Ensure workspace directory exists and has correct ownership
RUN mkdir -p "${AMENT_WS}" && \
    chown -R ${USER}:${USER} "${AMENT_WS}"

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}" && \
    chown -R ${USER}:${USER} "${AMENT_WS}"

# RMW Configurations
COPY docker/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
