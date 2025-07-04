ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/action/pure_pursuit_control pure_pursuit_control
COPY src/wato_msgs/world_modeling_msgs world_modeling_msgs

# Clone CARLA message definitions
RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
    | grep 'apt-get install' \
    | awk '{print $3}' \
    | sort > /tmp/colcon_install_list

################################ Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

# Install rosdep dependencies
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Clean up unnecessary cache
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build

# Build C++ ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Set up entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as deploy

# Clean source files for final image
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
