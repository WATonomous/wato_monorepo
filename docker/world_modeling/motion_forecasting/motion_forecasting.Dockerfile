ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/world_modeling/motion_forecasting motion_forecasting
COPY src/wato_msgs/sample_msgs sample_msgs

# Clone the autoware_common and autoware_msgs repositories
RUN git clone https://github.com/autowarefoundation/autoware_common.git
RUN git clone https://github.com/autowarefoundation/autoware_msgs.git
RUN git clone https://github.com/autowarefoundation/autoware_internal_msgs.git

# Clone the tier4_autoware_msgs repository with sparse checkout for tier4_planning_msgs
RUN git clone --depth 1 --filter=blob:none --sparse https://github.com/tier4/tier4_autoware_msgs.git && \
    cd tier4_autoware_msgs && \
    git sparse-checkout set tier4_planning_msgs tier4_debug_msgs && \
    cd ..

# Clone the specific tier4_autoware_utils directory from autoware.universe
RUN git clone --depth 1 --filter=blob:none --sparse https://github.com/autowarefoundation/autoware.universe.git && \
    cd autoware.universe && \
    git sparse-checkout set common/tier4_autoware_utils && \
    mv common/tier4_autoware_utils ../tier4_autoware_utils && \
    cd .. && rm -rf autoware.universe

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Install glog
RUN apt-get -qq update && apt-get -qq install -y libgoogle-glog-dev

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as deploy

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
