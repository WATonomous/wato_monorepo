ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04-devel
ARG RUNTIME_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} as source
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/perception/lane_detection                                lane_detection
COPY src/wato_msgs/perception_msgs/lane_detection_msgs            lane_detection_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | grep 'apt-get install' \
      | awk '{print $3}' \
      | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libopencv-dev python3-opencv tensorrt cuda-toolkit && \
    rm -rf /var/lib/apt/lists/*

ENV OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean  && \
    apt-get -qq clean      && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build ROS 2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM ${RUNTIME_IMAGE} as deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Runtime libs
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-cv-bridge tensorrt && \
    rm -rf /var/lib/apt/lists/*

# Copy the built workspace
COPY --from=build ${AMENT_WS} ${AMENT_WS}

WORKDIR ${AMENT_WS}
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
