ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/perception/camera_object_detection           camera_object_detection
COPY src/wato_msgs/perception_msgs/camera_object_detection_msgs  camera_object_detection_msgs

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | (grep 'apt-get install' || true) \
      | awk '{print $3}' \
      | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Base libraries and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip ffmpeg libsm6 libxext6 wget && \
    rm -rf /var/lib/apt/lists/*

# CUDA runtime (libcudart) for TensorRT inference
# Install NVIDIA CUDA keyring, then install cudart runtime and refresh ld cache
RUN set -eux; \
    apt-get update -qq && apt-get install -qq -y --no-install-recommends \
        ca-certificates gnupg wget; \
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb; \
    dpkg -i cuda-keyring_1.1-1_all.deb; \
    rm -f cuda-keyring_1.1-1_all.deb; \
    apt-get update -qq && apt-get install -qq -y --no-install-recommends \
        cuda-cudart-12-6; \
    echo "/usr/local/cuda-12.6/targets/x86_64-linux/lib" > /etc/ld.so.conf.d/cuda-12-6.conf; \
    ldconfig; \
    rm -rf /var/lib/apt/lists/*

# Python packages
WORKDIR /tmp
COPY src/perception/camera_object_detection/requirements.txt ./requirements.txt
RUN python3 -m pip install --no-cache-dir --ignore-installed -r requirements.txt && rm requirements.txt

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Extra runtime ROS package
RUN apt-get update && \
    apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-tf-transformations" && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# RMW Configurations
COPY docker/config/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/config/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/config/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
