# Multi-stage ROS2 development container (CPU/GPU compatible)
ARG ROS_DISTRO=humble
ARG NVIDIA_CONTAINER_TAG=12.4.1-cudnn-runtime-ubuntu22.04
ARG TENSORRT_RUNTIME_VERSION=10.7.0.23
ARG TENSORRT_CUDA_VERSION=12.6
ARG BASE_TYPE=gpu

# ===============================================
# CPU Base - Standard ROS2 image
# ===============================================
FROM ros:${ROS_DISTRO}-ros-base AS cpu-base

# ===============================================
# GPU Base - CUDA with manual ROS2 install
# ===============================================
FROM nvidia/cuda:${NVIDIA_CONTAINER_TAG} AS gpu-base
ARG ROS_DISTRO
ARG TENSORRT_RUNTIME_VERSION
ARG TENSORRT_CUDA_VERSION

# Install ROS2 manually since we're not using the ros: base image
# Set non-interactive to avoid geographic area prompts
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    wget \
    && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=interactive

RUN curl -fsSL -o cuda-keyring_1.1-1_all.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
    libnvinfer10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    libnvinfer-plugin10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    libnvonnxparsers10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    && rm cuda-keyring_1.1-1_all.deb

# ===============================================
# Install Common Development Tools in Either Base
# ===============================================
FROM ${BASE_TYPE}-base AS dev-tools
ARG ROS_DISTRO

# Install development tools not in base image
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    openssh-client \
    git \
    curl \
    ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace environment variables (matching other Dockerfiles)
ENV USER="bolty"
ENV AMENT_WS=/home/${USER}/ament_ws
ENV WATONOMOUS_INSTALL=/opt/watonomous

# Setup WATonomous Install Directory
RUN mkdir -p "${WATONOMOUS_INSTALL}"

# ===============================================
# Source - Gather Dependencies
# ===============================================
FROM dev-tools AS source

WORKDIR ${AMENT_WS}/src

# Clone deep_ros repository
RUN git clone --depth 1 --branch brian/tensorrt_ep \
      https://github.com/WATonomous/deep_ros.git deep_ros

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

# ===============================================
# Dependencies - Install Dependencies
# ===============================================
FROM dev-tools AS dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-get install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

# ===============================================
# Build - Build ROS2 packages
# ===============================================
FROM dependencies AS build
ARG ROS_DISTRO

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# RMW Configurations
COPY docker/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

# ===============================================
# Deploy - Final production stage
# ===============================================
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*

# Add User Configuration
ARG USERNAME
ARG USER_GID
ARG USER_UID
ARG ROS_DISTRO

# Cater image to user
SHELL ["/bin/bash", "-c"]
# hadolint ignore=SC2086
RUN set -euo pipefail \
    && EXISTING_GROUP="$(getent group ${USER_GID} | cut -d: -f1 || true)" \
    && if [ -z "${EXISTING_GROUP}" ]; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    elif [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
        groupmod -n ${USERNAME} "${EXISTING_GROUP}"; \
    fi \
    && EXISTING_USER="$(getent passwd ${USER_UID} | cut -d: -f1 || true)" \
    && if [ -z "${EXISTING_USER}" ]; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} --shell /bin/bash; \
    else \
        if [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
            usermod -l ${USERNAME} "${EXISTING_USER}"; \
        fi; \
        usermod -d /home/${USERNAME} -m ${USERNAME}; \
        usermod -g ${USER_GID} -s /bin/bash ${USERNAME}; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(ALL\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && cp /etc/skel/.bashrc /home/$USERNAME/.bashrc \
    && cp /etc/skel/.profile /home/$USERNAME/.profile \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc /home/$USERNAME/.profile \
    && rm -rf /var/lib/apt/lists/*

# Set the default user
USER $USERNAME

# Install Claude Code natively
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -fsSL https://claude.ai/install.sh | bash

# Source ROS in user's bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc
