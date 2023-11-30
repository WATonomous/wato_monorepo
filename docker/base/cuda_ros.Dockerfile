ARG UBUNTU_DISTRO
ARG CUDA_VERSION

####################### Nvidia CUDA Base Image #######################
FROM nvidia/cuda:${CUDA_VERSION}-devel-${UBUNTU_DISTRO} as base

# Save Environment Properties
ENV CUDA_VERSION=${CUDA_VERSION}
ENV UBUNTU_DISTRO=${UBUNTU_DISTRO}

########################## Install ROS2 Core ##########################
FROM base as core

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    software-properties-common \
    curl \
    && rm -rf /var/lib/apt/lists/*

# setup keys and sources.list
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) \
    main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# install ros2 core packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

######### Install ROS2 Developer Tools (rosdep, colcon, vcstools, apt-fast) #########
FROM core as devel

# install bootstrap tools
RUN apt-get update && apt-fast install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
