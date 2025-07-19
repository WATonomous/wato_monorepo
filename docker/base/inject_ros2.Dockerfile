ARG GENERIC_IMAGE

########################## Install ROS2 Core ##########################
FROM ${GENERIC_IMAGE} as core

# all RUN commands use bash + pipefail (DL4006)
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install helper packages
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
        dirmngr gnupg2 software-properties-common curl && \
    rm -rf /var/lib/apt/lists/*

# setup ROS 2 key and repo
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    printf "deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu %s main\n" \
        "$(dpkg --print-architecture)" \
        "$(source /etc/os-release && echo "$UBUNTU_CODENAME")" \
      > /etc/apt/sources.list.d/ros2.list

# setup environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# install ros-core
RUN apt-get update && \
    apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ros-core" && \
    rm -rf /var/lib/apt/lists*


######### Install ROS2 Developer Tools (rosdep, colcon, vcstools) #########
FROM core as devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# bootstrap tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential git \
        python3-colcon-common-extensions python3-colcon-mixin \
        python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# initialise rosdep
RUN rosdep init && rosdep update --rosdistro "${ROS_DISTRO}"

# add colcon mixins / metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros-base
RUN apt-get update && \
    apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ros-base" && \
    rm -rf /var/lib/apt/lists/*
