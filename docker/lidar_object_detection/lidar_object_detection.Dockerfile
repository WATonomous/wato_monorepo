ARG BASE_DIST=ubuntu20.04
ARG CUDA_VERSION=12.0.1
ARG OPENCV_VERSION=4.5.0
ARG ROS_DISTRO=humble
ARG cachebust=1

FROM nvidia/cuda:${CUDA_VERSION}-devel-${BASE_DIST} as perception-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install necessary tools
RUN apt-get update && apt-get install -y \
    locales \
    apt-utils \
    curl \
    gnupg2 \
    lsb-release 

# Set the locale to use UTF-8
RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt install -y software-properties-common
RUN add-apt-repository universe

# Add the ROS2 apt repository
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

# Update the package list
RUN apt-get update 

# Install ROS2 dev tools 
RUN apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# Install packages
RUN python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

# Get ROS2 Code
RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble 
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN apt upgrade -y

RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build ros2_humble

RUN colcon build --symlink-install

#####################

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# Add a docker user so we that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \                                                                     
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
    chown root:root /usr/local/bin/fixuid && \                                                                              
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \                                                                                               
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

USER docker:docker

RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM perception-base as repo

COPY src/lidar_object_detection lidar_object_detection
COPY src/wato_msgs/common_msgs common_msgs
COPY pointpillar.onnx pointpillar.onnx
WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ARG cachebust=0
RUN sudo apt install tree
RUN tree /home/docker/ament_ws

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "run", "lidar_object_detection", "lidar_object_detection"]
# Run with BUILDKIT_PROGRESS=plain ./watod2 up --build