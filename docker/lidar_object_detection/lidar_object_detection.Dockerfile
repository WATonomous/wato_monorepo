FROM nvcr.io/nvidia/tensorrt:23.04-py3 as base

# tensorrt:23.04 uses Ubuntu 20.04 so it is not possible to install humble
# from a package manager. Instead we have to build from source
ARG ROS_PKG=ros_base
ARG ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && apt-get install -y curl vim wget build-essential manpages-dev wget zlib1g software-properties-common git && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get --reinstall install sudo

# Set locale
RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Setup sources
RUN sudo apt install -y software-properties-common
RUN sudo add-apt-repository universe

# Add ROS2 GPG key
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Add repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt update -y
RUN sudo apt upgrade -y

RUN sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools


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

RUN sudo apt-get install -y libopencv-dev

# # install OpenCV with CUDA
# ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
# ARG OPENCV_DEB=OpenCV-4.5.0-amd64.tar.gz
# 
# COPY ./docker/lidar_object_detection//opencv_install.sh opencv_install.sh
# RUN ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB}

RUN RTI_NC_LICENSE_ACCEPTED=yes apt-get install rti-connext-dds-6.0.1


COPY ./docker/lidar_object_detection/ros2_build.sh ros2_build.sh
RUN ./ros2_build.sh

RUN echo "ROS_DISTRO $ROS_DISTRO"
RUN echo "ROS_ROOT   $ROS_ROOT"
RUN ls $ROS_ROOT

# RUN ls /opt/ros/humble/install/setup.bash

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN sudo apt-get install -y --no-install-recommends python3-rosdep && sudo rosdep init

ARG cachebust=1
RUN mkdir -p /home/docker/ament_ws
WORKDIR /home/docker/ament_ws

# Copy repo
COPY ./src/lidar_object_detection lidar_object_detection

# Build project
RUN . /opt/ros/$ROS_DISTRO/install/setup.bash && \
    rosdep update && \
    rosdep install -i --from-path . --rosdistro $ROS_DISTRO -y && \
    colcon build --packages-select lidar_object_detection \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda && \
    source install/setup.bash

RUN ls -la
RUN pwd

# # # setup container entrypoint
# COPY ./docker/lidar_object_detection/ros_entrypoint.sh /ros_entrypoint.sh
# RUN echo 'source` /ros_entrypoint.sh' >> /root/.bashrc
COPY ./docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY ./docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x /home/docker/wato_ros_entrypoint.sh
RUN sudo chmod 777 -R /opt/ros/humble
RUN sudo mkdir -p -m 777 /.ros/log
RUN pwd
RUN ls

# ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]


ENTRYPOINT ["tail", "-f", "/dev/null"]
# CMD ["ros2", "run", "lidar_object_detection", "lidar_object_detection"]