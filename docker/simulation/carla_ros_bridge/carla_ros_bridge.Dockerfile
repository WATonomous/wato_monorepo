ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:foxy-ubuntu20.04

ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/simulation/carla_config carla_config
COPY src/wato_msgs/simulation ros_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update --rosdistro foxy && \
    rosdep install --from-paths . -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build

ARG CARLA_VERSION

# Install dependencies
RUN apt-get update && \
      apt-fast install -qq -y --no-install-recommends lsb-release \
      libglu1-mesa-dev xorg-dev \
      software-properties-common \
      build-essential \
      wget \
      curl \
      python3-rosdep \
      python3-rospkg \
      git \
      python3-colcon-common-extensions \
      python3-pygame \
      ros-$ROS_DISTRO-tf2-geometry-msgs \
      ros-$ROS_DISTRO-tf2-eigen \
      ros-$ROS_DISTRO-ackermann-msgs \
      ros-$ROS_DISTRO-derived-object-msgs \
      ros-$ROS_DISTRO-cv-bridge \
      ros-$ROS_DISTRO-vision-opencv \
      ros-$ROS_DISTRO-rqt-image-view \
      ros-$ROS_DISTRO-rqt-gui-py \
      qt5-default \
      ros-$ROS_DISTRO-pcl-conversions \
      ros-$ROS_DISTRO-resource-retriever \
      ros-$ROS_DISTRO-yaml-cpp-vendor \
      ros-$ROS_DISTRO-urdf \
      ros-$ROS_DISTRO-map-msgs \
      ros-$ROS_DISTRO-laser-geometry \
      ros-$ROS_DISTRO-interactive-markers

#Install Python Carla API
COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla /opt/carla/PythonAPI
WORKDIR /opt/carla/PythonAPI
RUN python3.8 -m easy_install pip && \
      pip3 install carla==${CARLA_VERSION} && \
      pip install simple-pid==2.0.0 && \
      pip install transforms3d==0.4.1 && \
      pip install pexpect==4.9.0 && \
      pip install networkx==3.1

WORKDIR ${AMENT_WS}/src

# Download ROS Bridge
RUN git clone --depth 1 --branch master --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# Download rviz 
RUN git clone --branch foxy --recurse-submodules https://github.com/ros2/rviz.git

# Fix an error in the ackermann_control node 
RUN sed -i s/simple_pid.PID/simple_pid.pid/g ./ros-bridge/carla_ackermann_control/src/carla_ackermann_control/carla_ackermann_control_node.py



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
