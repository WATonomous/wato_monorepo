ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM ros:foxy-ros-core AS api

ARG CARLA_VERSION
RUN apt-get update && \
      apt-get install -y lsb-release \
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
      pip install simple-pid && \
      pip install transforms3d && \
      pip install pexpect && \
      pip install networkx

# fix user permissions when deving in container
COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
RUN chmod +x /project/fixuid_setup.sh
RUN /project/fixuid_setup.sh
USER docker:docker

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================

FROM api as base
# Set up workspace
RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

RUN git clone --depth 1 --branch master https://github.com/carla-simulator/ros-bridge.git
WORKDIR /home/docker/ament_ws/src/ros-bridge
RUN git submodule update --init --recursive
WORKDIR /home/docker/ament_ws/src

# Download rviz 
RUN git clone --branch foxy https://github.com/ros2/rviz.git
WORKDIR /home/docker/ament_ws/src/ros-bridge
RUN git submodule update --init --recursive
WORKDIR /home/docker/ament_ws/src


# Fix an error in the ackermann_control node 
RUN sed -i s/simple_pid.PID/simple_pid.pid/g /home/docker/ament_ws/src/ros-bridge/carla_ackermann_control/src/carla_ackermann_control/carla_ackermann_control_node.py

FROM base as repo
COPY src/wato_msgs ros_msgs

WORKDIR /home/docker/ament_ws

RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list || true && \
      . /opt/ros/$ROS_DISTRO/setup.sh && \
      sudo rosdep init && \
      sudo apt-get update && \
      rosdep update && \
      rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
      colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN rm -rf build

WORKDIR /home/docker/ament_ws/src

COPY src/simulation/carla_config carla_config

WORKDIR /home/docker/ament_ws

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-select carla_config"

RUN rm -rf build

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]
CMD ["ros2", "launch", "carla_config", "carla.launch.py"]