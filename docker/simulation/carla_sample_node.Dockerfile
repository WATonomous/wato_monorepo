
# ================= Dependencies ===================
FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

# fix user permissions when deving in container
COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
RUN chmod +x /project/fixuid_setup.sh
RUN /project/fixuid_setup.sh
USER docker:docker

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# Add any custom messages here
COPY src/wato_msgs/simulation/common_msgs common_msgs
COPY src/wato_msgs/simulation/embedded_msgs embedded_msgs
COPY src/wato_msgs/simulation/path_planning_msgs path_planning_msgs

# Carla specific messages
RUN git clone https://github.com/ros-drivers/ackermann_msgs.git --branch ros2 && \
    git clone https://github.com/ros-perception/image_common.git --branch $ROS_DISTRO && \
    git clone https://github.com/carla-simulator/ros-carla-msgs.git --branch master

RUN sudo apt-get -y update && sudo apt-get install -y python3-pip
RUN pip3 install setuptools==58.2.0

COPY src/simulation/carla_sample_node carla_sample_node

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]
# CMD ["ros2", "launch", "carla_sample_node", "carla_sample_node.launch.py"]
# CMD ["tail", "-f", "/dev/null"]