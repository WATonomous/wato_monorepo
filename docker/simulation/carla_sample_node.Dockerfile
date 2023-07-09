# ================= Dependencies ===================
FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl ros-humble-ros2bag ros-humble-rosbag2* ros-humble-foxglove-msgs&& \
    rm -rf /var/lib/apt/lists/*

# Add a docker user so that created files in the docker container are owned by a non-root user
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

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# Add any custom messages here for foxglove to interpret them
COPY src/wato_msgs/sample_msgs sample_msgs
COPY src/wato_msgs/common_msgs common_msgs
COPY src/wato_msgs/embedded_msgs embedded_msgs
COPY src/wato_msgs/path_planning_msgs path_planning_msgs

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

# WORKDIR /home/docker/ament_ws/src
# RUN ros2 pkg create --build-type ament_python carla_sample_node

USER root:root
RUN chmod a+w ./src/carla_sample_node/logs
USER docker:docker

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]
# CMD ["ros2", "launch", "carla_sample_node", "carla_sample_node.launch.py"]
# CMD ["tail", "-f", "/dev/null"]