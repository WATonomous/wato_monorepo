# ================= Dependencies ===================
FROM leungjch/cuda122-ubuntu2204-tensorrt-base:latest as base

# ADD DEPENDENCIES HERE

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN sudo apt update

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

COPY src/perception/lane_detection/ lane_detection
COPY src/wato_msgs/sample_msgs sample_msgs

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]
# CMD ["ros2", "launch", "lane_detection", "lane_detection.launch.py"]
CMD ["ros2", "run", "lane_detection", "lane_detection"]