# ================= Dependencies ===================
FROM leungjch/cuda122-ubuntu2204-tensorrt-base:latest as base

# ADD DEPENDENCIES HERE

# RUN apt-get update && apt-get install -y curl && \
#     rm -rf /var/lib/apt/lists/*

# # Add a docker user so that created files in the docker container are owned by a non-root user
# RUN addgroup --gid 1000 docker && \
#     adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
#     echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# USER docker:docker
# fix user permissions when deving in container
# COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
# RUN /project/fixuid_setup.sh

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

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