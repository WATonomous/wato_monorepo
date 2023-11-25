# ================= Dependencies ===================
FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    # Install ROS viz tools
    ros-humble-rqt* \
    ros-humble-rviz2 \
    # misc
    wget curl tmux

# fix user permissions when deving in container
COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
RUN /project/fixuid_setup.sh
USER docker:docker

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

COPY src/wato_msgs/sample_msgs sample_msgs
# If needed, you can copy over rviz configs here

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

# ================= Develop ===================
FROM repo as debug

USER root:root

RUN apt-get update -y && apt-get install -y wget curl gdb supervisor
EXPOSE 5900
RUN apt-get update && apt-get install -y lxde x11vnc xvfb mesa-utils && apt-get purge -y light-locker

COPY --chown=docker docker/infrastructure/vis_tools/supervisord.conf /etc/supervisor/supervisord.conf
RUN chown -R docker:docker /etc/supervisor
RUN chmod 777 /var/log/supervisor/
ENV DISPLAY=:1.0 
CMD ["/usr/bin/supervisord"]