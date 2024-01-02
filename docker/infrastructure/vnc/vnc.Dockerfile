ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies


RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    # Install ROS viz tools
    ros-humble-rqt* \
    ros-humble-rviz2 \
    # misc
    wget curl tmux

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies as build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as prod

RUN apt-get update -y && apt-get install -y wget curl gdb supervisor
EXPOSE 5900
RUN apt-get update && apt-get install -y lxde x11vnc xvfb mesa-utils && apt-get purge -y light-locker

COPY --chown=${USER} docker/infrastructure/vnc/supervisord.conf /etc/supervisor/supervisord.conf
RUN chown -R ${USER}:${USER} /etc/supervisor
RUN chmod 777 /var/log/supervisor/
ENV DISPLAY=:1.0 
CMD ["/usr/bin/supervisord"]

# Switching users, giving ownership only of the ament_ws
USER ${USER}
RUN sudo chown -R $USER:$USER ${AMENT_WS}
