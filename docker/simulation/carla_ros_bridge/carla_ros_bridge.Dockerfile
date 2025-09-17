ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:foxy-ubuntu20.04

ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

RUN git clone --depth 1 --branch master --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
WORKDIR ${AMENT_WS}/src/ros-bridge
RUN git checkout e9063d97ff5a724f76adbb1b852dc71da1dcfeec
WORKDIR ${AMENT_WS}/src

RUN sed -i s/simple_pid.PID/simple_pid.pid/g ./ros-bridge/carla_ackermann_control/src/carla_ackermann_control/carla_ackermann_control_node.py

COPY src/simulation/carla_config carla_config
COPY src/wato_msgs/simulation ros_msgs

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update --rosdistro foxy && \
    rosdep install --from-paths . -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# TODO(wato) this should be in rosdep
RUN apt-get update -qq && \
    apt-fast install -qq -y --no-install-recommends \
        lsb-release \
        libglu1-mesa-dev xorg-dev \
        software-properties-common \
        build-essential \
        python3-rosdep \
        python3-rospkg \
        python3-colcon-common-extensions \
        python3-pygame \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-eigen \
        ros-humble-ackermann-msgs \
        ros-humble-derived-object-msgs \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        ros-humble-rqt-image-view \
        ros-humble-rqt-gui-py \
        qt5-default \
        ros-humble-pcl-conversions \
        ros-humble-resource-retriever \
        ros-humble-yaml-cpp-vendor \
        ros-humble-urdf \
        ros-humble-map-msgs \
        ros-humble-laser-geometry \
        ros-humble-interactive-markers \
        ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

ARG CARLA_VERSION

COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla /opt/carla/PythonAPI
WORKDIR /opt/carla/PythonAPI
RUN cp -r ./agents /usr/local/lib/python3.8/dist-packages/
RUN curl -O https://bootstrap.pypa.io/pip/3.8/get-pip.py && \
    python3.8 get-pip.py "pip<24" && \
    rm get-pip.py && \
    pip3 install --no-cache-dir \
        carla==${CARLA_VERSION} \
        simple-pid==2.0.0 \
        transforms3d==0.4.1 \
        pexpect==4.9.0 \
        networkx==3.1

WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
