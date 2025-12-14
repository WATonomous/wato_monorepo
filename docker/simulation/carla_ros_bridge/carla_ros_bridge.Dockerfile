ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

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
COPY src/wato_msgs/interfacing_msgs interfacing_msgs

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths . --ignore-src -r -s > /tmp/rosdep_output && \
    (grep 'apt-get install' /tmp/rosdep_output || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list && \
    (grep 'pip3 install' /tmp/rosdep_output || true) \
        | sed 's/.*pip3 install //' \
        | sort  > /tmp/colcon_pip_install_list

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
        python3-pygame \
        qt5-default && \
    rm -rf /var/lib/apt/lists/*

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
COPY --from=source /tmp/colcon_pip_install_list /tmp/colcon_pip_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    if [ -s /tmp/colcon_pip_install_list ]; then \
        xargs -a /tmp/colcon_pip_install_list pip3 install --no-cache-dir; \
    fi

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

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# RMW Configurations
COPY docker/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
