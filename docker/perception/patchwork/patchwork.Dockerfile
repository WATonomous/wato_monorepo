ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

############################## Source ##############################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src
COPY src/perception/patchwork patchwork

############################ Dependencies ##########################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]

# Base libraries and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip ffmpeg libsm6 libxext6 wget \
      libpcl-dev pcl-tools libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# Build and install Patchwork++ from source so headers/libraries are available
WORKDIR /opt
RUN git clone --depth 1 --branch master https://github.com/url-kaist/patchwork-plusplus patchwork-plusplus && \
    cmake -S patchwork-plusplus/cpp -B patchwork-plusplus/build \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build patchwork-plusplus/build -j"$(nproc)" && \
    cmake --install patchwork-plusplus/build \
    && echo /usr/local/lib | tee /etc/ld.so.conf.d/usr-local.conf && ldconfig

# Copy source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Clean up apt caches
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

############################### Build ##############################
FROM dependencies AS build
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]

# Build ROS 2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


# Entrypoint will source ROS2 and workspace
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

############################### Prod ###############################
FROM build AS deploy

RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*
USER ${USER}
