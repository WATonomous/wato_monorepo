ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

############################## Source ##############################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src
COPY src/perception/patchwork patchwork

RUN git clone --depth 1 --branch master \
      https://github.com/url-kaist/patchwork-plusplus \
      patchwork/patchwork-plusplus

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list && \
    rm -rf /var/lib/apt/lists/*

############################ Dependencies ##########################
FROM ${BASE_IMAGE} AS dependencies

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Copy source code (including Patchwork++) from source stage
WORKDIR ${AMENT_WS}
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

COPY --from=source ${AMENT_WS}/src src

WORKDIR ${AMENT_WS}/src/patchwork/patchwork-plusplus
RUN cmake -S cpp -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    echo /usr/local/lib | tee /etc/ld.so.conf.d/usr-local.conf && ldconfig

# Clean up package manager caches
WORKDIR ${AMENT_WS}
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

############################### Build ##############################
FROM dependencies AS build

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
