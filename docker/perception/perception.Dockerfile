ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
<<<<<<< HEAD
# COPY src/perception perception
COPY src/perception/patchwork perception/patchwork
COPY src/perception/spatial_association perception/spatial_association
COPY src/perception/perception_bringup perception/perception_bringup
=======
COPY src/perception/perception_bringup perception_bringup
COPY src/perception/patchwork patchwork
>>>>>>> origin/main
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

# Bring in Patchwork++ third-party dependency (built later in dependencies stage)
RUN git clone --depth 1 --branch master \
      https://github.com/url-kaist/patchwork-plusplus \
      perception/patchwork/patchwork-plusplus

# Update CONTRIBUTING.md to pass ament_copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps across the copied sources
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install dependencies for spatial_association
RUN apt-get -qq update && apt-get install -qq -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pcl-conversions \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install rosdep requirements collected from source stage
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Ensure bash with pipefail for RUN commands with pipelines
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build and install Patchwork++
WORKDIR ${AMENT_WS}/src/perception/patchwork/patchwork-plusplus
RUN cmake -S cpp -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    echo /usr/local/lib | tee /etc/ld.so.conf.d/usr-local.conf && ldconfig

# RMW Configurations
COPY docker/dds_config.xml ${WATONOMOUS_INSTALL}/dds_config.xml

# Dependency Cleanup
WORKDIR ${AMENT_WS}
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# RMW Configurations
COPY docker/dds_config.xml ${WATONOMOUS_INSTALL}/dds_config.xml

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
