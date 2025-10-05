ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
# COPY src/perception perception
COPY src/perception/patchwork patchwork
COPY src/wato_msgs wato_msgs

# Include Patchwork++ ROS package sources
COPY src/perception/patchwork patchwork

# Bring in Patchwork++ third-party dependency (built later in dependencies stage)
RUN git clone --depth 1 --branch master \
      https://github.com/url-kaist/patchwork-plusplus \
      patchwork/patchwork-plusplus

# Update CONTRIBUTING.md to pass ament_copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps across the copied sources
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install rosdep requirements collected from source stage
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Build and install Patchwork++ 
WORKDIR ${AMENT_WS}/src/patchwork/patchwork-plusplus
RUN cmake -S cpp -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    echo /usr/local/lib | tee /etc/ld.so.conf.d/usr-local.conf && ldconfig

# Dependency Cleanup
WORKDIR ${AMENT_WS}
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages (now includes Patchwork ROS packages)
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
