ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for tracking_2d build
COPY src/perception/tracking_2d tracking_2d
COPY src/wato_msgs wato_msgs

# Prepare external dependencies
RUN git clone --depth 1 --branch main https://github.com/Vertical-Beach/ByteTrack-cpp.git /opt/ByteTrack-cpp

# Update CONTRIBUTING.md to pass ament_copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Install ByteTrack C++ library for native ROS wrappers (requires cmake/build tools from rosdep)
COPY --from=source /opt/ByteTrack-cpp /opt/ByteTrack-cpp
RUN cmake -S /opt/ByteTrack-cpp -B /opt/ByteTrack-cpp/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /opt/ByteTrack-cpp/build --config Release && \
    install -Dm755 /opt/ByteTrack-cpp/build/libbytetrack.so /usr/local/lib/libbytetrack.so && \
    install -d /usr/local/include/ByteTrack && \
    cp -r /opt/ByteTrack-cpp/include/ByteTrack/. /usr/local/include/ByteTrack/ && \
    ldconfig

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
