ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04-devel
ARG RUNTIME_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04
################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/perception/lane_detection lane_detection
COPY src/wato_msgs/sample_msgs sample_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

RUN apt-get update
# Intall opencv
RUN apt install -y libopencv-dev python3-opencv
RUN export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake

# Install TensorRT
RUN apt install -y tensorrt

# Install cuda toolkit
RUN apt-get install -y cuda-toolkit

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

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
# ENTRYPOINT ["./wato_ros_entrypoint.sh]

################################ Prod ################################
FROM ${RUNTIME_IMAGE} as deploy

# Install runtime libs
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    tensorrt

# Copy the compiled binary to the runtime image
COPY --from=build ${AMENT_WS} ${AMENT_WS}

WORKDIR ${AMENT_WS}

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}

ENTRYPOINT ["./wato_ros_entrypoint.sh"]