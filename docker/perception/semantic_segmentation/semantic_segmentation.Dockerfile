ARG BASE_BUILD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.0-humble-ubuntu22.04-devel
ARG BASE_PROD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.0-humble-ubuntu22.04
ARG PADDLE_INFERENCE_BUILD_URL=ghcr.io/watonomous/perception_paddlepaddle_inference_build_cuda-12.0
################################ Build library ################################
FROM ${PADDLE_INFERENCE_BUILD_URL} as PADDLE_INFERENCE_BUILD

################################ Source ################################
FROM ${BASE_BUILD_IMAGE} as source



WORKDIR ${AMENT_WS}/src

# Copy in the paddle inference library
RUN mkdir -p semantic_segmentation/src
COPY --from=PADDLE_INFERENCE_BUILD /paddle/paddle_inference_cuda120_build.tar semantic_segmentation/src/paddle_inference_cuda120_build.tar
RUN tar -xvf semantic_segmentation/src/paddle_inference_cuda120_build.tar -C semantic_segmentation/src
RUN rm semantic_segmentation/src/paddle_inference_cuda120_build.tar

# Copy in source code 
COPY src/perception/semantic_segmentation semantic_segmentation
COPY src/wato_msgs/sample_msgs sample_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_BUILD_IMAGE} as dependencies

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

# install tensorrt
RUN apt update && apt install -y tensorrt

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh

# Add /home/bolty/ament_ws/install/semantic_segmentation/lib/semantic_segmentation to LD_LIBRARY_PATH
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${AMENT_WS}/install/semantic_segmentation/lib/
# /home/bolty/ament_ws/install/semantic_segmentation/lib/semantic_segmentation/
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

# ################################ Prod ################################
# FROM build as deploy

# # Source Cleanup and Security Setup
# RUN chown -R $USER:$USER ${AMENT_WS}
# RUN rm -rf src/*

# USER ${USER}
