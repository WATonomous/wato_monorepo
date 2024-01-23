ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04
################################ Source ################################
FROM ${BASE_IMAGE} as source
WORKDIR ${AMENT_WS}/src
# Copy in source code 
COPY src/perception/lidar_object_detection lidar_object_detection
COPY src/wato_msgs/sample_msgs sample_msgs
# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list
################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies
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
################################ INSTALL OpenPCDet ################################
RUN apt update && apt install -y python3-pip
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
RUN pip3 install spconv-cu116
RUN apt update && apt install -y python3-setuptools
WORKDIR /home/bolty/
RUN git clone https://github.com/Kin-Zhang/OpenPCDet.git
RUN cd OpenPCDet && pip3 install -r requirements.txt
RUN pip3 install pyquaternion numpy==1.23 pillow==8.4 mayavi open3d
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
FROM build as deploy

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
