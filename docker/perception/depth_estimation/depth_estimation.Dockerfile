ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} as source

WORKDIR ${AMENT_WS}/src

# Install DepthAnythingV2
RUN git clone --depth 1 --branch main https://github.com/DepthAnything/Depth-Anything-V2 && \
    cd Depth-Anything-V2 && \
    git checkout e5a2732d3ea2cddc081d7bfd708fc0bf09f812f1

# Copy in source code 
COPY src/perception/depth_estimation depth_estimation

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

# Add pip 
RUN echo " python3-pip" >> /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# pip complains about blinker partial uninstall
RUN sudo apt remove python3-blinker -y

# Install dependencies from requirements.txt
WORKDIR ${AMENT_WS}/src/depth_estimation
COPY src/perception/depth_estimation/requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

#! faster build: required packages already included in depth_estimation/requirements.txt
# Install dependencies from DepthAnythingV2
#WORKDIR ${AMENT_WS}/src/Depth-Anything-V2
#RUN pip3 install -r requirements.txt
ENV PYTHONPATH="${PYTHONPATH}:/home/bolty/ament_ws/src/Depth-Anything-V2"

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
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as deploy

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
