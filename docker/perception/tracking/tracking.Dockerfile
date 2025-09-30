ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/perception/tracking tracking
COPY src/wato_msgs/sample_msgs sample_msgs
COPY src/wato_msgs/perception_msgs/camera_object_detection_msgs camera_object_detection_msgs

COPY src/wato_msgs/perception_msgs/tracking_msgs tracking_msgs
# Copy in source code TRACKING IS BROKEN AND NEEDS TO BE MERGED INTO THE MAIN PERCEPTION IMAGE
# COPY src/perception/tracking                                             tracking
COPY src/wato_msgs/sample_msgs                                           sample_msgs
COPY src/wato_msgs/perception_msgs/camera_object_detection_msgs          camera_object_detection_msgs

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | (grep 'apt-get install' || true) \
      | awk '{print $3}' \
      | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Base libraries, compiler tool-chain and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip ffmpeg libsm6 libxext6 wget \
      build-essential gcc gfortran libopenblas-dev liblapack-dev && \
    rm -rf /var/lib/apt/lists/*

# Python: scipy first (wheels build OpenBLAS)
RUN python3 -m pip install --no-cache-dir scipy==1.11.4
# Install pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    ffmpeg libsm6 libxext6 wget \
    build-essential \
    gcc \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-ros

# Other Python packages
WORKDIR /tmp
COPY src/perception/tracking/requirements.txt ./requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt && rm requirements.txt

# OpenCV runtime libs (no-recommendations)
RUN apt-get update && \
    apt-get install -y --no-install-recommends libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update
RUN apt-get install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)
RUN apt install ros-$ROS_DISTRO-tf-transformations -y

# install opencv
RUN apt-get install -y libopencv-dev
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch.  Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Source Cleanup and Security Setup
RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
