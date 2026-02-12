FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-lc"]

# Core build deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates \
    build-essential cmake \
  libboost-all-dev \
    python3 python3-dev python3-pip \
    libeigen3-dev \
    libpcl-dev \
    libceres-dev \
    libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev \
  ros-noetic-catkin \
    ros-noetic-pcl-ros ros-noetic-eigen-conversions \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros \
    ros-noetic-rosbag \
  && pip3 install --no-cache-dir matplotlib==3.7.5 \
  && rm -rf /var/lib/apt/lists/*

# Matplotlib (LI-Init includes matplotlibcpp)
ENV MPLBACKEND=Agg

# Catkin workspace
WORKDIR /catkin_ws/src

# LI-Init repo
# (We clone at build time so the monorepo doesn't vendor GPL code.)
ARG LI_INIT_REPO=https://github.com/hku-mars/lidar_imu_init.git
ARG LI_INIT_REF=main
RUN git clone --depth 1 --branch ${LI_INIT_REF} ${LI_INIT_REPO} lidar_imu_init

# livox_ros_driver is required to build headers even for non-Livox paths
RUN git clone --depth 1 --branch v2.6.0 https://github.com/Livox-SDK/livox_ros_driver.git

WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash \
  && catkin_init_workspace /catkin_ws/src \
  && catkin_make -DCMAKE_BUILD_TYPE=Release

# Tool entry
WORKDIR /tool
COPY run_li_init_from_bag.sh /tool/run_li_init_from_bag.sh
RUN chmod +x /tool/run_li_init_from_bag.sh

ENTRYPOINT ["/tool/run_li_init_from_bag.sh"]
