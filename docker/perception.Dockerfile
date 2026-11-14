ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.8.1-cudnn-runtime-ubuntu24.04

######################## Pinned Isaac ROS Sources ########################
FROM ${BASE_IMAGE} AS isaac_ros_source

WORKDIR /opt/isaac_ros_ws/src

# NITROS includes GXF libraries stored in Git LFS.
RUN apt-get update && \
    apt-get install -y --no-install-recommends git-lfs && \
    rm -rf /var/lib/apt/lists/*

# The lifecycle fork changes the NITROS C++ ABI. Build the camera pipeline's
# types and nodes against it too; release 4.5 binaries expect rclcpp::Node&
# callbacks and cannot be mixed with the fork's NitrosNodeInterfaces callbacks.
RUN R=/tmp/isaac_ros_nitros && \
    git clone https://github.com/WATonomous/isaac_ros_nitros.git $R && \
    git -C $R checkout a60cda942d21172b28a09e4be7ca959d7515e8e1 && \
    git -C $R lfs install && \
    git -C $R lfs pull --include="isaac_ros_nitros/lib/**" && \
    mv $R/isaac_ros_nitros $R/isaac_ros_managed_nitros . && \
    mv $R/isaac_ros_nitros_type/isaac_ros_nitros_image_type \
       $R/isaac_ros_nitros_type/isaac_ros_nitros_camera_info_type \
       $R/isaac_ros_nitros_type/isaac_ros_nitros_tensor_list_type \
       $R/isaac_ros_nitros_type/isaac_ros_nitros_compressed_image_type . && \
    rm -rf $R

# Isaac ROS v4.5-0 sources for the nodes that consume the fork's NITROS types.
# cvcuda_utils is built here too: NVIDIA's apt build depends on the stock
# NITROS type packages, which would reinstall duplicates of the fork.
RUN R=/tmp/isaac_ros_image_pipeline && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git $R && \
    git -C $R checkout 203e41be2a4a96382fe69abac08c0cec76720a7a && \
    mv $R/isaac_ros_image_proc $R/isaac_ros_cvcuda_utils . && \
    rm -rf $R

RUN R=/tmp/isaac_ros_compression && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_compression.git $R && \
    git -C $R checkout 818227579e3af2a448c542fdf453d0a871b0d213 && \
    mkdir isaac_ros_compression && \
    mv $R/isaac_ros_h264_encoder $R/codec isaac_ros_compression/ && \
    rm -rf $R

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Use bash with pipefail so piped RUN commands (e.g. curl | gpg) fail loudly.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install TensorRT dependencies for deep_ros (TRT 10) and bevfusion (TRT 11)
ARG TRT10_VERSION=10.16.1.11-1+cuda12.9
ARG TRT11_VERSION=11.4.0.106-1+cuda13.4
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libnvinfer10=${TRT10_VERSION} \
    libnvinfer-plugin10=${TRT10_VERSION} \
    libnvonnxparsers10=${TRT10_VERSION} \
    libnvinfer-dev=${TRT11_VERSION} \
    libnvinfer-plugin-dev=${TRT11_VERSION} \
    libnvonnxparsers-dev=${TRT11_VERSION} \
    protobuf-compiler \
    libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/*

# Binary dependencies of the Isaac ROS packages built from source below
# (NITROS fork, image_proc, h264_encoder). Their own apt packages are not
# installed: NVIDIA's builds target stock NITROS and would duplicate the fork.
# Stock rosdep cannot resolve these keys, so they are listed explicitly.
# ros-jazzy-isaac-ros-common brings in the CUDA 13 toolkit; nvv4l2 is the NVENC
# backend the H.264 encoder loads at runtime (not declared in its package.xml).
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates gnupg && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.5 noble main" \
      > /etc/apt/sources.list.d/nvidia-isaac-ros.list && \
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main" \
      > /etc/apt/sources.list.d/nvidia-jetson.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-isaac-ros-common \
      ros-jazzy-isaac-ros-gxf \
      ros-jazzy-isaac-ros-vpi-utils \
      ros-jazzy-isaac-ros-tensor-list-interfaces \
      ros-jazzy-gxf-isaac-atlas \
      ros-jazzy-gxf-isaac-gxf-helpers \
      ros-jazzy-gxf-isaac-message-compositor \
      ros-jazzy-gxf-isaac-optimizer \
      ros-jazzy-gxf-isaac-sight \
      ros-jazzy-negotiated \
      nvv4l2 libv4l-0t64 \
      libnvvpi4 vpi4-dev cvcuda0-dev && \
    rm -rf /var/lib/apt/lists/*

# Build the pinned lifecycle fork and its ABI-dependent camera pipeline once.
# Keep their sources/build outputs outside /ws so ordinary development builds
# only discover application packages. Both source and template images reuse
# this installation, independent of changes to application sources.
# rosdep skips the VPI/CV-CUDA keys (installed above) and cuda-toolkit, which
# stock rosdep maps to Ubuntu's nvidia-cuda-toolkit instead of CUDA 13.
ENV ISAAC_ROS_INSTALL=/opt/watonomous/isaac_ros
COPY --from=isaac_ros_source /opt/isaac_ros_ws/src /opt/isaac_ros_ws/src
WORKDIR /opt/isaac_ros_ws
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
      --dependency-types build --dependency-types buildtool \
      --dependency-types build_export --dependency-types buildtool_export \
      --dependency-types exec \
      --skip-keys "libnvvpi4 vpi4-dev cvcuda0-dev cuda-toolkit" && \
    colcon build --merge-install --install-base "${ISAAC_ROS_INSTALL}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf /opt/isaac_ros_ws /var/lib/apt/lists/*
WORKDIR ${AMENT_WS}

################################ Source ################################
# Inherit the compiled underlay so rosdep can also recognize its packages.
FROM dependencies AS source

WORKDIR ${AMENT_WS}/src

# git-lfs is needed for wato-cuda-bevfusion's assets.
RUN apt-get update && \
    apt-get install -y --no-install-recommends git-lfs && \
    rm -rf /var/lib/apt/lists/*

# Add COLCON_IGNORE to prevent the upstream package.xml from being discovered
# as a duplicate of our own bevfusion package.
RUN git clone https://github.com/WATonomous/deep_ros.git deep_ros && \
    git -C deep_ros checkout 23bb3018c6f785a6601c7b6624cb543555759771 && \
    git clone https://github.com/WATonomous/camera_aravis2_nitros.git camera_aravis2_nitros && \
    git -C camera_aravis2_nitros checkout 3b625b3100b962ee06342b36369116a6da486f77 && \
    git clone https://github.com/WATonomous/wato-cuda-bevfusion.git wato-cuda-bevfusion && \
    git -C wato-cuda-bevfusion checkout 08ec4db80ec768e9410c11e032e2a26dd24ded52 && \
    git -C wato-cuda-bevfusion submodule update --init --recursive && \
    touch wato-cuda-bevfusion/COLCON_IGNORE && \
    git -C wato-cuda-bevfusion lfs install && \
    git -C wato-cuda-bevfusion lfs pull

COPY src/perception perception
COPY src/world_modeling/world_model_msgs world_model_msgs
COPY src/world_modeling/lanelet_msgs lanelet_msgs
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/interfacing/calibration/camera_calib camera_calib
COPY src/interfacing/sensor_interfacing sensor_interfacing
COPY src/wato_test wato_test
