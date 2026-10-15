ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.8.1-cudnn-runtime-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# git-lfs must be installed here since wato-cuda-bevfusion uses LFS
RUN apt-get update && \
    apt-get install -y --no-install-recommends git-lfs && \
    rm -rf /var/lib/apt/lists/*

# Clone required repositories.
# Add COLCON_IGNORE to wato-cuda-bevfusion so colcon doesn't pick up the
# upstream CUDA-BEVFusion package.xml as a duplicate to our own bevfusion pkg.
RUN git clone https://github.com/WATonomous/deep_ros.git deep_ros && \
    git clone https://github.com/WATonomous/camera_aravis2_nitros.git camera_aravis2_nitros && \
    git clone --recurse-submodules https://github.com/WATonomous/wato-cuda-bevfusion.git wato-cuda-bevfusion && \
    touch wato-cuda-bevfusion/COLCON_IGNORE && \
    git -C wato-cuda-bevfusion lfs install && \
    git -C wato-cuda-bevfusion lfs pull

# Clone isaac_ros_nitros fork with lifecycle node support
RUN R=/tmp/isaac_ros_nitros && \
    git clone https://github.com/WATonomous/isaac_ros_nitros.git $R && \
    git -C $R checkout a60cda942d21172b28a09e4be7ca959d7515e8e1 && \
    git -C $R lfs install && \
    git -C $R lfs pull --include="isaac_ros_nitros/lib/**" && \
    mv $R/isaac_ros_nitros $R/isaac_ros_managed_nitros . && \
    rm -rf $R

COPY src/perception perception

COPY src/world_modeling/world_model_msgs world_model_msgs
COPY src/world_modeling/lanelet_msgs lanelet_msgs

COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/interfacing/calibration/camera_calib camera_calib
COPY src/interfacing/sensor_interfacing sensor_interfacing

COPY src/wato_test wato_test

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

# Isaac ROS (NITROS + image_proc + h264_encoder) for GPU-accelerated image
# rectification and NVENC H.264 compression of the rectified stream.
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
      ros-jazzy-isaac-ros-nitros \
      ros-jazzy-isaac-ros-image-proc \
      ros-jazzy-isaac-ros-h264-encoder && \
    rm -rf /var/lib/apt/lists/*
