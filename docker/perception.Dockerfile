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
RUN R=/tmp/isaac_ros_image_pipeline && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git $R && \
    git -C $R checkout 203e41be2a4a96382fe69abac08c0cec76720a7a && \
    mv $R/isaac_ros_image_proc . && \
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
      ros-jazzy-isaac-ros-h264-encoder \
      libnvvpi4 vpi4-dev cvcuda0-dev && \
    rm -rf /var/lib/apt/lists/*

# Build the pinned lifecycle fork and its ABI-dependent camera pipeline once.
# Keep their sources/build outputs outside /ws so ordinary development builds
# only discover application packages. Both source and template images reuse
# this installation, independent of changes to application sources.
# NVIDIA's VPI/CV-CUDA keys are installed above; the CUDA toolkit comes with
# the Isaac ROS binary dependencies. These keys are absent from stock rosdep.
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
    git clone https://github.com/WATonomous/camera_aravis2_nitros.git camera_aravis2_nitros && \
    git clone --recurse-submodules https://github.com/WATonomous/wato-cuda-bevfusion.git wato-cuda-bevfusion && \
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
