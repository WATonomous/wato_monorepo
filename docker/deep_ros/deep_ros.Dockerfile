ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04
ARG CUDA_BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04
ARG CUDA_VERSION=12.2
ARG TENSORRT_RUNTIME_VERSION=10.7.0.23
ARG TENSORRT_CUDA_VERSION=12.6

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Clone deep_ros repository
RUN git clone --depth 1 --branch gpu_runtime_test \
      https://github.com/WATonomous/deep_ros.git deep_ros

# Scan for rosdeps across the cloned repository
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths deep_ros --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${CUDA_BASE_IMAGE} AS dependencies

# Declare ARG variables for this stage (must match top-level ARG defaults)
ARG TENSORRT_RUNTIME_VERSION=10.7.0.23
ARG TENSORRT_CUDA_VERSION=12.6

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install common build dependencies for deep_ros (ONNX Runtime, etc.)
# System libraries that may not be caught by rosdep
# Note: catch2 is installed via rosdep from package.xml files
RUN apt-get -qq update && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    libyaml-cpp-dev \
    wget \
    curl \
    tar \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install TensorRT Runtime and cuDNN
# Note: TensorRT version format is MAJOR.MINOR.PATCH.BUILD-1+cudaVERSION
# CUDA 12.2 is compatible with TensorRT built for CUDA 12.6 (backward compatible)
# For CUDA 12.x, cuDNN packages are named libcudnn9-cuda-12
RUN curl -fsSL -o cuda-keyring_1.1-1_all.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update && apt-get install -qq -y --no-install-recommends \
    libnvinfer-lean10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    libcudnn9-cuda-12 \
    libcudnn9-dev-cuda-12 \
    && rm cuda-keyring_1.1-1_all.deb \
    && rm -rf /var/lib/apt/lists/*

# Install rosdep requirements collected from source stage
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src/deep_ros src/deep_ros

# Ensure bash with pipefail for RUN commands with pipelines
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# RMW Configurations
COPY docker/dds_config.xml ${WATONOMOUS_INSTALL}/dds_config.xml

# Dependency Cleanup
WORKDIR ${AMENT_WS}
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
# Build in stages: first build dependencies, then build packages that depend on them
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --packages-up-to deep_ort_gpu_backend_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# Add ONNX Runtime GPU vendor library to system library search path
# This ensures the library is found at runtime without needing LD_LIBRARY_PATH
# The library is installed in both the workspace (for development) and WATONOMOUS_INSTALL (for production)
# ONNX Runtime dynamically loads provider libraries (CUDA, TensorRT) at runtime, so they must be in the search path
RUN { \
        [ -d "${WATONOMOUS_INSTALL}/onnxruntime_gpu_vendor/lib" ] && \
        echo "${WATONOMOUS_INSTALL}/onnxruntime_gpu_vendor/lib"; \
        [ -d "${AMENT_WS}/install/onnxruntime_gpu_vendor/lib" ] && \
        echo "${AMENT_WS}/install/onnxruntime_gpu_vendor/lib"; \
    } | tee /etc/ld.so.conf.d/onnxruntime-gpu-vendor.conf && \
    ldconfig && \
    # Verify the libraries are in the cache
    ldconfig -p | grep -q onnxruntime || (echo "Warning: ONNX Runtime libraries not found in ldconfig cache" && exit 1)

# RMW Configurations
COPY docker/dds_config.xml ${WATONOMOUS_INSTALL}/dds_config.xml

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Keep source code for development/debugging (unlike other services which remove it for production)
# The source is needed for development, testing, and debugging deep_ros packages
# Source Cleanup and Security Setup
# Note: We keep the source code unlike other services since deep_ros is primarily a development container

# Fix ownership of workspace directories for development (build happens as root, but user needs write access)
RUN chown -R ${USER}:${USER} ${AMENT_WS}

# Create directory structure for ONNX model files (volume will be mounted at /perception_models)
# The symlink will be created at runtime via entrypoint or bashrc since the volume may not exist at build time
RUN mkdir -p /workspaces/deep_ros && \
    chown -R ${USER}:${USER} /workspaces

# Ensure ROS2 environment is sourced in bashrc for interactive shells
# (The base image already sources workspace setup, but we need ROS2 first)
# Also create symlink for ONNX file if volume is mounted
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc && \
    echo "if [ -f /perception_models/yolov8m.onnx ] && [ ! -f /workspaces/deep_ros/yolov8m.onnx ]; then ln -sf /perception_models/yolov8m.onnx /workspaces/deep_ros/yolov8m.onnx; fi" >> /home/${USER}/.bashrc

USER ${USER}
