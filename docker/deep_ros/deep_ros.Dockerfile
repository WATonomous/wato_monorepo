ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04
ARG CUDA_BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.2-humble-ubuntu22.04
ARG CUDA_VERSION=12.2
ARG TENSORRT_RUNTIME_VERSION=10.7.0.23
ARG TENSORRT_CUDA_VERSION=12.6
ARG CUDNN_VERSION=9.16.0.29
ARG ONNXRUNTIME_VERSION=1.22.0

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Clone deep_ros repository
RUN git clone --depth 1 --branch brian/tensorrt_ep \
      https://github.com/WATonomous/deep_ros.git deep_ros

################################# Dependencies ################################
FROM ${CUDA_BASE_IMAGE} AS dependencies

# Declare ARG variables for this stage (must match top-level ARG defaults)
ARG CUDA_VERSION=12.2
ARG TENSORRT_RUNTIME_VERSION=10.7.0.23
ARG TENSORRT_CUDA_VERSION=12.6
ARG CUDNN_VERSION=9.16.0.29
ARG ONNXRUNTIME_VERSION=1.22.0

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
    libopencv-core-dev \
    libopencv-imgproc-dev \
    libyaml-cpp-dev \
    wget \
    curl \
    tar \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* /var/cache/apt/archives/partial/*

# Install TensorRT Runtime + essential dev headers and cuDNN
# Split into runtime (smaller) and dev packages to manage disk space
# Static libraries are very large and often not needed
RUN curl -fsSL -o cuda-keyring_1.1-1_all.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    # Install TensorRT runtime packages \
    && apt-get install -qq -y --no-install-recommends \
        libnvinfer10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvinfer-plugin10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvinfer-dispatch10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvinfer-lean10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvinfer-vc-plugin10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvonnxparsers10=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    # Install TensorRT headers (needed for compilation, but no static libs) \
        libnvinfer-headers-dev=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
        libnvinfer-headers-plugin-dev=${TENSORRT_RUNTIME_VERSION}-1+cuda${TENSORRT_CUDA_VERSION} \
    # Install cuDNN \
        libcudnn9-cuda-12=${CUDNN_VERSION}-1 \
        libcudnn9-dev-cuda-12=${CUDNN_VERSION}-1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* /var/cache/apt/archives/partial/*

# Copy in source code from source stage and install rosdep requirements directly
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src/deep_ros src/deep_ros
RUN apt-get -qq update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Pre-download ONNX Runtime CPU/GPU archives to avoid build-time network flakes
RUN set -euo pipefail; \
    ORT_CPU_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"; \
    ORT_GPU_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz"; \
    TMP_DIR=$(mktemp -d); \
    mkdir -p "${AMENT_WS}/build/onnxruntime_vendor" "${AMENT_WS}/build/onnxruntime_gpu_vendor"; \
    for flavor in cpu gpu; do \
      if [ "$flavor" = "cpu" ]; then \
        url="$ORT_CPU_URL"; out_dir="${AMENT_WS}/build/onnxruntime_vendor"; prefix="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}"; \
      else \
        url="$ORT_GPU_URL"; out_dir="${AMENT_WS}/build/onnxruntime_gpu_vendor"; prefix="onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}"; \
      fi; \
      archive="${TMP_DIR}/${prefix}.tgz"; \
      echo "Downloading ONNX Runtime ${flavor} archive from ${url}"; \
      curl -fL --retry 5 --retry-delay 5 -o "$archive" "$url"; \
      tar -xzf "$archive" -C "$out_dir"; \
    done; \
    rm -rf "$TMP_DIR"

# Ensure bash with pipefail for RUN commands with pipelines
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

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
