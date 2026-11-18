ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.8.1-cudnn-runtime-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
RUN git clone https://github.com/WATonomous/deep_ros.git deep_ros && \
    git clone https://github.com/WATonomous/camera_aravis2_nitros.git camera_aravis2_nitros

COPY src/perception perception

COPY src/world_modeling/world_model_msgs world_model_msgs
COPY src/world_modeling/lanelet_msgs lanelet_msgs
COPY src/world_modeling/eidos_msgs eidos_msgs

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

# Install TensorRT dependencies for deep_ros
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libnvinfer10 \
    libnvinfer-plugin10 \
    libnvonnxparsers10 && \
    rm -rf /var/lib/apt/lists/*

# Isaac ROS (NITROS + image_proc + h264_encoder) for GPU-accelerated image
# rectification and NVENC H.264 compression of the rectified stream.
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates gnupg && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.4 noble main" \
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

# cuVSLAM (PyCuVSLAM) for GPU visual odometry (src/perception/visual_odometry).
# NVIDIA Community License: the wheel is fetched at build time, not vendored.
ARG CUVSLAM_VERSION=17.0.0
RUN pip3 install --no-cache-dir \
    "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v${CUVSLAM_VERSION}/cuvslam-${CUVSLAM_VERSION}%2Bcu12-cp312-abi3-manylinux_2_39_x86_64.whl"

# MapAnything (src/perception/map_anything): PyTorch and the model package, fetched at build time.
# MapAnything's own dependency list pulls numpy 2 (through rerun-sdk), which breaks ROS's
# numpy 1.26, so it and uniception are installed without their declared dependencies and the
# packages the inference path imports are listed here instead. Model weights are downloaded at
# run time into the mounted models directory.
ARG MAPANYTHING_COMMIT=3d10cf7a3016fc0f9bb13a071ee66c47b10be0d9
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cu130 \
      torch==2.13.0 torchvision==0.28.0 && \
    pip3 install --no-cache-dir --no-deps \
      "git+https://github.com/facebookresearch/map-anything.git@${MAPANYTHING_COMMIT}" \
      uniception==0.1.7 && \
    pip3 install --no-cache-dir \
      "numpy<2" \
      einops==0.8.2 \
      huggingface_hub==1.24.0 \
      hydra-core==1.3.8 \
      jaxtyping==0.3.7 \
      natsort==8.4.0 \
      orjson==3.13.0 \
      python-box==7.4.1 \
      safetensors==0.8.0 \
      timm==1.0.31 \
      tqdm==4.67.3
