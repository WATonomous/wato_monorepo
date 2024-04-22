ARG BASE_BUILD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.0-humble-ubuntu22.04-devel
ARG BASE_PROD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda12.0-humble-ubuntu22.04
ARG BASE_PYTORCH_IMAGE=pytorch/pytorch:1.11.0-cuda11.3-cudnn8-devel
# ################################ Build library ################################

FROM ${BASE_PYTORCH_IMAGE} as SEGFORMER_INFERENCE_BUILD

ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 7.0+PTX"
ENV TORCH_NVCC_FLAGS="-Xfatbin -compress-all"

# To fix GPG key error when running apt-get update
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

RUN apt-get update && apt-get install -y git ninja-build libglib2.0-0 libsm6 libxrender-dev libxext6 libgl1-mesa-dev  \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

#RUN conda clean --all
RUN apt update && apt install pip -y
# Install MMCV
ARG PYTORCH
ARG CUDA
ARG MMCV
RUN ["/bin/bash", "-c", "pip install openmim"]
RUN ["/bin/bash", "-c", "mim install mmengine"]
RUN ["/bin/bash", "-c", "mim install mmcv==2.0.1"]

# Install MMSegmentation
RUN git clone -b main https://github.com/open-mmlab/mmsegmentation.git /mmsegmentation
WORKDIR /mmsegmentation
ENV FORCE_CUDA="1"
RUN pip install -r requirements.txt
RUN pip install mmengine
RUN pip install --no-cache-dir -e .
RUN mim download mmsegmentation --config segformer_mit-b2_8xb1-160k_cityscapes-1024x1024 --dest ./model
#COPY src_video_360.mp4 /mmsegmentation/
COPY test2.mp4 /mmsegmentation/
COPY video.py /mmsegmentation/
ENV CV2_CUDABACKEND=0

# ################################ Source ################################
FROM ${BASE_BUILD_IMAGE} as source



WORKDIR ${AMENT_WS}/src


# # Copy in the paddle inference library
# # RUN mkdir -p semantic_segmentation/src
# # COPY --from=PADDLE_INFERENCE_BUILD /paddle/paddle_inference_cuda120_build.tar semantic_segmentation/src/paddle_inference_cuda120_build.tar




# Set up any additional configurations or data if needed

# Set the entry point
#ENTRYPOINT ["SegFormer"]


# # Copy in source code 
COPY src/perception/semantic_segmentation semantic_segmentation
COPY src/wato_msgs/sample_msgs sample_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_BUILD_IMAGE} as dependencies

# # install tensorrt
# RUN apt update && apt install -y tensorrt

RUN apt update && apt install -y ros-humble-cv-bridge libopencv-dev

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

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

# Add runtime libraries to path
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${AMENT_WS}/install/semantic_segmentation/lib/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as deploy

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
