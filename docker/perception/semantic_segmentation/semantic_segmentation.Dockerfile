ARG BASE_BUILD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda11.7-humble-ubuntu22.04-devel
ARG BASE_PROD_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda11.7-humble-ubuntu22.04
ARG BASE_PYTORCH_IMAGE=ghcr.io/watonomous/wato_monorepo/segformer_segmentation:latest
# ################################ Build library ################################

FROM ${BASE_PYTORCH_IMAGE} as Segformer

# # ################################ Source ################################


FROM ${BASE_BUILD_IMAGE} as source
WORKDIR ${AMENT_WS}/src

# # Copy in source code 
COPY src/perception/semantic_segmentation semantic_segmentation
COPY src/wato_msgs/sample_msgs sample_msgs
COPY --from=Segformer /mmsegmentation/model ${AMENT_WS}/src/semantic_segmentation/resource/model

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list


################################# Dependencies ################################
FROM ${BASE_BUILD_IMAGE} as dependencies

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    ninja-build \
    libglib2.0-0 \
    libsm6 \
    libxrender-dev \
    libxext6 \
    libgl1-mesa-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Segformer dependencies 
COPY --from=Segformer /tmp/pip_install_list.txt /tmp/pip_install_list.txt
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
RUN pip install cython
RUN ["/bin/bash", "-c", "pip install https://download.openmmlab.com/mmcv/dist/cu117/torch1.13.0/mmcv-2.0.0rc4-cp310-cp310-manylinux1_x86_64.whl"]
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


ENV FORCE_CUDA="1"


# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh

WORKDIR /mmsegmentation
COPY --from=Segformer /mmsegmentation /mmsegmentation
RUN pip install -r requirements.txt
RUN pip install --no-cache-dir -e .
RUN pip uninstall numpy -y 
RUN pip install numpy==1.26.4
WORKDIR ${AMENT_WS}
# Add runtime libraries to path
ENV CUDNN_DIR=/mmsegmentation/cuda
ENV CV2_CUDABACKEND=0
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${AMENT_WS}/install/semantic_segmentation/lib/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod ################################
FROM build as deploy

# Install runtime libs
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge


WORKDIR ${AMENT_WS}

RUN mkdir -p install/semantic_segmentation/lib/
# Add runtime libraries to path
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${AMENT_WS}/install/semantic_segmentation/lib/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
RUN rm -rf src/*

USER ${USER}
