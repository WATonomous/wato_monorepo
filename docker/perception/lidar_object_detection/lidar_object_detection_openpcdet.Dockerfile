ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04
################################ Source ################################
FROM ${BASE_IMAGE} as source
WORKDIR ${AMENT_WS}/src
# Copy in source code 
COPY src/perception/lidar_object_detection lidar_object_detection
COPY src/wato_msgs/sample_msgs sample_msgs
# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list
################################# Dependencies ################################
FROM ${BASE_IMAGE} as dependencies
# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)
# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src
# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*
################################ INSTALL OpenPCDet ################################
# Set environment variables
ENV NVENCODE_CFLAGS "-I/usr/local/cuda/include"
ENV CV_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive
# Get all dependencies
RUN apt-get update && apt-get install -y \
    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev
RUN apt-get update && apt-get install -y \
    pcl-tools \
    python3-pcl \
    xvfb \
    x11-utils
RUN rm -rf /var/lib/apt/lists/*
# OpenCV with CUDA support
WORKDIR /opencv
RUN git clone https://github.com/opencv/opencv.git -b $CV_VERSION &&\
    git clone https://github.com/opencv/opencv_contrib.git -b $CV_VERSION
# While using OpenCV 4.2.0 we have to apply some fixes to ensure that CUDA is fully supported, thanks @https://github.com/gismo07 for this fix
RUN mkdir opencvfix && cd opencvfix &&\
    git clone https://github.com/opencv/opencv.git -b 4.5.2 &&\
    cd opencv/cmake &&\
    cp -r FindCUDA /opencv/opencv/cmake/ &&\
    cp FindCUDA.cmake /opencv/opencv/cmake/ &&\
    cp FindCUDNN.cmake /opencv/opencv/cmake/ &&\
    cp OpenCVDetectCUDA.cmake /opencv/opencv/cmake/
WORKDIR /opencv/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D PYTHON_EXECUTABLE=$(which python2) \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D BUILD_opencv_python2=ON \
-D BUILD_opencv_python3=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
-D WITH_GSTREAMER=ON \
-D WITH_CUDA=ON \
-D ENABLE_PRECOMPILED_HEADERS=OFF \
.. &&\
make -j$(nproc) &&\
make install &&\
ldconfig &&\
rm -rf /opencv
WORKDIR /
ENV OpenCV_DIR=/usr/share/OpenCV
# PyTorch for CUDA 11.6
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
ENV TORCH_CUDA_ARCH_LIST="3.5;5.0;6.0;6.1;7.0;7.5;8.0;8.6+PTX"
# OpenPCDet
RUN pip3 install numpy==1.23.0 llvmlite numba tensorboardX easydict pyyaml scikit-image tqdm SharedArray open3d mayavi av2 pyquaternion
RUN pip3 install spconv-cu116
RUN pip3 install kornia==0.6.8
RUN pip3 install nuscenes-devkit==1.0.5
# Get extra kitti data
WORKDIR /
RUN git clone https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars.git
# Change based on if you want to retain changes to the OpenPCDet repo
RUN git clone https://github.com/open-mmlab/OpenPCDet.git
WORKDIR /OpenPCDet
# Set up xvfb (X Virtual FrameBuffer)
RUN echo '#!/bin/bash\nXvfb :99 -screen 0 1280x1024x24 &\nsleep 3\nexec "$@"' > /usr/local/bin/start-xvfb.sh \
    && chmod +x /usr/local/bin/start-xvfb.sh
# Set the environment variable for DISPLAY
ENV DISPLAY=:99
RUN python3 setup.py develop
WORKDIR /
ENV NVIDIA_VISIBLE_DEVICES="all" \
    OpenCV_DIR=/usr/share/OpenCV \
    NVIDIA_DRIVER_CAPABILITIES="video,compute,utility,graphics" \
    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib \
    QT_GRAPHICSSYSTEM="native"
CMD ["/usr/local/bin/start-xvfb.sh"]
################################ Build ################################
FROM dependencies as build
# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
################################ Prod ################################
# FROM build as deploy

# # Source Cleanup and Security Setup
# RUN chown -R $USER:$USER ${AMENT_WS}
# RUN rm -rf src/*

# USER ${USER}
