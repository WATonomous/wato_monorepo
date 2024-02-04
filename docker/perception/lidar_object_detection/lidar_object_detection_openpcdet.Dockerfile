ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda11.7-humble-ubuntu22.04-devel
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
# Set environment variables
ENV CUDA_HOME /usr/local/cuda
ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH /usr/local/cuda/bin:${PATH}
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
################################# INSTALL OpenCV with CUDA Support ################
RUN apt-get update && apt-get install -y \
    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim
WORKDIR /opt
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.5.5

RUN git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 4.5.5
WORKDIR /opt/opencv/build
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
WORKDIR /
################################ INSTALL OpenPCDet ################################
RUN apt update && apt install -y python3-pip
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
RUN pip3 install spconv-cu113
RUN apt update && apt install -y python3-setuptools
WORKDIR /home/bolty
RUN git clone https://github.com/Kin-Zhang/OpenPCDet.git
RUN cd OpenPCDet && pip3 install -r requirements.txt
RUN pip3 install pyquaternion numpy==1.23 pillow==8.4 mayavi open3d
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
