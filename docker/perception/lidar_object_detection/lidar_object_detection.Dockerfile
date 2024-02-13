ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda11.7-humble-ubuntu22.04-devel
FROM ${BASE_IMAGE} as dependencies
################################# Dependencies ################################
RUN apt-get update && apt-get install -y \
    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim \
    python3-pip python3-setuptools \
    && apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /var/lib/apt/lists/* /root/* /root/.ros /tmp/* /usr/share/doc
################################ INSTALL OpenCV with CUDA Support ##############
WORKDIR /opt
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 4.5.5
RUN git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && git checkout 4.5.5
WORKDIR /opt/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D PYTHON_EXECUTABLE=$(which python3) \
    -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
    -D BUILD_opencv_python3=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
    -D WITH_GSTREAMER=ON \
    -D WITH_CUDA=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    .. && make -j$(nproc) && make install && ldconfig
RUN rm -rf /opt/opencv /opt/opencv_contrib
# Set environment variables
ENV CUDA_HOME /usr/local/cuda
ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH /usr/local/cuda/bin:${PATH}
ENV OpenCV_DIR=/usr/share/OpenCV
# Install Python dependencies
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
RUN pip3 install torch-scatter -f https://data.pyg.org/whl/torch-1.13.1+cu116.html
ENV TORCH_CUDA_ARCH_LIST="3.5;5.0;6.0;6.1;7.0;7.5;8.0;8.6+PTX"
RUN pip3 install spconv-cu113 pyquaternion numpy==1.23 pillow==8.4 mayavi open3d av2
WORKDIR /home/bolty
RUN git clone https://github.com/WATonomous/OpenPCDet.git
# COPY /OpenPCDet /home/bolty/OpenPCDet
WORKDIR /home/bolty
RUN cd OpenPCDet && pip3 install -r requirements.txt
RUN pip3 install kornia==0.6.8
RUN pip3 install nuscenes-devkit==1.0.5
WORKDIR /home/bolty/OpenPCDet/
RUN python3 setup.py develop
################################ Source #######################################
FROM dependencies as build
WORKDIR ${AMENT_WS}/src
COPY src/perception/lidar_object_detection lidar_object_detection
COPY src/wato_msgs/sample_msgs sample_msgs
RUN apt-get update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y
RUN apt install -y ros-${ROS_DISTRO}-foxglove-bridge
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# Entrypoint setup
COPY /docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
################################ Prod #########################################
FROM build as deploy

# Source Cleanup and Security Setup
RUN chown -R $USER:$USER ${AMENT_WS}
# RUN rm -rf src/*

USER ${USER}