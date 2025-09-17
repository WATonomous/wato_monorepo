ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:cuda11.7-humble-ubuntu22.04-devel

################################ Source ################################
FROM ${BASE_IMAGE} AS source
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/perception/lidar_object_detection                  lidar_object_detection

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | (grep 'apt-get install' || true) \
      | awk '{print $3}' \
      | sort > /tmp/colcon_install_list

# Clone OpenPCDet
WORKDIR /home/bolty
RUN git clone https://github.com/WATonomous/OpenPCDet.git
WORKDIR /home/bolty/OpenPCDet
RUN git checkout 06fd4f862329625ff9ed850464330816e54531f8

################################ INSTALL OpenCV with CUDA Support ##############
WORKDIR /opt
RUN git clone -b 4.x https://github.com/opencv/opencv.git && \
    git clone -b 4.x https://github.com/opencv/opencv_contrib.git
WORKDIR /opt/opencv
RUN git checkout 4.5.5
WORKDIR /opt/opencv_contrib
RUN git checkout 4.5.5

WORKDIR /opt/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D INSTALL_C_EXAMPLES=OFF \
    -D PYTHON_EXECUTABLE="$(which python3)" \
    -D PYTHON3_INCLUDE_DIR="$(python3 -c 'from distutils.sysconfig import get_python_inc; print(get_python_inc())')" \
    -D PYTHON3_PACKAGES_PATH="$(python3 -c 'from distutils.sysconfig import get_python_lib; print(get_python_lib())')" \
    -D BUILD_opencv_python3=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
    -D WITH_GSTREAMER=ON -D WITH_CUDA=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    .. && make -j"$(nproc)" && make install && ldconfig

RUN rm -rf /opt/opencv /opt/opencv_contrib

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev \
      libatlas-base-dev libeigen3-dev software-properties-common build-essential cmake pkg-config \
      libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
      libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 \
      libcap2 libusb-1.0-0 libatk-adaptor neovim python3-pip python3-setuptools && \
    rm -rf /var/lib/apt/lists/*

# Environment
ENV CUDA_HOME=/usr/local/cuda \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH} \
    PATH=/usr/local/cuda/bin:${PATH} \
    OpenCV_DIR=/usr/share/OpenCV

# Python dependencies
RUN pip3 install --no-cache-dir \
      torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 \
      --extra-index-url https://download.pytorch.org/whl/cu116 && \
    pip3 install --no-cache-dir \
      torch-scatter==2.1.1+cu116 -f https://data.pyg.org/whl/torch-1.13.1+cu116.html && \
    pip3 install --no-cache-dir \
      spconv-cu116==2.3.6 \
      pyquaternion==0.9.9 \
      numpy==1.23 \
      pillow==8.4 \
      mayavi==4.8.1 \
      open3d==0.17.0 \
      av2==0.2.2

# Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Source tree
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src
COPY --from=source /home/bolty/OpenPCDet /home/bolty/OpenPCDet

# Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build #######################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Base libraries and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/bolty/OpenPCDet
RUN pip3 install --no-cache-dir -r requirements.txt && \
    pip3 install --no-cache-dir kornia==0.6.8 && \
    pip3 install --no-cache-dir nuscenes-devkit==1.0.5 && \
    python3 setup.py develop

# Build workspace
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

################################ Prod #########################################
FROM build AS deploy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN chown -R "${USER}:${USER}" "${AMENT_WS}" && rm -rf src/*

USER ${USER}
