ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy source code
COPY src/perception/patchwork perception/patchwork
COPY src/perception/spatial_association perception/spatial_association
COPY src/perception/perception_bringup perception/perception_bringup
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

# Clone Patchwork++
RUN git clone --depth 1 --branch master \
      https://github.com/url-kaist/patchwork-plusplus \
      perception/patchwork/patchwork-plusplus

# Update CONTRIBUTING.md for tests
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Generate rosdep list
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# --- CUDA INSTALLATION ---
# Install only essential CUDA packages (avoid optional tools like Nsight to save space)
# Essential packages: nvcc compiler, cudart runtime, basic libraries
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update && \
    apt-get install -qq -y --no-install-recommends \
        ca-certificates gnupg wget build-essential && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* && \
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm -f cuda-keyring_1.1-1_all.deb && \
    apt-get -qq update && \
    # Install only essential CUDA packages (skip optional profiling/visualization tools)
    # cuda-toolkit-11-8 pulls in Nsight/visual tools (~2GB) which we don't need
    # Install minimal packages: compiler, runtime, headers, and basic libraries only
    apt-get install -qq -y --no-install-recommends \
        -o APT::Keep-Downloaded-Packages="false" \
        cuda-nvcc-11-8 \
        cuda-cudart-dev-11-8 \
        cuda-nvml-dev-11-8 \
        cuda-command-line-tools-11-8 \
        libcurand-dev-11-8 && \
    # Clean immediately after installation
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* /tmp/* && \
    # Create symlink and configure library path
    ln -sfn /usr/local/cuda-11.8 /usr/local/cuda && \
    echo "/usr/local/cuda-11.8/lib64" > /etc/ld.so.conf.d/cuda-11-8.conf && \
    ldconfig

# Set CUDA Environment Variables
ENV PATH="/usr/local/cuda/bin:/usr/local/cuda-11.8/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}"
ENV CUDA_HOME="/usr/local/cuda"
ENV CUDAToolkit_ROOT="/usr/local/cuda"

# Install manual dependencies
RUN apt-get -qq update && apt-get install -qq -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pcl-conversions \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install generated rosdeps
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy Source
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Build Patchwork++
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${AMENT_WS}/src/perception/patchwork/patchwork-plusplus
RUN cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    ldconfig

# Cleanup (Preserving CUDA)
WORKDIR ${AMENT_WS}
RUN apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

WORKDIR ${AMENT_WS}

# Build ROS 2 packages
# We explicitly pass CUDA paths to ensure CMake finds the compiler we just installed
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCUDAToolkit_ROOT=${CUDA_HOME} \
        -DCMAKE_CUDA_COMPILER=${CUDA_HOME}/bin/nvcc && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

COPY docker/wato_entrypoint.sh ${AMENT_WS}/wato_entrypoint.sh
ENTRYPOINT ["./wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}