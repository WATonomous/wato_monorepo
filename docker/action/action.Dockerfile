ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/action action
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

RUN git clone --depth 1 https://github.com/carla-simulator/ros-carla-msgs.git --branch 1.3.0 carla_msgs

# Update CONTRIBUTING.md to pass copyright test
COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/carla_msgs/CONTRIBUTING.md
#COPY src/wato_msgs/simulation/mit_contributing.txt ${AMENT_WS}/src/ros-carla-msgs/CONTRIBUTING.md

# Scan for rosdeps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# INSTALL DEPENDENCIES HERE BEFORE THE ROSDEP
# Only do this as a last resort. Utilize ROSDEP first

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Install unzip (required for libtorch)
RUN apt-get update && apt-fast install -qq -y --no-install-recommends unzip && \
    rm -rf /var/lib/apt/lists/*

# Install wget (required for libtorch)
RUN apt-get update && apt-fast install -qq -y --no-install-recommends wget && \
    rm -rf /var/lib/apt/lists/*

#install casadi via pip
RUN apt-get update && apt-fast install -qq -y --no-install-recommends python3-pip && \
    python3 -m pip install --no-cache-dir "pip==24.2" && \
    # Install CasADi
    python3 -m pip install --no-cache-dir "casadi==3.6.5" && \
    # Verify installation
    python3 -c "import casadi; print('CasADi:', casadi.__version__);" && \
    rm -rf /var/lib/apt/lists/*

# ----------------- Install libtorch (PyTorch C++ API) -----------------
ARG LIBTORCH_URL=https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.4.0%2Bcpu.zip
RUN wget -qO /tmp/libtorch.zip "${LIBTORCH_URL}" && \
    unzip -q /tmp/libtorch.zip -d /opt && \
    rm -rf /tmp/libtorch.zip

# Make libtorch visible to CMake and the runtime linker
ENV Torch_DIR=/opt/libtorch/share/cmake/Torch \
    CMAKE_PREFIX_PATH=/opt/libtorch:${CMAKE_PREFIX_PATH} \
    LD_LIBRARY_PATH=/opt/libtorch/lib:${LD_LIBRARY_PATH}


# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}


# Clean previous build cache for safety
RUN rm -rf build/mppi_pkg install/mppi_pkg log/mppi_pkg || true

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf "${AMENT_WS:?}"/*
USER ${USER}
