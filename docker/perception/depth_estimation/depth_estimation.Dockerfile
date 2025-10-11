ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

# use bash + pipefail for every RUN that has a pipe (DL4006)
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR ${AMENT_WS}/src

# Install DepthAnythingV2
RUN git clone --depth 1 --branch main https://github.com/DepthAnything/Depth-Anything-V2
WORKDIR ${AMENT_WS}/src/Depth-Anything-V2
RUN git checkout e5a2732d3ea2cddc081d7bfd708fc0bf09f812f1
WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/perception/depth_estimation depth_estimation

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
      | (grep 'apt-get install' || true) \
      | awk '{print $3}' \
      | sort > /tmp/colcon_install_list

# Add pip (as apt package) to list
RUN echo "python3-pip" >> /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Base libraries and pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Remove conflicting blinker package cleanly
RUN apt-get update -qq && \
    apt-get remove -y --no-install-recommends python3-blinker && \
    rm -rf /var/lib/apt/lists/*

# Python deps for depth_estimation
WORKDIR ${AMENT_WS}/src/depth_estimation
COPY src/perception/depth_estimation/requirements.txt .
RUN python3 -m pip install --no-cache-dir -r requirements.txt && rm requirements.txt

# Depth-Anything V2 requirements are already included in depth_estimation/requirements.txt

ENV PYTHONPATH="${PYTHONPATH}:${AMENT_WS}/src/Depth-Anything-V2"

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
RUN apt-get -qq autoremove -y && \
    apt-get -qq autoclean && \
    apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_entrypoint.sh ${AMENT_WS}/wato_entrypoint.sh
ENTRYPOINT ["./wato_entrypoint.sh"]

################################ Prod ################################
FROM build AS deploy

# Source Cleanup and Security Setup
RUN rm -rf ${AMENT_WS}/*
USER ${USER}
