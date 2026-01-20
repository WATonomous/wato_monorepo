ARG MODULE_SOURCE=interfacing:source
ARG MODULE_DEPS=interfacing:deps

################################ Rosdep Scan ################################
# Reference the 'source' stage from the module image
FROM ${MODULE_SOURCE} AS rosdep_scan
ARG MODULE_SOURCE
ARG MODULE_DEPS

# Scan for rosdeps from the source stage
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -qq update && \
    rosdep install --from-paths . --ignore-src -r -s > /tmp/rosdep_output && \
    (grep 'apt-get install' /tmp/rosdep_output || true) \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list && \
    (grep 'pip3 install' /tmp/rosdep_output || true) \
        | sed 's/.*pip3 install //' \
        | sort > /tmp/colcon_pip_install_list

################################ Install Rosdeps ################################
# Use the dependencies stage from the module image
FROM ${MODULE_DEPS} AS rosdep_install
ARG MODULE_SOURCE
ARG MODULE_DEPS

# Install Rosdep requirements
COPY --from=rosdep_scan /tmp/colcon_install_list /tmp/colcon_install_list
COPY --from=rosdep_scan /tmp/colcon_pip_install_list /tmp/colcon_pip_install_list
RUN apt-get update && \
    if [ -s /tmp/colcon_install_list ]; then \
      xargs -a /tmp/colcon_install_list apt-fast install -qq -y --no-install-recommends; \
    fi && \
    rm -rf /var/lib/apt/lists/* && \
    if [ -s /tmp/colcon_pip_install_list ]; then \
      xargs -a /tmp/colcon_pip_install_list pip3 install --no-cache-dir; \
    fi

# Copy in source code from rosdep_scan stage (which comes from MODULE_SOURCE)
WORKDIR ${AMENT_WS}
COPY --from=rosdep_scan ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

# RMW Configurations
COPY docker/config/rmw_zenoh_router_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_router_config.json5
COPY docker/config/rmw_zenoh_session_config.json5 ${WATONOMOUS_INSTALL}/rmw_zenoh_session_config.json5

# Entrypoint
COPY docker/config/wato_entrypoint.sh ${WATONOMOUS_INSTALL}/wato_entrypoint.sh
ENTRYPOINT ["/opt/watonomous/wato_entrypoint.sh"]

################################ Build ################################
FROM rosdep_install AS build

# Build and Install ROS2 packages
WORKDIR ${AMENT_WS}
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/. "${WATONOMOUS_INSTALL}"

################################ Deploy ################################
FROM build AS deploy

# Source Cleanup, Security Setup, and Workspace Setup
RUN rm -rf "${AMENT_WS:?}"/* && \
    chown -R "${USER}":"${USER}" "${AMENT_WS}"
USER ${USER}

################################ Develop ################################
FROM rosdep_install AS develop

# Update Sources and Install Useful Developer Tools
# hadolint ignore=DL3009
RUN apt-get update && \
    apt-fast install -qq -y --no-install-recommends \
    tmux \
    git \
    curl \
    wget \
    htop \
    nano \
    tree

# Make ament_ws owned by bolty
RUN chown -R "${USER}":"${USER}" "${AMENT_WS}"
USER ${USER}

# Install Claude Code natively
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -fsSL https://claude.ai/install.sh | bash

# Setup dev bashrc
COPY docker/config/wato_dev.bashrc ${WATONOMOUS_INSTALL}/wato_dev.bashrc
RUN echo "source ${WATONOMOUS_INSTALL}/wato_dev.bashrc" >> ~/.bashrc

# Default to opening in the AMENT_WS
WORKDIR ${AMENT_WS}
