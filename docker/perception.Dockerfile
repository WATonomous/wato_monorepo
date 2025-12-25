# syntax=docker/dockerfile:1.4

ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
COPY src/perception/perception_bringup perception_bringup
COPY src/perception/patchwork patchwork
COPY src/perception/tracking_2d tracking_2d
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

# Bring in Patchwork++ third-party dependency (built later in dependencies stage)
RUN git clone --depth 1 --branch master \
      https://github.com/url-kaist/patchwork-plusplus \
      patchwork/patchwork-plusplus

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build and install Patchwork++
COPY --from=source ${AMENT_WS}/src/patchwork/patchwork-plusplus /tmp/patchwork-plusplus
WORKDIR /tmp/patchwork-plusplus
RUN cmake -S cpp -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    echo /usr/local/lib | tee /etc/ld.so.conf.d/usr-local.conf && ldconfig && \
    rm -rf /tmp/patchwork-plusplus
