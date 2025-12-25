# syntax=docker/dockerfile:1.4

ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in module-specific source code
COPY src/interfacing interfacing
COPY src/wato_msgs wato_msgs
COPY src/wato_test wato_test

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies here (non-rosdep)
# For interfacing, there are no extra dependencies needed
