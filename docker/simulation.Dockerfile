ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

COPY src/simulation simulation
COPY src/infrastructure/vision_msgs_markers vision_msgs_markers
COPY src/interfacing/eve_description eve_description

COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Install CARLA Client API (TODO, wait for 0.10.0 to exist in pypi)
# https://pypi.org/project/carla/#history
COPY src/simulation/carla_sim/dist /opt/carla/dist
RUN pip3 install --no-cache-dir /opt/carla/dist/carla-0.10.0-cp312-cp312-linux_x86_64.whl
