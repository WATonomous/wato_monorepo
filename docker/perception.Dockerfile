ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/perception/perception:source_main

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
RUN git clone https://github.com/WATonomous/deep_ros.git --branch jazzy-fix deep_ros
COPY src/perception perception
COPY src/infrastructure/wato_lifecycle_manager wato_lifecycle_manager
COPY src/wato_test wato_test

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies
