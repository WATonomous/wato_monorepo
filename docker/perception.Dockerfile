ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code needed for perception build
COPY src/perception perception
COPY src/wato_test wato_test
COPY src/infrastructure/wato_lifecycle_manager infrastructure/wato_lifecycle_manager

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies here (non-rosdep)
# For perception, there are no extra dependencies needed
