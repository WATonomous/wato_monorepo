ARG BASE_IMAGE=ghcr.io/watonomous/wato_monorepo/base:jazzy-ubuntu24.04

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/action action
COPY src/wato_test wato_test

COPY src/world_modeling/lanelet_msgs lanelet_msgs
COPY src/world_modeling/world_model_msgs world_model_msgs
COPY src/world_modeling/behaviour_msgs behaviour_msgs

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# No module-specific dependencies for action