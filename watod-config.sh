#!/bin/bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## ----------------------- watod Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a watod-config.local.sh
##       file that is untrackable by git and readable by watod.
##

############################ ACTIVE MODULE CONFIGURATION ############################
## List of active modules to run, defined in docker-compose.yaml.
## Add ":dev" suffix to run a module in development mode (with live code editing)
##
## Possible values:
##   - interfacing          :   starts up interfacing nodes (CAN and sensor interfaces)
##   - perception           :   starts perception nodes (Object detection, tracking)
##   - world_modeling       :   starts world modeling nodes (HD map, localization)
##   - action               :   starts action nodes (local planning, control)
##   - simulation           :   starts simulation (carla)
##
## Examples:
##   - "interfacing"                      : Interfacing in deploy mode
##   - "interfacing:dev"                  : Interfacing in dev mode (editable)
##   - "interfacing:dev perception:dev"   : Both in dev mode

export ACTIVE_MODULES="simulation simulation:dev world_modeling:dev"

############################## ADVANCED CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
# export COMPOSE_PROJECT_NAME=""

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
# export TAG=""

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# export REGISTRY_URL=""

# Directory where bags are stored and read. DEFAULT = "$MONO_DIR/bags"
# export BAG_DIRECTORY=""

############################### ROS 2 MIDDLEWARE SETTINGS ##############################
## Middleware to use for interprocess communication. DEFAULT = "rmw_zenoh_cpp"
# export RMW_IMPLEMENTATION=""

## Zenoh configuration - enables zero-copy transport for local communication
## Path to Zenoh router configuration. DEFAULT = "file:///opt/watonomous/rmw_zenoh_router_config.json5"
# export ZENOH_ROUTER_CONFIG_URI=""

## Path to Zenoh session configuration. DEFAULT ="file:///opt/watonomous/rmw_zenoh_session_config.json5"
# export ZENOH_SESSION_CONFIG_URI=""


## ROS 2 Domain ID for network isolation. DEFAULT = "<your_uid> % 230"
## Each user gets a unique domain to prevent cross-talk on shared networks
# export ROS_DOMAIN_ID=""

############################### FOXGLOVE SETTINGS ##################################
## Size of the outgoing websocket buffer (bytes) for foxglove_bridge. Increase to prevent drops; default is 256 MiB.
# export FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES="${FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES:-268435456}"

############################### SIMULATION SETTINGS ################################
## CARLA rendering mode: "gpu" for GPU-accelerated rendering, "no_gpu" for headless/software rendering
## Use "no_gpu" when running on machines without a GPU or when GPU resources are limited
## DEFAULT = "no_gpu"
# export CARLA_RENDER_MODE="no_gpu"
