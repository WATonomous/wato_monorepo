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
## Possible values:
##   - infrastructure     	:   starts visualization tools (foxglove and/or vnc and/or data_stream)
##   - interfacing          :   starts up interfacing nodes (CAN and sensor interfaces)
##	 - perception			:	starts perception nodes (Object detection, tracking)
##	 - world_modeling		:	starts world modeling nodes (HD map, localization)
##	 - action				:	starts action nodes (local planning, control)
##	 - simulation			:	starts simulation (carla)


#export ACTIVE_MODULES=""

################################# MODE OF OPERATION #################################
## Possible modes of operation when running watod.
## Possible values:
##	 - deploy (default)		:	runs production-grade containers (non-editable)
##	 - develop   		    :	runs developer containers (editable)

# export MODE_OF_OPERATION=""

############################## ADVANCED CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
# export COMPOSE_PROJECT_NAME=""

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
# export TAG=""

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# export REGISTRY_URL=""

############################### FOXGLOVE SETTINGS ##################################
## Size of the outgoing websocket buffer (bytes) for foxglove_bridge. Increase to prevent drops; default is 256 MiB.
# export FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES="${FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES:-268435456}"
