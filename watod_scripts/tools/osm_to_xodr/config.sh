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
# shellcheck shell=bash
# Configuration template for osm_to_xodr.sh.
# Copy this file to config.local.sh and set the values there.
# shellcheck disable=SC2034  # Variables are consumed after this file is sourced.

# Upstream map configuration. Common choices are:
#   example_latlon      - use LATITUDE/LONGITUDE as the geographic reference
#   example_mgrs_offset - use MGRS_GRID and subtract OFFSET_* from the map
MAP_CONFIG=""

# Settings for MAP_CONFIG="example_latlon".
LATITUDE=""
LONGITUDE=""
ALTITUDE=""

# Settings for MAP_CONFIG="example_mgrs_offset".
MGRS_GRID=""
OFFSET_X=""
OFFSET_Y=""
OFFSET_Z=""

# Permit replacing an existing XODR output. Keep this false unless overwriting
# the output on every run is intentional.
FORCE=false

# Any additional upstream Hydra overrides can be added here, for example:
#   EXTRA_OPTIONS=("map.use_local_coordinates=true" "some.setting=value")
EXTRA_OPTIONS=()
