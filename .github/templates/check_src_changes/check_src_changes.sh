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
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps

# Action
if [ $ACTION_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected action changes"
    MODIFIED_MODULES+="action "
fi

# Interfacing
if [ $INTERFACING_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected interfacing changes"
    MODIFIED_MODULES+="interfacing "
fi

# Perception
if [ $PERCEPTION_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected perception changes"
    MODIFIED_MODULES+="perception "
fi

# Samples
if [ $SAMPLES_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected samples changes"
    MODIFIED_MODULES+="samples "
fi

# Simulation
if [ $SIMULATION_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected simulation changes"
    MODIFIED_MODULES+="simulation "
fi

# World-modeling
if [ $WORLD_MODELING_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "Detected world_modeling changes"
    MODIFIED_MODULES+="world_modeling "
fi

# Infrastructure
if [ $INFRASTRUCTURE_CHANGED == 'true' || $ALL_CHANGED == 'true' ]; then
    echo "::notice:: Detected infrastructure changes"
    MODIFIED_MODULES+="infrastructure "
else
    echo "::notice:: MODIFIED_MODULES are $MODIFIED_MODULES"
fi

if [ $ALL_CHANGED == 'true' ]; then
    echo "::notice:: Detected a change that constitutes testing all modules. This is caused by any changes outside src."
fi

# Output list
echo "modified_modules=$MODIFIED_MODULES" >> $GITHUB_OUTPUT
