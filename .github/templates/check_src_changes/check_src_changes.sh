#!/bin/bash
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps



# Action
if [ $ACTION_CHANGED == 'true' ]; then
    echo "Detected action changes"
    MODIFIED_MODULES+="action "
fi

# Interfacing
if [ $INTERFACING_CHANGED == 'true' ]; then
    echo "Detected interfacing changes"
    MODIFIED_MODULES+="interfacing "
fi

# Perception
if [ $PERCEPTION_CHANGED == 'true' ]; then
    echo "Detected perception changes"
    MODIFIED_MODULES+="perception "
fi

# Samples
if [ $SAMPLES_CHANGED == 'true' ]; then
    echo "Detected samples changes"
    MODIFIED_MODULES+="samples "
fi

# Simulation
if [ $SIMULATION_CHANGED == 'true' ]; then
    echo "Detected simulation changes"
    MODIFIED_MODULES+="simulation "
fi

# World-modeling
if [ $WORLD_MODELING_CHANGED == 'true' ]; then
    echo "Detected world_modeling changes"
    MODIFIED_MODULES+="world_modeling"
fi

# Infrastructure
if [$INFRASTRUCTURE_CHANGED == 'true']; then
    echo "Detected infrastructure changes"
    echo "::notice:: Detected infrastructure changes, testing entire repo"
    MODIFIED_MODULES="infrastructure"
else
    echo "::notice:: MODIFIED_MODULES are $MODIFIED_MODULES" 
fi

# Output list
echo "modified_modules=$MODIFIED_MODULES" >> $GITHUB_OUTPUT