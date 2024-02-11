#!/bin/bash
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps

# Action
if [ $ACTION_CHANGED == 'true' ]; then
    echo "Detected action changes"
    CHANGED_MODULES+="action "
fi

# Interfacing
if [ $INTERFACING_CHANGED == 'true' ]; then
    echo "Detected interfacing changes"
    CHANGED_MODULES+="interfacing "
fi

# Perception
if [ $PERCEPTION_CHANGED == 'true' ]; then
    echo "Detected perception changes"
    CHANGED_MODULES+="perception "
fi

# Samples
if [ $SAMPLES_CHANGED == 'true' ]; then
    echo "Detected samples changes"
    CHANGED_MODULES+="samples "
fi

# Simulation
if [ $SIMULATION_CHANGED == 'true' ]; then
    echo "Detected simulation changes"
    CHANGED_MODULES+="simulation "
fi

# World-modeling
if [ $WORLD_MODELING_CHANGED == 'true' ]; then
    echo "Detected world_modeling changes"
    CHANGED_MODULES+="world_modeling"
fi

# Output list
echo "::notice:: CHANGED_MODULES is $CHANGED_MODULES" 
echo "CHANGED_MODULES=$CHANGED_MODULES" >> $GITHUB_OUTPUT