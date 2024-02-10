#!/bin/bash
set -e

################# Create a space delimited list of modified modules #################
# Outputs a list of modified modules by comparing changes between main and current commit
# References previous GitHub workflow steps

# Action
if [ ${{ steps.changed-files-action.outputs.any_changed }} == 'true' ]; then
    echo "Detected action changes"
    CHANGED_MODULES+="action "
fi

# Interfacing
if [ ${{ steps.changed-files-interfacing.outputs.any_changed }} == 'true' ]; then
    echo "Detected interfacing changes"
    CHANGED_MODULES+="interfacing "
fi

# Perception
if [ ${{ steps.changed-files-perception.outputs.any_changed }} == 'true' ]; then
    echo "Detected perception changes"
    CHANGED_MODULES+="perception "
fi

# Samples
if [ ${{ steps.changed-files-samples.outputs.any_changed }} == 'true' ]; then
    echo "Detected samples changes"
    CHANGED_MODULES+="samples "
fi

# Simulation
if [ ${{ steps.changed-files-simulation.outputs.any_changed }} == 'true' ]; then
    echo "Detected simulation changes"
    CHANGED_MODULES+="simulation "
fi

# World-modeling
if [ ${{ steps.changed-files-world-modeling.outputs.any_changed }} == 'true' ]; then
    echo "Detected world_modeling changes"
    CHANGED_MODULES+="world_modeling"
fi

# Output list
echo "CHANGED_MODULES=$CHANGED_MODULES" >> $GITHUB_OUTPUT