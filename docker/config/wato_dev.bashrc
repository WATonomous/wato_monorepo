# WATonomous development bashrc additions
# This file is sourced by ~/.bashrc in development containers

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Add ~/.local/bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# Add any custom aliases or functions here
