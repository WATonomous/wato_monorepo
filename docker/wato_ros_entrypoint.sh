#!/bin/bash
set -e

# setup ros2 environment
# echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> "/home/docker/.bashrc"
# echo "source /home/docker/ament_ws/install/setup.bash" >> "/home/docker/.bashrc"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/docker/ament_ws/install/setup.bash"

exec "/usr/local/bin/fixuid" "-q" "$@"
