#!/bin/bash
set -e

# setup ROS environment
source /home/docker/ament_ws/install/setup.bash
echo "source /home/docker/ament_ws/install/setup.bash" >> ~/.bashrc

exec "/usr/local/bin/fixuid" "-q" "$@"