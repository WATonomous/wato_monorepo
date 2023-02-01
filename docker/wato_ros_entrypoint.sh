#!/bin/bash
set -e

# setup ROS2 environment
source /home/docker/colcon_ws/install/setup.bash

exec "/usr/local/bin/fixuid" "-q" "$@"