#!/bin/bash
set -e

# setup ROS environment
echo "SOURCING SETUP.BASH"
source /home/docker/ament_ws/install/setup.bash

exec "$@"
