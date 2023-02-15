#!/bin/bash
set -e

# setup ROS2 environment
source /home/docker/ament_ws/install/setup.bash

exec "$@"
