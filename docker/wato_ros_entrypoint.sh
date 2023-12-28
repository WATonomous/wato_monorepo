#!/bin/bash
set -e

# setup ROS2 environment
source /home/bolty/ament_ws/install/setup.bash

exec "$@"
