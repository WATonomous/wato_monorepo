#!/bin/bash
set -e

# setup ROS environment
source /home/docker/ament_ws/install/setup.bash

exec "/usr/local/bin/fixuid" "-q" "$@"