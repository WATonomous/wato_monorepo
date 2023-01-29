#!/bin/bash
# set -e

# setup ROS2 environment
source /home/docker/src/ros-bridge/install/setup.bash

export CARLA_ROOT=/home/docker/src/carla
export SCENARIO_RUNNER_ROOT=/home/docker/src/scenario_runner
export LEADERBOARD_ROOT=/home/docker/src/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}

# exec "/usr/local/bin/fixuid" "-q" "$@"