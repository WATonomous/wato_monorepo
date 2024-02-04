#!/bin/bash
set -e

docker pull -q $IMAGE:$TAG
docker run $IMAGE:$TAG /bin/bash -c "source /home/bolty/ament_ws/install/setup.bash; colcon test; colcon test-result --verbose"
