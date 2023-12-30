#!/bin/bash
set -e

echo "Running $SERVICE tests..."
CONTAINER_NAME="$SERVICE-github-actions"

# Run docker-compose service in detached mode and prevent early exit
bash watod -dev pull "$SERVICE"
bash watod -dev up -d --name "$CONTAINER_NAME" "$SERVICE"

# Run tests for ros2 packages, on failure script will exit with error message
docker exec "$CONTAINER_NAME" /bin/bash -c "source /home/bolty/ament_ws/install/setup.bash; colcon test; colcon test-result --verbose"

docker stop "$CONTAINER_NAME"
docker rm "$CONTAINER_NAME"
