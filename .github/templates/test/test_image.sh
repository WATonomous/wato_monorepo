#!/bin/bash
set -e

bash dev_config.sh

echo "Running $SERVICE tests..."
CONTAINER_NAME="$SERVICE-github-actions"

DOCKER_BUILDKIT=1
COMPOSE_DOCKER_CLI_BUILD=1
BUILDKIT_INLINE_CACHE=1

# Run docker-compose service in detached mode and prevent early exit
bash watod2 run -d --name "$CONTAINER_NAME" "$SERVICE" tail -f /dev/null

# Run tests for ros2 packages, on failure script will exit with error message
docker exec "$CONTAINER_NAME" /bin/bash -c "source ~/.bashrc; colcon test; colcon test-result --verbose"

docker stop "$CONTAINER_NAME"
docker rm "$CONTAINER_NAME"

