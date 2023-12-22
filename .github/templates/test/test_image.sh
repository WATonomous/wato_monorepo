#!/bin/bash
set -e

bash watod_scripts/watod-setup-env.sh

echo "Running $SERVICE tests..."
CONTAINER_NAME="$SERVICE-github-actions"

# Run docker-compose service in detached mode and prevent early exit
bash watod run -d --name "$CONTAINER_NAME" "$SERVICE" tail -f /dev/null

# Run tests for ros2 packages, on failure script will exit with error message
docker exec "$CONTAINER_NAME" /bin/bash -c "source ~/.bashrc; colcon test; colcon test-result --verbose"

docker stop "$CONTAINER_NAME"
docker rm "$CONTAINER_NAME"
