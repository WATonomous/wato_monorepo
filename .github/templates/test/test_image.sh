#!/bin/bash
set -e

bash scripts/watod-setup-env.sh

echo "Running $IMAGE tests..."

# Run tests for ros2 packages, on failure script will exit with error message
docker run --rm "$IMAGE" /bin/bash -c "source ~/.bashrc; colcon test; colcon test-result --verbose"
