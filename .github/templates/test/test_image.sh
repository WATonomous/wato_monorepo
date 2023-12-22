#!/bin/bash
set -e

echo "Running $IMAGE tests..."

# Run tests for ros2 packages, on failure script will exit with error message
docker run --user="$FIXUID:$FIXGID" --rm "$IMAGE" /bin/bash -c "source ~/.bashrc; colcon test; colcon test-result --verbose"
