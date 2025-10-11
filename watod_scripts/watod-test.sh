#!/usr/bin/env bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -euo pipefail

# Parse arguments: module files (-f flags), then service names
declare -a MODULES=()
declare -a TEST_SERVICES=()

while [[ $# -gt 0 && "$1" == "-f" ]]; do
  MODULES+=("$1" "$2")
  shift 2
done
TEST_SERVICES=("$@")

# Get monorepo directory
MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MONO_DIR"

# Use unique ROS_DOMAIN_ID for test isolation
TEST_ROS_DOMAIN_ID=${TEST_ROS_DOMAIN_ID:-99}

# Run tests for a service
run_tests() {
  local service=$1
  local image=$(docker compose "${MODULES[@]}" config --images "$service" 2>/dev/null | head -n1)
  
  if [[ -z "$image" ]]; then
    echo "Error: Could not find image for service $service"
    return 1
  fi
  
  echo "Testing $service (image: $image)"
  docker run --rm \
    -e ROS_DOMAIN_ID=$TEST_ROS_DOMAIN_ID \
    --name "${service}_test_$$" \
    "$image" \
    /bin/bash -c "colcon test; colcon test-result --verbose"
}

# Main logic
if [[ ${#TEST_SERVICES[@]} -gt 0 ]]; then
  # Test specific services
  for service in "${TEST_SERVICES[@]}"; do
    run_tests "$service"
  done
else
  # Test all services
  readarray -t SERVICES < <(docker compose "${MODULES[@]}" config --services 2>/dev/null)
  
  if [[ ${#SERVICES[@]} -eq 0 ]]; then
    echo "No services found in active modules."
    exit 1
  fi
  
  for service in "${SERVICES[@]}"; do
    run_tests "$service"
  done
fi

echo "Testing complete!"