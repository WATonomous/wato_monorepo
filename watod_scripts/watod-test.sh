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

# watod-test.sh - Builds test images and runs tests
# Usage: watod-test.sh --pre-profiles <profiles...> --all-profiles <profiles...> [--services <services...>] [--packages <packages...>]

# Get monorepo directory
MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MONO_DIR"

# Parse arguments
declare -a PRE_PROFILES=()
declare -a ALL_PROFILES=()
declare -a TEST_PRE_PROFILES=()
declare -a TEST_ALL_PROFILES=()
declare -a PROFILE_MODULES=()
declare -a TEST_SERVICES=()
declare -a FILTER_PACKAGES=()

# Parse profile arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --pre-profiles)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        PRE_PROFILES+=("$1")
        shift
      done
      ;;
    --all-profiles)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        ALL_PROFILES+=("$1")
        shift
      done
      ;;
    --packages)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        FILTER_PACKAGES+=("$1")
        shift
      done
      ;;
    --services)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        TEST_SERVICES+=("$1")
        shift
      done
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

# Convert profiles to test profiles and extract module names
for profile in "${ALL_PROFILES[@]}"; do
  # Remove _dev suffix to get module name
  module_name="${profile%_dev}"
  PROFILE_MODULES+=("$module_name")
  # Convert to test profile (e.g., "action" -> "action_test", "action_dev" -> "action_test")
  TEST_ALL_PROFILES+=("${module_name}_test")
done

# Keep pre-profiles as-is (they're used for building)
TEST_PRE_PROFILES=("${PRE_PROFILES[@]}")

# Use unique ROS_DOMAIN_ID for test isolation
TEST_ROS_DOMAIN_ID=${TEST_ROS_DOMAIN_ID:-99}

# Services to skip (non-ROS services that don't have colcon tests)
SKIP_SERVICES=("log_viewer" "network_namespace")

# Track test results
declare -a TESTED_SERVICES=()
declare -a TEST_RESULTS=()
declare -a TEST_PACKAGES=()
declare -a TEST_COUNTS=()

# Compose files for testing
# PRE-BUILD uses standard dep files to build source/deps stages
declare -a WATCLOUD_COMPOSE_FILES=()
if [[ "${WATCLOUD_MODE:-false}" == "true" ]]; then
  WATCLOUD_COMPOSE_FILES=("modules/docker-compose.watcloud.yaml")
fi

declare -a TEST_PRE_COMPOSE_FILES=("modules/docker-compose.yaml" "${WATCLOUD_COMPOSE_FILES[@]}" "modules/docker-compose.dep.yaml")
# Main BUILD uses dep files (for profile assignments) + test files (for test service overrides)
declare -a TEST_ALL_COMPOSE_FILES=("modules/docker-compose.yaml" "${WATCLOUD_COMPOSE_FILES[@]}" "modules/docker-compose.dep.yaml" "modules/docker-compose.test.yaml")

# Run tests for a service
run_tests() {
  local service=$1
  local test_service="${service}_test"

  # Build colcon test command
  local colcon_cmd="colcon test --event-handlers console_direct+"
  if [[ ${#FILTER_PACKAGES[@]} -gt 0 ]]; then
    colcon_cmd+=" --packages-select ${FILTER_PACKAGES[*]}"
  fi

  # Capture test output
  local test_output
  test_output=$( "$MONO_DIR/watod_scripts/watod-compose.sh" run \
    --pre-profiles "${TEST_PRE_PROFILES[@]}" \
    --all-profiles "${TEST_ALL_PROFILES[@]}" \
    --compose-files "${TEST_ALL_COMPOSE_FILES[@]}" \
    --rm \
    -e ROS_DOMAIN_ID="$TEST_ROS_DOMAIN_ID" \
    --name "${service}_test_$$" \
    -w /ws \
    "$test_service" \
    /bin/bash -c "source /opt/watonomous/setup.bash && $colcon_cmd && colcon test-result --verbose" 2>&1)

  local exit_code=$?
  echo "$test_output"

  # Extract summary information
  local packages
  local tests
  packages=$(echo "$test_output" | grep -oP "Summary: \K\d+(?= packages? finished)" | tail -1)
  tests=$(echo "$test_output" | grep -oP "Summary: \K\d+(?= tests)" | tail -1)

  # Store results
  TESTED_SERVICES+=("$service")
  if [[ $exit_code -eq 0 ]]; then
    TEST_RESULTS+=("PASSED")
  else
    TEST_RESULTS+=("FAILED")
  fi
  TEST_PACKAGES+=("${packages:-0}")
  TEST_COUNTS+=("${tests:-0}")

  if [[ $exit_code -ne 0 ]]; then
    echo "Error: Tests failed for $service" >&2
    return 1
  fi
}

# Build test images using watod-compose.sh with test compose files
echo "Building test images..."
if ! "$MONO_DIR/watod_scripts/watod-compose.sh" build \
  --pre-profiles "${TEST_PRE_PROFILES[@]}" \
  --all-profiles "${TEST_ALL_PROFILES[@]}" \
  --pre-compose-files "${TEST_PRE_COMPOSE_FILES[@]}" \
  --compose-files "${TEST_ALL_COMPOSE_FILES[@]}"; then
  echo "Error: Failed to build test images" >&2
  exit 1
fi

# Main logic
if [[ ${#TEST_SERVICES[@]} -gt 0 ]]; then
  # Test specific services (convert module name to bringup service name)
  for service in "${TEST_SERVICES[@]}"; do
    # If service doesn't end with _bringup, append it (e.g., world_modeling -> world_modeling_bringup)
    if [[ ! "$service" =~ _bringup$ ]]; then
      service="${service}_bringup"
    fi
    run_tests "$service" || exit 1
  done
else
  # Test all services - use watod-compose.sh for config
  echo "Getting list of services to test..."
  readarray -t SERVICES < <("$MONO_DIR/watod_scripts/watod-compose.sh" config \
    --pre-profiles "${TEST_PRE_PROFILES[@]}" \
    --all-profiles "${TEST_ALL_PROFILES[@]}" \
    --compose-files "${TEST_ALL_COMPOSE_FILES[@]}" \
    --services 2>/dev/null)

  if [[ ${#SERVICES[@]} -eq 0 ]]; then
    echo "Error: No services found in active modules." >&2
    exit 1
  fi

  echo "Found ${#SERVICES[@]} service(s) in compose files"

  for service in "${SERVICES[@]}"; do
    # Skip non-ROS services
    if [[ " ${SKIP_SERVICES[*]} " =~ \ ${service}\  ]]; then
      echo "Skipping $service (in skip list)"
      continue
    fi

    # Only test *_bringup_test services
    if [[ ! "$service" =~ _bringup_test$ ]]; then
      continue
    fi

    # Extract module name from test service (e.g., perception_bringup_test -> perception)
    module_name="${service%_bringup_test}"

    # Only test if this module is in the active profiles
    if [[ ! " ${PROFILE_MODULES[*]} " =~ \ ${module_name}\  ]]; then
      echo "Skipping $service (module not in active profiles)"
      continue
    fi

    # Convert test service name back to base service name for run_tests
    base_service="${service%_test}"
    run_tests "$base_service" || exit 1
  done
fi

# Print test summary
# ANSI color codes
readonly BLUE='\033[0;34m'
readonly GREEN='\033[0;32m'
readonly CYAN='\033[0;36m'
readonly RED='\033[0;31m'
readonly YELLOW='\033[0;33m'
readonly RESET='\033[0m'
readonly BOLD='\033[1m'

echo ""
echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${RESET}"
echo -e "${BLUE}║${RESET}  ${BOLD}Test Summary${RESET}                                             ${BLUE}║${RESET}"
echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"

total_packages=0
total_tests=0
all_passed=true

for i in "${!TESTED_SERVICES[@]}"; do
  service="${TESTED_SERVICES[$i]}"
  result="${TEST_RESULTS[$i]}"
  packages="${TEST_PACKAGES[$i]}"
  tests="${TEST_COUNTS[$i]}"

  total_packages=$((total_packages + packages))
  total_tests=$((total_tests + tests))

  # Format service name with result
  if [[ "$result" == "PASSED" ]]; then
    printf "${BLUE}║${RESET}    ${GREEN}✓${RESET} ${CYAN}%-30s${RESET} %2d pkgs, %4d tests ${BLUE}%-2s║${RESET}\n" "$service" "$packages" "$tests"
  else
    printf "${BLUE}║${RESET}    ${RED}✗${RESET} ${CYAN}%-30s${RESET} ${RED}FAILED${RESET}              ${BLUE}║${RESET}\n" "$service"
    all_passed=false
  fi
done

echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
# Calculate dynamic padding for the Total line (box content width is 57 chars after "  ")
total_text="Total: ${total_packages} packages, ${total_tests} tests"
padding=$((57 - ${#total_text}))
printf "${BLUE}║${RESET}  ${YELLOW}${BOLD}%s${RESET}%${padding}s${BLUE}║${RESET}\n" "$total_text" ""
if $all_passed; then
  echo -e "${BLUE}║${RESET}  ${GREEN}${BOLD}Status: ALL TESTS PASSED${RESET}                                 ${BLUE}║${RESET}"
else
  echo -e "${BLUE}║${RESET}  ${RED}${BOLD}Status: SOME TESTS FAILED${RESET}                              ${BLUE}║${RESET}"
fi
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${RESET}"
echo ""
