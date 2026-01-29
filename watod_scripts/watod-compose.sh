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

# watod-compose.sh - Handles all docker compose operations with preprocessing
# Usage: watod-compose.sh <command> --pre-profiles <profile1> <profile2> ... --all-profiles <profile1> <profile2> ... [--pre-compose-files <files...>] [--compose-files <file1> <file2>...] [compose-args...]

# Helper function to run docker compose with standard settings
run_docker_compose() {
  DOCKER_BUILDKIT=${DOCKER_BUILDKIT:-1} \
    COMPOSE_BAKE=${COMPOSE_BAKE:-true} \
    docker compose "$@"
}

# Parse arguments
COMPOSE_CMD=()
declare -a PRE_PROFILES=()
declare -a ALL_PROFILES=()
declare -a CUSTOM_PRE_COMPOSE_FILES=()
declare -a CUSTOM_ALL_COMPOSE_FILES=()
declare -a EXTRA_COMPOSE_ARGS=()

# First argument is the compose command
if [[ $# -lt 1 ]]; then
  echo "Error: Missing compose command" >&2
  echo "Usage: watod-compose.sh <command> --pre-profiles <profiles...> --all-profiles <profiles...> [--compose-files <files...>]" >&2
  exit 1
fi

COMPOSE_CMD=("$1")
shift

# Parse profile and file arguments
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
    --pre-compose-files)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        CUSTOM_PRE_COMPOSE_FILES+=("-f" "$1")
        shift
      done
      ;;
    --compose-files)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        CUSTOM_ALL_COMPOSE_FILES+=("-f" "$1")
        shift
      done
      ;;
    *)
      # Additional compose command arguments (e.g., -d for up, --images for config)
      EXTRA_COMPOSE_ARGS+=("$1")
      shift
      ;;
  esac
done

# Get monorepo directory
MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MONO_DIR"

# Standard compose file sets
declare -a DEFAULT_PRE_COMPOSE_FILES=("-f" "modules/docker-compose.yaml" "-f" "modules/docker-compose.dep.yaml")
declare -a DEFAULT_ALL_COMPOSE_FILES=("-f" "modules/docker-compose.yaml" "-f" "modules/docker-compose.dep.yaml" "-f" "modules/docker-compose.dev.yaml")

# Use custom compose files if provided, otherwise use defaults
if [[ ${#CUSTOM_PRE_COMPOSE_FILES[@]} -gt 0 ]]; then
  PRE_COMPOSE_FILES=("${CUSTOM_PRE_COMPOSE_FILES[@]}")
else
  PRE_COMPOSE_FILES=("${DEFAULT_PRE_COMPOSE_FILES[@]}")
fi

if [[ ${#CUSTOM_ALL_COMPOSE_FILES[@]} -gt 0 ]]; then
  ALL_COMPOSE_FILES=("${CUSTOM_ALL_COMPOSE_FILES[@]}")
else
  ALL_COMPOSE_FILES=("${DEFAULT_ALL_COMPOSE_FILES[@]}")
fi

# If build, pull base images first to ensure they're tagged locally
if [[ "${COMPOSE_CMD[0]}" == "build" ]]; then
  echo "Pulling base images..."
  base_images=$(grep -h "^ARG BASE_IMAGE=" docker/*.Dockerfile 2>/dev/null | \
                sed 's/ARG BASE_IMAGE=//' | \
                sort -u)

  if [[ -n "$base_images" ]]; then
    while IFS= read -r base_image; do
      echo "  Pulling $base_image..."
      docker pull "$base_image" 2>/dev/null || \
        echo "    (Skipped - using cached version or offline)"
    done <<< "$base_images"
  fi
fi

# Convert profile names to --profile flags
declare -a PRE_PROFILE_FLAGS=()
for p in "${PRE_PROFILES[@]}"; do
  PRE_PROFILE_FLAGS+=("--profile" "$p")
done

declare -a ALL_PROFILE_FLAGS=()
for p in "${ALL_PROFILES[@]}"; do
  ALL_PROFILE_FLAGS+=("--profile" "$p")
done

# If build, run PRE-BUILD stage (source and dependency stages)
if [[ "${COMPOSE_CMD[0]}" == "build" && ${#PRE_PROFILES[@]} -gt 0 ]]; then
  echo "RUNNING PRE-BUILD"
  run_docker_compose "${PRE_COMPOSE_FILES[@]}" "${PRE_PROFILE_FLAGS[@]}" build "${EXTRA_COMPOSE_ARGS[@]}"

  # In CI, push PRE-BUILD images to registry
  if [[ -n ${CI:-} || -n ${GITHUB_ACTIONS:-} ]]; then
    echo "CI detected: Pushing PRE-BUILD images to registry..."
    run_docker_compose "${PRE_COMPOSE_FILES[@]}" "${PRE_PROFILE_FLAGS[@]}" push
  fi
fi

# Handle 'up' command preprocessing: always run in detached mode
SHOW_STATUS_PANEL=false
if [[ "${COMPOSE_CMD[0]}" == "up" && ! "${EXTRA_COMPOSE_ARGS[*]}" =~ -d ]]; then
  EXTRA_COMPOSE_ARGS+=( -d )
  SHOW_STATUS_PANEL=true
fi

# Run main compose command
if [[ "${COMPOSE_CMD[0]}" == "build" ]]; then
  echo "RUNNING BUILD"
fi

# Run compose command with or without profiles
run_docker_compose "${ALL_COMPOSE_FILES[@]}" "${ALL_PROFILE_FLAGS[@]}" "${COMPOSE_CMD[@]}" "${EXTRA_COMPOSE_ARGS[@]}"

# In CI, push final images after successful build
if [[ "${COMPOSE_CMD[0]}" == "build" && ( -n ${CI:-} || -n ${GITHUB_ACTIONS:-} ) && ${#ALL_PROFILES[@]} -gt 0 ]]; then
  echo "CI detected: Pushing final images to registry..."
  run_docker_compose "${ALL_COMPOSE_FILES[@]}" "${ALL_PROFILE_FLAGS[@]}" push
fi

# Display status panel after 'up' command
if [[ "$SHOW_STATUS_PANEL" == "true" ]]; then
  # ANSI color codes
  readonly BLUE='\033[0;34m'
  readonly GREEN='\033[0;32m'
  readonly CYAN='\033[0;36m'
  readonly RED='\033[0;31m'
  readonly YELLOW='\033[0;33m'
  readonly RESET='\033[0m'
  readonly BOLD='\033[1m'

  # Source .env to get ports and project name
  if [[ -f "$MONO_DIR/modules/.env" ]]; then
    # shellcheck disable=SC1091
    set -a
    source "$MONO_DIR/modules/.env"
    set +a
  fi

  # Determine URLs based on connection type
  foxglove_url="ws://localhost:${FOXGLOVE_BRIDGE_PORT}"
  log_viewer_url="http://localhost:${LOG_VIEWER__PORT}"
  pygame_hud_url="http://localhost:${PYGAME_HUD_PORT}"

  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    # SSH_CONNECTION format: client_ip client_port server_ip server_port
    server_ip=$(echo "$SSH_CONNECTION" | awk '{print $3}')
    foxglove_url="ws://${server_ip}:${FOXGLOVE_BRIDGE_PORT}"
    log_viewer_url="http://${server_ip}:${LOG_VIEWER__PORT}"
    pygame_hud_url="http://${server_ip}:${PYGAME_HUD_PORT}"

    # Show SSH warning
    echo ""
    echo -e "${YELLOW}⚠️  SSH Connection Detected!${RESET}"
    echo ""
    echo "If the URLs below don't work, forward ports on your local machine:"
    ssh_ports="-L ${FOXGLOVE_BRIDGE_PORT}:localhost:${FOXGLOVE_BRIDGE_PORT} -L ${LOG_VIEWER__PORT}:localhost:${LOG_VIEWER__PORT}"
    if [[ "${PYGAME_HUD_ENABLED:-}" == "true" ]]; then
      ssh_ports="${ssh_ports} -L ${PYGAME_HUD_PORT}:localhost:${PYGAME_HUD_PORT}"
    fi
    echo -e "  ${CYAN}ssh ${ssh_ports} <host>${RESET}"
    echo ""
  fi

  # Get list of running containers (match both project prefix and log_viewer)
  mapfile -t containers < <(docker ps --format "{{.Names}}" | grep -E "(^${COMPOSE_PROJECT_NAME}-|^watod_log_viewer)" | sort)

  # Display status panel
  echo ""
  echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${RESET}"
  echo -e "${BLUE}║${RESET}  ${BOLD}WATonomous Services Started${RESET}                              ${BLUE}║${RESET}"
  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  echo -e "${BLUE}║${RESET}  ${GREEN}Running Containers:${RESET}                                      ${BLUE}║${RESET}"

  for container in "${containers[@]}"; do
    # Remove project prefix
    service_name="${container#"${COMPOSE_PROJECT_NAME}"-}"
    service_name="${service_name%-[0-9]*}"  # Remove trailing -1, -2, etc.

    # Check if it's a dev container
    if [[ "$service_name" == *"_dev" ]]; then
      # Format: "    - service_name (DevContainer)" padded to 59 chars
      printf "${BLUE}║${RESET}    - ${CYAN}%-37s${RESET} ${RED}(DevContainer)${RESET} ${BLUE}║${RESET}\n" "${service_name}"
    else
      # Format: "    - service_name" padded to 59 chars
      printf "${BLUE}║${RESET}    - ${CYAN}%-52s${RESET} ${BLUE}║${RESET}\n" "${service_name}"
    fi
  done

  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  printf "${BLUE}║${RESET}  ${GREEN}Foxglove:${RESET}    ${CYAN}%-43s${RESET} ${BLUE}║${RESET}\n" "${foxglove_url}"
  printf "${BLUE}║${RESET}  ${GREEN}Log Viewer:${RESET}  ${CYAN}%-43s${RESET} ${BLUE}║${RESET}\n" "${log_viewer_url}"
  # Only show pygame HUD if enabled AND simulation is running
  simulation_running=false
  for container in "${containers[@]}"; do
    if [[ "$container" == *"simulation"* || "$container" == *"carla"* ]]; then
      simulation_running=true
      break
    fi
  done
  if [[ "${PYGAME_HUD_ENABLED:-}" == "true" && "$simulation_running" == "true" ]]; then
    printf "${BLUE}║${RESET}  ${GREEN}Pygame HUD:${RESET}  ${CYAN}%-43s${RESET} ${BLUE}║${RESET}\n" "${pygame_hud_url}"
  fi
  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  echo -e "${BLUE}║${RESET}  ${YELLOW}View container logs at the Log Viewer URL above.${RESET}         ${BLUE}║${RESET}"
  echo -e "${BLUE}║${RESET}  Use ${BOLD}./watod down${RESET} to stop all services.                   ${BLUE}║${RESET}"
  echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${RESET}"
  echo ""
fi
