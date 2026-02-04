#!/bin/bash

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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MONO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Available modules for dependency extraction
AVAILABLE_MODULES=("infrastructure" "interfacing" "perception" "world_modeling" "action" "simulation")

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Temp resources to clean up
TEMP_DEPENDENCES_DIR="/tmp/deps"
DEP_MOUNT_CONTAINER_NAME="dep-mount-temp"

cleanup() {
    # Stop spinner if running
    if [[ -n "${SPINNER_PID:-}" ]]; then
        kill "$SPINNER_PID" 2>/dev/null || true
        tput cnorm 2>/dev/null || true  # Show cursor
    fi
    docker rm -f "${DEP_MOUNT_CONTAINER_NAME}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# Spinner for long-running operations
SPINNER_PID=""
SPINNER_FRAMES=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")

spinner_start() {
    local msg="${1:-Working...}"
    tput civis 2>/dev/null || true  # Hide cursor
    (
        i=0
        while true; do
            printf "\r  ${BLUE}%s${NC} %s" "${SPINNER_FRAMES[$i]}" "$msg"
            i=$(( (i + 1) % ${#SPINNER_FRAMES[@]} ))
            sleep 0.08
        done
    ) &
    SPINNER_PID=$!
}

spinner_stop() {
    local success="${1:-true}"
    local msg="${2:-}"
    if [[ -n "$SPINNER_PID" ]]; then
        kill "$SPINNER_PID" 2>/dev/null || true
        wait "$SPINNER_PID" 2>/dev/null || true
        SPINNER_PID=""
    fi
    printf "\r\033[K"  # Clear line
    tput cnorm 2>/dev/null || true  # Show cursor
    if [[ -n "$msg" ]]; then
        if [[ "$success" == "true" ]]; then
            log_success "$msg"
        else
            log_error "$msg"
        fi
    fi
}

usage() {
    cat << EOF
${BOLD}Usage:${NC} watod -s [MODULE...]

Sets up VSCode development environment by extracting dependencies from built containers.

${BOLD}Available modules:${NC}
  ${AVAILABLE_MODULES[*]}

${BOLD}Examples:${NC}
  watod -s                      # Extract from all modules (default)
  watod -s infrastructure       # Extract from infrastructure only (fastest)
  watod -s world_modeling       # Extract from world_modeling module only
  watod -s perception action    # Extract from multiple modules (merged)

${BOLD}Tip:${NC} Use 'watod -s infrastructure' for the fastest setup with base dependencies.
EOF
    exit 0
}

validate_module() {
    local module="$1"
    for valid in "${AVAILABLE_MODULES[@]}"; do
        [[ "$module" == "$valid" ]] && return 0
    done
    return 1
}

if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
    usage
fi

# Determine which modules to extract from
if [[ -z "${1:-}" ]] || [[ "$1" == "all" ]]; then
    MODULES=("${AVAILABLE_MODULES[@]}")
else
    MODULES=()
    for arg in "$@"; do
        if validate_module "$arg"; then
            MODULES+=("$arg")
        else
            log_error "Unknown module: '$arg'"
            echo -e "Available modules: ${AVAILABLE_MODULES[*]}"
            exit 1
        fi
    done
fi

# Define run_compose wrapper to use watod script
run_compose() {
    "$MONO_DIR/watod" -m all "$@"
}

echo ""
log_info "Setting up dev environment from modules: ${BOLD}${MODULES[*]}${NC}"
echo ""

mkdir -p "${TEMP_DEPENDENCES_DIR}"

# Track progress
TOTAL_MODULES=${#MODULES[@]}
CURRENT_MODULE=0
SUCCESSFUL_MODULES=0

# Extract dependencies from each module
FIRST_MODULE=true
for MODULE_NAME in "${MODULES[@]}"; do
    ((CURRENT_MODULE++)) || true

    echo -e "${BOLD}[$CURRENT_MODULE/$TOTAL_MODULES]${NC} ${BLUE}$MODULE_NAME${NC}"

    # Get the image name from docker compose config
    IMAGE_NAME=$(run_compose config --images 2>/dev/null | grep "${MODULE_NAME}" | head -1)
    if [[ -z "$IMAGE_NAME" ]]; then
        log_warn "  Could not find image, skipping..."
        continue
    fi

    # Check if image exists locally first (avoid slow pulls)
    IMAGE_NAME_MAIN="${IMAGE_NAME%:*}:main"
    if docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo -e "  Using local image"
    elif docker image inspect "$IMAGE_NAME_MAIN" &>/dev/null; then
        echo -e "  Using local main image"
        IMAGE_NAME="$IMAGE_NAME_MAIN"
    else
        # Need to pull - try branch first, then main
        spinner_start "Pulling ${IMAGE_NAME##*/}..."
        if docker pull "$IMAGE_NAME" >/dev/null 2>&1; then
            spinner_stop "true" "Pulled image"
        else
            spinner_stop "false"
            spinner_start "Trying main branch..."
            if docker pull "$IMAGE_NAME_MAIN" >/dev/null 2>&1; then
                spinner_stop "true" "Pulled main image"
                IMAGE_NAME="$IMAGE_NAME_MAIN"
            else
                spinner_stop "false" "Could not pull image, skipping"
                continue
            fi
        fi
    fi

    # Create container directly from image (no build)
    docker rm -f "${DEP_MOUNT_CONTAINER_NAME}" &>/dev/null || true
    docker create --name "${DEP_MOUNT_CONTAINER_NAME}" "$IMAGE_NAME" tail -f /dev/null >/dev/null

    # Copy ROS and system dependencies only from first module (they're identical across images)
    if [[ "$FIRST_MODULE" == true ]]; then
        spinner_start "Copying ROS dependencies..."
        docker cp "${DEP_MOUNT_CONTAINER_NAME}:/opt/ros" "${TEMP_DEPENDENCES_DIR}" >/dev/null 2>&1
        spinner_stop "true" "Copied ROS dependencies"

        mkdir -p "${TEMP_DEPENDENCES_DIR}/usr"
        spinner_start "Copying system headers..."
        docker cp "${DEP_MOUNT_CONTAINER_NAME}:/usr/include" "${TEMP_DEPENDENCES_DIR}/usr/include" >/dev/null 2>&1 || true
        spinner_stop "true" "Copied system headers"
        FIRST_MODULE=false
    fi

    # Copy custom built packages (includes custom message headers)
    spinner_start "Copying custom packages..."
    docker cp "${DEP_MOUNT_CONTAINER_NAME}:/opt/watonomous/." "${TEMP_DEPENDENCES_DIR}/watonomous/" >/dev/null 2>&1 || true
    spinner_stop "true" "Copied custom packages"

    docker rm -f "${DEP_MOUNT_CONTAINER_NAME}" >/dev/null
    ((SUCCESSFUL_MODULES++)) || true
    echo ""
done

if [[ $SUCCESSFUL_MODULES -eq 0 ]]; then
    log_error "No modules were successfully processed!"
    exit 1
fi

# Detect ROS distro and Python version from extracted dependencies
ROS_DISTRO=$(find "${TEMP_DEPENDENCES_DIR}/ros/" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | head -1)
if [[ -z "$ROS_DISTRO" ]]; then
    log_warn "Could not detect ROS distro, defaulting to 'jazzy'"
    ROS_DISTRO="jazzy"
fi

PYTHON_PATH=$(find "${TEMP_DEPENDENCES_DIR}/ros/${ROS_DISTRO}" -type d -name "python3.*" 2>/dev/null | head -1)
if [[ -n "$PYTHON_PATH" ]]; then
    PYTHON_VERSION=$(basename "$PYTHON_PATH")
else
    log_warn "Could not detect Python version, defaulting to 'python3.12'"
    PYTHON_VERSION="python3.12"
fi

log_info "Detected ROS distro: ${BOLD}$ROS_DISTRO${NC}"
log_info "Detected Python: ${BOLD}$PYTHON_VERSION${NC}"

VSCODE_DIR="$MONO_DIR/.vscode"
mkdir -p "$VSCODE_DIR"

# Write c_cpp_properties.json
cat << EOF > "$VSCODE_DIR/c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "\${workspaceFolder}/**",
                "/tmp/deps/ros/${ROS_DISTRO}/include/**",
                "/tmp/deps/watonomous/include/**",
                "/tmp/deps/usr/include/**",
                "/tmp/deps/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
EOF

# Write settings.json
cat << EOF > "$VSCODE_DIR/settings.json"
{
    "python.analysis.extraPaths": [
        "/tmp/deps/ros/${ROS_DISTRO}/local/lib/${PYTHON_VERSION}/dist-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/tmp/deps/ros/${ROS_DISTRO}/local/lib/${PYTHON_VERSION}/dist-packages"
    ],
    "[python]": {
        "editor.formatOnSave": true,
        "editor.defaultFormatter": "charliermarsh.ruff"
    },
    "python.analysis.autoSearchPaths": true
}
EOF

# Write extensions.json
cat << EOF > "$VSCODE_DIR/extensions.json"
{
    "recommendations": [
        "ms-iot.vscode-ros",
        "charliermarsh.ruff"
    ]
}
EOF

log_success "VSCode configuration files created in .vscode/"

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}${BOLD}  ✓ VSCode Dev Environment Ready!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${BOLD}Next step:${NC} Reload VSCode to activate IntelliSense"
echo -e "  Press ${YELLOW}Ctrl+Shift+P${NC} (or ${YELLOW}Cmd+Shift+P${NC}) → ${BOLD}Reload Window${NC}"
echo ""
echo -e "  ${BOLD}Summary:${NC}"
echo -e "    • Processed ${SUCCESSFUL_MODULES}/${TOTAL_MODULES} modules"
echo -e "    • Dependencies extracted to: ${BLUE}/tmp/deps${NC}"
echo -e "    • ROS distro: ${BLUE}${ROS_DISTRO}${NC}"
echo ""
