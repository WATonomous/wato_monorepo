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
MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "$1" ]] || [[ "$1" == "all" ]]; then
  MODULE_NAME="infrastructure"
else
  MODULE_NAME=$1
fi

SERVICE_NAME="${MODULE_NAME}_bringup"

# Define run_compose wrapper to use watod script
run_compose() {
    "$MONO_DIR/../watod" -m all "$@"
}

echo "Starting dependency mount using service: $SERVICE_NAME"


TEMP_DEPENDENCES_DIR="/tmp/deps"
DEP_MOUNT_CONTAINER_NAME=dep-mount-temp

mkdir -p "${TEMP_DEPENDENCES_DIR}"

# Dynamically create a Docker Compose service
run_compose run --rm --detach --name "${DEP_MOUNT_CONTAINER_NAME}" "${SERVICE_NAME}" tail -f /dev/null

# Copy dependencies from the service's container to a temporary directory
docker cp "${DEP_MOUNT_CONTAINER_NAME}:/opt/ros" "${TEMP_DEPENDENCES_DIR}"

# Copy Eigen3 headers
mkdir -p "${TEMP_DEPENDENCES_DIR}/usr/include"
docker cp "${DEP_MOUNT_CONTAINER_NAME}:/usr/include/eigen3" "${TEMP_DEPENDENCES_DIR}/usr/include/eigen3"

# Stop the service
docker stop "${DEP_MOUNT_CONTAINER_NAME}"

# Create the .vscode directory if it doesn't exist
# Define the .vscode directory path
VSCODE_DIR="$MONO_DIR/../.vscode"


# Create the .vscode directory if it doesn't exist
mkdir -p "$VSCODE_DIR"

# Write to c_cpp_properties.json
cat << EOF > "$VSCODE_DIR"/c_cpp_properties.json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "\${workspaceFolder}/**",
                "/tmp/deps/ros/jazzy/include/**",
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

# Write to settings.json
cat << EOF > "$VSCODE_DIR"/settings.json
{
    "python.analysis.extraPaths": [
        "/tmp/deps/ros/jazzy/local/lib/python3.10/dist-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/tmp/deps/ros/jazzy/local/lib/python3.10/dist-packages"
    ],
    "[python]": {
        "editor.formatOnSave": true,
        "editor.defaultFormatter": "charliermarsh.ruff"
    },
    "python.analysis.autoSearchPaths": true,
}
EOF

# Write to settings.json
cat << EOF > "$VSCODE_DIR"/extensions.json
{
    "recommendations": [
        "ms-iot.vscode-ros",
        "charliermarsh.ruff"
    ],
}
EOF

echo "Configuration files created successfully in .vscode/"

# Display the message with yellow text and borders
echo -e "\033[1;33m##############################################\033[0m"
echo -e "\033[1;33m#                                            #\033[0m"
echo -e "\033[1;33m#   VScode Dev Environment Set Up!           #\033[0m"
echo -e "\033[1;33m#   Type CMD/CTRL + SHIFT + P > Reload       #\033[0m"
echo -e "\033[1;33m#   Window to activate IntelliSense          #\033[0m"
echo -e "\033[1;33m#                                            #\033[0m"
echo -e "\033[1;33m##############################################\033[0m"
