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

# watod-install.sh - Install watod to ~/.local/bin for global access

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="/usr/local/bin"
WATOD_PATH="$MONO_DIR/watod"

# Check if watod already exists
if [[ -e "$INSTALL_DIR/watod" ]]; then
  echo "Error: watod is already installed at $INSTALL_DIR/watod" >&2
  echo "Remove it first if you want to reinstall." >&2
  exit 1
fi

# Create symlink (requires sudo)
sudo ln -s "$WATOD_PATH" "$INSTALL_DIR/watod"
echo "✓ watod installed successfully to $INSTALL_DIR/watod"
echo ""
echo "You can now run 'watod' from any directory within a monorepo!"
