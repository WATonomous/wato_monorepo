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

# watod-install.sh - Install watod and bash completion for global access

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_BIN_DIR="/usr/local/bin"
WATOD_PATH="$MONO_DIR/watod"
COMPLETION_SRC="$MONO_DIR/watod_scripts/watod_completion.bash"

# Find bash completion directory
COMPLETION_DIR=""
for dir in "/usr/share/bash-completion/completions" "/etc/bash_completion.d"; do
  if [[ -d "$dir" ]]; then
    COMPLETION_DIR="$dir"
    break
  fi
done

echo "Checking watod installation status..."

# Check if both binary and completion are already correctly installed as symlinks
# to the current monorepo paths.
BIN_OK=false
if [[ -L "$INSTALL_BIN_DIR/watod" && "$(readlink "$INSTALL_BIN_DIR/watod")" == "$WATOD_PATH" ]]; then
  BIN_OK=true
fi

COMP_OK=false
if [[ -n "$COMPLETION_DIR" ]]; then
  if [[ -L "$COMPLETION_DIR/watod" && "$(readlink "$COMPLETION_DIR/watod")" == "$COMPLETION_SRC" ]]; then
    COMP_OK=true
  fi
else
  # If no completion directory found, we can't install it, so we treat it as OK to avoid infinite loop
  # but we should warn the user.
  echo "Warning: No bash-completion directory found. Autocomplete will not be installed."
  COMP_OK=true
fi

if [[ "$BIN_OK" == "true" && "$COMP_OK" == "true" ]]; then
  echo "✓ watod and bash completion are already installed and up-to-date for this monorepo."
else
  echo "Installing/Updating watod and bash completion..."
  echo "NOTE: Sudo privileges required for system-wide installation. (Optional)"

  # Install/Update binary
  if [[ -e "$INSTALL_BIN_DIR/watod" && ! -L "$INSTALL_BIN_DIR/watod" ]]; then
    echo "Error: $INSTALL_BIN_DIR/watod exists and is not a symlink. Please remove it manually." >&2
  else
    if sudo ln -sf "$WATOD_PATH" "$INSTALL_BIN_DIR/watod"; then
        echo "✓ watod symlink updated at $INSTALL_BIN_DIR/watod"
    else
        echo "⚠ Failed to update watod symlink (sudo required). You can use 'watod' via local path or alias."
    fi
  fi

  # Install/Update completion
  if [[ -n "$COMPLETION_DIR" ]]; then
    if [[ -e "$COMPLETION_DIR/watod" && ! -L "$COMPLETION_DIR/watod" ]]; then
      echo "Warning: $COMPLETION_DIR/watod exists and is not a symlink. Skipping completion update." >&2
    else
      if sudo ln -sf "$COMPLETION_SRC" "$COMPLETION_DIR/watod"; then
        echo "✓ Bash completion symlink updated at $COMPLETION_DIR/watod"
      else
         echo "⚠ Failed to update system-wide bash completion (sudo required). User-local completion setup in .bashrc is sufficient."
      fi
    fi
  fi
fi

echo ""
echo "Installation check complete!"
echo "You can now run 'watod' from any directory within a monorepo with autocomplete!"
echo "NOTE: You may need to restart your terminal or run 'source ~/.bashrc' for changes to take effect."
