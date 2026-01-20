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

echo "Checking watod installation..."

# 1. Install/Update watod binary symlink
if [[ -L "$INSTALL_BIN_DIR/watod" ]]; then
  echo "Notice: watod symlink already exists at $INSTALL_BIN_DIR/watod. Updating."
  sudo ln -sf "$WATOD_PATH" "$INSTALL_BIN_DIR/watod"
elif [[ -e "$INSTALL_BIN_DIR/watod" ]]; then
  echo "Notice: watod already exists at $INSTALL_BIN_DIR/watod. Skipping binary symlink creation."
else
  sudo ln -s "$WATOD_PATH" "$INSTALL_BIN_DIR/watod"
  echo "✓ watod installed successfully to $INSTALL_BIN_DIR/watod"
fi

# 2. Install/Update bash completion
COMPLETION_SRC="$MONO_DIR/watod_completion.bash"
COMPLETION_INSTALLED=false

echo "Checking bash completion..."

for dir in "/usr/share/bash-completion/completions" "/etc/bash_completion.d"; do
  if [[ -d "$dir" ]]; then
    COMPLETION_DEST="$dir/watod"
    if [[ -L "$COMPLETION_DEST" ]]; then
      echo "Notice: Bash completion symlink already exists. Updating."
      sudo ln -sf "$COMPLETION_SRC" "$COMPLETION_DEST"
    elif [[ -e "$COMPLETION_DEST" ]]; then
      echo "Notice: Bash completion already exists at $COMPLETION_DEST. Skipping."
    else
      sudo ln -s "$COMPLETION_SRC" "$COMPLETION_DEST"
      echo "✓ Bash completion installed to $COMPLETION_DEST"
    fi
    COMPLETION_INSTALLED=true
    break
  fi
done

if [[ "$COMPLETION_INSTALLED" = false ]]; then
  echo "Warning: Could not find bash-completion directory. Autocomplete not installed."
fi

echo ""
echo "You can now run 'watod' from any directory within a monorepo with autocomplete!"
