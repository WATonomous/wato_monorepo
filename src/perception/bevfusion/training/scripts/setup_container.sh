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
# Usage: scripts/setup_container.sh
# Runs inside the training container at the start of every job, before any other script.
# Connects the image's wato-cuda-bevfusion clone to the weights and data on storage (paths from params.env):
#   1. NVIDIA's published weights:  $SHARED/cuda-bevfusion-model/model    -> $CB/model
#   2. The ResNet-50 configs:       $CB/configs (in the clone)            -> copied into $BF/configs
#   3. The nuScenes data:           $NUSC_DIR                             -> $BF/data/nuscenes, $CB/data/nuscenes
# Changes are made in the container only and disappear when it exits, so this is safe to rerun.
set -euo pipefail
source "$(dirname "$0")/env.sh"

MODEL_SRC="$SHARED/cuda-bevfusion-model"

# Stop early with a clear message if storage is not mounted or not set up yet.
for path in "$MODEL_SRC/model" "$NUSC_DIR"; do
  if [[ ! -d "$path" ]]; then
    echo "missing: $path (is $STORAGE mounted? paths are set in params.env)" >&2
    exit 1
  fi
done

# Replace a path with a symlink. A real directory already there (e.g. from the clone) is moved
# to <path>.orig first, because ln would otherwise put the link inside it.
# Prints: linked <container path> -> <storage path>
link() {
  local target="$1" link_path="$2"
  if [[ -d "$link_path" && ! -L "$link_path" ]]; then
    mv "$link_path" "$link_path.orig"
  fi
  mkdir -p "$(dirname "$link_path")"
  ln -sfn "$target" "$link_path"
  echo "linked $link_path -> $target"
}

# 1. Weights
link "$MODEL_SRC/model" "$CB/model"

# 2. Configs. The ResNet-50 configs ship in the clone at $CB/configs, not in model.zip. NVIDIA's
# qat/README.md step is `cp -r configs bevfusion`: merge them into BEVFusion's own configs folder,
# next to the upstream ones (so copied, not linked).
cp -r "$CB/configs/." "$BF/configs/"
echo "copied $CB/configs into $BF/configs"

# 3. Data. Upstream scripts use ./data/nuscenes relative to $BF, and the qat/ export scripts
# relative to $CB.
link "$NUSC_DIR" "$BF/data/nuscenes"
link "$NUSC_DIR" "$CB/data/nuscenes"

# Check that the files the training and export scripts need are now reachable.
missing=0
for f in "$CB/model/resnet50/bevfusion-det.pth" "$R50_FUSION_CFG"; do
  if [[ -e "$f" ]]; then echo "ok:      $f"; else echo "MISSING: $f" >&2; missing=1; fi
done
exit "$missing"
