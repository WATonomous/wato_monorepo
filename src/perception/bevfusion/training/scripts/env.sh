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
# shellcheck shell=bash
# Shared settings for scripts running in the training container. Source this file:
#   source "$(dirname "$0")/env.sh"
# Machine-specific data, weights and output paths are configured in params.env.
# NGPUS defaults to all GPUs visible to the container and can be overridden.

# Locations derived from this file, so they work wherever the monorepo is mounted
# (/workspace in the job script, or a local checkout).
TRAINING_DIR="${TRAINING_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../params.env
source "$TRAINING_DIR/params.env"
REPO_ROOT="${REPO_ROOT:-$(cd "$TRAINING_DIR/../../../.." && pwd)}"

# Set by ENV in docker/Dockerfile. Missing means the script is running outside the training image.
: "${CB:?CB is not set: run inside the training image (docker/Dockerfile)}"
BF="${BF:-$CB/bevfusion}"

# GPUs: count from CUDA_VISIBLE_DEVICES if set, otherwise from nvidia-smi
if [[ -z "${NGPUS:-}" ]]; then
  if [[ -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
    NGPUS=$(tr ',' '\n' <<<"$CUDA_VISIBLE_DEVICES" | grep -c .)
  else
    NGPUS=$(nvidia-smi -L 2>/dev/null | grep -c '^GPU') || NGPUS=1
  fi
  [[ "$NGPUS" -ge 1 ]] || NGPUS=1
fi

# ResNet-50 camera+LiDAR config, copied from $CB/configs into $BF/configs by setup_container.sh.
# Absolute, so it resolves from any working directory.
R50_FUSION_CFG="${R50_FUSION_CFG:-$BF/configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml}"

export REPO_ROOT TRAINING_DIR CB BF SHARED NUSC_DIR RUNS NGPUS R50_FUSION_CFG
export STORAGE NUSC_MINI_SRC NUSC_FULL_SRC NUSC_MAP_DIR LOAD_RETRIES
