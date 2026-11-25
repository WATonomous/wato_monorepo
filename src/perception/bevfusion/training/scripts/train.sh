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
# Usage: scripts/train.sh head-only|head-fuser-decoder RUN_NAME [config overrides...]
# Fine-tunes the selected layers in the training container after setup_container.sh.
# Reads the nuScenes data and starting checkpoint; writes checkpoints and logs to $RUNS/RUN_NAME.
#   INIT_CKPT: starting weights (default: published ResNet-50 fusion checkpoint)
#   RESUME=1 continues from $RUNS/RUN_NAME/latest.pth
# The upstream LR (2e-4) is tuned for 8 GPUs x 4 samples. On 1 GPU, scale it down with
# --optimizer.lr (e.g. 2.5e-5 for 1 x 4) and put non-default settings in RUN_NAME.
set -euo pipefail
source "$(dirname "$0")/env.sh"
LAYERS="${1:?head-only or head-fuser-decoder}"; RUN_NAME="${2:?run name}"; shift 2
case "$LAYERS" in
  head-only)          export TRAINABLE="heads.object" ;;
  head-fuser-decoder) export TRAINABLE="fuser,decoder,heads.object" ;;
  *) echo "unknown layer choice: $LAYERS" >&2; exit 1 ;;
esac
RUN_DIR="$RUNS/$RUN_NAME"
ARGS=(--run-dir "$RUN_DIR" --load_from "$(realpath "${INIT_CKPT:-$CB/model/resnet50/bevfusion-det.pth}")")
if [[ "${RESUME:-0}" == 1 && -e "$RUN_DIR/latest.pth" ]]; then
  ARGS+=(--resume_from "$RUN_DIR/latest.pth")
fi
cd "$BF"
torchpack dist-run -np "$NGPUS" python "$TRAINING_DIR/tools/train_frozen.py" "$R50_FUSION_CFG" "${ARGS[@]}" "$@"
