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
# Usage: scripts/eval.sh CHECKPOINT [CONFIG]
# nuScenes detection eval (mAP/NDS) of a training checkpoint on the val split of $NUSC_DIR.
# Example: scripts/eval.sh "$CB/model/resnet50/bevfusion-det.pth" scores NVIDIA's
# published weights (about 67.9 mAP on full val).
set -euo pipefail
source "$(dirname "$0")/env.sh"
CKPT="$(realpath "${1:?checkpoint}")"
CFG="${2:-$R50_FUSION_CFG}"
cd "$BF"
torchpack dist-run -np "$NGPUS" python tools/test.py "$CFG" "$CKPT" --eval bbox
