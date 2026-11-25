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
# Usage: scripts/export.sh CHECKPOINT VERSION
# Runs in the training container after setup_container.sh; exports a checkpoint to
# $RUNS/export/VERSION with five ONNX models and a model card.
# Optional environment: PTQ (1 or 0, default 1), CALIB_BATCHES (300), EXAMPLE_DATA.
set -euo pipefail
source "$(dirname "$0")/env.sh"
CKPT="$(realpath "${1:?checkpoint}")"
VERSION="${2:?version, e.g. r50-nusc-f2-v1}"
OUT="$RUNS/export/$VERSION"
if [[ -e "$OUT" ]]; then
  echo "Export destination already exists: $OUT" >&2
  exit 1
fi

PTQ="${PTQ:-1}"
CALIB_BATCHES="${CALIB_BATCHES:-300}"
cd "$CB"
rm -rf qat/onnx_int8 example-data
# Prepare example data for tracing camera input shapes.
if [[ -n "${EXAMPLE_DATA:-}" ]]; then
  if [[ ! -f "$EXAMPLE_DATA/example-data.pth" ]]; then
    echo "Missing example data: $EXAMPLE_DATA/example-data.pth" >&2
    exit 1
  fi
  ln -sfn "$EXAMPLE_DATA" example-data
else
  mkdir -p example-data
  # export-camera.py only traces shapes; values are unused.
  python - <<'PY'
import torch
from mmcv.parallel import DataContainer

torch.save(
    {
        "img": DataContainer([torch.zeros(1, 6, 3, 256, 704)]),
        "points": DataContainer([[torch.zeros(1, 5)]]),
    },
    "example-data/example-data.pth",
)
PY
fi

# Calibrate with PTQ, or reuse an already calibrated checkpoint.
case "$PTQ" in
  1)
    python qat/ptq.py --config="$R50_FUSION_CFG" --ckpt="$CKPT" --calibrate_batch "$CALIB_BATCHES"
    PTQ_CKPT=qat/ckpt/bevfusion_ptq.pth
    mkdir -p "$RUNS/ptq"
    cp "$PTQ_CKPT" "$RUNS/ptq/$VERSION.pth"
    PTQ_NOTE="yes, $CALIB_BATCHES train batches from $NUSC_DIR; PTQ model: $RUNS/ptq/$VERSION.pth"
    ;;
  0)
    PTQ_CKPT="$CKPT"
    PTQ_NOTE="skipped (checkpoint is already a PTQ model)"
    ;;
  *)
    echo "PTQ must be 1 or 0: $PTQ" >&2
    exit 1
    ;;
esac

# Export camera, fusion, detection and LiDAR ONNX models.
python qat/export-camera.py --ckpt="$PTQ_CKPT"
python qat/export-transfuser.py --ckpt="$PTQ_CKPT"
python qat/export-scn.py --ckpt="$PTQ_CKPT" --save=qat/onnx_int8/lidar.backbone.onnx
# Patch external ONNX I/O to FP16 for TensorRT 11.
python "$TRAINING_DIR/tools/onnx_regen_trt11.py" qat/onnx_int8
# Check the exported I/O against the deployment contract.
python "$TRAINING_DIR/tools/check_onnx_io.py" qat/onnx_int8

# Copy the models and record their provenance in the model card.
mkdir -p "$OUT"
cp qat/onnx_int8/{camera.backbone,camera.vtransform,fuser,head.bbox,lidar.backbone.xyz}.onnx "$OUT/"

# Escape sed replacement metacharacters, including backslashes in supplied values.
sed_value() {
  printf '%s' "$1" | sed 's/[\\&|]/\\&/g'
}
MONOREPO_COMMIT="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
FORK_COMMIT="$(git -C "$CB" rev-parse --short HEAD 2>/dev/null || echo unknown)"
EXPORTED_UTC="$(date -u +%Y-%m-%dT%H:%MZ)"
sed \
  -e "s|{{VERSION}}|$(sed_value "$VERSION")|g" \
  -e "s|{{CHECKPOINT}}|$(sed_value "$CKPT")|g" \
  -e "s|{{PTQ}}|$(sed_value "$PTQ_NOTE")|g" \
  -e "s|{{MONOREPO_COMMIT}}|$(sed_value "$MONOREPO_COMMIT")|g" \
  -e "s|{{FORK_COMMIT}}|$(sed_value "$FORK_COMMIT")|g" \
  -e "s|{{EXPORTED_UTC}}|$(sed_value "$EXPORTED_UTC")|g" \
  "$TRAINING_DIR/MODEL_CARD.md" > "$OUT/MODEL_CARD.md"
ls -l "$OUT"
