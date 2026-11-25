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
# Usage: scripts/prepare_data.sh mini|full
# Run in the training container after setup_container.sh, using the linked nuScenes data.
# Writes nuscenes_infos_*.pkl, nuscenes_dbinfos_train.pkl and the GT database into $NUSC_DIR,
# which must be a writable root laid out by scripts/link_nuscenes.sh.
set -euo pipefail
source "$(dirname "$0")/env.sh"
case "${1:?mini or full}" in
  mini) VERSION_ARGS=(--version v1.0-mini); EXPECTED="323 / 81" ;;
  full) VERSION_ARGS=(); EXPECTED="28130 / 6019" ;;   # default v1.0 = trainval + test
  *) echo "unknown split: $1" >&2; exit 1 ;;
esac
cd "$BF"
python tools/create_data.py nuscenes --root-path ./data/nuscenes \
  --out-dir ./data/nuscenes --extra-tag nuscenes --workers "${WORKERS:-16}" "${VERSION_ARGS[@]}"
echo "expected train / val: $EXPECTED"
python - <<'EOF'
import pickle
for split in ("train", "val"):
    with open(f"data/nuscenes/nuscenes_infos_{split}.pkl", "rb") as f:
        print(split, len(pickle.load(f)["infos"]))
EOF
