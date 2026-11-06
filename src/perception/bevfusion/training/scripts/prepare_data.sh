#!/usr/bin/env bash
# Usage: scripts/prepare_data.sh mini|full
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
