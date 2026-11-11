# Shared paths and settings for the training scripts. Source it, don't run it:
#   source "$(dirname "$0")/env.sh"
# Every variable can be overridden by setting it before sourcing (e.g. with `-e` in job.sbatch).
# Other scripts read these variables and must not hard-code paths. Changes go through Sub-task 1.
#
#   SHARED    project folder on cluster storage   (default: /mnt/wato-drive2/bevfusion_training_image)
#             holds cuda-bevfusion-model/ (NVIDIA's unzipped model.zip), the image tar and runs/
#   NUSC_DIR  writable nuScenes root built by scripts/link_nuscenes.sh
#                                                 (default: $SHARED/nuscenes-mini)
#   RUNS      training/eval/export outputs        (default: $SHARED/runs)
#   NGPUS     GPUs for torchpack dist-run         (default: all GPUs visible to the container)

# Locations derived from this file, so they work wherever the monorepo is mounted
# (/workspace in job.sbatch, or a local checkout).
TRAINING_DIR="${TRAINING_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
REPO_ROOT="${REPO_ROOT:-$(cd "$TRAINING_DIR/../../../.." && pwd)}"

# Set by ENV in docker/Dockerfile. Missing means the script is running outside the training image.
: "${CB:?CB is not set: run inside the training image (docker/Dockerfile)}"
BF="${BF:-$CB/bevfusion}"

# Cluster storage. job.sbatch must mount /mnt/wato-drive2 into the container at the same path.
SHARED="${SHARED:-/mnt/wato-drive2/bevfusion_training_image}"
NUSC_DIR="${NUSC_DIR:-$SHARED/nuscenes-mini}"   # use $SHARED/nuscenes-full for F1/F2
RUNS="${RUNS:-$SHARED/runs}"

# GPUs: count from CUDA_VISIBLE_DEVICES if set, otherwise from nvidia-smi
if [[ -z "${NGPUS:-}" ]]; then
  if [[ -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
    NGPUS=$(tr ',' '\n' <<<"$CUDA_VISIBLE_DEVICES" | grep -c .)
  else
    NGPUS=$(nvidia-smi -L 2>/dev/null | grep -c '^GPU') || NGPUS=1
  fi
  [[ "$NGPUS" -ge 1 ]] || NGPUS=1
fi

# ResNet-50 camera+LiDAR config, copied into $BF/configs from model.zip by setup_container.sh.
# Absolute, so it resolves from any working directory.
R50_FUSION_CFG="${R50_FUSION_CFG:-$BF/configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml}"

export REPO_ROOT TRAINING_DIR CB BF SHARED NUSC_DIR RUNS NGPUS R50_FUSION_CFG
