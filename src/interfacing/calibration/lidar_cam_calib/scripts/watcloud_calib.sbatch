#!/bin/bash
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
#SBATCH --job-name=lidar_cam_calib
#SBATCH --cpus-per-task=8
#SBATCH --mem=16G
#SBATCH --gres=tmpdisk:51200
#SBATCH --time=04:00:00
#SBATCH --output=%x-%j.out
#
# LiDAR-camera calibration on WATcloud. STEP picks the work:
#   extract    find rest/creep segments in every bag under BAG_DIR, cut them into OUT_DIR
#   calibrate  calibrate cameras; as a job array, task N does the Nth camera of config/calib.yaml
#   summarize  results table and URDF snippet
#   all        the three in one job, cameras one after another
#
# Submit from your home folder so the log lands there. Full pipeline, cameras in parallel:
#   cd ~ && JOB=$(STEP=extract sbatch --parsable ~/wato_monorepo/src/interfacing/calibration/lidar_cam_calib/scripts/watcloud_calib.sbatch)
#   cd ~ && JOB2=$(STEP=calibrate sbatch --parsable --dependency=afterok:$JOB --array=1-6 ~/wato_monorepo/src/interfacing/calibration/lidar_cam_calib/scripts/watcloud_calib.sbatch)
#   cd ~ && STEP=summarize sbatch --dependency=afterany:$JOB2 --cpus-per-task=2 --mem=4G --time=00:20:00 ~/wato_monorepo/src/interfacing/calibration/lidar_cam_calib/scripts/watcloud_calib.sbatch
# Another try on the same segments: pass a new RUN (e.g. RUN=run2) to calibrate and summarize.
#
# Settings (environment variables, all optional):
#   BAG_DIR  bags to search                       (default ~/wato_monorepo/bags)
#   OUT_DIR  segments, previews and results       (default $BAG_DIR/lidar_cam_calib)
#   RUN      name of this batch of results        (default run1)
#   MODE     creep (preprocess -d) or static      (default creep)
#   CAMERA   do only this camera (non-array jobs)
#   FORCE    1 to redo finished work

MONO=$HOME/wato_monorepo
PKG=$MONO/src/interfacing/calibration/lidar_cam_calib
SCRIPTS=/ws/src/interfacing/calibration/lidar_cam_calib/scripts
IMAGE=lidar_cam_calib:latest
IMAGE_TAR=/mnt/wato-drive2/$USER/images/lidar_cam_calib.tar
STEP=${STEP:-all}
RUN=${RUN:-run1}
MODE=${MODE:-creep}
export BAG_DIR=${BAG_DIR:-$MONO/bags}
export OUT_DIR=${OUT_DIR:-$BAG_DIR/lidar_cam_calib}
export IMAGE
# Some nodes (thor-slurm1) set DISPLAY in batch jobs although no X server is reachable;
# clear it so the toolbox's viewer runs under xvfb instead of crashing
unset DISPLAY
say() { echo "[$(date +%H:%M:%S)] $*"; }

FORCE_ARG=()
if [ "${FORCE:-0}" = 1 ]; then
  FORCE_ARG=(--force)
fi

# Camera for this array task: the Nth entry of `cameras:` in calib.yaml
CAMERA_ARG=()
if [ -n "${SLURM_ARRAY_TASK_ID:-}" ] && [ "$STEP" = calibrate ]; then
  mapfile -t CAMS < <(sed -n '/^cameras:/,/^[^ ]/p' "$PKG/config/calib.yaml" | grep -oE 'camera_[a-z_]+')
  CAM=${CAMS[$((SLURM_ARRAY_TASK_ID - 1))]:-}
  if [ -z "$CAM" ]; then
    say "Array task $SLURM_ARRAY_TASK_ID: no camera #$SLURM_ARRAY_TASK_ID in calib.yaml (${#CAMS[@]} listed)"
    exit 0
  fi
  CAMERA_ARG=(--camera "$CAM")
elif [ -n "${CAMERA:-}" ]; then
  CAMERA_ARG=(--camera "$CAMERA")
fi

say "Job ${SLURM_JOB_ID:-local} on $(hostname): STEP=$STEP RUN=$RUN MODE=$MODE ${CAMERA_ARG[*]}"
if [ ! -d "$BAG_DIR" ]; then
  say "No bag folder at $BAG_DIR"
  exit 1
fi
mkdir -p "$OUT_DIR"

slurm-start-dockerd.sh || { say "Docker failed to start"; exit 1; }
# Batch jobs don't get DOCKER_HOST set like interactive shells do
export DOCKER_HOST=unix:///tmp/run/docker.sock

if [ -f "$IMAGE_TAR" ]; then
  say "Loading saved image"
  docker load -i "$IMAGE_TAR" || exit 1
else
  say "No saved image at $IMAGE_TAR; building"
  docker build -t "$IMAGE" "$PKG/docker" || { say "docker build FAILED"; exit 1; }
  # Only one job saves, so array tasks never write the same file
  if [ "$STEP" != calibrate ]; then
    mkdir -p "$(dirname "$IMAGE_TAR")"
    docker save "$IMAGE" -o "$IMAGE_TAR"
  fi
fi

run() { bash "$PKG/scripts/docker_run.sh" "$@"; }

if [ "$STEP" = extract ] || [ "$STEP" = all ]; then
  say "Extracting segments from $BAG_DIR"
  run python3 "$SCRIPTS/extract_segments.py" /bags /out "${FORCE_ARG[@]}" || { say "extract FAILED"; exit 1; }
fi
if [ "$STEP" = calibrate ] || [ "$STEP" = all ]; then
  say "Calibrating (run $RUN, mode $MODE)"
  run python3 "$SCRIPTS/run_calibration.py" /out --run "$RUN" --mode "$MODE" "${CAMERA_ARG[@]}" "${FORCE_ARG[@]}" \
    || say "calibrate reported failures, see the logs under $OUT_DIR/calib/$RUN"
fi
if [ "$STEP" = summarize ] || [ "$STEP" = all ]; then
  run python3 "$SCRIPTS/summarize_results.py" /out --run "$RUN" || { say "summarize FAILED"; exit 1; }
fi
say "Job finished; results under $OUT_DIR"
