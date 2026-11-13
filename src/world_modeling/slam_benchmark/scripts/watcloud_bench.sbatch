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
#SBATCH --job-name=slam_bench
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --gres=tmpdisk:102400
#SBATCH --time=08:00:00
#SBATCH --output=%x-%j.out
#
# Batch version of Steps 2-6 of the WATcloud SLAM benchmark guide.
# BENCH picks the benchmark (default ringroad):
#   ringroad   inhouse_bag/run_benchmark.sh: Eidos (no GPS), RTAB-Map, Eidos (GPS) on the car's bag
#   boreas     boreas/boreas_benchmark.sh:   Eidos (no GPS), RTAB-Map on a Boreas bag, vs Applanix ground truth
#
# Submit from your home folder so the log lands there:
#   cd ~ && sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
#   cd ~ && BENCH=boreas sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Test with one repeat first:
#   cd ~ && RUNS=1 sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Optional settings, passed through to the benchmark script:
#   ringroad: BAG PREFIX OSM RUNS RATE EVAL_ONLY
#   boreas:   BAG SEQUENCE DATASET RUNS RATE METHODS EVAL_ONLY
# Boreas data, inside ~/wato_monorepo/bags (the only bag folder the container sees, as /ws/bags):
#   bags/boreas-2020-11-26-13-58-lidar-imu/                      the bag
#   bags/boreas-2020-11-26-13-58/applanix/gps_post_process.csv   the ground truth
#
# Parallel: submit as a job array, one test per task (needs the saved images from a normal job first).
# The array size is (tests per repeat) x RUNS: 3 x 3 = 1-9 for ringroad, 2 x 3 = 1-6 for boreas.
#   cd ~ && JOB=$(BENCH=boreas sbatch --parsable --array=1-6%4 --time=01:30:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch)
#   BENCH=boreas EVAL_ONLY=1 sbatch --dependency=afterany:$JOB --cpus-per-task=4 --mem=16G --time=01:00:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Pass the same BENCH (and PREFIX/DATASET, if set) to both commands.
# ringroad tasks: 1-3 = repeat 1 (Eidos no GPS, RTAB-Map, Eidos GPS), 4-6 = repeat 2, 7-9 = repeat 3.
# boreas tasks:   1-2 = test 1 (Eidos no GPS, RTAB-Map), 3-4 = test 2, 5-6 = test 3.
# A full Boreas run replays an 18-minute bag at RATE 0.5 (about 40 min per test), so give each
# Boreas task at least --time=01:30:00 and --mem=32G (Eidos holds its ~10 GB map in RAM).

MONO=$HOME/wato_monorepo
IMAGES=/mnt/wato-drive2/$USER/images/wato_rtabmap.tar
SCRIPTS=/ws/src/world_modeling/slam_benchmark/scripts
say() { echo "[$(date +%H:%M:%S)] $*"; }

BENCH=${BENCH:-ringroad}
case $BENCH in
  ringroad)
    SCRIPT=$SCRIPTS/inhouse_bag/run_benchmark.sh
    # One prefix for the runs, the array task names and the EVAL_ONLY job (run_benchmark.sh's default)
    export PREFIX=${PREFIX:-ringroad}
    TESTS=(eidos_nogps rtabmap eidos_gps)
    PASS=(BAG PREFIX OSM RUNS RATE EVAL_ONLY ONLY)
    ;;
  boreas)
    SCRIPT=$SCRIPTS/boreas/boreas_benchmark.sh
    TESTS=(eidos_nogps rtabmap)
    PASS=(BAG SEQUENCE DATASET RUNS RATE METHODS EVAL_ONLY ONLY)
    ;;
  *)
    say "Unknown BENCH=$BENCH; use ringroad or boreas"
    exit 1
    ;;
esac

say "Job $SLURM_JOB_ID on $(hostname), BENCH=$BENCH"

# Check the Boreas data before spending minutes on Docker: the container sees $MONO/bags as /ws/bags
if [ "$BENCH" = boreas ]; then
  bag=${BAG:-/ws/bags/boreas-2020-11-26-13-58-lidar-imu}
  seq=${SEQUENCE:-/ws/bags/boreas-2020-11-26-13-58}
  if [ "${EVAL_ONLY:-0}" != 1 ] && [ ! -f "$MONO/bags/${bag#/ws/bags/}/metadata.yaml" ]; then
    say "No bag at $MONO/bags/${bag#/ws/bags/} (the container's $bag)."
    say "Fix: copy or hard-link the bag folder into ~/wato_monorepo/bags, see the slam_benchmark README."
    exit 1
  fi
  if [ ! -f "$MONO/bags/${seq#/ws/bags/}/applanix/gps_post_process.csv" ]; then
    say "No ground truth at $MONO/bags/${seq#/ws/bags/}/applanix/gps_post_process.csv, see the slam_benchmark README."
    exit 1
  fi
fi

if [ -n "${SLURM_ARRAY_TASK_ID:-}" ]; then
  # One test per array task
  n=${#TESTS[@]}
  i=$((SLURM_ARRAY_TASK_ID - 1))
  r=$((i / n + 1))
  if [ "$r" -gt "${RUNS:-3}" ]; then
    say "Array task $SLURM_ARRAY_TASK_ID is past RUNS=${RUNS:-3} x $n tests; nothing to do"
    exit 0
  fi
  if [ "$BENCH" = boreas ]; then
    ONLY=test$r/${TESTS[$((i % n))]}
  else
    ONLY=${PREFIX}_${TESTS[$((i % n))]}_r$r
  fi
  export ONLY
  # Two tasks on the same node must not publish the same host ports
  export BASE_PORT=$((20000 + ((SLURM_ARRAY_JOB_ID * 10 + SLURM_ARRAY_TASK_ID) % 2000) * 20))
  say "Array task $SLURM_ARRAY_TASK_ID: $ONLY (ports from $BASE_PORT)"
  if [ ! -f "$IMAGES" ]; then
    say "No saved images at $IMAGES. Run one normal (non-array) job first so it builds and saves them."
    exit 1
  fi
fi
slurm-start-dockerd.sh || { say "Docker failed to start"; exit 1; }
# Batch jobs don't get DOCKER_HOST set like interactive shells do
export DOCKER_HOST=unix:///tmp/run/docker.sock
cd "$MONO" || exit 1

if [ -f "$IMAGES" ]; then
  say "Loading saved Docker images"
  docker load -i "$IMAGES" || exit 1
else
  say "No saved images at $IMAGES; building (slow the first time)"
  ./watod build || { say "watod build FAILED"; exit 1; }
  say "Saving images for later jobs"
  mkdir -p "$(dirname "$IMAGES")"
  # shellcheck disable=SC2046
  docker save $(docker images --format '{{.Repository}}:{{.Tag}}' | grep -E 'world_modeling|infrastructure') -o "$IMAGES"
fi

./watod up || { say "watod up FAILED"; exit 1; }
sleep 15

ENV_ARGS=()
for v in "${PASS[@]}"; do
  [ -n "${!v:-}" ] && ENV_ARGS+=(-e "$v=${!v}")
done

./watod exec -T "${ENV_ARGS[@]}" world_modeling_bringup_dev bash "$SCRIPT"

./watod down
say "Job finished"
