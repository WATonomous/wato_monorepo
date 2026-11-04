#!/bin/bash
#SBATCH --job-name=slam_bench
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --gres=tmpdisk:102400
#SBATCH --time=08:00:00
#SBATCH --output=%x-%j.out
#
# Batch version of Steps 2-6 of the WATcloud SLAM benchmark guide.
# Submit from your home folder so the log lands there:
#   cd ~ && sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Test with one repeat first:
#   cd ~ && RUNS=1 sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Optional settings (passed through to run_benchmark.sh): BAG PREFIX OSM RUNS RATE EVAL_ONLY
#
# Parallel: submit as a job array, one run per task (needs the saved images from a normal job first):
#   cd ~ && JOB=$(sbatch --parsable --array=1-9%4 --time=01:30:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch)
#   EVAL_ONLY=1 sbatch --dependency=afterany:$JOB --cpus-per-task=4 --mem=16G --time=01:00:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
# Tasks 1-3 = repeat 1 (Eidos no GPS, RTAB-Map, Eidos GPS), 4-6 = repeat 2, 7-9 = repeat 3.

MONO=$HOME/wato_monorepo
IMAGES=/mnt/wato-drive2/$USER/images/wato_rtabmap.tar
say() { echo "[$(date +%H:%M:%S)] $*"; }

say "Job $SLURM_JOB_ID on $(hostname)"

if [ -n "${SLURM_ARRAY_TASK_ID:-}" ]; then
  # One run per array task
  i=$((SLURM_ARRAY_TASK_ID - 1))
  r=$((i / 3 + 1))
  case $((i % 3)) in
    0) ONLY=${PREFIX:-rr}_eidos_nogps_r$r ;;
    1) ONLY=${PREFIX:-rr}_rtabmap_r$r ;;
    2) ONLY=${PREFIX:-rr}_eidos_gps_r$r ;;
  esac
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
for v in BAG PREFIX OSM RUNS RATE EVAL_ONLY ONLY; do
  [ -n "${!v:-}" ] && ENV_ARGS+=(-e "$v=${!v}")
done

./watod exec -T "${ENV_ARGS[@]}" world_modeling_bringup_dev \
  bash /ws/src/world_modeling/slam_benchmark/scripts/run_benchmark.sh

./watod down
say "Job finished"
