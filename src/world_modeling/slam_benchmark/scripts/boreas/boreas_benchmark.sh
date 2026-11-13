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
#
# Eidos vs RTAB-Map on a Boreas bag, scored against the Applanix post-processed ground truth.
# Runs INSIDE the world_modeling dev container. This is the Boreas counterpart of
# inhouse_bag/run_benchmark.sh: the bags have no GNSS topic, so Eidos runs LiDAR-inertial only
# (there is no eidos/ GPS test and no car/ comparison), and the reference comes from the
# sequence's applanix/gps_post_process.csv.
#
# Order per test: Eidos (no GPS) -> RTAB-Map. A test whose output already exists is skipped,
# so rerunning resumes.
#
# Output layout, under maps/bench/results/<DATASET>/ (= /opt/watonomous/maps/bench/results/<DATASET> here):
#   test1/eidos_nogps/   eidos.map, poses.csv, poses.tum, launch/play/save/status logs
#   test1/rtabmap/       rtabmap.db, poses.txt, launch/play/export logs
#   test1/               results.csv, trajectories.png, errors.png, eval.log
#   test2/ test3/        same as test1
#   reference/           gnss.tum (the Applanix ground truth scored against) + gnss.log
#   logs/                build and Python setup logs
#   summary.md           mean +- spread over all tests
#
# Settings (environment variables, all optional):
#   DATASET    output folder name             (default boreas; PREFIX is accepted too)
#   BAG        bag directory to replay        (default /ws/bags/boreas-2020-11-26-13-58-lidar-imu)
#   SEQUENCE   Boreas sequence with applanix/ (default /ws/bags/boreas-2020-11-26-13-58)
#   RUNS       tests per method               (default 3)
#   RATE       bag playback rate              (default 0.5)
#   METHODS    methods to run                 (default "eidos_nogps rtabmap")
#   EVAL_ONLY  set to 1 to skip the runs and only evaluate
#   ONLY       do just one test, e.g. test2/rtabmap, and skip evaluation (parallel job array)

SB_DATASET=${DATASET:-${PREFIX:-boreas}}
SB_BAG=${BAG:-/ws/bags/boreas-2020-11-26-13-58-lidar-imu}
SB_SEQUENCE=${SEQUENCE:-/ws/bags/boreas-2020-11-26-13-58}
SB_RUNS=${RUNS:-3}
SB_RATE=${RATE:-0.5}
SB_METHODS=${METHODS:-eidos_nogps rtabmap}
SB_EVAL_ONLY=${EVAL_ONLY:-0}
SB_ONLY=${ONLY:-}

SB_DS=/opt/watonomous/maps/bench/results/$SB_DATASET
SB_REFDIR=$SB_DS/reference
SB_LOG=$SB_DS/logs
SB_SCRIPTS=/ws/src/world_modeling/slam_benchmark/scripts
SB_TOPICS=(/boreas/lidar/points /boreas/imu/data /tf_static)
SB_NODE_PATTERN='eidos_node|wato_lifecycle_manager_node|rtabmap'

say() { echo "[$(date +%H:%M:%S)] $*"; }
if ! mkdir -p "$SB_DS" "$SB_REFDIR" "$SB_LOG" || ! touch "$SB_LOG/.write_test"; then
  say "Cannot write to $SB_DS from inside the container (rootless Docker maps your user to a different ID)."
  say "Fix: on the login node run   chmod -R a+rwX ~/wato_monorepo/maps   then resubmit."
  exit 1
fi
rm -f "$SB_LOG/.write_test"

# The container sees only ~/wato_monorepo/bags, as /ws/bags (see the slam_benchmark README)
if [ "$SB_EVAL_ONLY" != 1 ] && [ ! -f "$SB_BAG/metadata.yaml" ]; then
  say "No bag at $SB_BAG. Put the bag folder in ~/wato_monorepo/bags, or set BAG."
  exit 1
fi
if [ ! -f "$SB_SEQUENCE/applanix/gps_post_process.csv" ]; then
  say "No ground truth at $SB_SEQUENCE/applanix/gps_post_process.csv; copy it there or set SEQUENCE."
  exit 1
fi

# Wait up to 90 s for a command to succeed.
wait_for() {
  local what=$1 cmd=$2
  for _ in $(seq 45); do
    if eval "$cmd" > /dev/null 2>&1; then return 0; fi
    sleep 2
  done
  say "Timed out waiting for $what"
  return 1
}

# Stop a launch process and any nodes it leaves behind.
stop_launch() {
  local pid=$1
  kill -INT "$pid" 2> /dev/null
  for _ in $(seq 60); do
    kill -0 "$pid" 2> /dev/null || break
    sleep 1
  done
  kill -KILL "$pid" 2> /dev/null
  wait "$pid" 2> /dev/null
  pkill -INT -f "$SB_NODE_PATTERN" 2> /dev/null
  sleep 3
  pkill -KILL -f "$SB_NODE_PATTERN" 2> /dev/null
}

play_bag() {  # $1 = method folder
  ros2 bag play "$SB_BAG" --clock --rate "$SB_RATE" --disable-keyboard-controls \
    --topics "${SB_TOPICS[@]}" > "$1/play.log" 2>&1 < /dev/null
}

eidos_run() {  # $1 = test number
  local name=test$1/eidos_nogps
  local dir=$SB_DS/$name
  if [ -s "$dir/poses.csv" ]; then
    say "$name: already done, skipping"
    return
  fi
  mkdir -p "$dir"
  say "$name: starting Eidos (LiDAR-inertial)"
  ros2 launch slam_benchmark boreas_eidos_bench.launch.yaml \
    > "$dir/launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "Eidos save_map service" \
    "ros2 service list --no-daemon | grep -q /world_modeling/slam/save_map"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $dir/launch.log"
    return
  fi
  sleep 5
  # Typed, so echo doesn't give up when it starts before discovery has seen the topic
  ros2 topic echo /world_modeling/slam/status eidos_msgs/msg/SlamStatus --field state \
    > "$dir/status.log" 2>&1 < /dev/null &
  local epid=$!

  say "$name: playing bag at rate $SB_RATE"
  play_bag "$dir"
  sleep 15
  kill "$epid" 2> /dev/null
  say "$name: SLAM states seen, count then state (3 = TRACKING): $(grep -E '^[0-9]+$' "$dir/status.log" | sort | uniq -c | tr -s ' \n' ' ')"

  timeout 300 ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap \
    "{filepath: '$dir/eidos.map'}" > "$dir/save.log" 2>&1
  stop_launch "$lpid"

  if [ -s "$dir/eidos.map" ] && eidos poses "$dir/eidos.map" > "$dir/poses.csv" 2>> "$dir/save.log" \
    && [ -s "$dir/poses.csv" ]; then
    say "$name: done ($(wc -l < "$dir/poses.csv") lines in poses.csv)"
  else
    rm -f "$dir/poses.csv"
    say "$name: FAILED, see the logs in $dir"
  fi
}

rtabmap_run() {  # $1 = test number
  local name=test$1/rtabmap
  local dir=$SB_DS/$name
  if [ -s "$dir/poses.txt" ]; then
    say "$name: already done, skipping"
    return
  fi
  if ! ros2 pkg prefix rtabmap_examples > /dev/null 2>&1; then
    say "$name: RTAB-Map is not installed in this image; rebuild it with ./watod build. Skipping"
    return
  fi
  mkdir -p "$dir"
  say "$name: starting RTAB-Map"
  rm -f ~/.ros/rtabmap.db
  ros2 launch rtabmap_examples lidar3d.launch.py use_sim_time:=true \
    lidar_topic:=/boreas/lidar/points frame_id:=boreas_base \
    imu_topic:=/boreas/imu/data voxel_size:=0.5 \
    > "$dir/launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "RTAB-Map node" "ros2 node list --no-daemon | grep -q rtabmap"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $dir/launch.log"
    return
  fi
  sleep 10

  say "$name: playing bag at rate $SB_RATE"
  play_bag "$dir"
  sleep 15
  stop_launch "$lpid"  # RTAB-Map writes its database on shutdown

  if [ -s ~/.ros/rtabmap.db ]; then
    cp ~/.ros/rtabmap.db "$dir/rtabmap.db"
    # rtabmap-export names its output <database>_poses.txt
    (cd "$dir" && rtabmap-export --poses --poses_format 10 rtabmap.db) > "$dir/export.log" 2>&1 \
      && mv -f "$dir/rtabmap_poses.txt" "$dir/poses.txt" 2>> "$dir/export.log"
  fi
  if [ -s "$dir/poses.txt" ]; then
    say "$name: done ($(wc -l < "$dir/poses.txt") poses)"
  else
    say "$name: FAILED, see the logs in $dir"
  fi
}

evaluate() {
  say "Evaluating"
  local venv=/tmp/slam-bench-venv
  local py=$venv/bin/python
  if [ ! -x "$py" ]; then
    if ! python3 -m venv --system-site-packages "$venv" > "$SB_LOG/venv.log" 2>&1; then
      {
        sudo apt-get update -qq
        sudo apt-get install -y -qq python3-venv
        python3 -m venv --system-site-packages "$venv"
      } >> "$SB_LOG/venv.log" 2>&1
    fi
    if ! "$venv/bin/pip" install -q evo pyproj >> "$SB_LOG/venv.log" 2>&1; then
      say "Python setup FAILED, see $SB_LOG/venv.log"
      return
    fi
  fi

  local ref=$SB_REFDIR/gnss.tum
  if [ ! -s "$ref" ]; then
    say "Converting Applanix ground truth"
    "$py" "$SB_SCRIPTS/boreas/boreas_to_tum.py" "$SB_SEQUENCE" -o "$ref" 2>&1 | tee "$SB_REFDIR/gnss.log"
  fi
  if [ ! -s "$ref" ]; then
    say "Ground truth conversion FAILED, see $SB_REFDIR/gnss.log"
    return
  fi

  for r in $(seq "$SB_RUNS"); do
    local est=() csv=$SB_DS/test$r/eidos_nogps/poses.csv
    if [ -s "$csv" ] && "$py" "$SB_SCRIPTS/inhouse_bag/to_tum.py" eidos "$csv" -o "${csv%.csv}.tum" > /dev/null; then
      est+=(--est "Eidos (no GPS)=${csv%.csv}.tum")
    fi
    [ -s "$SB_DS/test$r/rtabmap/poses.txt" ] && est+=(--est "RTAB-Map=$SB_DS/test$r/rtabmap/poses.txt")
    if [ ${#est[@]} -eq 0 ]; then continue; fi

    say "Scoring test $r"
    "$py" "$SB_SCRIPTS/evaluate.py" --ref "$ref" "${est[@]}" \
      --out "$SB_DS/test$r" 2>&1 | tee "$SB_DS/test$r/eval.log"
  done

  # Mean +- spread over the tests
  "$py" - "$SB_DS" << 'EOF' | tee "$SB_DS/summary.md"
import csv
import glob
import statistics as st
import sys

root = sys.argv[1]
files = sorted(glob.glob(f"{root}/test*/results.csv"))
rows = {}
for f in files:
    with open(f) as fh:
        for r in csv.DictReader(fh):
            rows.setdefault(r["method"], []).append(r)
cols = ["ate_rms", "ate_mean", "ate_max"]
print(f"\nSummary over {len(files)} test(s) vs Applanix ground truth: mean ± std, metres\n")
print("| method | n | " + " | ".join(cols) + " |")
print("|---|---|" + "---|" * len(cols))
for method, rs in rows.items():
    cells = []
    for c in cols:
        v = [float(r[c]) for r in rs if r.get(c)]
        if not v:
            cells.append("-")
        elif len(v) == 1:
            cells.append(f"{v[0]:.3f}")
        else:
            cells.append(f"{st.mean(v):.3f} ± {st.stdev(v):.3f}")
    print(f"| {method} | {len(rs)} | " + " | ".join(cells) + " |")
EOF
}

# ------------------------------------------------------------------ main
source /opt/ros/jazzy/setup.bash
cd /ws || exit 1
say "Building workspace"
if ! colcon build > "$SB_LOG/colcon_build.log" 2>&1; then
  say "colcon build FAILED, see $SB_LOG/colcon_build.log"
  exit 1
fi
source /ws/install/setup.bash  # note: this sets generic variables such as PREFIX, hence the SB_ names

want() { [[ " $SB_METHODS " == *" $1 "* ]]; }
# $1 = method, $2 = test number
run_this() { want "$1" && { [ -z "$SB_ONLY" ] || [ "$SB_ONLY" = "test$2/$1" ]; }; }

say "Dataset: $SB_DATASET | bag: $SB_BAG | tests per method: $SB_RUNS | rate: $SB_RATE | methods: $SB_METHODS${SB_ONLY:+ | only: $SB_ONLY}"
# Eidos maps keep every keyframe's 128-beam cloud: about 10 GB per test of the full 2020-11-26 drive
free_gb=$(df -P -BG "$SB_DS" | awk 'NR == 2 {print int($4)}')
if [ "$SB_EVAL_ONLY" != 1 ] && want eidos_nogps && [ "$free_gb" -lt $((10 * SB_RUNS)) ]; then
  say "WARNING: only ${free_gb} GB free under maps/, and each Eidos map of the full drive is about 10 GB"
fi
if [ "$SB_EVAL_ONLY" != 1 ]; then
  for r in $(seq "$SB_RUNS"); do
    run_this eidos_nogps "$r" && eidos_run "$r"
    run_this rtabmap "$r" && rtabmap_run "$r"
  done
fi
if [ -n "$SB_ONLY" ]; then
  say "Single test finished; evaluation runs in its own job"
  exit 0
fi
evaluate
say "All done. Everything is in maps/bench/results/$SB_DATASET/"
