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
# run_benchmark.sh: the bags have no GNSS topic, so Eidos runs LiDAR-inertial only, and the
# reference comes from the sequence's applanix/gps_post_process.csv.
#
# Order per repeat: Eidos -> RTAB-Map. A run whose output already exists is skipped,
# so rerunning resumes.
#
# Settings (environment variables, all optional):
#   BAG        bag directory to replay        (default /ws/bags/boreas/bags/boreas-2020-11-26-13-58-lidar-imu)
#   SEQUENCE   Boreas sequence with applanix/ (default /ws/bags/boreas/boreas-2020-11-26-13-58)
#   PREFIX     output file prefix             (default: SEQUENCE's folder name)
#   RUNS       repeats per method             (default 3)
#   RATE       bag playback rate              (default 0.5)
#   METHODS    methods to run                 (default "eidos rtabmap")
#   EVAL_ONLY  set to 1 to skip the runs and only evaluate

SB_BAG=${BAG:-/ws/bags/boreas/bags/boreas-2020-11-26-13-58-lidar-imu}
SB_SEQUENCE=${SEQUENCE:-/ws/bags/boreas/boreas-2020-11-26-13-58}
SB_PREFIX=${PREFIX:-$(basename "$SB_SEQUENCE")}
SB_RUNS=${RUNS:-3}
SB_RATE=${RATE:-0.5}
SB_METHODS=${METHODS:-eidos rtabmap}
SB_EVAL_ONLY=${EVAL_ONLY:-0}

SB_OUT=/opt/watonomous/maps/bench
SB_LOG=$SB_OUT/logs
SB_SCRIPTS=/ws/src/world_modeling/slam_benchmark/scripts
SB_TOPICS=(/boreas/lidar/points /boreas/imu/data /tf_static)
SB_NODE_PATTERN='eidos_node|wato_lifecycle_manager_node|rtabmap'

say() { echo "[$(date +%H:%M:%S)] $*"; }
if ! mkdir -p "$SB_OUT" "$SB_LOG" "$SB_OUT/results" || ! touch "$SB_LOG/.write_test"; then
  say "Cannot write to $SB_OUT from inside the container."
  say "Fix: on the host, create <repo>/maps as your user (or chmod -R a+rwX it) before ./watod up."
  exit 1
fi
rm -f "$SB_LOG/.write_test"

if [ "$SB_EVAL_ONLY" != 1 ] && [ ! -f "$SB_BAG/metadata.yaml" ]; then
  say "No bag at $SB_BAG. The dev container only sees <repo>/bags (as /ws/bags);"
  say "bind-mount the Boreas data under it before ./watod up, see the slam_benchmark README."
  exit 1
fi
if [ ! -f "$SB_SEQUENCE/applanix/gps_post_process.csv" ]; then
  say "No ground truth at $SB_SEQUENCE/applanix/gps_post_process.csv; set SEQUENCE."
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

play_bag() {
  ros2 bag play "$SB_BAG" --clock --rate "$SB_RATE" --disable-keyboard-controls \
    --topics "${SB_TOPICS[@]}" > "$SB_LOG/$1.play.log" 2>&1 < /dev/null
}

eidos_run() {
  local name=$1
  if [ -s "$SB_OUT/$name.csv" ]; then
    say "$name: already done, skipping"
    return
  fi
  say "$name: starting Eidos (LiDAR-inertial)"
  ros2 launch slam_benchmark boreas_eidos_bench.launch.yaml \
    > "$SB_LOG/$name.launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "Eidos save_map service" \
    "ros2 service list --no-daemon | grep -q /world_modeling/slam/save_map"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $SB_LOG/$name.launch.log"
    return
  fi
  sleep 5
  ros2 topic echo /world_modeling/slam/status --field state \
    > "$SB_LOG/$name.status.log" 2>&1 < /dev/null &
  local epid=$!

  say "$name: playing bag at rate $SB_RATE"
  play_bag "$name"
  sleep 15
  kill "$epid" 2> /dev/null
  say "$name: SLAM states seen, count then state (3 = TRACKING): $(grep -E '^[0-9]+$' "$SB_LOG/$name.status.log" | sort | uniq -c | tr -s ' \n' ' ')"

  timeout 300 ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap \
    "{filepath: '$SB_OUT/$name.map'}" > "$SB_LOG/$name.save.log" 2>&1
  stop_launch "$lpid"

  if [ -s "$SB_OUT/$name.map" ] && eidos poses "$SB_OUT/$name.map" > "$SB_OUT/$name.csv" 2>> "$SB_LOG/$name.save.log" \
    && [ -s "$SB_OUT/$name.csv" ]; then
    say "$name: done ($(wc -l < "$SB_OUT/$name.csv") lines in $name.csv)"
  else
    rm -f "$SB_OUT/$name.csv"
    say "$name: FAILED, see $SB_LOG/$name.*.log"
  fi
}

rtabmap_run() {
  local name=$1
  if [ -s "$SB_OUT/${name}_poses.txt" ]; then
    say "$name: already done, skipping"
    return
  fi
  if ! ros2 pkg prefix rtabmap_examples > /dev/null 2>&1; then
    say "$name: RTAB-Map is not installed in this image; rebuild it with ./watod build. Skipping"
    return
  fi
  say "$name: starting RTAB-Map"
  rm -f ~/.ros/rtabmap.db
  ros2 launch rtabmap_examples lidar3d.launch.py use_sim_time:=true \
    lidar_topic:=/boreas/lidar/points frame_id:=boreas_base \
    imu_topic:=/boreas/imu/data voxel_size:=0.5 \
    > "$SB_LOG/$name.launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "RTAB-Map node" "ros2 node list --no-daemon | grep -q rtabmap"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $SB_LOG/$name.launch.log"
    return
  fi
  sleep 10

  say "$name: playing bag at rate $SB_RATE"
  play_bag "$name"
  sleep 15
  stop_launch "$lpid"  # RTAB-Map writes its database on shutdown

  if [ -s ~/.ros/rtabmap.db ]; then
    cp ~/.ros/rtabmap.db "$SB_OUT/$name.db"
    (cd "$SB_OUT" && rtabmap-export --poses --poses_format 10 "$name.db") > "$SB_LOG/$name.export.log" 2>&1
  fi
  if [ -s "$SB_OUT/${name}_poses.txt" ]; then
    say "$name: done ($(wc -l < "$SB_OUT/${name}_poses.txt") poses)"
  else
    say "$name: FAILED, see $SB_LOG/$name.*.log"
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

  local gt=$SB_OUT/${SB_PREFIX}_gt.tum
  if [ ! -s "$gt" ]; then
    say "Converting Applanix ground truth"
    "$py" "$SB_SCRIPTS/boreas_to_tum.py" "$SB_SEQUENCE" -o "$gt" 2>&1 | tee "$SB_LOG/gt.log"
  fi
  if [ ! -s "$gt" ]; then
    say "Ground truth conversion FAILED, see $SB_LOG/gt.log"
    return
  fi

  for r in $(seq "$SB_RUNS"); do
    local est=() csv=$SB_OUT/${SB_PREFIX}_eidos_nogps_r$r.csv
    if [ -s "$csv" ] && "$py" "$SB_SCRIPTS/to_tum.py" eidos "$csv" -o "${csv%.csv}.tum" > /dev/null; then
      est+=(--est "Eidos (no GPS)=${csv%.csv}.tum")
    fi
    [ -s "$SB_OUT/${SB_PREFIX}_rtabmap_r${r}_poses.txt" ] && est+=(--est "RTAB-Map=$SB_OUT/${SB_PREFIX}_rtabmap_r${r}_poses.txt")
    if [ ${#est[@]} -eq 0 ]; then continue; fi

    say "Scoring repeat $r"
    "$py" "$SB_SCRIPTS/evaluate.py" --ref "$gt" "${est[@]}" \
      --out "$SB_OUT/results/${SB_PREFIX}_r$r" 2>&1 | tee "$SB_LOG/eval_r$r.log"
  done

  # Mean +- spread over the repeats
  "$py" - "$SB_OUT/results" "$SB_PREFIX" << 'EOF' | tee "$SB_OUT/results/${SB_PREFIX}_summary.md"
import csv
import glob
import statistics as st
import sys

root, prefix = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"{root}/{prefix}_r*/results.csv"))
rows = {}
for f in files:
    with open(f) as fh:
        for r in csv.DictReader(fh):
            rows.setdefault(r["method"], []).append(r)
cols = ["ate_rms", "ate_mean", "ate_max"]
print(f"\nSummary over {len(files)} repeat(s) vs Applanix ground truth: mean ± std, metres\n")
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

say "Bag: $SB_BAG | runs per method: $SB_RUNS | rate: $SB_RATE | methods: $SB_METHODS"
# Eidos maps keep every keyframe's 128-beam cloud: about 10 GB per run of the full 2020-11-26 drive
free_gb=$(df -P -BG "$SB_OUT" | awk 'NR == 2 {print int($4)}')
if [ "$SB_EVAL_ONLY" != 1 ] && want eidos && [ "$free_gb" -lt $((10 * SB_RUNS)) ]; then
  say "WARNING: only ${free_gb} GB free under maps/, and each Eidos map of the full drive is about 10 GB"
fi
if [ "$SB_EVAL_ONLY" != 1 ]; then
  for r in $(seq "$SB_RUNS"); do
    want eidos && eidos_run "${SB_PREFIX}_eidos_nogps_r$r"
    want rtabmap && rtabmap_run "${SB_PREFIX}_rtabmap_r$r"
  done
fi
evaluate
say "All done. Results: maps/bench/results/   Logs: maps/bench/logs/"
