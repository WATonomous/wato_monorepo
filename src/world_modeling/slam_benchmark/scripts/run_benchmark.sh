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
# Full Eidos vs RTAB-Map benchmark on one bag, then evaluation.
# Runs INSIDE the world_modeling dev container (watcloud_bench.sbatch starts it).
#
# Order per repeat: Eidos (no GPS) -> RTAB-Map -> Eidos (GPS), so every repeat
# finishes a complete comparison set before the next one starts.
# A run whose output already exists is skipped, so resubmitting resumes.
#
# Settings (environment variables, all optional):
#   BAG        bag to replay              (default /ws/bags/ring_road_corrected_0.mcap)
#   PREFIX     output file prefix         (default rr)
#   OSM        lane map for evaluation    (default /opt/watonomous/maps/ring_road.osm)
#   RUNS       repeats per method         (default 3)
#   RATE       bag playback rate          (default 0.5)
#   EVAL_ONLY  set to 1 to skip the runs and only evaluate
#   ONLY       do just this one run (e.g. rr_rtabmap_r2) and skip evaluation;
#              used by the parallel job array

BAG=${BAG:-/ws/bags/ring_road_corrected_0.mcap}
PREFIX=${PREFIX:-rr}
OSM=${OSM:-/opt/watonomous/maps/ring_road.osm}
RUNS=${RUNS:-3}
RATE=${RATE:-0.5}
EVAL_ONLY=${EVAL_ONLY:-0}
ONLY=${ONLY:-}

B=/opt/watonomous/maps/bench
LOG=$B/logs
S=/ws/src/world_modeling/slam_benchmark/scripts
TOPICS=(/lidar_cc/velodyne_points /novatel/oem7/imu/data /novatel/oem7/fix /tf_static)
NODE_PATTERN='eidos_node|wato_lifecycle_manager_node|rtabmap'

say() { echo "[$(date +%H:%M:%S)] $*"; }
if ! mkdir -p "$B" "$LOG" "$B/results" || ! touch "$LOG/.write_test"; then
  say "Cannot write to $B from inside the container (rootless Docker maps your user to a different ID)."
  say "Fix: on the login node run   chmod -R a+rwX ~/wato_monorepo/maps   then resubmit."
  exit 1
fi
rm -f "$LOG/.write_test"

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
  pkill -INT -f "$NODE_PATTERN" 2> /dev/null
  sleep 3
  pkill -KILL -f "$NODE_PATTERN" 2> /dev/null
}

play_bag() {
  ros2 bag play "$BAG" --clock --rate "$RATE" --disable-keyboard-controls \
    --topics "${TOPICS[@]}" > "$LOG/$1.play.log" 2>&1 < /dev/null
}

eidos_run() {
  local name=$1 gps=$2
  if [ -s "$B/$name.csv" ]; then
    say "$name: already done, skipping"
    return
  fi
  say "$name: starting Eidos (use_gps:=$gps)"
  ros2 launch slam_benchmark eidos_bench.launch.yaml use_gps:="$gps" \
    > "$LOG/$name.launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "Eidos save_map service" \
    "ros2 service list --no-daemon | grep -q /world_modeling/slam/save_map"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $LOG/$name.launch.log"
    return
  fi
  sleep 5
  ros2 topic echo /world_modeling/slam/status --field state \
    > "$LOG/$name.status.log" 2>&1 < /dev/null &
  local epid=$!

  say "$name: playing bag at rate $RATE"
  play_bag "$name"
  sleep 15
  kill "$epid" 2> /dev/null
  say "$name: SLAM states seen, count then state (3 = TRACKING): $(grep -E '^[0-9]+$' "$LOG/$name.status.log" | sort | uniq -c | tr -s ' \n' ' ')"

  timeout 300 ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap \
    "{filepath: '$B/$name.map'}" > "$LOG/$name.save.log" 2>&1
  stop_launch "$lpid"

  if [ -s "$B/$name.map" ] && eidos poses "$B/$name.map" > "$B/$name.csv" 2>> "$LOG/$name.save.log" \
    && [ -s "$B/$name.csv" ]; then
    say "$name: done ($(wc -l < "$B/$name.csv") lines in $name.csv)"
  else
    rm -f "$B/$name.csv"
    say "$name: FAILED, see $LOG/$name.*.log"
  fi
}

rtabmap_run() {
  local name=$1
  if [ -s "$B/${name}_poses.txt" ]; then
    say "$name: already done, skipping"
    return
  fi
  say "$name: starting RTAB-Map"
  rm -f ~/.ros/rtabmap.db
  ros2 launch rtabmap_examples lidar3d.launch.py use_sim_time:=true \
    lidar_topic:=/lidar_cc/velodyne_points frame_id:=base_footprint \
    imu_topic:=/novatel/oem7/imu/data voxel_size:=0.5 \
    > "$LOG/$name.launch.log" 2>&1 < /dev/null &
  local lpid=$!
  if ! wait_for "RTAB-Map node" "ros2 node list --no-daemon | grep -q rtabmap"; then
    stop_launch "$lpid"
    say "$name: FAILED to start, see $LOG/$name.launch.log"
    return
  fi
  sleep 10

  say "$name: playing bag at rate $RATE"
  play_bag "$name"
  sleep 15
  stop_launch "$lpid"  # RTAB-Map writes its database on shutdown

  if [ -s ~/.ros/rtabmap.db ]; then
    cp ~/.ros/rtabmap.db "$B/$name.db"
    (cd "$B" && rtabmap-export --poses --poses_format 10 "$name.db") > "$LOG/$name.export.log" 2>&1
  fi
  if [ -s "$B/${name}_poses.txt" ]; then
    say "$name: done ($(wc -l < "$B/${name}_poses.txt") poses)"
  else
    say "$name: FAILED, see $LOG/$name.*.log"
  fi
}

evaluate() {
  say "Evaluating"
  local venv=/tmp/slam-bench-venv
  local py=$venv/bin/python
  if [ ! -x "$py" ]; then
    if ! python3 -m venv --system-site-packages "$venv" > "$LOG/venv.log" 2>&1; then
      sudo apt-get update -qq >> "$LOG/venv.log" 2>&1
      sudo apt-get install -y -qq python3-venv >> "$LOG/venv.log" 2>&1
      python3 -m venv --system-site-packages "$venv" >> "$LOG/venv.log" 2>&1
    fi
    if ! "$venv/bin/pip" install -q evo pyproj >> "$LOG/venv.log" 2>&1; then
      say "Python setup FAILED, see $LOG/venv.log"
      return
    fi
  fi

  local gnss=$B/${PREFIX}_gnss.tum car=$B/${PREFIX}_eidos_car.tum
  if [ ! -s "$gnss" ]; then
    say "Converting GNSS reference (reads the whole bag, can take a few minutes)"
    "$py" "$S/to_tum.py" gnss "$BAG" -o "$gnss" 2>&1 | tee "$LOG/gnss.log"
  fi
  if [ ! -s "$car" ]; then
    "$py" "$S/to_tum.py" pose "$BAG" --topic /world_modeling/slam/pose -o "$car" > "$LOG/car_pose.log" 2>&1 \
      || say "No live Eidos pose in this bag; skipping the car row"
  fi
  if [ ! -s "$gnss" ]; then
    say "GNSS conversion FAILED, see $LOG/gnss.log"
    return
  fi

  local osm_args=()
  if [ -f "$OSM" ]; then osm_args=(--osm "$OSM"); else say "Lane map $OSM not found; scoring ATE only"; fi

  for r in $(seq "$RUNS"); do
    local est=() m label csv
    for m in nogps gps; do
      csv=$B/${PREFIX}_eidos_${m}_r$r.csv
      [ "$m" = gps ] && label="Eidos (GPS)" || label="Eidos (no GPS)"
      if [ -s "$csv" ] && "$py" "$S/to_tum.py" eidos "$csv" -o "${csv%.csv}.tum" > /dev/null; then
        est+=(--est "$label=${csv%.csv}.tum")
      fi
    done
    [ -s "$B/${PREFIX}_rtabmap_r${r}_poses.txt" ] && est+=(--est "RTAB-Map=$B/${PREFIX}_rtabmap_r${r}_poses.txt")
    if [ ${#est[@]} -eq 0 ]; then continue; fi
    [ -s "$car" ] && est+=(--est "Eidos (car, live)=$car")

    say "Scoring repeat $r"
    "$py" "$S/evaluate.py" --ref "$gnss" "${est[@]}" "${osm_args[@]}" \
      --out "$B/results/${PREFIX}_r$r" 2>&1 | tee "$LOG/eval_r$r.log"
  done

  # Mean +- spread over the repeats
  "$py" - "$B/results" "$PREFIX" << 'EOF' | tee "$B/results/${PREFIX}_summary.md"
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
cols = ["ate_rms", "ate_max", "lane_rms", "lane_p95"]
print(f"\nSummary over {len(files)} repeat(s): mean ± std, metres\n")
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
if ! colcon build > "$LOG/colcon_build.log" 2>&1; then
  say "colcon build FAILED, see $LOG/colcon_build.log"
  exit 1
fi
source /ws/install/setup.bash

want() { [ -z "$ONLY" ] || [ "$ONLY" = "$1" ]; }

say "Bag: $BAG | runs per method: $RUNS | rate: $RATE${ONLY:+ | only: $ONLY}"
if [ "$EVAL_ONLY" != 1 ]; then
  for r in $(seq "$RUNS"); do
    want "${PREFIX}_eidos_nogps_r$r" && eidos_run "${PREFIX}_eidos_nogps_r$r" false
    want "${PREFIX}_rtabmap_r$r" && rtabmap_run "${PREFIX}_rtabmap_r$r"
    want "${PREFIX}_eidos_gps_r$r" && eidos_run "${PREFIX}_eidos_gps_r$r" true
  done
fi
if [ -n "$ONLY" ]; then
  say "Single run finished; evaluation runs in its own job"
  exit 0
fi
evaluate
say "All done. Results: maps/bench/results/   Logs: maps/bench/logs/"
