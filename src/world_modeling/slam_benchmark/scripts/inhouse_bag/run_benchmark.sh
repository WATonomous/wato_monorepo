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
#   PREFIX     output file prefix         (default ringroad)
#   OSM        lane map for evaluation    (default /opt/watonomous/maps/ring_road.osm)
#   RUNS       repeats per method         (default 3)
#   RATE       bag playback rate          (default 0.5)
#   EVAL_ONLY  set to 1 to skip the runs and only evaluate
#   ONLY       do just this one run (e.g. ringroad_rtabmap_r2) and skip evaluation;
#              used by the parallel job array

SB_BAG=${BAG:-/ws/bags/ring_road_corrected_0.mcap}
SB_PREFIX=${PREFIX:-ringroad}
SB_OSM=${OSM:-/opt/watonomous/maps/ring_road.osm}
SB_RUNS=${RUNS:-3}
SB_RATE=${RATE:-0.5}
SB_EVAL_ONLY=${EVAL_ONLY:-0}
SB_ONLY=${ONLY:-}

SB_OUT=/opt/watonomous/maps/bench
SB_LOG=$SB_OUT/logs
SB_SCRIPTS=/ws/src/world_modeling/slam_benchmark/scripts
SB_TOPICS=(/lidar_cc/velodyne_points /novatel/oem7/imu/data /novatel/oem7/fix /tf_static)
SB_NODE_PATTERN='eidos_node|wato_lifecycle_manager_node|rtabmap'

say() { echo "[$(date +%H:%M:%S)] $*"; }
if ! mkdir -p "$SB_OUT" "$SB_LOG" "$SB_OUT/results" || ! touch "$SB_LOG/.write_test"; then
  say "Cannot write to $SB_OUT from inside the container (rootless Docker maps your user to a different ID)."
  say "Fix: on the login node run   chmod -R a+rwX ~/wato_monorepo/maps   then resubmit."
  exit 1
fi
rm -f "$SB_LOG/.write_test"

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
  local name=$1 gps=$2
  if [ -s "$SB_OUT/$name.csv" ]; then
    say "$name: already done, skipping"
    return
  fi
  say "$name: starting Eidos (use_gps:=$gps)"
  ros2 launch slam_benchmark eidos_bench.launch.yaml use_gps:="$gps" \
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
  say "$name: starting RTAB-Map"
  rm -f ~/.ros/rtabmap.db
  ros2 launch rtabmap_examples lidar3d.launch.py use_sim_time:=true \
    lidar_topic:=/lidar_cc/velodyne_points frame_id:=base_footprint \
    imu_topic:=/novatel/oem7/imu/data voxel_size:=0.5 \
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

  local gnss=$SB_OUT/${SB_PREFIX}_gnss.tum car=$SB_OUT/${SB_PREFIX}_eidos_car.tum
  if [ ! -s "$gnss" ]; then
    say "Converting GNSS reference (reads the whole bag, can take a few minutes)"
    "$py" "$SB_SCRIPTS/inhouse_bag/to_tum.py" gnss "$SB_BAG" -o "$gnss" 2>&1 | tee "$SB_LOG/gnss.log"
  fi
  if [ ! -s "$car" ]; then
    "$py" "$SB_SCRIPTS/inhouse_bag/to_tum.py" pose "$SB_BAG" --topic /world_modeling/slam/pose -o "$car" > "$SB_LOG/car_pose.log" 2>&1 \
      || say "No live Eidos pose in this bag; skipping the car row"
  fi
  if [ ! -s "$gnss" ]; then
    say "GNSS conversion FAILED, see $SB_LOG/gnss.log"
    return
  fi

  local osm_args=()
  if [ -f "$SB_OSM" ]; then osm_args=(--osm "$SB_OSM"); else say "Lane map $SB_OSM not found; scoring ATE only"; fi

  for r in $(seq "$SB_RUNS"); do
    local est=() m label csv
    for m in nogps gps; do
      csv=$SB_OUT/${SB_PREFIX}_eidos_${m}_r$r.csv
      [ "$m" = gps ] && label="Eidos (GPS)" || label="Eidos (no GPS)"
      if [ -s "$csv" ] && "$py" "$SB_SCRIPTS/inhouse_bag/to_tum.py" eidos "$csv" -o "${csv%.csv}.tum" > /dev/null; then
        est+=(--est "$label=${csv%.csv}.tum")
      fi
    done
    [ -s "$SB_OUT/${SB_PREFIX}_rtabmap_r${r}_poses.txt" ] && est+=(--est "RTAB-Map=$SB_OUT/${SB_PREFIX}_rtabmap_r${r}_poses.txt")
    if [ ${#est[@]} -eq 0 ]; then continue; fi

    say "Scoring repeat $r"
    "$py" "$SB_SCRIPTS/evaluate.py" --ref "$gnss" "${est[@]}" "${osm_args[@]}" \
      --out "$SB_OUT/results/${SB_PREFIX}_r$r" 2>&1 | tee "$SB_LOG/eval_r$r.log"
  done

  # The car's live Eidos is scored on its own: it covers only part of the drive, and in the
  # same evaluation it would shrink the shared time window for every other method.
  local gps1=$SB_OUT/${SB_PREFIX}_eidos_gps_r1.tum
  if [ -s "$car" ] && [ -s "$gps1" ]; then
    say "Scoring the car's live Eidos against offline Eidos (GPS) r1"
    "$py" "$SB_SCRIPTS/evaluate.py" --ref "$gnss" \
      --est "Eidos (car, live)=$car" --est "Eidos (GPS, offline r1)=$gps1" "${osm_args[@]}" \
      --out "$SB_OUT/results/${SB_PREFIX}_car" 2>&1 | tee "$SB_LOG/eval_car.log"
  fi

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
if ! colcon build > "$SB_LOG/colcon_build.log" 2>&1; then
  say "colcon build FAILED, see $SB_LOG/colcon_build.log"
  exit 1
fi
source /ws/install/setup.bash  # note: this sets generic variables such as PREFIX, hence the SB_ names

want() { [ -z "$SB_ONLY" ] || [ "$SB_ONLY" = "$1" ]; }

say "Bag: $SB_BAG | runs per method: $SB_RUNS | rate: $SB_RATE${SB_ONLY:+ | only: $SB_ONLY}"
if [ "$SB_EVAL_ONLY" != 1 ]; then
  for r in $(seq "$SB_RUNS"); do
    want "${SB_PREFIX}_eidos_nogps_r$r" && eidos_run "${SB_PREFIX}_eidos_nogps_r$r" false
    want "${SB_PREFIX}_rtabmap_r$r" && rtabmap_run "${SB_PREFIX}_rtabmap_r$r"
    want "${SB_PREFIX}_eidos_gps_r$r" && eidos_run "${SB_PREFIX}_eidos_gps_r$r" true
  done
fi
if [ -n "$SB_ONLY" ]; then
  say "Single run finished; evaluation runs in its own job"
  exit 0
fi
evaluate
say "All done. Results: maps/bench/results/   Logs: maps/bench/logs/"
