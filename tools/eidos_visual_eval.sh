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

# Run eidos with the visual plugins over a recorded bag and record what is needed to score them.
#
#   1. Starts perception's map_anything server in the perception dev container (if not running).
#   2. Starts eidos + eidos_transform (sim time) in the world_modeling dev container, with the
#      parameters of eidos_slam.yaml (MODE=slam) or eidos_localization.yaml (MODE=localization)
#      plus the overrides in PARAMS.
#   3. Records the eidos outputs and the plugins' measurement topics.
#   4. Plays the bag's lidar, IMU, GNSS, camera and /tf_static topics from a throwaway container
#      (the bag may live anywhere on the host).
#   5. Optionally saves the map at the end (SAVE_MAP=1).
#
# Usage:   tools/eidos_visual_eval.sh <bag dir or .mcap on the host> <run name>
# Output:  bags/eidos_visual_eval/<run name>/{out/ (rosbag), map.map, eidos.log, play.log, ...}
# Score:   tools/eidos_visual_eval.py
#
# Environment:
#   RATE=0.5            playback rate (the pose service needs about a second per keyframe)
#   MODE=slam           slam | localization
#   PARAMS=<yaml>       eidos parameter overrides (host file), applied last
#   START=0 DURATION=0  play from START seconds for DURATION seconds (0 = to the end)
#   SAVE_MAP=0          1 = call slam/save_map into the run directory when playback ends
#   CHECKPOINT=facebook/map-anything-apache    model the server is started with
#   CAMERAS="camera_pano_nw camera_pano_nn camera_pano_ne"    cameras to play
#   EXTRA_RECORD_TOPICS=""   more topics to record (e.g. /world_modeling/slam/visualization/map)
#   EVAL_DOMAIN_ID=""        ROS domain for the run (default: the stack's), to keep it apart from
#                            anything else playing on the stack's domain
#   TF_STATIC=""             static TF set (tools/tf_static_override.py extract) published instead of
#                            the bag's /tf_static, e.g. with corrected extrinsics
#   GPS_OUTAGE=""            GNSS outage windows "start:end[,start:end]" in s after the bag start:
#                            tools/gps_outage_relay.py republishes /novatel/oem7/fix on /eval/gps_fix
#                            without them. Point gps_factor.gps_topic at /eval/gps_fix in PARAMS.
#
# Prereqs: ./watod -m world_modeling:dev -m perception:dev up, with eidos, eidos_transform and
#          wato_lifecycle_manager built in world_modeling and map_anything built in perception.
set -euo pipefail

BAG="$(realpath "${1:?usage: $0 <bag dir or .mcap> <run name>}")"
RUN="${2:?usage: $0 <bag dir or .mcap> <run name>}"
RATE="${RATE:-0.5}"
MODE="${MODE:-slam}"
PARAMS="${PARAMS:-}"
START="${START:-0}"
DURATION="${DURATION:-0}"
SAVE_MAP="${SAVE_MAP:-0}"
CHECKPOINT="${CHECKPOINT:-facebook/map-anything-apache}"
read -r -a CAMERAS <<<"${CAMERAS:-camera_pano_nw camera_pano_nn camera_pano_ne}"
read -r -a EXTRA_RECORD_TOPICS <<<"${EXTRA_RECORD_TOPICS:-}"

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$MONO_DIR/modules/.env" ]] || { echo "modules/.env not found; run ./watod up first"; exit 1; }
set -a
# shellcheck disable=SC1091
source "$MONO_DIR/modules/.env"
set +a

OUT_HOST="$MONO_DIR/bags/eidos_visual_eval/$RUN"
OUT_WM="/ws/bags/eidos_visual_eval/$RUN"
[[ -e "$BAG" ]] || { echo "Missing $BAG"; exit 1; }
[[ ! -e "$OUT_HOST/out" ]] || { echo "$OUT_HOST/out already exists; remove it or pick another run name"; exit 1; }
case "$MODE" in
  slam) CONFIG=/ws/src/world_modeling/world_modeling_bringup/config/eidos_slam.yaml ;;
  localization) CONFIG=/ws/src/world_modeling/world_modeling_bringup/config/eidos_localization.yaml ;;
  *) echo "MODE must be slam or localization"; exit 1 ;;
esac

container() {  # container <service>: name of the running (dev) container of a module
  docker ps --format '{{.Names}}' | grep -E "^${COMPOSE_PROJECT_NAME}-$1(_dev)?-1$" | head -1 || true
}
WM="$(container world_modeling_bringup)"
PC="$(container perception_bringup)"
[[ -n "$WM" && -n "$PC" ]] || { echo "world_modeling and perception containers must be running"; exit 1; }

mkdir -p "$OUT_HOST"
DOMAIN_ID="${EVAL_DOMAIN_ID:-$ROS_DOMAIN_ID}"
wm_exec() { docker exec -i -e PYTHONUNBUFFERED=1 -e ROS_DOMAIN_ID="$DOMAIN_ID" "$WM" bash -c "source /ws/install/setup.bash && $1"; }
pc_exec() { docker exec -i -e PYTHONUNBUFFERED=1 -e ROS_DOMAIN_ID="$DOMAIN_ID" "$PC" bash -c "source /ws/install/setup.bash && $1"; }

EIDOS_PROCS='lib/eidos/eidos_node|lib/eidos_transform/eidos_transform_node|eidos_lifecycle_manager'
PLAYER="${COMPOSE_PROJECT_NAME}-visual_eval_player"
stop_eidos() {
  docker exec "$WM" pkill -INT -f 'eidos_visual_eval.launch.yaml' 2>/dev/null || true
  for _ in $(seq 20); do
    docker exec "$WM" pgrep -f "$EIDOS_PROCS" >/dev/null 2>&1 || return 0
    sleep 1
  done
  docker exec "$WM" pkill -KILL -f "$EIDOS_PROCS" 2>/dev/null || true
}
cleanup() {
  docker stop "$PLAYER" >/dev/null 2>&1 || true
  docker exec "$WM" pkill -INT -f "tf_static_override.py publish" 2>/dev/null || true
  docker exec "$WM" pkill -INT -f "gps_outage_relay.py" 2>/dev/null || true
  docker exec "$WM" pkill -INT -f "bag record -o $OUT_WM/out" 2>/dev/null || true
  stop_eidos
}
trap cleanup EXIT

# ---- 1. map_anything server ----
if ! docker exec "$PC" pgrep -f 'lib/map_anything/server_node' >/dev/null 2>&1; then
  echo "==> Starting the map_anything server ($CHECKPOINT) in $PC"
  docker exec -d -e ROS_DOMAIN_ID="$DOMAIN_ID" "$PC" bash -c "source /ws/install/setup.bash && ros2 launch map_anything map_anything.launch.yaml \
    checkpoint:=$CHECKPOINT use_sim_time:=true > /tmp/map_anything_server.log 2>&1"
fi
for _ in $(seq 180); do
  docker exec "$PC" grep -q "ready after" /tmp/map_anything_server.log 2>/dev/null && break
  sleep 2
done
docker exec "$PC" grep -q "ready after" /tmp/map_anything_server.log \
  || { echo "map_anything server did not become ready; see /tmp/map_anything_server.log in $PC"; exit 1; }
docker exec "$PC" grep -E "loading|ready after" /tmp/map_anything_server.log | tail -2 | sed 's/^/    /'

# ---- 2. eidos ----
docker exec "$WM" pgrep -f "$EIDOS_PROCS" >/dev/null 2>&1 && { echo "==> Stopping the eidos already running"; stop_eidos; }
docker exec "$WM" mkdir -p /tmp/eidos_visual_eval "$OUT_WM"
EXTRA_FROM=""
if [[ -n "$PARAMS" ]]; then
  docker exec -i "$WM" bash -c 'cat > /tmp/eidos_visual_eval/extra.yaml' <"$PARAMS"
  cp "$PARAMS" "$OUT_HOST/params.yaml"
  EXTRA_FROM="- from: /tmp/eidos_visual_eval/extra.yaml"
fi
# slam_rate and tick_rate are wall timers: scale them so they run at their configured rates in bag
# time. The bag does not start with the car parked, so LISO must not wait for it to be stationary.
wm_exec "python3 - '$CONFIG' '$RATE'" <<'PY'
import sys

import yaml

config, rate = sys.argv[1], float(sys.argv[2])
with open(config) as f:
    cfg = yaml.safe_load(f)
slam_rate = cfg['/**/eidos_node']['ros__parameters']['slam_rate'] * rate
tick_rate = cfg['/**/eidos_transform_node']['ros__parameters']['tick_rate'] * rate
with open('/tmp/eidos_visual_eval/base.yaml', 'w') as f:
    f.write(f'''/**/eidos_node:
  ros__parameters:
    slam_rate: {slam_rate}
    liso_factor:
      initialization:
        warmup_samples: 10
        stationary_gyr_threshold: 1000.0
/**/eidos_transform_node:
  ros__parameters:
    tick_rate: {tick_rate}
''')
PY
docker exec -i "$WM" bash -c 'cat > /tmp/eidos_visual_eval/eidos_visual_eval.launch.yaml' <<EOF
launch:
  - node:
      pkg: eidos_transform
      exec: eidos_transform_node
      name: eidos_transform_node
      namespace: world_modeling
      output: screen
      param:
        - from: $CONFIG
        - from: /tmp/eidos_visual_eval/base.yaml
        $EXTRA_FROM
        - name: use_sim_time
          value: true
  - node:
      pkg: eidos
      exec: eidos_node
      name: eidos_node
      namespace: world_modeling
      output: screen
      param:
        - from: $CONFIG
        - from: /tmp/eidos_visual_eval/base.yaml
        $EXTRA_FROM
        - name: use_sim_time
          value: true
  - node:
      pkg: wato_lifecycle_manager
      exec: wato_lifecycle_manager_node
      name: eidos_lifecycle_manager
      namespace: world_modeling
      output: screen
      param:
        - name: node_names
          value:
            - /world_modeling/eidos_transform_node
            - /world_modeling/eidos_node
        - name: autostart
          value: true
        - name: transition_timeout_s
          value: 60.0
        - name: bond_timeout_s
          value: 4.0
        - name: bond_enabled
          value: false
EOF

echo "==> Starting eidos ($MODE) in $WM (log: $OUT_HOST/eidos.log)"
wm_exec 'exec ros2 launch /tmp/eidos_visual_eval/eidos_visual_eval.launch.yaml' >"$OUT_HOST/eidos.log" 2>&1 &
EIDOS_PID=$!
for _ in $(seq 120); do
  grep -q "WARMING_UP" "$OUT_HOST/eidos.log" && break
  kill -0 "$EIDOS_PID" 2>/dev/null || { echo "eidos exited during startup:"; tail -20 "$OUT_HOST/eidos.log"; exit 1; }
  sleep 1
done
grep -q "WARMING_UP" "$OUT_HOST/eidos.log" || { echo "eidos did not activate; see $OUT_HOST/eidos.log"; exit 1; }

if [[ -n "${TF_STATIC:-}" ]]; then
  echo "==> Publishing the static TF of $(basename "$TF_STATIC") instead of the bag's"
  docker exec "$WM" pkill -INT -f "tf_static_override.py publish" 2>/dev/null || true
  docker exec -i "$WM" bash -c 'cat > /tmp/eidos_visual_eval/tf_static.yaml' <"$TF_STATIC"
  docker exec -i "$WM" bash -c 'cat > /tmp/eidos_visual_eval/tf_static_override.py' <"$MONO_DIR/tools/tf_static_override.py"
  cp "$TF_STATIC" "$OUT_HOST/tf_static.yaml"
  wm_exec 'exec python3 /tmp/eidos_visual_eval/tf_static_override.py publish /tmp/eidos_visual_eval/tf_static.yaml' \
    >"$OUT_HOST/tf_static.log" 2>&1 &
  sleep 3
fi

if [[ -n "${GPS_OUTAGE:-}" ]]; then
  ORIGIN="$(python3 - "$BAG" <<'PY'
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

with AnyReader([Path(sys.argv[1])]) as reader:
    print(f"{reader.start_time / 1e9:.6f}")
PY
)"
  echo "==> GPS outage $GPS_OUTAGE s after the bag start ($ORIGIN): /novatel/oem7/fix -> /eval/gps_fix"
  docker exec "$WM" pkill -INT -f "gps_outage_relay.py" 2>/dev/null || true
  docker exec -i "$WM" bash -c 'cat > /tmp/eidos_visual_eval/gps_outage_relay.py' <"$MONO_DIR/tools/gps_outage_relay.py"
  echo "$GPS_OUTAGE" >"$OUT_HOST/gps_outage.txt"
  wm_exec "exec python3 /tmp/eidos_visual_eval/gps_outage_relay.py --origin $ORIGIN --windows $GPS_OUTAGE" \
    >"$OUT_HOST/gps_outage.log" 2>&1 &
  sleep 2
fi

# ---- 3. Record ----
RECORD_TOPICS=(
  /world_modeling/liso/odometry /world_modeling/slam/odometry /world_modeling/slam/pose
  /world_modeling/slam/status /world_modeling/slam/initial_pose /world_modeling/transform/odometry
  /world_modeling/visual_loop_closure_factor/measurement
  /world_modeling/euclidean_distance_loop_closure_factor/measurement
  /world_modeling/visual_relocalization/estimate
  /world_modeling/multiview_odometry_factor/measurement
  /world_modeling/gps_factor/utm_to_map_tf
  /novatel/oem7/odom
  "${EXTRA_RECORD_TOPICS[@]}"
)
[[ -z "${GPS_OUTAGE:-}" ]] || RECORD_TOPICS+=(/eval/gps_fix)
wm_exec "exec ros2 bag record -o '$OUT_WM/out' --use-sim-time --storage mcap ${RECORD_TOPICS[*]}" \
  >"$OUT_HOST/record.log" 2>&1 &
REC_PID=$!
sleep 5  # let the recorder subscribe before data flows

# ---- 4. Play ----
PLAY_TOPICS=(/lidar_cc/velodyne_points /novatel/oem7/imu/data /novatel/oem7/imu/data_raw /novatel/oem7/fix
  /novatel/oem7/odom)
[[ -n "${TF_STATIC:-}" ]] || PLAY_TOPICS+=(/tf_static)
for cam in "${CAMERAS[@]}"; do
  PLAY_TOPICS+=("/$cam/image_rect_compressed" "/$cam/camera_info")
done
PLAY_ARGS=(--clock 1000 -r "$RATE" --topics "${PLAY_TOPICS[@]}")
[[ "$START" == "0" ]] || PLAY_ARGS+=(--start-offset "$START")
[[ "$DURATION" == "0" ]] || PLAY_ARGS+=(--playback-duration "$DURATION")
echo "==> Playing $(basename "$BAG") at ${RATE}x (start $START s, duration ${DURATION} s; 0 = to the end)"
docker run --rm --ulimit memlock=-1 --ipc host --network host --name "$PLAYER" \
  -v "$(dirname "$BAG"):/eval_bags:ro" \
  -e "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -e "ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI" -e "ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" \
  "$INFRASTRUCTURE_IMAGE:$TAG" \
  ros2 bag play "/eval_bags/$(basename "$BAG")" "${PLAY_ARGS[@]}" >"$OUT_HOST/play.log" 2>&1

echo "==> Playback done; letting pending measurements finish"
sleep 10
if [[ "$SAVE_MAP" == "1" ]]; then
  echo "==> Saving the map -> $OUT_HOST/map.map"
  wm_exec "ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap \"{filepath: '$OUT_WM/map.map'}\"" \
    | tail -1 | sed 's/^/    /'
fi
docker exec "$WM" pkill -INT -f "bag record -o $OUT_WM/out" || true
wait "$REC_PID" || true
stop_eidos
wait "$EIDOS_PID" 2>/dev/null || true

grep -E "\[(visual_loop_closure_factor|euclidean_distance_loop_closure_factor)\] (keyframes|initialized)" \
  "$OUT_HOST/eidos.log" | tail -3 | sed 's/^.*\]: /    /' || true
grep -E "\[multiview_odometry_factor\] keyframes" "$OUT_HOST/eidos.log" | tail -1 | sed 's/^.*\]: /    /' || true
echo "==> Done: $OUT_HOST"
