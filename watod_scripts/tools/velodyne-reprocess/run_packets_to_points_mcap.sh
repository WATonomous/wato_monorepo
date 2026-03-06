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
set -euo pipefail

usage() {
  cat <<'EOF'
Generate a timed PointCloud2 from Velodyne packets inside a ROS2 MCAP bag.

This is meant to fix bags where PointCloud2 has a `time` field that is constant
(all zeros), which breaks LI-Init.

Usage:
  run_packets_to_points_mcap.sh \
    --src <ros2_bag_dir_or_mcap> \
    --dst <output_bag_dir_name> \
    --packets-topic <topic> \
    --points-topic <topic> \
    --imu-topic <topic>
      [--rate <float>] \
      [--start-offset <seconds>] \
      [--duration <seconds>] \
      [--timeout <seconds>]

Defaults:
  --packets-topic /lidar_top/velodyne_packets
  --points-topic  /lidar_top/velodyne_points_timed
  --imu-topic     /novatel/oem7/imu/data_raw
  --pointcloud-config VLP32C
  --calibration-file  VeloView-VLP-32C.yaml
  --rate 1.0
  --timeout 0   (0 = no timeout)

Notes:
- This tool plays the source bag and records ONLY the regenerated pointcloud
  and IMU topic into a new MCAP bag (under /bags).
- Requires the source bag to contain Velodyne packets.
- `--duration` is intended to mean bag-time seconds (if your ROS2 `ros2 bag play`
  supports `--duration`). If your machine can't keep up, wall-clock runtime can
  be longer than the bag-time slice.
EOF
}

src=""
dst=""
packets_topic="/lidar_top/velodyne_packets"
points_topic="/lidar_top/velodyne_points_timed"
imu_topic="/novatel/oem7/imu/data_raw"
pointcloud_config="VLP32C"
calibration_file="VeloView-VLP-32C.yaml"
rate="1.0"
start_offset=""
duration=""
timeout_s="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --src)
      src="${2:-}"; shift 2 ;;
    --dst)
      dst="${2:-}"; shift 2 ;;
    --packets-topic)
      packets_topic="${2:-}"; shift 2 ;;
    --points-topic)
      points_topic="${2:-}"; shift 2 ;;
    --imu-topic)
      imu_topic="${2:-}"; shift 2 ;;
    --pointcloud-config)
      pointcloud_config="${2:-}"; shift 2 ;;
    --calibration-file)
      calibration_file="${2:-}"; shift 2 ;;
    --rate)
      rate="${2:-}"; shift 2 ;;
    --start-offset)
      start_offset="${2:-}"; shift 2 ;;
    --duration)
      duration="${2:-}"; shift 2 ;;
    --timeout)
      timeout_s="${2:-}"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Run with --help for usage." >&2
      exit 2
      ;;
  esac
done

if [[ -z "$src" || -z "$dst" ]]; then
  echo "Error: --src and --dst are required." >&2
  echo "Run with --help for usage." >&2
  exit 2
fi

if [[ ! -d /bags ]]; then
  echo "Error: /bags is not mounted." >&2
  exit 2
fi

if [[ ! -e "/bags/$src" ]]; then
  echo "Error: source bag not found at /bags/$src" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ROS setup scripts may reference unset variables; don't run them under nounset.
set +u
source /opt/ros/jazzy/setup.bash
set -u

params_dir="$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud"
transform_params="$params_dir/config/${pointcloud_config}-velodyne_transform_node-params.yaml"
calib_path="$params_dir/params/${calibration_file}"

if [[ ! -f "$transform_params" ]]; then
  echo "Error: missing velodyne_pointcloud config: $transform_params" >&2
  echo "Try --pointcloud-config VLP16 or VLP32C (and matching calibration file)." >&2
  exit 2
fi

if [[ ! -f "$calib_path" ]]; then
  echo "Error: missing velodyne_pointcloud calibration: $calib_path" >&2
  exit 2
fi

out_dir="/bags/$dst"
if [[ -e "$out_dir" ]]; then
  echo "Error: output already exists: $out_dir" >&2
  exit 2
fi

debug_root="/bags/velodyne_reprocess_debug"
stamp="$(date +%Y%m%d_%H%M%S)"
debug_dir="$debug_root/${dst}_${stamp}"
mkdir -p "$debug_dir" || true
echo "Logs: $debug_dir" >&2

transform_pid=""
record_pid=""
play_pid=""

auto_kill() {
  set +e

  # Persist logs to the mounted volume for post-mortem.
  for f in /tmp/velodyne_transform.log /tmp/ros2_record.log /tmp/ros2_play.log /tmp/ros2_bag_info.txt; do
    if [[ -e "$f" ]]; then
      cp -f "$f" "$debug_dir/$(basename "$f")" >/dev/null 2>&1 || true
    fi
  done

  if [[ -n "$play_pid" ]] && kill -0 "$play_pid" 2>/dev/null; then
    kill -INT -- "-$play_pid" >/dev/null 2>&1 || kill -INT "$play_pid" 2>/dev/null || true
  fi
  if [[ -n "$transform_pid" ]] && kill -0 "$transform_pid" 2>/dev/null; then
    kill -INT -- "-$transform_pid" >/dev/null 2>&1 || kill -INT "$transform_pid" 2>/dev/null || true
    sleep 0.5
    kill -KILL -- "-$transform_pid" >/dev/null 2>&1 || kill -KILL "$transform_pid" 2>/dev/null || true
  fi
  if [[ -n "$record_pid" ]] && kill -0 "$record_pid" 2>/dev/null; then
    kill -INT -- "-$record_pid" >/dev/null 2>&1 || kill -INT "$record_pid" 2>/dev/null || true
    sleep 0.5
    kill -TERM -- "-$record_pid" >/dev/null 2>&1 || kill -TERM "$record_pid" 2>/dev/null || true
  fi
}
trap auto_kill EXIT

echo "Inspecting source bag topics..." >&2
ros2 bag info "/bags/$src" >/tmp/ros2_bag_info.txt 2>&1 || true
if ! grep -Fq "$packets_topic" /tmp/ros2_bag_info.txt; then
  echo "Error: packets topic not found in bag: $packets_topic" >&2
  echo "See: $debug_dir/ros2_bag_info.txt" >&2
  exit 2
fi
if ! grep -Fq "$imu_topic" /tmp/ros2_bag_info.txt; then
  echo "Error: IMU topic not found in bag: $imu_topic" >&2
  echo "See: $debug_dir/ros2_bag_info.txt" >&2
  exit 2
fi

# Start velodyne packet->pointcloud transform
# Start each process in its own session/process-group so we can reliably
# signal the whole group (ros2 CLI sometimes spawns children).
setsid ros2 run velodyne_pointcloud velodyne_transform_node \
  --ros-args \
  --params-file "$transform_params" \
  -p use_sim_time:=true \
  -p calibration:="$calib_path" \
  -r velodyne_packets:="$packets_topic" \
  -r velodyne_points:="$points_topic" \
  >/tmp/velodyne_transform.log 2>&1 &
transform_pid=$!

# Start recording before playing
setsid ros2 bag record \
  --storage mcap \
  --compression-mode none \
  --storage-preset-profile fastwrite \
  --max-cache-size 0 \
  --disable-keyboard-controls \
  -o "$out_dir" \
  --topics "$points_topic" "$imu_topic" \
  >/tmp/ros2_record.log 2>&1 &
record_pid=$!

# Give recorder time to subscribe
sleep 1.0

# Play the source bag
play_args=("/bags/$src" "--clock" "--rate" "$rate")
play_args+=("--topics" "$packets_topic" "$imu_topic")
if [[ -n "$start_offset" ]]; then
  play_args+=("--start-offset" "$start_offset")
fi

# Prefer native bag-time slicing when available.
# Jazzy uses `--playback-duration` (bag-time seconds), not `--duration`.
if [[ -n "$duration" ]] && ros2 bag play --help 2>/dev/null | grep -q -- "--playback-duration"; then
  play_args+=("--playback-duration" "$duration")
  duration=""
fi

setsid ros2 bag play \
  --disable-keyboard-controls \
  --read-ahead-queue-size 10000 \
  "${play_args[@]}" \
  >/tmp/ros2_play.log 2>&1 &
play_pid=$!

# Watchdog: periodically report progress and fail fast if a key process dies.
(
  start_ts=$(date +%s)
  last_report=0
  while kill -0 "$play_pid" >/dev/null 2>&1; do
    now_ts=$(date +%s)
    elapsed=$(( now_ts - start_ts ))

    if [[ "$timeout_s" != "0" ]] && (( elapsed > timeout_s )); then
      echo "ERROR: timeout after ${timeout_s}s while playing bag." >&2
      kill -INT "$play_pid" >/dev/null 2>&1 || true
      kill -INT "$record_pid" >/dev/null 2>&1 || true
      exit 124
    fi

    if ! kill -0 "$transform_pid" >/dev/null 2>&1; then
      echo "ERROR: velodyne_transform_node exited early." >&2
      tail -n 200 /tmp/velodyne_transform.log >&2 || true
      kill -INT "$play_pid" >/dev/null 2>&1 || true
      kill -INT "$record_pid" >/dev/null 2>&1 || true
      exit 125
    fi

    if ! kill -0 "$record_pid" >/dev/null 2>&1; then
      echo "ERROR: ros2 bag record exited early." >&2
      tail -n 200 /tmp/ros2_record.log >&2 || true
      kill -INT "$play_pid" >/dev/null 2>&1 || true
      exit 126
    fi

    # Report size every ~10s
    if (( now_ts - last_report >= 10 )); then
      size=$(du -sh "$out_dir" 2>/dev/null | awk '{print $1}')
      echo "[progress] elapsed=${elapsed}s out_size=${size:-?}" >&2
      last_report=$now_ts
    fi

    sleep 1
  done
) &
watchdog_pid=$!

if [[ -n "$duration" ]]; then
  # Not all ROS2 distros/patchlevels support `ros2 bag play --duration`.
  # Implement it ourselves by stopping playback after N seconds.
  (
    sleep "$duration"
    if kill -0 "$play_pid" >/dev/null 2>&1; then
      # Kill the whole process group to avoid orphaned children.
      kill -INT -- "-$play_pid" >/dev/null 2>&1 || kill -INT "$play_pid" >/dev/null 2>&1 || true
    fi
  ) &
fi

play_rc=0
wait "$play_pid" || play_rc=$?

watchdog_rc=0
wait "$watchdog_pid" || watchdog_rc=$?

if (( watchdog_rc != 0 )); then
  echo "Error: watchdog triggered (rc=$watchdog_rc). Logs: $debug_dir" >&2
  exit 1
fi

if (( play_rc != 0 )); then
  echo "Error: ros2 bag play exited with rc=$play_rc" >&2
  echo "---- /tmp/ros2_play.log (tail) ----" >&2
  tail -n 200 /tmp/ros2_play.log >&2 || true
  echo "---- /tmp/ros2_record.log (tail) ----" >&2
  tail -n 200 /tmp/ros2_record.log >&2 || true
  echo "---- /tmp/velodyne_transform.log (tail) ----" >&2
  tail -n 200 /tmp/velodyne_transform.log >&2 || true
  exit 1
fi

# Stop transform node first (so recorder stops receiving new data)
echo "Stopping transform..." >&2
kill -INT -- "-$transform_pid" >/dev/null 2>&1 || kill -INT "$transform_pid" >/dev/null 2>&1 || true

transform_deadline_s=${VELODYNE_REPROCESS_TRANSFORM_STOP_TIMEOUT_S:-120}
transform_start_ts=$(date +%s)
while kill -0 "$transform_pid" >/dev/null 2>&1; do
  now_ts=$(date +%s)
  if (( now_ts - transform_start_ts > transform_deadline_s )); then
    echo "WARN: transform did not exit after ${transform_deadline_s}s; force killing." >&2
    kill -KILL "$transform_pid" >/dev/null 2>&1 || true
    break
  fi
  sleep 1
done
wait "$transform_pid" >/dev/null 2>&1 || true

# Stop recording cleanly and allow time for metadata finalization.
echo "Stopping recorder (may take time to finalize metadata)..." >&2
kill -INT -- "-$record_pid" >/dev/null 2>&1 || kill -INT "$record_pid" >/dev/null 2>&1 || true

record_deadline_s=${VELODYNE_REPROCESS_RECORD_STOP_TIMEOUT_S:-1800}
record_int_to_term_s=${VELODYNE_REPROCESS_RECORD_INT_TO_TERM_S:-30}
record_start_ts=$(date +%s)
last_report_ts=0
last_size=""
sent_term=0
while kill -0 "$record_pid" >/dev/null 2>&1; do
  now_ts=$(date +%s)
  if (( now_ts - last_report_ts >= 10 )); then
    size=$(du -sh "$out_dir" 2>/dev/null | awk '{print $1}')
    if [[ -n "$size" && "$size" != "$last_size" ]]; then
      echo "[finalize] out_size=$size" >&2
      last_size="$size"
    fi
    last_report_ts=$now_ts
  fi

  if (( now_ts - record_start_ts > record_deadline_s )); then
    echo "ERROR: recorder did not exit after ${record_deadline_s}s; bag may be incomplete." >&2
    echo "See logs: $debug_dir" >&2
    exit 1
  fi

  if (( sent_term == 0 )) && (( now_ts - record_start_ts > record_int_to_term_s )); then
    echo "WARN: recorder still running after ${record_int_to_term_s}s; sending SIGTERM..." >&2
    kill -TERM -- "-$record_pid" >/dev/null 2>&1 || kill -TERM "$record_pid" >/dev/null 2>&1 || true
    sent_term=1
  fi
  sleep 1
done
wait "$record_pid" >/dev/null 2>&1 || true

if [[ ! -f "$out_dir/metadata.yaml" ]]; then
  echo "ERROR: output bag missing metadata.yaml (recorder may have been killed or hung)." >&2
  echo "See logs: $debug_dir" >&2
  echo "---- /tmp/ros2_record.log (tail) ----" >&2
  tail -n 200 /tmp/ros2_record.log >&2 || true
  exit 1
fi

if ! ros2 bag info "$out_dir" >/dev/null 2>&1; then
  echo "ERROR: output bag is not readable by ros2 bag info." >&2
  echo "See logs: $debug_dir" >&2
  exit 1
fi

echo "Wrote: $out_dir"
echo "Recorded topics: $points_topic , $imu_topic"
echo "Logs: $debug_dir" >&2
