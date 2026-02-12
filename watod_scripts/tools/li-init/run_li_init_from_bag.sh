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
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
  cat <<EOF
Usage: run_li_init_from_bag.sh --bag /bags/file.bag [--lidar-topic TOPIC] [--imu-topic TOPIC] [--out /bags/out.txt]

Runs HKU-MARS LI-Init (ROS1) inside this container against a ROS1 bag.
EOF
}

bag=""
lidar_topic="/lidar_top/velodyne_points"
imu_topic="/novatel/oem7/imu/data_raw"
out="/bags/li_init_result.txt"
debug_dir_default="/bags/li_init_debug"
debug_dir="${LI_INIT_DEBUG_DIR:-$debug_dir_default}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      bag="$2"; shift 2
      ;;
    --lidar-topic)
      lidar_topic="$2"; shift 2
      ;;
    --imu-topic)
      imu_topic="$2"; shift 2
      ;;
    --out)
      out="$2"; shift 2
      ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$bag" ]]; then
  echo "Error: --bag is required" >&2
  exit 2
fi

if [[ ! -f "$bag" ]]; then
  echo "Error: bag not found: $bag" >&2
  exit 2
fi

# ROS Noetic's catkin profile scripts may reference ROS_MASTER_URI, and this
# script runs with `set -u`.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

result_src="/catkin_ws/src/lidar_imu_init/result/Initialization_result.txt"

# Clean previous result
rm -f "$result_src" || true

# Ensure ROS master
roscore >/tmp/roscore.log 2>&1 &
roscore_pid=$!

# Wait for roscore to become reachable
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

if ! rosparam list >/dev/null 2>&1; then
  echo "Failed to contact roscore (see /tmp/roscore.log)" >&2
  exit 1
fi

cleanup() {
  exit_code=$?

  if (( exit_code != 0 )); then
    mkdir -p "$debug_dir" || true
    stamp="$(date +%Y%m%d_%H%M%S)"

    # Copy whatever logs exist to the mounted bags volume so debugging doesn't
    # require re-running the container without --rm.
    for f in /tmp/li_init.log /tmp/rosbag_play.log /tmp/rosbag_probe.log /tmp/rosbag_info.txt /tmp/roscore.log; do
      if [[ -s "$f" ]]; then
        cp -f "$f" "$debug_dir/$(basename "$f" .log)_${stamp}.log" >/dev/null 2>&1 || true
      fi
    done
    # Preserve raw text names (rosbag_info may not be .log)
    if [[ -s /tmp/rosbag_info.txt ]]; then
      cp -f /tmp/rosbag_info.txt "$debug_dir/rosbag_info_${stamp}.txt" >/dev/null 2>&1 || true
    fi
    echo "Debug logs copied to: $debug_dir" >&2
  fi

  # best-effort cleanup
  if [[ -n "${play_pid:-}" ]] && kill -0 "$play_pid" >/dev/null 2>&1; then
    kill "$play_pid" >/dev/null 2>&1 || true
  else
    pkill -f "rosbag play" >/dev/null 2>&1 || true
  fi

  if [[ -n "${li_init_pid:-}" ]] && kill -0 "$li_init_pid" >/dev/null 2>&1; then
    kill "$li_init_pid" >/dev/null 2>&1 || true
  else
    pkill -f "li_init" >/dev/null 2>&1 || true
  fi

  kill "$roscore_pid" >/dev/null 2>&1 || true

  exit "$exit_code"
}
trap cleanup EXIT

echo "Inspecting bag topics..." >&2
rosbag info "$bag" >/tmp/rosbag_info.txt 2>&1 || true
if ! grep -Fq "$lidar_topic" /tmp/rosbag_info.txt; then
  echo "Error: LiDAR topic not found in bag: $lidar_topic" >&2
  echo "See /tmp/rosbag_info.txt (will be copied to $debug_dir on exit)." >&2
  exit 1
fi
if ! grep -Fq "$imu_topic" /tmp/rosbag_info.txt; then
  echo "Error: IMU topic not found in bag: $imu_topic" >&2
  echo "See /tmp/rosbag_info.txt (will be copied to $debug_dir on exit)." >&2
  exit 1
fi

echo "Probing first LiDAR/IMU messages..." >&2

# Play just long enough for one message to be observed.
rosbag play --quiet --clock --topics "$lidar_topic" "$imu_topic" "$bag" >/tmp/rosbag_probe.log 2>&1 &
probe_play_pid=$!

LIDAR_TOPIC="$lidar_topic" IMU_TOPIC="$imu_topic" python3 - <<'PY'
import os
import sys

import rospy
from sensor_msgs.msg import Imu, PointCloud2

lidar_topic = os.environ.get("LIDAR_TOPIC", "/lidar_top/velodyne_points")
imu_topic = os.environ.get("IMU_TOPIC", "/novatel/oem7/imu/data_raw")

rospy.init_node("li_init_bag_probe", anonymous=True, disable_signals=True)

try:
    pc = rospy.wait_for_message(lidar_topic, PointCloud2, timeout=10.0)
except Exception as e:
    print(f"ERROR: did not receive PointCloud2 on {lidar_topic}: {e}", file=sys.stderr)
    sys.exit(2)

try:
    _ = rospy.wait_for_message(imu_topic, Imu, timeout=10.0)
except Exception as e:
    print(f"ERROR: did not receive Imu on {imu_topic}: {e}", file=sys.stderr)
    sys.exit(2)

field_names = [f.name for f in pc.fields]
print("PointCloud2 fields:", ", ".join(field_names), file=sys.stderr)

# LI-Init commonly expects per-point timing + ring for Velodyne clouds.
required_any = [
    {"time", "t", "timestamp"},
    {"ring"},
]

missing_groups = []
for group in required_any:
    if not any(name in field_names for name in group):
        missing_groups.append("/".join(sorted(group)))

if missing_groups:
    print(
        "ERROR: PointCloud2 is missing required fields for LI-Init: "
        + ", ".join(missing_groups)
        + ".\n"
        + "If your driver publishes a cloud without ring/time, re-record using a Velodyne pipeline that includes these fields (e.g., velodyne_pointcloud with ring+time).",
        file=sys.stderr,
    )
    sys.exit(3)

sys.exit(0)
PY

probe_rc=$?

kill "$probe_play_pid" >/dev/null 2>&1 || true
wait "$probe_play_pid" >/dev/null 2>&1 || true

if (( probe_rc != 0 )); then
  echo "Probe failed (rc=$probe_rc). See /tmp/rosbag_probe.log" >&2
  exit 1
fi

# Configure parameters for Velodyne VLP-16-ish defaults.
# LI-Init reads these as global params (nh.param("common/..."))
rosparam set common/lid_topic "$lidar_topic"
rosparam set common/imu_topic "$imu_topic"

# preprocess/lidar_type: enum LID_TYPE { AVIA=1, VELO=2, OUSTER=3, L515=4, PANDAR=5, ROBOSENSE=6 }
rosparam set preprocess/lidar_type 2
rosparam set preprocess/scan_line 16
rosparam set preprocess/feature_extract_en false
rosparam set preprocess/blind 1.0
rosparam set point_filter_num 2

# initialization
rosparam set initialization/cut_frame true
rosparam set initialization/cut_frame_num 2
rosparam set initialization/orig_odom_freq 10
rosparam set initialization/online_refine_time 20.0
rosparam set initialization/mean_acc_norm 9.805

# mapping
rosparam set mapping/filter_size_surf 0.5
rosparam set mapping/filter_size_map 0.5
rosparam set mapping/det_range 300.0

# Turn off publishing to reduce overhead
rosparam set publish/path_en false
rosparam set publish/scan_publish_en false
rosparam set publish/dense_publish_en false
rosparam set publish/scan_bodyframe_pub_en false

# Start LI-Init node
rosrun lidar_imu_init li_init >/tmp/li_init.log 2>&1 &
li_init_pid=$!

# Play bag (no /clock needed; LI-Init uses msg header stamps)
rosbag play --quiet --clock "$bag" >/tmp/rosbag_play.log 2>&1 &
play_pid=$!

# Wait for result file to be created and non-empty

timeout_s=${LI_INIT_TIMEOUT_S:-600}
start_ts=$(date +%s)

while true; do
  if [[ -s "$result_src" ]]; then
    break
  fi
  now_ts=$(date +%s)
  if (( now_ts - start_ts > timeout_s )); then
    echo "Timed out waiting for LI-Init result after ${timeout_s}s" >&2
    echo "Logs: /tmp/li_init.log, /tmp/rosbag_play.log" >&2
    exit 1
  fi
  sleep 1
  # If LI-Init crashed, fail early
  if ! kill -0 "$li_init_pid" >/dev/null 2>&1; then
    echo "LI-Init exited before producing result. See /tmp/li_init.log" >&2
    exit 1
  fi
done

mkdir -p "$(dirname "$out")"
cp -f "$result_src" "$out"

echo "Wrote: $out"
