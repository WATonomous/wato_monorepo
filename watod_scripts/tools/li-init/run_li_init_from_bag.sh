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
}
trap cleanup EXIT

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
