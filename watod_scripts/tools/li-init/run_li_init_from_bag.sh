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
time_shift_mode="${LI_INIT_TIME_SHIFT_MODE:-bag}"

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
    for f in /tmp/li_init.log /tmp/rosbag_play.log /tmp/rosbag_probe.log /tmp/rosbag_info.txt /tmp/roscore.log /tmp/li_init_pc_time_shift.log /tmp/li_init_time_shift_bag.log; do
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

  if [[ -n "${time_shift_pid:-}" ]] && kill -0 "$time_shift_pid" >/dev/null 2>&1; then
    kill "$time_shift_pid" >/dev/null 2>&1 || true
  else
    pkill -f "li_init_pc_time_shift" >/dev/null 2>&1 || true
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
# Note: rosbag play expects BAGFILE before options; otherwise BAGFILE can be
# parsed as part of --topics and you'll get "You must specify at least 1 bag".
rosbag play "$bag" --quiet --clock --topics "$lidar_topic" "$imu_topic" >/tmp/rosbag_probe.log 2>&1 &
probe_play_pid=$!

LIDAR_TOPIC="$lidar_topic" IMU_TOPIC="$imu_topic" python3 - <<'PY'
import os
import sys

import rospy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2

lidar_topic = os.environ.get("LIDAR_TOPIC", "/lidar_top/velodyne_points")
imu_topic = os.environ.get("IMU_TOPIC", "/novatel/oem7/imu/data_raw")

rospy.init_node("li_init_bag_probe", anonymous=True, disable_signals=True)

try:
  _ = rospy.wait_for_message(imu_topic, Imu, timeout=10.0)
except Exception as e:
  print(f"ERROR: did not receive Imu on {imu_topic}: {e}", file=sys.stderr)
  sys.exit(2)

# Sample a few PointCloud2 messages to avoid being fooled by a single weird scan.
pcs = []
for i in range(3):
  try:
    pcs.append(rospy.wait_for_message(lidar_topic, PointCloud2, timeout=10.0))
  except Exception as e:
    print(
      f"ERROR: did not receive PointCloud2 on {lidar_topic} (sample {i+1}/3): {e}",
      file=sys.stderr,
    )
    sys.exit(2)

pc = pcs[0]

field_names = [f.name for f in pc.fields]
print("PointCloud2 fields:", ", ".join(field_names), file=sys.stderr)

# Compute basic stats for ring/time for sanity + auto-config.
ring_min = None
ring_max = None
time_min = None
time_max = None
time_varies = False

sample_limit = 20000

want_fields = []
for name in ("ring", "time"):
  if name in field_names:
    want_fields.append(name)

def ingest_one(msg: PointCloud2) -> None:
  global ring_min, ring_max, time_min, time_max, time_varies

  if not want_fields:
    return

  count = 0
  local_time_min = None
  local_time_max = None

  for p in point_cloud2.read_points(msg, field_names=want_fields, skip_nans=True):
    for idx, fname in enumerate(want_fields):
      val = p[idx]
      if fname == "ring":
        try:
          rval = int(val)
        except Exception:
          continue
        ring_min = rval if ring_min is None else min(ring_min, rval)
        ring_max = rval if ring_max is None else max(ring_max, rval)
      elif fname == "time":
        try:
          tval = float(val)
        except Exception:
          continue
        time_min = tval if time_min is None else min(time_min, tval)
        time_max = tval if time_max is None else max(time_max, tval)
        local_time_min = tval if local_time_min is None else min(local_time_min, tval)
        local_time_max = tval if local_time_max is None else max(local_time_max, tval)

    count += 1
    if count >= sample_limit:
      break

  if local_time_min is not None and local_time_max is not None and abs(local_time_max - local_time_min) > 1e-9:
    time_varies = True

for msg in pcs:
  ingest_one(msg)

if ring_min is not None and ring_max is not None:
  # LI-Init wants scan_line = number of rings.
  scan_line = ring_max + 1
  # Clamp to something reasonable to avoid exploding memory on bad data.
  if scan_line < 4 or scan_line > 256:
    print(
      f"ERROR: ring field looks suspicious (min={ring_min}, max={ring_max}); refusing to auto-set scan_line.",
      file=sys.stderr,
    )
    sys.exit(4)

  print(f"Detected ring range: min={ring_min} max={ring_max} -> scan_line={scan_line}", file=sys.stderr)
  with open("/tmp/li_init_probe.env", "w", encoding="utf-8") as f:
    f.write(f"LI_INIT_SCAN_LINE={scan_line}\n")
else:
  print("WARN: could not compute ring range from PointCloud2 sample.", file=sys.stderr)

if time_min is not None and time_max is not None:
  print(f"Detected time range (sample): min={time_min} max={time_max}", file=sys.stderr)
  with open("/tmp/li_init_probe.env", "a", encoding="utf-8") as f:
    f.write(f"LI_INIT_TIME_MIN={time_min}\n")
    f.write(f"LI_INIT_TIME_MAX={time_max}\n")
  if not time_varies:
    print(
      "ERROR: PointCloud2 'time' field appears constant across sampled points/scans. "
      "LI-Init typically requires per-point time within each scan for deskew; constant time often leads to invalid lag estimates and crashes.\n"
      "Fix options:\n"
      "  - Re-record using a Velodyne pointcloud pipeline that populates per-point time (not all drivers do).\n"
      "  - Prefer packet topics (velodyne_packets) + generate a timed PointCloud2 using velodyne_pointcloud before running LI-Init.",
      file=sys.stderr,
    )
    sys.exit(5)

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

li_init_scan_line=""
li_init_time_min=""
li_init_time_max=""
if [[ -f /tmp/li_init_probe.env ]]; then
  # shellcheck disable=SC1091
  source /tmp/li_init_probe.env
  li_init_scan_line="${LI_INIT_SCAN_LINE:-}"
  li_init_time_min="${LI_INIT_TIME_MIN:-}"
  li_init_time_max="${LI_INIT_TIME_MAX:-}"  # shellcheck disable=SC2034
fi

li_init_lidar_topic="$lidar_topic"
bag_to_play="$bag"

# LI-Init has been observed to crash on some inputs when the per-point `time`
# field is negative. Prefer rewriting the bag to shift per-point time to be
# non-negative (per scan) so LI-Init can run without a slow Python republisher.
need_time_shift=0
if [[ -n "$li_init_time_min" ]]; then
  python3 - <<PY >/dev/null 2>&1 || true
import sys
try:
  v = float("$li_init_time_min")
except Exception:
  sys.exit(1)
sys.exit(0 if v < 0.0 else 2)
PY
  neg_rc=$?
  if (( neg_rc == 0 )); then
    need_time_shift=1
  fi
fi

if (( need_time_shift == 1 )); then
  if [[ "$time_shift_mode" == "topic" ]]; then
    shifted_topic="${lidar_topic}_time_shifted"
    echo "Detected negative per-point time (min=$li_init_time_min). Republishing to: $shifted_topic" >&2

    LIDAR_IN="$lidar_topic" LIDAR_OUT="$shifted_topic" python3 - <<'PY' >/tmp/li_init_pc_time_shift.log 2>&1 &
import os
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField

lidar_in = os.environ.get("LIDAR_IN", "/lidar_top/velodyne_points")
lidar_out = os.environ.get("LIDAR_OUT", lidar_in + "_time_shifted")

rospy.init_node("li_init_pc_time_shift", anonymous=True, disable_signals=True)

pub = rospy.Publisher(lidar_out, PointCloud2, queue_size=2)

time_offset = None
time_datatype = None
point_step = None
shift = None

def _dtype_fmt(datatype: int):
  if datatype == PointField.FLOAT32:
    return "<f", 4
  if datatype == PointField.FLOAT64:
    return "<d", 8
  return None, None

def _init_layout(msg: PointCloud2):
  global time_offset, time_datatype, point_step, shift

  if time_offset is not None:
    return

  for f in msg.fields:
    if f.name == "time":
      time_offset = f.offset
      time_datatype = f.datatype
      break
  point_step = msg.point_step
  fmt, _size = _dtype_fmt(time_datatype)
  if fmt is None:
    rospy.logerr("Unsupported PointField datatype for time: %s", str(time_datatype))
    shift = 0.0
    return

  data = msg.data
  npts = (len(data) // point_step) if point_step else 0
  if npts <= 0:
    shift = 0.0
    return

  tmin = None
  for i in range(npts):
    base = i * point_step + time_offset
    (t,) = struct.unpack_from(fmt, data, base)
    if tmin is None or t < tmin:
      tmin = t
  if tmin is None or tmin >= 0.0:
    shift = 0.0
  else:
    shift = -float(tmin)
  rospy.logwarn("Shifting per-point time by %+0.9f (first-scan min=%+0.9f).", shift, tmin if tmin is not None else 0.0)

def cb(msg: PointCloud2):
  global shift
  _init_layout(msg)
  if shift is None:
    shift = 0.0
  if shift == 0.0:
    pub.publish(msg)
    return

  fmt, _size = _dtype_fmt(time_datatype)
  if fmt is None:
    pub.publish(msg)
    return

  data = bytearray(msg.data)
  npts = (len(data) // point_step) if point_step else 0
  for i in range(npts):
    base = i * point_step + time_offset
    (t,) = struct.unpack_from(fmt, data, base)
    t2 = float(t) + shift
    if t2 < 0.0:
      t2 = 0.0
    struct.pack_into(fmt, data, base, t2)

  out = PointCloud2()
  out.header = msg.header
  out.height = msg.height
  out.width = msg.width
  out.fields = msg.fields
  out.is_bigendian = msg.is_bigendian
  out.point_step = msg.point_step
  out.row_step = msg.row_step
  out.is_dense = msg.is_dense
  out.data = bytes(data)
  pub.publish(out)

rospy.Subscriber(lidar_in, PointCloud2, cb, queue_size=1, buff_size=64 * 1024 * 1024)
rospy.spin()
PY
    time_shift_pid=$!
    li_init_lidar_topic="$shifted_topic"
  else
    shifted_bag="/tmp/li_init_time_shifted.bag"
    echo "Detected negative per-point time (min=$li_init_time_min). Rewriting bag to: $shifted_bag" >&2

    IN_BAG="$bag" OUT_BAG="$shifted_bag" LIDAR_TOPIC="$lidar_topic" python3 - <<'PY' >/tmp/li_init_time_shift_bag.log 2>&1
import os
import sys

import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2, PointField

in_bag = os.environ["IN_BAG"]
out_bag = os.environ["OUT_BAG"]
lidar_topic = os.environ["LIDAR_TOPIC"]

def _np_dtype(datatype: int):
  if datatype == PointField.FLOAT32:
    return np.float32
  if datatype == PointField.FLOAT64:
    return np.float64
  return None

def shift_cloud(msg: PointCloud2) -> PointCloud2:
  time_field = None
  for f in msg.fields:
    if f.name == "time":
      time_field = f
      break
  if time_field is None:
    return msg

  dtype = _np_dtype(time_field.datatype)
  if dtype is None:
    return msg

  point_step = int(msg.point_step)
  if point_step <= 0:
    return msg

  data = bytearray(msg.data)
  npts = len(data) // point_step
  if npts <= 0:
    return msg

  # Writable strided view into the time field.
  times = np.ndarray(
    shape=(npts,),
    dtype=dtype,
    buffer=data,
    offset=int(time_field.offset),
    strides=(point_step,),
  )

  tmin = float(times.min(initial=0.0)) if times.size else 0.0
  if tmin < 0.0:
    times[:] = times - tmin
    # guard tiny negatives from float rounding
    times[times < 0.0] = 0.0

  out = msg
  out.data = bytes(data)
  return out

with rosbag.Bag(out_bag, "w") as out:
  with rosbag.Bag(in_bag, "r") as bag:
    for topic, msg, t in bag.read_messages():
      if topic == lidar_topic and isinstance(msg, PointCloud2):
        msg = shift_cloud(msg)
      out.write(topic, msg, t)

print(f"Wrote shifted bag: {out_bag}")
PY

    if [[ ! -s "$shifted_bag" ]]; then
      echo "ERROR: shifted bag was not created (see /tmp/li_init_time_shift_bag.log)." >&2
      exit 1
    fi
    bag_to_play="$shifted_bag"
  fi
fi

# Configure parameters for Velodyne VLP-16-ish defaults.
# LI-Init reads these as global params (nh.param("common/..."))
rosparam set common/lid_topic "$li_init_lidar_topic"
rosparam set common/imu_topic "$imu_topic"

# preprocess/lidar_type: enum LID_TYPE { AVIA=1, VELO=2, OUSTER=3, L515=4, PANDAR=5, ROBOSENSE=6 }
rosparam set preprocess/lidar_type 2
if [[ -n "${li_init_scan_line:-}" ]]; then
  rosparam set preprocess/scan_line "$li_init_scan_line"
else
  rosparam set preprocess/scan_line 16
fi
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
rosbag play "$bag_to_play" --quiet --clock --delay=2 >/tmp/rosbag_play.log 2>&1 &
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
