#!/usr/bin/env bash
# Run eidos SLAM offline over a recorded bag and merge its output into a copy of the bag.
#
#   0. Recovers the input with `mcap recover` if it is truncated.
#   1. Starts eidos + eidos_transform (mapping mode, sim time) in the running world_modeling
#      container, stopping any eidos already running there.
#   2. Plays the bag with /clock and records the eidos topics.
#   3. Rewrites each eidos message's log time to its header stamp, so it sits at the sensor time
#      it was computed from instead of when eidos finished computing it, and checks every
#      per-scan pose against the bag's lidar scan stamps.
#   4. Merges that into <input>_with_eidos.mcap with the host `mcap` CLI.
#
# Offline overrides (written to the container's /tmp/eidos_offline):
#   - Static TFs base_footprint -> ... -> lidar_cc are taken from the input bag's own /tf_static,
#     so the eidos poses agree with the extrinsics a consumer of the merged bag will read. imu_link
#     also comes from the bag; bags recorded before the lidar/IMU calibration have none (LISO then
#     sits in warmup forever), so it is attached under lidar_cc with the current eve_description
#     joint (eve_kia_soul_ev.xacro). The bag's /tf_static is not played.
#   - slam_rate and eidos_transform's tick_rate are wall timers; they are scaled by the playback
#     rate so the SLAM tick and the transform output run at their configured rates in bag time.
#   - LISO warmup no longer needs the car to be stationary: the Novatel INS orientation is valid
#     while moving, so a few samples are enough for gravity alignment.
#   - Loop closure cloud dumping is disabled.
#
# Prereqs: ./watod up -d with eidos, eidos_transform and wato_lifecycle_manager built in the
#          world_modeling container, and the mcap CLI on the host.
# Usage:   tools/eidos_offline_to_bag.sh <input.mcap> [rate]   (input relative to $BAG_DIRECTORY)
#          EIDOS_EXTRA_PARAMS=<host yaml> applies further eidos/eidos_transform parameter overrides
#          last, e.g. to compare runs with one setting changed.
set -euo pipefail

IN_BAG="${1:?usage: $0 <input.mcap> [rate]}"
RATE="${2:-0.5}"
EXTRA_PARAMS="${EIDOS_EXTRA_PARAMS:-}"

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[[ -f "$MONO_DIR/watod-config.local.sh" ]] && source "$MONO_DIR/watod-config.local.sh"
BAG_DIRECTORY="$(realpath "${BAG_DIRECTORY:-$MONO_DIR/bags}")"
PROJECT="${COMPOSE_PROJECT_NAME:-$USER}"

STEM="$(basename "${IN_BAG%.mcap}")"
RECOVERED="${STEM}_recovered.mcap"
EIDOS_RAW="${STEM}_eidos_raw"
EIDOS_RESTAMPED="${STEM}_eidos"
MERGED="${STEM}_with_eidos.mcap"
LOG_DIR="$BAG_DIRECTORY/${STEM}_eidos_logs"

LIDAR_TOPIC=/lidar_cc/velodyne_points
PLAY_TOPICS=(/novatel/oem7/fix /novatel/oem7/imu/data "$LIDAR_TOPIC")
RECORD_TOPICS=(
  /world_modeling/liso/odometry /world_modeling/liso/odometry_incremental
  /world_modeling/slam/pose /world_modeling/slam/odometry /world_modeling/slam/status
  /world_modeling/transform/odometry /tf
  # Debugging / visualization (the factor graph plugin only publishes while subscribed)
  /world_modeling/slam/visualization/map /world_modeling/slam/visualization/factor_graph
  /world_modeling/slam/visualization/loop_closure_source /world_modeling/slam/visualization/loop_closure_target
  /world_modeling/slam/visualization/loop_closure_markers /world_modeling/liso/submap
  /world_modeling/gps_factor/utm_pose /world_modeling/gps_factor/utm_to_map_tf
)
CLOCK_HZ=1000  # /clock resolution in wall time; sim-time stamps are quantized to RATE / CLOCK_HZ
# Launches and processes this script may need to stop inside the container
LAUNCH_PATTERN='world_modeling.launch.yaml|eidos.launch.yaml|eidos_offline.launch.yaml'
EIDOS_PROCS='lib/eidos/eidos_node|lib/eidos_transform/eidos_transform_node|eidos_lifecycle_manager|__node:=offline_tf_'

# ---- Checks ----
command -v mcap >/dev/null || { echo "mcap CLI not found on the host"; exit 1; }
[[ -f "$BAG_DIRECTORY/$IN_BAG" ]] || { echo "Missing $BAG_DIRECTORY/$IN_BAG"; exit 1; }
for out in "$EIDOS_RAW" "$EIDOS_RESTAMPED" "$MERGED"; do
  [[ ! -e "$BAG_DIRECTORY/$out" ]] || { echo "$BAG_DIRECTORY/$out already exists; remove it first"; exit 1; }
done

WM="$(docker ps --format '{{.Names}}' | grep -E "^${PROJECT}-world_modeling_bringup(_dev)?-1$" | head -1 || true)"
[[ -n "$WM" ]] || { echo "world_modeling container for '$PROJECT' is not running (./watod up -d)"; exit 1; }

# Where BAG_DIRECTORY is mounted inside the world_modeling container
WM_BAGS="$(docker inspect -f '{{range .Mounts}}{{println .Source .Destination}}{{end}}' "$WM" \
  | awk -v src="$BAG_DIRECTORY" '$1 == src {print $2}')"
[[ -n "$WM_BAGS" ]] || { echo "$BAG_DIRECTORY is not mounted in $WM"; exit 1; }

mkdir -p "$LOG_DIR"

# ---- 0. Recover a truncated input ----
if ! mcap info "$BAG_DIRECTORY/$IN_BAG" >/dev/null 2>&1; then
  if [[ -f "$BAG_DIRECTORY/$RECOVERED" ]]; then
    echo "==> $IN_BAG is truncated; using existing $RECOVERED"
  else
    need=$(stat -c %s "$BAG_DIRECTORY/$IN_BAG")
    avail=$(df --output=avail -B1 "$BAG_DIRECTORY" | tail -1)
    (( avail > 2 * need + 2 * 1024 ** 3 )) || {
      echo "Not enough disk to recover and merge: need ~$((2 * need / 1024 ** 3 + 2)) GiB, have $((avail / 1024 ** 3)) GiB"
      exit 1
    }
    echo "==> $IN_BAG is truncated; recovering -> $RECOVERED"
    mcap recover "$BAG_DIRECTORY/$IN_BAG" -o "$BAG_DIRECTORY/$RECOVERED"
  fi
  IN_BAG="$RECOVERED"
fi

need=$(stat -c %s "$BAG_DIRECTORY/$IN_BAG")
avail=$(df --output=avail -B1 "$BAG_DIRECTORY" | tail -1)
(( avail > need + 2 * 1024 ** 3 )) || {
  echo "Not enough disk to merge: need ~$((need / 1024 ** 3 + 2)) GiB free, have $((avail / 1024 ** 3)) GiB"
  exit 1
}

wm_exec() {  # wm_exec <bash command>, run in the world_modeling container with the workspace sourced
  docker exec -i -e PYTHONUNBUFFERED=1 -e RCUTILS_LOGGING_BUFFERED_STREAM=0 "$WM" \
    bash -c "source /ws/install/setup.bash && $1"
}

# The dev container mounts the sources but may not have world_modeling_bringup built
EIDOS_CONFIG="$(wm_exec 'for f in "$(ros2 pkg prefix world_modeling_bringup 2>/dev/null)/share/world_modeling_bringup/config/eidos_slam.yaml" \
  /ws/src/world_modeling/world_modeling_bringup/config/eidos_slam.yaml; do [[ -f "$f" ]] && { echo "$f"; break; }; done' || true)"
[[ -n "$EIDOS_CONFIG" ]] || { echo "eidos_slam.yaml not found in $WM"; exit 1; }

stop_eidos() {
  docker exec "$WM" pkill -INT -f "$LAUNCH_PATTERN" 2>/dev/null || true
  for _ in $(seq 20); do
    docker exec "$WM" pgrep -f "$EIDOS_PROCS" >/dev/null 2>&1 || return 0
    sleep 1
  done
  docker exec "$WM" pkill -KILL -f "$EIDOS_PROCS" 2>/dev/null || true
}

cleanup() {
  # docker exec does not forward Ctrl-C, so stop the container-side processes explicitly
  docker exec "$WM" pkill -INT -f "bag (play|record) .*(${IN_BAG}|${EIDOS_RAW})" 2>/dev/null || true
  stop_eidos
}
trap cleanup EXIT

# ---- 1. Start eidos ----
if docker exec "$WM" pgrep -f "$EIDOS_PROCS" >/dev/null 2>&1; then
  echo "==> Stopping the eidos already running in $WM"
  stop_eidos
fi

echo "==> Reading static TFs from $IN_BAG (config: $EIDOS_CONFIG)"
docker exec "$WM" mkdir -p /tmp/eidos_offline
EXTRA_FROM=""
if [[ -n "$EXTRA_PARAMS" ]]; then
  echo "    extra parameters from $EXTRA_PARAMS:"; sed 's/^/      /' "$EXTRA_PARAMS"
  docker exec -i "$WM" bash -c 'cat > /tmp/eidos_offline/extra_params.yaml' <"$EXTRA_PARAMS"
  EXTRA_FROM="- from: /tmp/eidos_offline/extra_params.yaml"
fi
wm_exec "python3 - '$WM_BAGS/$IN_BAG' '$EIDOS_CONFIG' '$RATE'" <<'PY' | tee "$LOG_DIR/offline_config.txt"
import sys

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

bag, config, rate = sys.argv[1], sys.argv[2], float(sys.argv[3])
BASE, LIDAR, IMU = 'base_footprint', 'lidar_cc', 'imu_link'
# eve_description eve_kia_soul_ev.xacro imu_mount joint (parent lidar_cc), used when the bag has no imu_link
IMU_JOINT = {'xyz': (-0.973762, -0.033626, -1.316802), 'rpy': (0.0, 0.0, -0.083116)}

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
edges = {}  # child -> TransformStamped
while reader.has_next():
    for tf in deserialize_message(reader.read_next()[1], TFMessage).transforms:
        old = edges.get(tf.child_frame_id)
        if old is not None and (old.header.frame_id, old.transform) != (tf.header.frame_id, tf.transform):
            sys.exit(f'/tf_static changes {tf.child_frame_id} during the bag; refusing to pick one')
        edges[tf.child_frame_id] = tf


def chain(frame):
    out = []
    while frame != BASE:
        if frame not in edges:
            return None
        out.append(edges[frame])
        frame = edges[frame].header.frame_id
    return out


lidar_chain = chain(LIDAR)
if lidar_chain is None:
    sys.exit(f'/tf_static has no {BASE} -> {LIDAR} chain')
imu_chain = chain(IMU)
nodes = {(t.header.frame_id, t.child_frame_id): t for t in lidar_chain + (imu_chain or [])}


def node(name, parent, child, args):
    return (f'  - node:\n      pkg: tf2_ros\n      exec: static_transform_publisher\n'
            f'      name: offline_tf_{name}\n      args: "{args} --frame-id {parent} --child-frame-id {child}"\n')


launch = 'launch:\n'
for (parent, child), t in nodes.items():
    tr, q = t.transform.translation, t.transform.rotation
    launch += node(child, parent, child, f'--x {tr.x!r} --y {tr.y!r} --z {tr.z!r} '
                   f'--qx {q.x!r} --qy {q.y!r} --qz {q.z!r} --qw {q.w!r}')
    print(f'  bag /tf_static   {parent} -> {child}: xyz ({tr.x:.6f}, {tr.y:.6f}, {tr.z:.6f}) '
          f'q ({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})')
if imu_chain is None:
    (x, y, z), (r, p, yw) = IMU_JOINT['xyz'], IMU_JOINT['rpy']
    launch += node(IMU, LIDAR, IMU, f'--x {x} --y {y} --z {z} --roll {r} --pitch {p} --yaw {yw}')
    print(f'  eve_description  {LIDAR} -> {IMU}: xyz ({x}, {y}, {z}) rpy ({r}, {p}, {yw})  [bag has no {IMU}]')

with open(config) as f:
    cfg = yaml.safe_load(f)
slam_rate = cfg['/**/eidos_node']['ros__parameters']['slam_rate'] * rate
tick_rate = cfg['/**/eidos_transform_node']['ros__parameters']['tick_rate'] * rate
print(f'  wall timers at {rate}x: slam_rate {slam_rate} Hz, eidos_transform tick_rate {tick_rate} Hz')

with open('/tmp/eidos_offline/static_tfs.launch.yaml', 'w') as f:
    f.write(launch)
with open('/tmp/eidos_offline/params.yaml', 'w') as f:
    f.write(f'''/**/eidos_node:
  ros__parameters:
    slam_rate: {slam_rate}
    liso_factor:
      initialization:
        warmup_samples: 10
        stationary_gyr_threshold: 1000.0
    loop_closure_cloud_visualization:
      dump_dir: ""
/**/eidos_transform_node:
  ros__parameters:
    tick_rate: {tick_rate}
''')
PY

docker exec -i "$WM" bash -c 'cat > /tmp/eidos_offline/eidos_offline.launch.yaml' <<EOF
launch:
  - include:
      file: /tmp/eidos_offline/static_tfs.launch.yaml

  - node:
      pkg: eidos_transform
      exec: eidos_transform_node
      name: eidos_transform_node
      namespace: world_modeling
      output: screen
      param:
        - from: $EIDOS_CONFIG
        - from: /tmp/eidos_offline/params.yaml
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
        - from: $EIDOS_CONFIG
        - from: /tmp/eidos_offline/params.yaml
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
          value: 10.0
        - name: bond_timeout_s
          value: 4.0
        - name: bond_enabled
          value: false
EOF

echo "==> Starting eidos in $WM (log: $LOG_DIR/eidos.log)"
wm_exec 'exec ros2 launch /tmp/eidos_offline/eidos_offline.launch.yaml' >"$LOG_DIR/eidos.log" 2>&1 &
EIDOS_PID=$!

for _ in $(seq 60); do
  grep -q "WARMING_UP" "$LOG_DIR/eidos.log" && break
  kill -0 "$EIDOS_PID" 2>/dev/null || { echo "eidos exited during startup:"; tail -20 "$LOG_DIR/eidos.log"; exit 1; }
  sleep 1
done
grep -q "WARMING_UP" "$LOG_DIR/eidos.log" || { echo "eidos did not activate within 60 s; see $LOG_DIR/eidos.log"; exit 1; }

# Surface state changes while the bag plays
tail -n 0 -f --pid="$EIDOS_PID" "$LOG_DIR/eidos.log" \
  | grep --line-buffered -E "warmup complete|TRACKING.*Beginning|GICP failed|stale|ERROR|FATAL" &

# ---- 2. Record + play ----
echo "==> Recording eidos topics -> $EIDOS_RAW"
wm_exec "cd '$WM_BAGS' && exec ros2 bag record -o '$EIDOS_RAW' --use-sim-time --storage mcap ${RECORD_TOPICS[*]}" \
  >"$LOG_DIR/record.log" 2>&1 &
REC_PID=$!
sleep 5  # let the recorder subscribe before data flows

echo "==> Playing $IN_BAG at ${RATE}x"
wm_exec "exec ros2 bag play '$WM_BAGS/$IN_BAG' --clock $CLOCK_HZ -r $RATE --topics ${PLAY_TOPICS[*]}" \
  >"$LOG_DIR/play.log" 2>&1

echo "==> Playback done; flushing eidos output"
sleep 5
docker exec "$WM" pkill -INT -f "bag record -o $EIDOS_RAW" || true
wait "$REC_PID" || true
stop_eidos
wait "$EIDOS_PID" 2>/dev/null || true

grep -q "TRACKING" "$LOG_DIR/eidos.log" || { echo "eidos never reached TRACKING; see $LOG_DIR/eidos.log"; exit 1; }

# ---- 3. Log time := header stamp for the eidos messages, then check them against the scans ----
echo "==> Restamping eidos messages -> $EIDOS_RESTAMPED"
wm_exec "python3 - '$WM_BAGS/$EIDOS_RAW' '$WM_BAGS/$EIDOS_RESTAMPED' '$WM_BAGS/$IN_BAG' '$LIDAR_TOPIC'" <<'PY' | tee "$LOG_DIR/restamp.txt"
import struct
import sys
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from visualization_msgs.msg import Marker, MarkerArray

src, dst, in_bag, lidar_topic = sys.argv[1:5]


def open_reader(uri, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def header_stamp_ns(topic, data):
    # CDR: 4-byte encapsulation header, then std_msgs/Header.stamp (int32 sec, uint32 nanosec).
    # tf2_msgs/TFMessage starts with a uint32 sequence length before the first transform's header.
    endian = '<' if data[1] & 1 else '>'
    offset = 4
    if topic == '/tf':
        if len(data) < 16 or struct.unpack_from(endian + 'I', data, 4)[0] == 0:
            return None
        offset = 8
    if len(data) < offset + 8:
        return None
    sec, nanosec = struct.unpack_from(endian + 'iI', data, offset)
    return sec * 1_000_000_000 + nanosec


def marker_stamp_ns(data):
    # MarkerArray has no top-level header, and eidos leads each one with a stamp-0 DELETEALL.
    # The drawing markers all carry the render time.
    stamps = {m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
              for m in deserialize_message(data, MarkerArray).markers if m.action != Marker.DELETEALL}
    return max(stamps) if stamps else None


# Scan stamps and time span of the input bag, to place and check the eidos output
reader = open_reader(in_bag, [lidar_topic])
scans = []
while reader.has_next():
    topic, data, _ = reader.read_next()
    scans.append(header_stamp_ns(topic, data))
meta = reader.get_metadata()
bag_start = meta.starting_time.nanoseconds
bag_end = bag_start + meta.duration.nanoseconds
del reader
scan_set = set(scans)

reader = open_reader(src)
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
marker_topics = set()
for topic_meta in reader.get_all_topics_and_types():
    writer.create_topic(topic_meta)
    if topic_meta.type == 'visualization_msgs/msg/MarkerArray':
        marker_topics.add(topic_meta.name)

# eidos stamps these with the latest keyframe's time and republishes them every tick until the
# next keyframe. Keep the first message per keyframe (the estimate as of that keyframe).
KEYFRAME_TOPICS = {'/world_modeling/slam/pose', '/world_modeling/slam/odometry'}
# Stamped with the lidar scan they were computed from
SCAN_TOPICS = {'/world_modeling/liso/odometry', '/world_modeling/liso/odometry_incremental'}

kept, dropped, fallback = Counter(), Counter(), Counter()
last_stamp = {}
stamps = {t: [] for t in SCAN_TOPICS | KEYFRAME_TOPICS}
while reader.has_next():
    topic, data, recorded = reader.read_next()
    stamp = marker_stamp_ns(data) if topic in marker_topics else header_stamp_ns(topic, data)
    if stamp is None and topic in marker_topics:
        stamp = recorded  # only DELETEALL: no render time to use
        fallback[topic] += 1
    if stamp is None or not bag_start <= stamp <= bag_end:
        dropped[(topic, 'stamp outside the bag')] += 1
        continue
    if topic in KEYFRAME_TOPICS:
        if last_stamp.get(topic) == stamp:
            dropped[(topic, 'keyframe republish')] += 1
            continue
        last_stamp[topic] = stamp
    if topic in stamps:
        stamps[topic].append(stamp)
    writer.write(topic, data, stamp)
    kept[topic] += 1
del writer

for topic in sorted(kept):
    print(f'  {topic}: kept {kept[topic]}')
for (topic, why), n in sorted(dropped.items()):
    print(f'  {topic}: dropped {n} ({why})')
for topic, n in sorted(fallback.items()):
    print(f'  {topic}: {n} with no drawn markers kept at their recorded time')

# Every per-scan and keyframe pose must sit exactly on a scan stamp, once per scan
ok = True
print(f'  {lidar_topic}: {len(scans)} scans')
for topic in sorted(stamps):
    s = stamps[topic]
    off = [x for x in s if x not in scan_set]
    dup = len(s) - len(set(s))
    print(f'  {topic}: {len(s)} msgs, {len(s) - len(off)} on a scan stamp, {len(off)} off, {dup} duplicated')
    ok &= not off and not dup
liso = sorted(set(stamps['/world_modeling/liso/odometry']))
if liso:
    tracked = [s for s in scans if liso[0] <= s <= liso[-1]]
    missing = sorted(set(tracked) - set(liso))
    print(f'  scans from the first to the last LISO pose: {len(tracked)}, without a LISO pose: {len(missing)}')
    print(f'  scans before the first LISO pose: {sum(s < liso[0] for s in scans)}, after the last: '
          f'{sum(s > liso[-1] for s in scans)}')
    if missing:
        print('  first missing scan stamps: ' + ' '.join(str(m) for m in missing[:10]))
if not ok:
    sys.exit('eidos output is not aligned with the lidar scans')
PY

# ---- 4. Merge ----
RESTAMPED_MCAP="$(find "$BAG_DIRECTORY/$EIDOS_RESTAMPED" -name '*.mcap' | head -1)"
echo "==> Merging -> $MERGED"
mcap merge "$BAG_DIRECTORY/$IN_BAG" "$RESTAMPED_MCAP" --allow-duplicate-metadata -o "$BAG_DIRECTORY/$MERGED"

echo "==> Done: $BAG_DIRECTORY/$MERGED"
mcap info "$BAG_DIRECTORY/$MERGED" | grep -E "duration|messages:|/world_modeling|/tf "
echo "Intermediate files ($EIDOS_RAW, $EIDOS_RESTAMPED, logs in $LOG_DIR) can be deleted."
