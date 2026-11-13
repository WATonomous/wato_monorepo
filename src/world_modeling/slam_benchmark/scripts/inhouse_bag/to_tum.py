#!/usr/bin/env python3
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
"""
Convert trajectories into TUM format ("stamp x y z qx qy qz qw") for evaluate.py / evo.

  gnss   NavSatFix topic in a bag -> UTM positions (reference trajectory)
  pose   PoseStamped / Odometry topic in a bag (e.g. /world_modeling/slam/pose)
  eidos  CSV from `eidos poses <map_file>` (optimized keyframes)

RTAB-Map needs no conversion: `rtabmap-export --poses --poses_format 10` already writes TUM.
"""

import argparse
import csv
import math
import sys


def open_bag_reader(uri, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_id = "mcap" if uri.endswith(".mcap") else ""
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(
            f"Topic '{topic}' not in bag. Available:\n  " + "\n  ".join(sorted(types))
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(types[topic])
    while reader.has_next():
        _, data, _ = reader.read_next()
        yield deserialize_message(data, msg_type)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rpy_to_quat(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) Euler angles to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def write_tum(path, rows, header):
    rows = sorted(rows, key=lambda r: r[0])
    with open(path, "w") as f:
        f.write(f"# {header}\n# timestamp x y z qx qy qz qw\n")
        for r in rows:
            f.write(
                " ".join(f"{v:.9f}" if i == 0 else f"{v:.6f}" for i, v in enumerate(r))
                + "\n"
            )
    print(f"Wrote {len(rows)} poses to {path}")


def cmd_gnss(args):
    from pyproj import Transformer

    rows, transformer, skipped, statuses, stds = [], None, 0, {}, []
    for msg in open_bag_reader(args.bag, args.topic):
        statuses[msg.status.status] = statuses.get(msg.status.status, 0) + 1
        # status < 0 means no fix; position_covariance[0] is east variance (m^2)
        if msg.status.status < 0 or msg.position_covariance[0] > args.max_cov:
            skipped += 1
            continue
        stds.append(math.sqrt(msg.position_covariance[0]))
        if transformer is None:
            zone = int((msg.longitude + 180.0) / 6.0) + 1
            epsg = (32600 if msg.latitude >= 0 else 32700) + zone
            transformer = Transformer.from_crs(
                "EPSG:4326", f"EPSG:{epsg}", always_xy=True
            )
            print(f"Projecting to UTM zone {zone} (EPSG:{epsg})")
        e, n = transformer.transform(msg.longitude, msg.latitude)
        rows.append(
            (stamp_to_sec(msg.header.stamp), e, n, msg.altitude, 0.0, 0.0, 0.0, 1.0)
        )
    if not rows:
        sys.exit("No usable fixes found.")
    print(f"Skipped {skipped} fixes (no fix or variance > {args.max_cov} m^2)")
    # NavSatStatus: 0 = single-point fix (~1 m), 1 = SBAS, 2 = GBAS/RTK (~cm)
    print(f"Fix status counts: {dict(sorted(statuses.items()))}")
    print(
        f"Median reported east std of kept fixes: {sorted(stds)[len(stds) // 2]:.2f} m"
    )
    write_tum(args.output, rows, f"GNSS {args.topic} from {args.bag}, UTM EPSG:{epsg}")


def cmd_pose(args):
    rows, at_origin = [], 0
    for msg in open_bag_reader(args.bag, args.topic):
        pose = (
            msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        )  # Odometry vs PoseStamped
        p, q = pose.position, pose.orientation
        # Eidos publishes an all-zero pose until it reaches TRACKING; those are not estimates
        if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
            at_origin += 1
            continue
        rows.append((stamp_to_sec(msg.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    print(f"Dropped {at_origin} poses at exactly (0, 0, 0)")
    write_tum(args.output, rows, f"{args.topic} from {args.bag}")


def cmd_eidos(args):
    rows = []
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            q = rpy_to_quat(float(r["roll"]), float(r["pitch"]), float(r["yaw"]))
            rows.append(
                (float(r["time"]), float(r["x"]), float(r["y"]), float(r["z"]), *q)
            )
    write_tum(args.output, rows, f"Eidos keyframes from {args.csv}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("gnss", help="NavSatFix topic -> UTM reference trajectory")
    p.add_argument("bag")
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--topic", default="/novatel/oem7/fix")
    p.add_argument(
        "--max-cov",
        type=float,
        default=5.0,
        help="Drop fixes with east variance above this (m^2, same default as Eidos)",
    )
    p.set_defaults(func=cmd_gnss)

    p = sub.add_parser("pose", help="PoseStamped or Odometry topic -> TUM")
    p.add_argument("bag")
    p.add_argument("--topic", required=True)
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(func=cmd_pose)

    p = sub.add_parser("eidos", help="`eidos poses` CSV -> TUM")
    p.add_argument("csv")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(func=cmd_eidos)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
