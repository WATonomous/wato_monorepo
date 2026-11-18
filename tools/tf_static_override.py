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
"""A recorded bag's static TF with some transforms replaced, for replaying it with fixed extrinsics.

    # host (rosbags): the bag's /tf_static, overrides applied, as one YAML
    tools/tf_static_override.py extract --bag <bag> --override <overrides.yaml> --out <tf_static.yaml>

    # in a ROS container: publish that YAML on /tf_static (latched) until stopped
    python3 tf_static_override.py publish <tf_static.yaml>

Overrides and output share one format, a list of transforms with URDF-style fixed-axis rpy:

    - {parent: base_link, child: lidar_cc, xyz: [0, 0, 0.35], rpy: [-0.006167, 0.042198, 0.125664]}

An override must name a transform the bag has (a typo would otherwise add a second tree branch).
tools/eidos_visual_eval.sh plays the bag without its /tf_static and publishes this instead when
TF_STATIC is set.
"""

import argparse
import sys
from pathlib import Path

import yaml


def extract(bag: Path, override: Path, out: Path) -> int:
    import numpy as np
    from rosbags.highlevel import AnyReader
    from scipy.spatial.transform import Rotation

    transforms = {}
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == "/tf_static"]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            for tr in msg.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
                transforms[(tr.header.frame_id, tr.child_frame_id)] = {
                    "parent": tr.header.frame_id,
                    "child": tr.child_frame_id,
                    "xyz": [round(float(v), 6) for v in (p.x, p.y, p.z)],
                    "rpy": [round(float(v), 6) for v in rpy],
                }
    if not transforms:
        print(f"no /tf_static in {bag}")
        return 1
    for entry in yaml.safe_load(override.read_text()) or []:
        key = (entry["parent"], entry["child"])
        if key not in transforms:
            print(f"override {key[0]} -> {key[1]} is not in the bag's /tf_static")
            return 1
        old = transforms[key]
        transforms[key] = {
            "parent": key[0],
            "child": key[1],
            "xyz": [float(v) for v in entry["xyz"]],
            "rpy": [float(v) for v in entry["rpy"]],
        }
        moved = np.linalg.norm(np.subtract(entry["xyz"], old["xyz"]))
        turned = Rotation.from_euler("xyz", old["rpy"]).inv() * Rotation.from_euler(
            "xyz", entry["rpy"]
        )
        print(
            f"{key[0]} -> {key[1]}: moved {moved:.3f} m, turned {np.degrees(turned.magnitude()):.3f} deg"
        )
    out.write_text(
        "# Static TF of "
        + bag.name
        + " with overrides from "
        + override.name
        + "\n"
        + yaml.safe_dump(list(transforms.values()), default_flow_style=None, sort_keys=False)
    )
    print(f"wrote {len(transforms)} transforms to {out}")
    return 0


def _quaternion(rpy):
    import math

    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    # fixed-axis x, y, z (URDF rpy): q = qz * qy * qx
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _publish(path, rclpy, TransformStamped, QoSProfile, DurabilityPolicy, TFMessage):
    entries = yaml.safe_load(Path(path).read_text())
    rclpy.init()
    node = rclpy.create_node("tf_static_override")
    pub = node.create_publisher(
        TFMessage,
        "/tf_static",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    msg = TFMessage()
    stamp = node.get_clock().now().to_msg()
    for e in entries:
        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = e["parent"]
        tr.child_frame_id = e["child"]
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = (
            float(v) for v in e["xyz"]
        )
        q = _quaternion([float(v) for v in e["rpy"]])
        (
            tr.transform.rotation.x,
            tr.transform.rotation.y,
            tr.transform.rotation.z,
            tr.transform.rotation.w,
        ) = q
        msg.transforms.append(tr)
    pub.publish(msg)
    node.get_logger().info(f"published {len(msg.transforms)} static transforms from {path}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    ex = sub.add_parser("extract")
    ex.add_argument("--bag", type=Path, required=True)
    ex.add_argument("--override", type=Path, required=True)
    ex.add_argument("--out", type=Path, required=True)
    pub = sub.add_parser("publish")
    pub.add_argument("yaml", type=Path)
    args = parser.parse_args(argv)
    if args.command == "extract":
        return extract(args.bag, args.override, args.out)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    return _publish(args.yaml, rclpy, TransformStamped, QoSProfile, DurabilityPolicy, TFMessage)


if __name__ == "__main__":
    sys.exit(main())
