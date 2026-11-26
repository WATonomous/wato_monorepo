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
"""Bag-testing tool: broadcast map -> base_footprint TF from recorded odometry.

For replaying bags without localization. Do not run it alongside anything else
that publishes the map frame.

The odometry child frame (the IMU/GNSS frame) already has a parent in the robot
TF tree, so the transform is moved to the tree's root frame before broadcasting.
Map translations are relative to the first pose received, which keeps them small
enough for the float32 transforms used by SemVecNet.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def to_matrix(translation, rotation):
    q = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def to_quaternion(r):
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
    # Each row is the quaternion scaled by 4 times one of its components.
    candidates = np.array(
        [
            [
                1 + r[0, 0] - r[1, 1] - r[2, 2],
                r[0, 1] + r[1, 0],
                r[0, 2] + r[2, 0],
                r[2, 1] - r[1, 2],
            ],
            [
                r[0, 1] + r[1, 0],
                1 - r[0, 0] + r[1, 1] - r[2, 2],
                r[1, 2] + r[2, 1],
                r[0, 2] - r[2, 0],
            ],
            [
                r[0, 2] + r[2, 0],
                r[1, 2] + r[2, 1],
                1 - r[0, 0] - r[1, 1] + r[2, 2],
                r[1, 0] - r[0, 1],
            ],
            [
                r[2, 1] - r[1, 2],
                r[0, 2] - r[2, 0],
                r[1, 0] - r[0, 1],
                1 + r[0, 0] + r[1, 1] + r[2, 2],
            ],
        ]
    )
    # Use the row for the largest component to avoid dividing by a small number.
    best = candidates[np.argmax(np.diag(candidates))]
    return best / np.linalg.norm(best)


class OdomToMapTf(Node):
    def __init__(self):
        super().__init__("odom_to_map_tf")
        odom_topic = self.declare_parameter("odom_topic", "/novatel/oem7/odom").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.root_frame = self.declare_parameter("root_frame", "base_footprint").value

        self.child_from_root = {}
        self.origin = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Waiting for {odom_topic} to broadcast {self.map_frame} -> {self.root_frame}"
        )

    def on_odom(self, msg):
        child = msg.child_frame_id.lstrip("/")
        pose = msg.pose.pose
        q = pose.orientation
        values = [pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w]
        if not np.all(np.isfinite(values)) or np.linalg.norm(values[3:]) < 1e-6:
            self.get_logger().warning(
                "Skipping invalid odometry pose", throttle_duration_sec=5.0
            )
            return

        if child not in self.child_from_root:
            try:
                # The sensor mount is fixed, so one lookup is enough.
                mount = self.tf_buffer.lookup_transform(child, self.root_frame, Time())
            except TransformException as error:
                self.get_logger().warning(
                    f"Waiting for the {child} <- {self.root_frame} transform: {error}",
                    throttle_duration_sec=5.0,
                )
                return
            self.child_from_root[child] = to_matrix(
                mount.transform.translation, mount.transform.rotation
            )

        map_from_root = to_matrix(pose.position, q) @ self.child_from_root[child]
        if self.origin is None:
            self.origin = map_from_root[:3, 3].copy()
            self.get_logger().info(
                f"Broadcasting {self.map_frame} -> {self.root_frame} "
                f"(odometry {msg.header.frame_id} -> {child})"
            )

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.root_frame
        x, y, z = (float(v) for v in map_from_root[:3, 3] - self.origin)
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = x, y, z
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = (
            float(v) for v in to_quaternion(map_from_root[:3, :3])
        )
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomToMapTf())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
