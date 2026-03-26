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
"""Subscribes to all 3 lidar topics and GPS, prints timestamp deltas in real time.

Usage (inside the interfacing container):
    python3 pps_sync_test.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from novatel_oem7_msgs.msg import BESTPOS


class PPSSyncTest(Node):
    def __init__(self):
        super().__init__("pps_sync_test")
        self.latest = {}

        for topic in [
            "/lidar_cc/velodyne_points",
            "/lidar_ne/velodyne_points",
            "/lidar_nw/velodyne_points",
        ]:
            self.create_subscription(
                PointCloud2, topic, lambda msg, t=topic: self._on_lidar(msg, t), 10
            )

        self.create_subscription(BESTPOS, "/novatel/oem7/bestpos", self._on_gps, 10)

    def _stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_lidar(self, msg, topic):
        self.latest[topic] = self._stamp_to_sec(msg.header.stamp)
        self._print_report()

    def _on_gps(self, msg):
        self.latest["gps"] = self._stamp_to_sec(msg.header.stamp)
        self._print_report()

    def _print_report(self):
        cc = self.latest.get("/lidar_cc/velodyne_points")
        ne = self.latest.get("/lidar_ne/velodyne_points")
        nw = self.latest.get("/lidar_nw/velodyne_points")
        gps = self.latest.get("gps")

        if not (cc and ne and nw):
            return

        cc_ne_ms = (cc - ne) * 1000
        cc_nw_ms = (cc - nw) * 1000
        ne_nw_ms = (ne - nw) * 1000
        gps_str = f"  cc-gps: {(cc - gps) * 1000:+.1f}ms" if gps else ""

        self.get_logger().info(
            f"cc-ne: {cc_ne_ms:+.1f}ms | cc-nw: {cc_nw_ms:+.1f}ms | "
            f"ne-nw: {ne_nw_ms:+.1f}ms{gps_str}"
        )

        for label, delta in [("cc-ne", cc_ne_ms), ("cc-nw", cc_nw_ms)]:
            if abs(delta) > 50:
                self.get_logger().warn(
                    f"PPS SYNC FAIL: {label} delta {delta:.1f}ms > 50ms!"
                )


def main():
    rclpy.init()
    node = PPSSyncTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
