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
"""Republish GNSS fixes with outage windows cut out, to test eidos without GPS for a while.

    gps_outage_relay.py --origin <bag start, s since epoch> --windows 130:200[,250:260] \
        [--input /novatel/oem7/fix] [--output /eval/gps_fix]

A fix is dropped when its header stamp, in seconds after --origin, falls in any window. Point
eidos' gps_factor.gps_topic at --output; anything else that times itself on the GNSS (e.g.
MultiViewOdometryFactor's keyframe timing) can keep the original topic, so keyframes do not change
between runs with and without an outage. Used by tools/eidos_visual_eval.sh (GPS_OUTAGE).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


def parse_windows(text: str):
    windows = []
    for part in text.split(","):
        start, end = (float(v) for v in part.split(":"))
        if end <= start:
            raise ValueError(f"window {part}: end must be after start")
        windows.append((start, end))
    return windows


class GpsOutageRelay(Node):
    def __init__(self, args):
        super().__init__("gps_outage_relay")
        self.origin = args.origin
        self.windows = parse_windows(args.windows)
        self.passed = self.dropped = 0
        self.pub = self.create_publisher(NavSatFix, args.output, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, args.input, self.on_fix, qos_profile_sensor_data)
        self.get_logger().info(
            f"{args.input} -> {args.output}, dropping fixes "
            + ", ".join(f"{a:.1f}-{b:.1f} s" for a, b in self.windows)
            + f" after {self.origin:.3f}"
        )

    def on_fix(self, msg: NavSatFix):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec - self.origin
        if any(a <= t < b for a, b in self.windows):
            self.dropped += 1
        else:
            self.passed += 1
            self.pub.publish(msg)
        if (self.passed + self.dropped) % 1000 == 0:
            self.get_logger().info(f"passed {self.passed}, dropped {self.dropped}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--origin", type=float, required=True)
    parser.add_argument("--windows", required=True)
    parser.add_argument("--input", default="/novatel/oem7/fix")
    parser.add_argument("--output", default="/eval/gps_fix")
    args = parser.parse_args(argv)
    rclpy.init()
    node = GpsOutageRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done: passed {node.passed}, dropped {node.dropped}")
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
