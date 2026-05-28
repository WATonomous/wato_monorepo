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
"""Live SIL limit monitor.

Watches the ego state, controller command, and actors during the run and publishes a
diagnostic per defined limit so violations are visible in real time (and on the Foxglove
timeline). The authoritative pass/fail verdict is produced post-hoc by sil_analyzer; this
node is for live visibility and optional early abort.
"""

import math
import os

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

from sil_testing.sil_metrics import distance_point_to_polyline
from sil_testing.sil_scenario_schema import load_scenario


class SilMonitorNode(Node):
    def __init__(self):
        super().__init__("sil_monitor_node")
        self.declare_parameter("scenario_file", os.environ.get("SIL_SCENARIO_FILE", ""))
        self.declare_parameter("odom_topic", "/ego/odom")
        self.declare_parameter("command_topic", "/carla/ackermann_control/command")
        self.declare_parameter("detections_topic", "/carla/detections_3d")
        self.declare_parameter("diagnostics_topic", "/sil/diagnostics")
        self.declare_parameter("wheelbase", 2.5667)
        self.declare_parameter("publish_rate_hz", 10.0)

        scenario_file = self.get_parameter("scenario_file").value
        if not scenario_file:
            raise RuntimeError("scenario_file/SIL_SCENARIO_FILE not provided")
        scenario = load_scenario(scenario_file)
        self.limits = scenario.limits
        self.route_xy = (
            [[wp.x, wp.y] for wp in scenario.route.waypoints] if scenario.route else None
        )
        self.wheelbase = self.get_parameter("wheelbase").value

        self.speed = None
        self.ego_xy = None
        self.steering = None
        self.last_steering = None
        self.last_steering_time = None
        self.steering_rate = 0.0
        self.actor_xy = []

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._odom_cb, 10)
        self.create_subscription(AckermannDriveStamped,
                                 self.get_parameter("command_topic").value,
                                 self._command_cb, 10)
        self.create_subscription(Detection3DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._detections_cb, 10)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
        )
        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self._publish_diagnostics)

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        self.ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _command_cb(self, msg: AckermannDriveStamped):
        self.steering = msg.drive.steering_angle
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_steering is not None and self.last_steering_time is not None:
            dt = now - self.last_steering_time
            if dt > 1e-6:
                self.steering_rate = (self.steering - self.last_steering) / dt
        self.last_steering = self.steering
        self.last_steering_time = now

    def _detections_cb(self, msg: Detection3DArray):
        self.actor_xy = [
            (d.bbox.center.position.x, d.bbox.center.position.y) for d in msg.detections
        ]

    def _metric_value(self, name):
        """Return the current value for a limit name, or None if not yet computable."""
        if name == "max_speed_mps":
            return self.speed
        if name == "max_lateral_accel_mps2":
            if self.speed is None or self.steering is None:
                return None
            # Magnitude: lateral accel is signed (negative on left turns).
            return abs(self.speed**2 * math.tan(self.steering) / self.wheelbase)
        if name == "max_steering_rate_radps":
            return abs(self.steering_rate)
        if name == "max_cross_track_error_m":
            if self.ego_xy is None or self.route_xy is None:
                return None
            return distance_point_to_polyline(self.ego_xy[0], self.ego_xy[1], self.route_xy)
        if name == "min_distance_to_actor_m":
            if self.ego_xy is None or not self.actor_xy:
                return None
            return min(
                math.hypot(ax - self.ego_xy[0], ay - self.ego_xy[1])
                for ax, ay in self.actor_xy
            )
        return None

    @staticmethod
    def _violated(name, value, limit):
        # Signed metrics are already reduced to magnitude in _metric_value.
        return value > limit if name.startswith("max_") else value < limit

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        for name, limit in self.limits.items():
            value = self._metric_value(name)
            status = DiagnosticStatus()
            status.name = f"sil/{name}"
            status.hardware_id = "sil_monitor"
            if value is None:
                status.level = DiagnosticStatus.STALE
                status.message = "waiting for data"
            else:
                violated = self._violated(name, value, limit)
                status.level = DiagnosticStatus.ERROR if violated else DiagnosticStatus.OK
                status.message = "VIOLATION" if violated else "ok"
                status.values = [
                    KeyValue(key="value", value=f"{value:.4f}"),
                    KeyValue(key="limit", value=f"{limit:.4f}"),
                ]
            arr.status.append(status)

        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SilMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
