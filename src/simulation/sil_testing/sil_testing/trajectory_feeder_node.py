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
"""Publishes a reference Trajectory built from the SIL scenario route.

In motion-control SIL this replaces the planner: it converts the scenario's route
(waypoints + target speed, ROS map frame) into a wato_trajectory_msgs/Trajectory that
the controller-under-test consumes. Republished at a fixed rate so the controller's
idle/staleness check never trips.
"""

import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from wato_trajectory_msgs.msg import Trajectory, TrajectoryPoint

from sil_testing.sil_scenario_schema import load_scenario


class TrajectoryFeederNode(Node):
    def __init__(self):
        super().__init__("trajectory_feeder_node")
        self.declare_parameter("scenario_file", os.environ.get("SIL_SCENARIO_FILE", ""))
        self.declare_parameter("output_topic", "/sil/reference_trajectory")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 10.0)

        scenario_file = self.get_parameter("scenario_file").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("output_topic").value

        if not scenario_file:
            raise RuntimeError("scenario_file/SIL_SCENARIO_FILE not provided")

        scenario = load_scenario(scenario_file)
        if scenario.route is None:
            raise RuntimeError("scenario has no 'route'; cannot feed a trajectory")
        self.route = scenario.route

        self.publisher = self.create_publisher(Trajectory, topic, 10)
        rate = self.get_parameter("publish_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Feeding {len(self.route.waypoints)} waypoints @ "
            f"{self.route.target_speed_mps} m/s on '{topic}'"
        )

    def _build_points(self):
        wps = self.route.waypoints
        points = []
        for i, wp in enumerate(wps):
            nxt = wps[min(i + 1, len(wps) - 1)]
            yaw = math.atan2(nxt.y - wp.y, nxt.x - wp.x)
            pose = Pose()
            pose.position.x = wp.x
            pose.position.y = wp.y
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            point = TrajectoryPoint()
            point.pose = pose
            point.max_speed = self.route.target_speed_mps
            points.append(point)
        return points

    def _publish(self):
        msg = Trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.points = self._build_points()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFeederNode()
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
