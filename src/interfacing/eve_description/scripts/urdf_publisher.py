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
Publishes URDF description with volatile QoS for visualization in Foxglove.
Avoids transient_local QoS which is not supported by iceoryx.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import sys


class URDFPublisher(Node):
    def __init__(self, urdf_content):
        super().__init__("urdf_publisher")

        # Create QoS profile with volatile durability (compatible with iceoryx)
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(String, "/robot_description_viz", qos)

        self.urdf_content = urdf_content

        # Publish periodically at 1 Hz for visualization tools
        self.timer = self.create_timer(1.0, self.publish_urdf)

        self.get_logger().info(
            "URDF publisher started, publishing to /robot_description_viz"
        )

    def publish_urdf(self):
        msg = String()
        msg.data = self.urdf_content
        self.publisher.publish(msg)


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: urdf_publisher.py <path_to_urdf>")
        sys.exit(1)

    urdf_path = sys.argv[1]

    try:
        with open(urdf_path, "r") as f:
            urdf_content = f.read()
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        sys.exit(1)

    rclpy.init(args=args)
    node = URDFPublisher(urdf_content)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
