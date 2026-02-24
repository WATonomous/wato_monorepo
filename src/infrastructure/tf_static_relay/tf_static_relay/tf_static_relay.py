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
"""Republish /tf_static at a fixed rate.

Temporary workaround so that rosbag2 captures static transforms in every
split chunk when recording with --max-bag-size or --max-bag-duration.

Remove once https://github.com/ros2/rosbag2/pull/2342 is released.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelay(Node):
    def __init__(self):
        super().__init__("tf_static_relay")

        self.declare_parameter("publish_rate", 1.0)
        rate = self.get_parameter("publish_rate").value

        tf_static_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._transforms = {}  # (parent, child) -> TransformStamped

        self._sub = self.create_subscription(
            TFMessage, "/tf_static", self._cb, tf_static_qos
        )
        self._pub = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"tf_static relay running at {rate} Hz")

    def _cb(self, msg):
        for t in msg.transforms:
            self._transforms[(t.header.frame_id, t.child_frame_id)] = t

    def _publish(self):
        if self._transforms:
            msg = TFMessage()
            msg.transforms = list(self._transforms.values())
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfStaticRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
