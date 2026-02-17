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
# Dummy subscriber for all 12 compressed camera topics.
# Triggers the lazy compressed_image_transport publishers for profiling.
# Usage: python3 dummy_compressed_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

CAMERAS = [
    "camera_pano_nn",
    "camera_pano_ne",
    "camera_pano_ee",
    "camera_pano_se",
    "camera_pano_ss",
    "camera_pano_sw",
    "camera_pano_ww",
    "camera_pano_nw",
    "camera_lower_ne",
    "camera_lower_se",
    "camera_lower_sw",
    "camera_lower_nw",
]


class DummyCompressedSubscriber(Node):
    def __init__(self):
        super().__init__("dummy_compressed_subscriber")
        self.subs = []
        for cam in CAMERAS:
            sub = self.create_subscription(
                CompressedImage,
                f"/{cam}/image_rect_compressed",
                lambda msg, c=cam: None,
                10,
            )
            self.subs.append(sub)
        self.get_logger().info(f"Subscribed to {len(self.subs)} compressed topics")


def main():
    rclpy.init()
    node = DummyCompressedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
