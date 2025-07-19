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
import rclpy
from rclpy.node import Node
from vision_msgs.msg import VisionInfo


class LabelServer(Node):
    def __init__(self):
        super().__init__("label_server")
        self.publisher_ = self.create_publisher(VisionInfo, "vision_info", 10)
        self.label_mapping = {
            1: "car",
            2: "pedestrian",
            3: "cyclist",
        }
        self.publish_labels()

    def publish_labels(self):
        vision_info_msg = VisionInfo()
        vision_info_msg.database_location = "memory"
        vision_info_msg.database_version = "1.0"
        for label_id, class_name in self.label_mapping.items():
            vision_info_msg.class_map[label_id] = class_name
        self.publisher_.publish(vision_info_msg)


def main(args=None):
    rclpy.init(args=args)
    label_server = LabelServer()
    rclpy.spin(label_server)
    label_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
