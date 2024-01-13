# Copyright 2023 WATonomous
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

from sensor_msgs.msg import Image
from traffic_light_detection.traffic_light_detection_core import TrafficLightDetectionCore


class TrafficLightDetection(Node):

    def __init__(self):
        super().__init__('traffic_light_detection')
        # Declare and get the parameters
        self.declare_parameter('version', 1)

        # Initialize Traffic Light Detection Core Logic
        self.__traffic_light_detection = TrafficLightDetectionCore()

        # Initialize ROS2 Constructs
        self.publisher_ = self.create_publisher(Image, '/publish_topic', 10)
        self.subscription = self.create_subscription(Image, '/camera_topic',
                                                     self.image_callback, 10)

    def image_callback(self, msg):
        if not self.check_msg_validity(msg):
            self.get_logger().info('INVALID MSG')
            return

    def check_msg_validity(self, msg):
        return msg.valid


def main(args=None):
    rclpy.init(args=args)

    traffic_light_detection = TrafficLightDetection()

    rclpy.spin(traffic_light_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traffic_light_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
