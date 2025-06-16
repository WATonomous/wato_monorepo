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
import cv2
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage, PointCloud2
#from anomaly_detection_msgs.msg import Anomaly


class Anomaly(Node):

    def __init__(self):
        super().__init__('Anomaly_Node_Init')
        # Declare and get the parameters
        self.declare_parameter("model_path")
        #self.declare_parameter("model_config_path")
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.model_path = self.get_parameter("model_path").value
        #self.model_config_path = self.get_parameter("model_config_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value
        self.camera_data = self.get_parameter("camera_topic").value

        # self.__buffer_capacity = self.get_parameter('buffer_capacity') \
        #     .get_parameter_value().integer_value
        # Initialize Anomaly Core Logic for Deserialization
        # self.__anomaly = AnomalyCore()

        # Pubs and Subs
        #self.publisher_ = self.create_publisher(Anomaly, '/anomaly', 10)
        self.subscription = self.create_subscription(
            Image, self.camera_data, self.camera_callback, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.lidar_data, self.lidar_callback, 10)

    def lidar_callback(self, msg):
        pass
    def camera_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    python_anomaly = Anomaly()

    rclpy.spin(python_anomaly)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_anomaly.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
