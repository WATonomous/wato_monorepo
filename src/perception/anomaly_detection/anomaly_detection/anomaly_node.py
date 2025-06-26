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

#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from anomaly_detection_msgs.msg import Anomaly

from cv_bridge import CvBridge, CvBridgeError

from pathlib import Path

from anomalib.data import PredictDataset
from anomalib.engine import Engine
from anomalib.models import EfficientAd

class Anomaly(Node):

    def __init__(self):
        super().__init__('Anomaly_Node_Init')
        self.bridge = CvBridge()
        # Declare and get the parameters
        self.declare_parameter("model_path")
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.model_path = self.get_parameter("model_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value
        self.camera_data = self.get_parameter("camera_topic").value

        # Pubs and Subs
        self.publisher_ = self.create_publisher(Anomaly, '/anomaly', 10)
        self.subscription = self.create_subscription(
            Image, self.camera_data, self.camera_callback, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.lidar_data, self.lidar_callback, 10)

    def lidar_callback(self, msg):
        pass

    def camera_callback(self, msg):
        try:
            openCVImage = self.bridge.imgmsg_to_cv2(msg)
            score = anomaly_inference(openCVImage, self.model_path)
            self.get_logger().info(f'Image received. The anomaly score is {score}')
            # Optionally, publish the anomaly score as a message
            # anomaly_msg = Anomaly()
            # anomaly_msg.score = score
            # self.publisher_.publish(anomaly_msg)
        except Exception as e:
            self.get_logger().error(f"Error in camera_callback: {e}")

    def anomaly_inference(img, model_path):
        model = EfficientAd()
        engine = Engine()

        # Save the incoming image to a temporary file for PredictDataset
        temp_img_path = "/tmp/anomaly_input.jpg"
        cv2.imwrite(temp_img_path, img)

        dataset = PredictDataset(
            path=Path(temp_img_path),
            image_size=(256, 256),
        )

        predictions = engine.predict(
            model=model,
            dataset=dataset,
            ckpt_path=model_path,
        )

        if predictions is not None:
            for prediction in predictions:
                pred_label = prediction.pred_label  # 0: normal, 1: anomalous
                pred_score = prediction.pred_score
                print(f'the prediction label is {pred_label} and pred score is {pred_score}')
                return pred_score
        return None

def main(args=None):
    rclpy.init(args=args)
    python_anomaly = Anomaly()
    rclpy.spin(python_anomaly)
    python_anomaly.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
