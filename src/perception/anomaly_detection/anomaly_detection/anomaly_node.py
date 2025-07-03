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
from anomaly_detection_msgs.msg import Anomaly, BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path
from anomalib.data import PredictDataset
from anomalib.engine import Engine
from anomalib.models import EfficientAd

"""
Initialize self.model = EfficientAd() and self.engine = Engine() in the __init__ method 
of the Anomaly class, and then just use self.model and self.engine in anomaly_inference.
"""

class Anomaly(Node):
    MIN_BOUNDING_BOX_AREA = 100 # Minimum area for a detected bounding box (in pixels^2)
    ANOMALY_MASK_THRESHOLD = 0.5 # Threshold for the anomaly mask (0.0 to 1.0)

    def __init__(self):
        super().__init__('Anomaly_Node_Init')
        self.bridge = CvBridge()
        # Declare and get the parameters
        self.declare_parameter("model_path")
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.declare_parameter("debug_image_topic", "/anomaly_detection/debug_image")

        self.model_path = self.get_parameter("model_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value
        self.camera_data = self.get_parameter("camera_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        # Pubs and Subs
        self.publisher_ = self.create_publisher(Anomaly, '/anomaly', 10)
        self.subscription_camera = self.create_subscription(
            Image, self.camera_data, self.camera_callback, 10)
        self.subscription_lidar = self.create_subscription(
            PointCloud2, self.lidar_data, self.lidar_callback, 10)
        self.publisher_debug_image = self.create_publisher(Image, self.debug_image_topic, 10)

    def lidar_callback(self, msg):
        pass

    def camera_callback(self, msg):
        try:
            openCVImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #score = anomaly_inference(openCVImage, self.model_path)
            score, boxes = self.anomaly_inference(openCVImage, self.model_path)
            if score !=  None:
                self.get_logger().info(f'Image received. The anomaly score is {score:.4f}, Detected Boxes: {len(boxes)}')
            else:
                self.get_logger().warn('Anomaly inference returned no score or boxes.')
            #self.get_logger().info(f'Image received. The anomaly score is {score}')
            debug_image = openCVImage.copy() # Work on a copy to preserve original
             # --- Draw bounding boxes on a copy of the image and publish ---
            for (x, y, w, h) in boxes:
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 2) #red rectangle
            
            # ------------------------------- publishing ----------------------------------
            # anomaly_msg = Anomaly()
            # anomaly_msg.score = score
            # self.publisher_.publish(anomaly_msg)
            
            try:
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_image_msg.header = msg.header
                self.publisher_debug_image.publish(debug_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Error while converting and publishing debug image: {e}")

            if score is not None:
                anomaly_msg = Anomaly()
                anomaly_msg.score = float(score)
                # Might add more fields to Anomaly message like bounding box list
                #self.publisher_.publish(anomaly_msg)

                 # Populate the bounding_boxes list in the Anomaly message
                for (x, y, w, h) in boxes:
                    bbox_msg = BoundingBox2D()
                    bbox_msg.x_min = int(x)
                    bbox_msg.y_min = int(y)
                    bbox_msg.width = int(w)
                    bbox_msg.height = int(h)
                    anomaly_msg.bounding_boxes.append(bbox_msg)
                    
                self.publisher_anomaly.publish(anomaly_msg)

        except CvBridgeError as e_cv:
            self.get_logger().error(f"CvBridge Error in camera_callback: {e}")

        except Exception as e:
            self.get_logger().error(f"General error in camera_callback: {e}")

    def anomaly_inference(self, img, model_path):
        model = EfficientAd()
        engine = Engine()

        # Save the incoming image to a temporary file for PredictDataset
        temp_img_path = "/tmp/anomaly_input.jpg"
        try:
            cv2.imwrite(temp_img_path, img)
        except:
            self.get_logger().error(f"Failed to write temporary image file: {e}")
            return None, []

        #dataset = PredictDataset(
        #    path=Path(temp_img_path),
         #   image_size=(256, 256),
        #)

        #predictions = engine.predict(
        #    model=model,
        #    dataset=dataset,
         #   ckpt_path=model_path,
        #)
        try:
            dataset = PredictDataset(
                path=Path(temp_img_path),
                image_size=(256, 256), # matches model's input
            )

            predictions = self.engine.predict(
                model=self.model,
                dataset=dataset,
                ckpt_path=model_path,
            )
        except Exception as e:
            self.get_logger().error(f"Error during anomalib prediction: {e}")
            return None, []
        finally:
            # Clean up the temporary file after use
            if Path(temp_img_path).exists():
                Path(temp_img_path).unlink()

        bounding_boxes = []
        if predictions is not None and len(predictions) > 0:
            for prediction in predictions:
                pred_label = prediction.pred_label  # 0: normal, 1: anomalous
                pred_score = prediction.pred_score
                print(f'the prediction label is {pred_label} and pred score is {pred_score}')

                mask = prediction.pred_mask.cpu().numpy()

                # Threshold the mask to get binary image
                # --- ANOMALY_MASK_THRESHOLD ---
                #0.5 is a starting threshold. Way may need to change it.

                mask_bin = (mask > self.ANOMALY_MASK_THRESHOLD).astype('uint8') * 255

                # Ensure the mask is correctly resized to the original image dimensions
                mask_bin = cv2.resize(mask_bin, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
                
                contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w * h > self.MIN_BOUNDING_BOX_AREA:  # Filter out small noise boxes
                        bounding_boxes.append((x, y, w, h))

                return pred_score, bounding_boxes
        return None, []

def main(args=None):
    rclpy.init(args=args)
    python_anomaly = Anomaly()
    rclpy.spin(python_anomaly)
    python_anomaly.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
