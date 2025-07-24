"""
Train robust anomaly detection models per camera that can identify:
Blurred or cracked lenses
Unusual road conditions (construction, obstacles)
Rain, snow, fog, or visibility degradation

For each camera:
datasets/
  left/
    train/good/
    test/good/
  center/
    train/good/
    test/good/
  right/
    train/good/
    test/good/

For testing evaluation: 
Even if Anomalib is trained only on train/good, the test/bad is used for evaluation and threshold tuning.
datasets/
  left/
    test/bad/
  center/
    test/bad/
  right/
    test/bad/

Note: Blurring is not available in Anovox so we will have to simulate that using OpenCV. Anovox = test/bad

eg:
import cv2
import numpy as np

def add_blur(image):
    return cv2.GaussianBlur(image, (15, 15), 0)

def add_crack_overlay(image, crack_path="crack.png"):
    crack = cv2.imread(crack_path, cv2.IMREAD_UNCHANGED)  # PNG with alpha
    crack = cv2.resize(crack, (image.shape[1], image.shape[0]))

    # Separate crack RGBA
    crack_rgb = crack[:, :, :3]
    crack_alpha = crack[:, :, 3] / 255.0

    # Blend
    for c in range(3):
        image[:, :, c] = image[:, :, c] * (1 - crack_alpha) + crack_rgb[:, :, c] * crack_alpha
    return image


ALL IMAGES RESIZED TO 256x256

"""

#!/usr/bin/env python3
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
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from anomaly_detection_msgs.msg import Anomaly, BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path
import numpy as np
import time
import torch
from typing import List, Tuple, Union, Optional
import tempfile
import os

# Anomalib imports - corrected
from anomalib import Engine
from anomalib.models import EfficientAd
from anomalib.data import PredictDataset

class AnomalyProcessor(Node):
    MIN_BOUNDING_BOX_AREA = 100  # Minimum area for a detected bounding box (in pixels^2)
    ANOMALY_MASK_THRESHOLD = 0.5  # Threshold for the anomaly mask (0.0 to 1.0)
    ANOMALY_SCORE_THRESHOLD = 0.5  # Threshold for considering something anomalous

    def __init__(self):
        super().__init__('anomaly_node_processor')
        self.bridge = CvBridge()
        
        # Declare and get the parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.declare_parameter("debug_image_topic", "/anomaly_detection/debug_image")
        self.declare_parameter("anomaly_publish_topic", "/anomaly_detection/results")
        self.declare_parameter("inference_image_width", 256)  # Common anomalib size
        self.declare_parameter("inference_image_height", 256)  # Common anomalib size
        self.declare_parameter("use_compressed_image", False)
        self.declare_parameter("camera_sensor_id", "camera_right")
        
        # Get parameters
        self.model_path = self.get_parameter("model_path").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.anomaly_publish_topic = self.get_parameter("anomaly_publish_topic").value
        self.inference_image_width = self.get_parameter("inference_image_width").value
        self.inference_image_height = self.get_parameter("inference_image_height").value
        self.use_compressed_image = self.get_parameter("use_compressed_image").value
        self.camera_sensor_id = self.get_parameter("camera_sensor_id").value

        # Initialize model and engine
        self.model = None
        self.engine = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load model
        self._load_model()
        
        # Create temporary directory for inference
        self.temp_dir = Path(tempfile.mkdtemp(prefix="anomalib_"))
        self.get_logger().info(f"Created temp directory: {self.temp_dir}")
        
        # Publishers and Subscribers
        self.anomaly_publisher = self.create_publisher(
            Anomaly, self.anomaly_publish_topic, 10
        )
        
        if self.use_compressed_image:
            self.subscription_camera = self.create_subscription(
                CompressedImage, self.camera_topic, self.camera_callback, 10
            )
            self.get_logger().info(f'Subscribing to {self.camera_topic} (CompressedImage)')
        else:
            self.subscription_camera = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 10
            )
            self.get_logger().info(f'Subscribing to {self.camera_topic} (Image)')

        self.subscription_lidar = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, 10
        )
        
        self.publisher_debug_image = self.create_publisher(
            Image, self.debug_image_topic, 10
        )
        
        self.get_logger().info("Anomaly detection node initialized successfully")

    def _load_model(self):
        """Load the Anomalib model"""
        try:
            if not self.model_path or not Path(self.model_path).exists():
                self.get_logger().error(f"Model path does not exist: {self.model_path}")
                return
            
            # Initialize model and engine
            self.model = EfficientAd()
            self.engine = Engine()
            
            # Load model weights
            checkpoint = torch.load(self.model_path, map_location=self.device)
            
            # Handle different checkpoint formats
            if 'state_dict' in checkpoint:
                state_dict = checkpoint['state_dict']
            elif 'model' in checkpoint:
                state_dict = checkpoint['model']
            else:
                state_dict = checkpoint
                
            # Remove 'model.' prefix if present
            new_state_dict = {}
            for key, value in state_dict.items():
                if key.startswith('model.'):
                    new_key = key[6:]  # Remove 'model.' prefix
                else:
                    new_key = key
                new_state_dict[new_key] = value
            
            self.model.load_state_dict(new_state_dict, strict=False)
            self.model.eval()
            
            self.get_logger().info(f"Anomaly model loaded successfully from {self.model_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load anomaly model: {e}")
            self.model = None
            self.engine = None

    def lidar_callback(self, msg):
        """Process LIDAR data for anomalies"""
        # TODO: Implement LIDAR anomaly detection
        # - Check point density for rain/snow interference
        # - Validate LIDAR configuration
        # - Check for blocked sensors
        pass

    def _msg_to_cv_image(self, msg: Union[Image, CompressedImage]) -> Optional[np.ndarray]:
        """Convert ROS Image/CompressedImage to OpenCV numpy array"""
        try:
            if isinstance(msg, Image):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    self.get_logger().error("Failed to decode compressed image.")
                    return None
            else:
                self.get_logger().error(f"Unsupported image message type: {type(msg)}")
                return None
            return cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error converting image message: {e}")
            return None

    def _create_bbox_msg(self, x: int, y: int, w: int, h: int, 
                        label: str = "", score_region: float = 0.0) -> BoundingBox2D:
        """Create BoundingBox2D message"""
        bbox_msg = BoundingBox2D()
        bbox_msg.x_min = int(x)
        bbox_msg.y_min = int(y)
        bbox_msg.width = int(w)
        bbox_msg.height = int(h)
        bbox_msg.label = label
        bbox_msg.score_region = score_region
        return bbox_msg

    def classify_anomaly_type(self, img: np.ndarray, mask: np.ndarray) -> str:
        """
        Classify the likely cause of an anomaly using image content and anomaly mask.
        """
        # Convert mask to binary if needed
        if mask.dtype != np.uint8:
            mask_binary = (mask > self.ANOMALY_MASK_THRESHOLD).astype(np.uint8) * 255
        else:
            mask_binary = mask
            
        # Check if there are any anomalous pixels
        if np.sum(mask_binary) == 0:
            return "normal"

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 1. Check for blur using Laplacian variance
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        if lap_var < 30:
            return "sensor_blur"
        
        # 2. Check for environmental conditions
        mask_fraction = np.sum(mask_binary > 0) / (mask_binary.shape[0] * mask_binary.shape[1])
        contrast = gray.std()
        
        if mask_fraction > 0.4 and contrast < 25:
            mean_brightness = np.mean(gray)
            if mean_brightness > 200:
                return "environmental_snow"
            elif mean_brightness < 120:
                return "environmental_fog"
            else:
                return "environmental_rain_or_overcast"

        # 3. Check for cracks or lens artifacts
        edges_in_mask = cv2.Canny(gray, 50, 150)
        edges_in_mask[mask_binary == 0] = 0
        
        if np.sum(mask_binary) > 0:
            edge_density_in_anomaly = np.sum(edges_in_mask > 0) / np.sum(mask_binary)
            if edge_density_in_anomaly > 0.05:
                return "sensor_crack_or_lens_damage"

        # 4. Check brightness-based lighting issues
        brightness = np.mean(gray)
        if brightness < 50:
            return "environmental_too_dark_or_occluded"
        elif brightness > 230:
            return "environmental_overexposed_or_direct_sun"
        
        # 5. Object-like anomaly / Road anomaly
        contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            significant_contours = [c for c in contours if cv2.contourArea(c) > self.MIN_BOUNDING_BOX_AREA]
            if len(significant_contours) > 1:
                return "scene_multiple_unknown_objects"
            else:
                h, w = img.shape[:2]
                for contour in significant_contours:
                    x, y, cw, ch = cv2.boundingRect(contour)
                    if y > h / 2 and cw / w > 0.3:
                        return "road_obstacle_or_condition"
                return "scene_unknown_object"
        
        return "unknown_anomaly_type"

    def camera_callback(self, msg: Union[Image, CompressedImage]):
        """Callback for camera image messages"""
        try:
            # Convert ROS message to OpenCV image
            cv_image = self._msg_to_cv_image(msg)
            if cv_image is None:
                return

            # Perform basic image validation
            if not self._validate_image(cv_image, msg.header.frame_id):
                return

            # Perform anomaly inference
            if self.model is None:
                self.get_logger().warn("Model not loaded, skipping inference")
                return
                
            score, mask, boxes = self.anomaly_inference(cv_image)

            if score is None:
                self.get_logger().warn('Anomaly inference failed')
                return

            self.get_logger().info(f'Anomaly score: {score:.4f}, Detected boxes: {len(boxes)}')

            # Create and publish anomaly message
            self._publish_anomaly_results(msg, score, mask, boxes, cv_image)

        except Exception as e:
            self.get_logger().error(f"Error in camera_callback: {e}")

    def _validate_image(self, image: np.ndarray, frame_id: str) -> bool:
        """Validate image format and resolution"""
        try:
            # Check image dimensions
            if len(image.shape) != 3 or image.shape[2] != 3:
                self.get_logger().error(f"Invalid image format in {frame_id}: {image.shape}")
                return False
            
            # Check image size (basic validation)
            h, w = image.shape[:2]
            if h < 100 or w < 100:
                self.get_logger().error(f"Image too small in {frame_id}: {w}x{h}")
                return False
            
            # Check if image is not corrupted (all zeros or all same value)
            if np.all(image == 0) or np.all(image == image.flat[0]):
                self.get_logger().error(f"Corrupted image detected in {frame_id}")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating image: {e}")
            return False

    def anomaly_inference(self, img: np.ndarray) -> Tuple[Optional[float], np.ndarray, List[Tuple[int, int, int, int]]]:
        """Perform anomaly inference using the loaded Anomalib model"""
        overall_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        
        # Create unique temporary file
        temp_img_path = self.temp_dir / f"input_{time.time_ns()}.jpg"
        
        try:
            # Save image temporarily
            cv2.imwrite(str(temp_img_path), img)
            
            # Create prediction dataset
            dataset = PredictDataset(
                path=temp_img_path,
                image_size=(self.inference_image_height, self.inference_image_width),
            )
            
            # Perform prediction
            predictions = self.engine.predict(
                model=self.model,
                dataset=dataset,
            )
            
            if not predictions or len(predictions) == 0:
                return None, overall_mask, []
            
            # Process first prediction
            prediction = predictions[0]
            pred_score = float(prediction.pred_score)
            
            # Get anomaly mask
            if hasattr(prediction, 'anomaly_map'):
                mask = prediction.anomaly_map.cpu().numpy()
            elif hasattr(prediction, 'pred_mask'):
                mask = prediction.pred_mask.cpu().numpy()
            else:
                self.get_logger().warn("No anomaly mask found in prediction")
                return pred_score, overall_mask, []
            
            # Process mask
            if mask.ndim == 3:
                mask = mask.squeeze()
            
            # Threshold mask
            mask_bin = (mask > self.ANOMALY_MASK_THRESHOLD).astype(np.uint8) * 255
            
            # Resize mask to original image size
            if mask_bin.shape[:2] != img.shape[:2]:
                mask_bin = cv2.resize(mask_bin, (img.shape[1], img.shape[0]), 
                                    interpolation=cv2.INTER_NEAREST)
            
            overall_mask = mask_bin
            
            # Extract bounding boxes
            bounding_boxes = self._extract_bounding_boxes(mask_bin)
            
            return pred_score, overall_mask, bounding_boxes
            
        except Exception as e:
            self.get_logger().error(f"Error during anomaly inference: {e}")
            return None, overall_mask, []
        finally:
            # Clean up temporary file
            if temp_img_path.exists():
                temp_img_path.unlink()

    def _extract_bounding_boxes(self, mask_bin: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Extract bounding boxes from binary mask"""
        bounding_boxes = []
        
        contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > self.MIN_BOUNDING_BOX_AREA:
                bounding_boxes.append((x, y, w, h))
        
        return bounding_boxes

    def _publish_anomaly_results(self, msg: Union[Image, CompressedImage], 
                                score: float, mask: np.ndarray, 
                                boxes: List[Tuple[int, int, int, int]], 
                                cv_image: np.ndarray):
        """Publish anomaly detection results"""
        try:
            # Create anomaly message
            anomaly_msg = Anomaly()
            anomaly_msg.header = msg.header
            anomaly_msg.sensor_id = self.camera_sensor_id
            anomaly_msg.overall_score = float(score)
            
            # Classify anomaly type
            anomaly_type = self.classify_anomaly_type(cv_image, mask)
            anomaly_msg.anomaly_type = anomaly_type
            
            # Set description
            anomaly_msg.description = f"Anomaly: {anomaly_type} detected with score {score:.4f}"
            
            # Add bounding boxes
            for (x, y, w, h) in boxes:
                bbox_msg = self._create_bbox_msg(x, y, w, h, 
                                               label="anomalous_region", 
                                               score_region=score)
                anomaly_msg.boxes.append(bbox_msg)
            
            # Publish anomaly message
            self.anomaly_publisher.publish(anomaly_msg)
            
            # Publish debug image
            self._publish_debug_image(msg, cv_image, boxes)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing anomaly results: {e}")

    def _publish_debug_image(self, msg: Union[Image, CompressedImage], 
                           cv_image: np.ndarray, 
                           boxes: List[Tuple[int, int, int, int]]):
        """Publish debug image with bounding boxes"""
        try:
            debug_image = cv_image.copy()
            
            # Draw bounding boxes
            for (x, y, w, h) in boxes:
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(debug_image, "Anomaly", (x, y - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Convert and publish
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header = msg.header
            self.publisher_debug_image.publish(debug_image_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

    def destroy_node(self):
        """Clean up resources"""
        try:
            # Clean up temporary directory
            if hasattr(self, 'temp_dir') and self.temp_dir.exists():
                import shutil
                shutil.rmtree(self.temp_dir)
                self.get_logger().info("Cleaned up temporary directory")
        except Exception as e:
            self.get_logger().error(f"Error cleaning up: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        anomaly_node = AnomalyProcessor()
        rclpy.spin(anomaly_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'anomaly_node' in locals():
            anomaly_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()