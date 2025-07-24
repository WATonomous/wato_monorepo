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
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from anomaly_detection_msgs.msg import Anomaly, BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path
from anomalib.data import PredictDataset
from anomalib import Engine
from anomalib.models import EfficientAd

import numpy as np # Added for classify_anomaly_type
import time # Added for unique temporary file names
import torch # Added for loading model weights
from typing import List, Tuple, Union

from yolo_utils import YOLOTensorRT

class AnomalyProcessor(Node):
    MIN_BOUNDING_BOX_AREA = 100 # Minimum area for a detected bounding box (in pixels^2)
    ANOMALY_MASK_THRESHOLD = 0.5 # Threshold for the anomaly mask (0.0 to 1.0)
    BLUR_THRESHOLD = 30.0
    MASK_FRACTION_THRESHOLD = 0.4
    CONTRAST_THRESHOLD = 25.0
    EDGE_DENSITY_THRESHOLD = 0.05
    DARK_THRESHOLD = 50.0
    LIGHT_THRESHOLD = 230.0
    COLOR_VAR_LOW = 500.0   # low color variance indicates fog
    COLOR_VAR_HIGH = 2000.0

    def __init__(self):
        super().__init__('Anomaly_Node_Processor')
        self.bridge = CvBridge()
        # Declare and get the parameters
        self.declare_parameter("model_path")
        self.declare_parameter("engine_path")  # for YOLO TensorRT engine
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.declare_parameter("debug_image_topic", "/anomaly_detection/debug_image")

        self.declare_parameter("anomaly_publish_topic", "/anomaly_detection/results") # for anomaly output topic
        self.declare_parameter("inference_image_width", 640) # model input width
        self.declare_parameter("inference_image_height", 640) # model input height
        self.declare_parameter("use_compressed_image", False) # toggle CompressedImage subscription
        self.declare_parameter("camera_sensor_id", "camera_right") # anomaly_msg.sensor_id
        
        self.model_path = self.get_parameter("model_path").value
        self.lidar_data = self.get_parameter("lidar_topic").value
        self.camera_data = self.get_parameter("camera_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.anomaly_publish_topic = self.get_parameter("anomaly_publish_topic").value
        self.inference_image_width = self.get_parameter("inference_image_width").value
        self.inference_image_height = self.get_parameter("inference_image_height").value
        self.use_compressed_image = self.get_parameter("use_compressed_image").value
        self.camera_sensor_id = self.get_parameter("camera_sensor_id").value

        self.model = EfficientAd()
        self.engine = Engine()

        # ----- Load Weights ------
        if not Path(self.model_path).exists():
            self.get_logger().error(f"Model path does not exist: {self.model_path}")
            rclpy.shutdown()
        else:
            state = torch.load(str(self.model_path))
            # Load the model state dictionary. Anomalib models typically save this way.
            self.model.load_state_dict(state['state_dict'])
            self.get_logger().info(f"Anomaly model loaded successfully from {self.model_path}")

        # Initialize YOLO detector
        self.get_logger().info(f"Loading YOLO TensorRT engine from {self.engine_path}")
        self.detector = YOLOTensorRT(
            engine_path=str(self.engine_path),
            batch_size=1,
            input_size=(self.input_h, self.input_w),
            conf_thres=0.5,
            iou_thres=0.45
        )
        # ----- Pubs and Subs ------

        self.anomaly_publisher = self.create_publisher(Anomaly, '/anomaly', 10)
        
        if self.use_compressed_image:
            self.subscription_camera = self.create_subscription(
                CompressedImage, self.camera_topic, self.camera_callback, 10)
            self.get_logger().info(f'Subscribing to {self.camera_topic} (CompressedImage)')
        else:
            self.subscription_camera = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 10)
            self.get_logger().info(f'Subscribing to {self.camera_topic} (Image)')

        self.subscription_lidar = self.create_subscription(
            PointCloud2, self.lidar_data, self.lidar_callback, 10)
        self.get_logger().info(f'Subscribing to {self.lidar_topic} (PointCloud2)')

        self.publisher_debug_image = self.create_publisher(Image, self.debug_image_topic, 10)

    def lidar_callback(self, msg):
        pass
    
    # Helper function to convert ROS Image/CompressedImage to OpenCV numpy array
    def _msg_to_rgb(self, msg: Union[Image, CompressedImage]) -> np.ndarray: #????
        try:
            if isinstance(msg, Image):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif isinstance(msg, CompressedImage):
                # Decode compressed image
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
            self.get_logger().error(f"CvBridge Error in _msg_to_rgb: {e}")
            return None

        except Exception as e:
            self.get_logger().error(f"Error converting image message: {e}")
            return None
    
    def camera_callback(self, msg: Union[Image, CompressedImage]):
        """
        Callback for camera image messages. Performs anomaly inference and publishes results.
        """
        img = self._msg_to_rgb(msg)
        if img is None:
            return

        # 1) Run YOLO detection
        detections = self.detector.infer([img])[0]
        yolo_boxes = [d['bbox'] for d in detections]
        yolo_labels = [d['class'] for d in detections]

        # 2) Run VAE anomaly inference
        #camera_callback would need to be modified to also subscribe to bounding boxes
        score, masks, boxes = self.anomaly_inference(img)#, self.model_path)

        if score is not None:
            self.get_logger().info(f'Image received. The anomaly score is {score:.4f}, Detected Boxes: {len(boxes)}')
        else:
            self.get_logger().warn('Anomaly inference returned no score or boxes.')
            return

         # 3) Fuse results: only keep anomaly boxes that overlap YOLO or annotate semantics
        fused_boxes = []
        for (x,y,w,h) in boxes:
            for (xmin, ymin, xmax, ymax) in [(int(x),int(y),int(x+w),int(y+h))]:
                fused_boxes.append((xmin, ymin, xmax-xmin, ymax-ymin))

        # 4) Prepare debug image
        debug_image = image.copy() # Work on a copy to preserve original

         # --- Draw bounding boxes on the copy of the image ---
        # Draw YOLO in green
        for (xmin,ymin,xw,yh), lbl in zip(yolo_boxes, yolo_labels):
            cv2.rectangle(debug_image, (int(xmin),int(ymin)), (int(xmin+xw),int(ymin+yh)), (0,255,0),2)
            
        # Draw anomalies in red
        for (x,y,w,h) in fused_boxes:
            cv2.rectangle(debug_image, (x,y), (x+w,y+h), (0,0,255),2)
            
        # ------------------------------- publishing ----------------------------------
        try:
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header = msg.header
            self.publisher_debug_image.publish(debug_image_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error while converting and publishing debug image: {e}")
        
        #5) Publish Anomaly message
        anomaly_msg = Anomaly()
        anomaly_msg.header = msg.header
        anomaly_msg.sensor_id = self.camera_sensor_id
        anomaly_msg.overall_score = float(score) if score is not None else 0.0   
        anomaly_msg.anomaly_type = self.classify_anomaly_type(img, masks, yolo_boxes, yolo_labels)
        #anomaly_type = Anomaly.classify_anomaly_type(openCVImage, masks, yolo_objects_from_subscription)
        anomaly_msg.description = f"Anomaly Type: {anomaly_msg.anomaly_type} detected with anomaly score {anomaly_msg.overall_score:.3f}."

        # Populate the bounding_boxes list in the Anomaly message       
        for (x, y, w, h) in fused_boxes:
            bbox_msg = self._create_bbox_msg(x, y, w, h, label=anomaly_msg.anomaly_type, score_region=an_msg.overall_score)
            anomaly_msg.boxes.append(bbox_msg)
        self.anomaly_publisher.publish(anomaly_msg)

    def anomaly_inference(self, img: np.ndarray) -> Tuple[float, np.ndarray, List[Tuple[int,int,int,int]]]:
        """
        Performs anomaly inference using the loaded Anomalib model.
        Returns the overall anomaly score, the full anomaly mask, and a list of anomaly bounding boxes.
        """

       #model = self.model
       #engine = self.engine
        # --- 1. Use Unique Temporary File Name ---
        # Save the incoming image to a temporary file for PredictDataset
        temp_dir = Path("/tmp/anomalib_inputs")
        temp_dir.mkdir(exist_ok=True) # Ensure the directory exists
        temp_img_path = temp_dir / f"anomaly_input_{time.time_ns()}.jpg" # Unique name using nanoseconds
        
        overall_mask = np.zeros(img_np.shape[:2], dtype=np.uint8)
        try:
            cv2.imwrite(str(temp_img_path), img)
        except Exception as e: # Catch specific exception for better logging
            self.get_logger().error(f"Failed to write temporary image file: {e}")
            return None, overall_mask, [] 

        predictions = None

        #temp_img_path = "/tmp/anomaly_input.jpg"

        try:
            # --- 2. Parameterize Image Size ---
            dataset = PredictDataset(
                path=Path(temp_img_path),
                image_size=(self.inference_image_width, self.inference_image_height), # matches model's input
            )

            predictions = self.engine.predict(
                model=self.model,
                dataset=dataset,
                #ckpt_path=model_path,
            )

        except Exception as e:
            self.get_logger().error(f"Error during Anomalib prediction: {e}")
            temp_img_path.unlink()
            return None, overall_mask, []

        temp_img_path.unlink()
        if not preds:
            return None, overall_mask, []

        bounding_boxes = []

        #for prediction in predictions:
        prediction = predictions[0] 
        mask = prediction.pred_mask.cpu().numpy() #pixel-level anomaly mask
        pred_label = prediction.pred_label  # 0: normal, 1: anomalous
        pred_score = prediction.pred_score
        #print(f'the prediction label is {pred_label} and pred score is {pred_score}')

        # Threshold for the mask to get binary image
        # --- ANOMALY_MASK_THRESHOLD ---
        #0.5 is a starting threshold. Way may need to change it.

        mask_bin = (mask > self.ANOMALY_MASK_THRESHOLD).astype('uint8') * 255

        # Ensure the mask is correctly resized to the original image dimensions
        if mask_bin.shape[:2] != img.shape[:2]: #or mask_bin.shape != img.shape[:2]
            mask_bin = cv2.resize(mask_bin, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            
        contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = [(x,y,w,h) for c in contours if (x:=cv2.boundingRect(c)[0])
                 or True for x,y,w,h in [cv2.boundingRect(c)] if w*h > self.MIN_BOUNDING_BOX_AREA]
        """         
        for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h > self.MIN_BOUNDING_BOX_AREA:  # Filter out small noise boxes
                    bounding_boxes.append((x, y, w, h))
        """
        return pred_score, bin_mask, bounding_boxes

    #This helper function will be used in camera_callback:
    def classify_anomaly_type(self, img: np.ndarray, mask: np.ndarray, yolo_boxes: List[Tuple[int,int,int,int]], yolo_labels: List[str] = None) -> str: ##
        """
        Classify the likely cause of an anomaly using image content and anomaly mask.
        Args:
            img (np.ndarray): The original BGR image (H, W, 3).
            mask (np.ndarray): Binary anomaly mask (H, W), values 0 or 255.
            yolo_objects (List[str], optional): Object labels from a YOLO detection pipeline.
                                                Not used in the callback directly yet, but useful for future integration.
        Returns:
            str: Detected anomaly type (e.g., "blur", "fog", "construction", etc.).
        """
        # No anomaly detected
        # Check if there are any anomalous pixels at all
        if mask is None or np.sum(mask) == 0:
            return "normal"

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # --- 1. Check for Blur using Laplacian Variance ---
        if yolo_boxes:
            for (x, y, w, h) in yolo_boxes:
                roi = gray[y:y+h, x:x+w]
                lap_var_roi = cv2.Laplacian(roi, cv2.CV_64F).var()
                if lap_var_roi < self.BLUR_THRESHOLD:
                    return "sensor_blur"
        # global check
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        if lap_var < self.BLUR_THRESHOLD:
            return "sensor_blur"
        
        # --- 2. Check for environmental conditions (Fog/Snow/Rain/Overcast) ---
        mask_fraction = np.sum(mask > 0) / (mask.shape[0] * mask.shape[1])
        # Consider a regional standard deviation if anomaly is localized, otherwise overall
        contrast = gray.std() 

        # If a large portion of the image is anomalous and low contrast
        if mask_fraction > self.MASK_FRACTION_THRESHOLD and contrast < self.CONTRAST_THRESHOLD: # Thresholds can be tuned
        # Compute per-channel variances
            b, g, r = cv2.split(img)
            var_b = np.var(b)
            var_g = np.var(g)
            var_r = np.var(r)
            avg_var = (var_b + var_g + var_r) / 3.0

            # Fog: low overall variance
            if avg_var < self.COLOR_VAR_LOW:
                return "environmental_fog"
            # Rain: high variance due to bright raindrops
            elif avg_var > self.COLOR_VAR_HIGH:
                return "environmental_rain"
            # Overcast: mid-range variance
            else:
                return "environmental_overcast"

            """
            mean_brightness = gray.mean()
            if mean_brightness > 200: # Very bright, low contrast, widespread anomaly
                return "environmental_snow"
            elif mean_brightness < 120: # Dark, low contrast, widespread anomaly
                return "environmental_fog"
            else: # Medium brightness, low contrast, widespread anomaly
                return "environmental_rain_or_overcast"
            """
        # --- 3. Check for cracks or lens artifacts using edge density within anomaly mask ---
        # This is more robust if applied only to the anomalous region
        edges_in_mask = cv2.Canny(gray, 50, 150) # Canny edges on full image
        edges_in_mask[mask == 0] = 0 # Mask out non-anomalous areas
        
        # If there are many fine, linear edges within the anomalous region, might indicate crack
        if np.sum(mask) > 0: # Avoid division by zero
            edge_density_in_anomaly = np.sum(edges_in_mask > 0) / np.sum(mask)
            if edge_density_in_anomaly > self.EDGE_DENSITY_THRESHOLD: # Threshold can be tuned
                return "sensor_crack_or_lens_damage"

        # --- 4. Check brightness-based lighting issue (whole image) ---
        brightness = np.mean(gray)
        if brightness < self.DARK_THRESHOLD: # Dark overall
            return "environmental_too_dark_or_occluded"
        elif brightness > self.LIGHT_THRESHOLD: # Very bright overall
            return "environmental_overexposed_or_direct_sun"

        # 5. Object-level anomaly: use YOLO labels for context
        if yolo_labels:
            # Return label of first detected object marked anomalous
            return f"{yolo_labels[0]}_anomaly"

        if yolo_boxes:
            return "object_anomaly"

        return "unknown_anomaly" # Fallback if mask is not empty but no specific type found

    # Helper function to create BoundingBox2D message
    def _create_bbox_msg(self, x: int, y: int, w: int, h: int, label: str = "", score_region: float = 0.0) -> BoundingBox2D:##
        bbox_msg = BoundingBox2D()
        bbox_msg.x_min = int(x)
        bbox_msg.y_min = int(y)
        bbox_msg.width = int(w)
        bbox_msg.height = int(h)
        bbox_msg.label = label
        bbox_msg.score_region = score_region
        return bbox_msg

    def destroy_node(self):
        # Clean up YOLO resources
        self.detector.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    python_anomaly = AnomalyProcessor()
    rclpy.spin(python_anomaly)
    python_anomaly.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
