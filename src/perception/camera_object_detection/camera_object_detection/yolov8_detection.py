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
import os
from message_filters import Subscriber, ApproximateTimeSynchronizer
from camera_object_detection_msgs.msg import BatchDetection, EveBatchDetection
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    ObjectHypothesisWithPose,
    Detection2D,
    Detection2DArray,
)
import concurrent.futures
from pathlib import Path
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.utils.plotting import Annotator

# Ultralytics API changed across versions; provide a fallback NMS
try:
    from ultralytics.utils.ops import non_max_suppression as _yolo_nms  # type: ignore
except ImportError:
    _yolo_nms = None  # will use local fallback

import math


import cv2
import numpy as np

import torch
import tensorrt as trt
from typing import Tuple
from dataclasses import dataclass
from numpy import ndarray

# import pycuda.driver as cuda
from cuda import cudart


@dataclass
class Tensor:
    name: str
    dtype: np.dtype
    shape: Tuple
    cpu: ndarray
    gpu: int


class Model:
    def __init__(self, model_path, device):
        self.model_path = model_path
        self.model = AutoBackend(self.model_path, device=device, dnn=False, fp16=False)
        self.names = (
            self.model.module.names
            if hasattr(self.model, "module")
            else self.model.names
        )
        self.stride = int(self.model.stride)


class CameraDetectionNode(Node):
    def __init__(self):
        super().__init__("camera_object_detection_node")
        self.get_logger().info("Creating batched camera detection node...")

        # Eve config params
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("left_camera_topic", "/camera/left/image_color")
        self.declare_parameter("center_camera_topic", "/camera/center/image_color")
        self.declare_parameter("image_width", 1600)
        self.declare_parameter("image_height", 900)

        # Nuscenes
        self.declare_parameter("nuscenes", False)
        self.declare_parameter(
            "front_center_camera_topic", "/CAM_FRONT/image_rect_compressed"
        )
        self.declare_parameter(
            "front_right_camera_topic", "/CAM_FRONT_RIGHT/image_rect_compressed"
        )
        self.declare_parameter(
            "front_left_camera_topic", "/CAM_FRONT_LEFT/image_rect_compressed"
        )

        self.declare_parameter(
            "back_center_camera_topic", "/CAM_BACK/image_rect_compressed"
        )
        self.declare_parameter(
            "back_right_camera_topic", "/CAM_BACK_RIGHT/image_rect_compressed"
        )
        self.declare_parameter(
            "back_left_camera_topic", "/CAM_BACK_LEFT/image_rect_compressed"
        )

        # tensorRT. extensions
        self.declare_parameter("onnx_model_path", "/perception_models/tensorRT.onnx")
        self.declare_parameter(
            "tensorRT_model_path", "/perception_models/tensorRT.engine"
        )
        self.declare_parameter(
            "eve_tensorRT_model_path", "/perception_models/eve.engine"
        )
        self.declare_parameter("publish_vis_topic", False)
        self.declare_parameter("batch_publish_detection_topic", "/batch_detections")
        self.declare_parameter("model_path", "/perception_models/yolov8m.pt")
        self.declare_parameter("compressed", False)
        self.declare_parameter("crop_mode", "LetterBox")
        self.declare_parameter("save_detections", False)

        # Declare batch topic
        self.declare_parameter("batch_inference_topic", "/batched_camera_message")
        self.declare_parameter(
            "eve_batch_inference_topic", "/eve_batched_camera_message"
        )

        # Batch inference topic
        self.batch_inference_topic = self.get_parameter("batch_inference_topic").value
        self.eve_batch_inference_topic = self.get_parameter(
            "eve_batch_inference_topic"
        ).value
        self.batch_publish_detection_topic = self.get_parameter(
            "batch_publish_detection_topic"
        ).value
        self.onnx_model_path = self.get_parameter("onnx_model_path").value
        self.tensorRT_model_path = self.get_parameter("tensorRT_model_path").value
        self.eve_tensorRT_model_path = self.get_parameter(
            "eve_tensorRT_model_path"
        ).value

        # Camera topics for eve
        self.camera_topic = self.get_parameter("camera_topic").value
        self.left_camera_topic = self.get_parameter("left_camera_topic").value
        self.center_camera_topic = self.get_parameter("center_camera_topic").value

        # Camera topics for nuscenes
        self.nuscenes = self.get_parameter("nuscenes").value
        self.front_center_camera_topic = self.get_parameter(
            "front_center_camera_topic"
        ).value
        self.front_right_camera_topic = self.get_parameter(
            "front_right_camera_topic"
        ).value
        self.front_left_camera_topic = self.get_parameter(
            "front_left_camera_topic"
        ).value
        self.back_center_camera_topic = self.get_parameter(
            "back_center_camera_topic"
        ).value
        self.back_right_camera_topic = self.get_parameter(
            "back_right_camera_topic"
        ).value
        self.back_left_camera_topic = self.get_parameter("back_left_camera_topic").value

        # Bool publish vis topic
        self.publish_vis_topic = bool(self.get_parameter("publish_vis_topic").value)

        # Model Path and configs
        self.model_path = self.get_parameter("model_path").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.compressed = self.get_parameter("compressed").value
        self.crop_mode = self.get_parameter("crop_mode").value
        self.save_detections = bool(self.get_parameter("save_detections").value)
        self.counter = 0  # For saving detections
        if self.save_detections:
            if not os.path.exists("detections"):
                os.makedirs("detections")

        self.line_thickness = 1
        self.half = False

        self.batch_size = getattr(self, "num_cameras", 3)
        self.input_c = 3
        self.input_h = 640
        self.input_w = 640
        self.batched_images_buffer = None

        # Start logger
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace="")

        # Initialize TensorRT model
        self.weight = (
            Path(self.tensorRT_model_path)
            if isinstance(self.tensorRT_model_path, str)
            else self.tensorRT_model_path
        )
        with trt.Runtime(self.logger) as runtime:
            self.tensorRT_model = runtime.deserialize_cuda_engine(
                self.weight.read_bytes()
            )
        self.execution_context = self.tensorRT_model.create_execution_context()
        if not self.execution_context:
            self.get_logger().error("Failed to create execution context")
            return 1

        input_name = self.tensorRT_model.get_tensor_name(
            0
        )  # usually the first I/O is your input
        self.execution_context.set_input_shape(
            input_name, (self.batch_size, self.input_c, self.input_h, self.input_w)
        )

        # Allocate GPU memory for input and output tensors
        self._collect_io_names()
        self._alloc_cuda_buffers()

        # Subscription for Nuscenes
        if self.nuscenes:
            self.front_center_camera_subscription = Subscriber(
                self, CompressedImage, self.front_center_camera_topic
            )
            self.front_right_camera_subscription = Subscriber(
                self, CompressedImage, self.front_right_camera_topic
            )
            self.front_left_camera_subscription = Subscriber(
                self, CompressedImage, self.front_left_camera_topic
            )
            self.back_center_camera_subscription = Subscriber(
                self, CompressedImage, self.back_center_camera_topic
            )
            self.back_right_camera_subscription = Subscriber(
                self, CompressedImage, self.back_right_camera_topic
            )
            self.back_left_camera_subscription = Subscriber(
                self, CompressedImage, self.back_left_camera_topic
            )
            self.ats = ApproximateTimeSynchronizer(
                [
                    self.front_left_camera_subscription,
                    self.front_center_camera_subscription,
                    self.front_right_camera_subscription,
                ],
                queue_size=10,
                slop=0.1,
            )
            self.ats.registerCallback(self.batch_inference_callback)
        else:
            # Subscription for Eve cameras
            self.right_camera_subscription = Subscriber(self, Image, self.camera_topic)
            self.left_camera_subscription = Subscriber(
                self, Image, self.left_camera_topic
            )
            self.center_camera_subscription = Subscriber(
                self, Image, self.center_camera_topic
            )
            self.eve_ats = ApproximateTimeSynchronizer(
                [
                    self.right_camera_subscription,
                    self.left_camera_subscription,
                    self.center_camera_subscription,
                ],
                queue_size=10,
                slop=0.1,
            )

            # Create this callback function tomorrow
            self.eve_ats.registerCallback(self.eve_batch_inference_callback)
            self.get_logger().info(f"TENSORT VERSION:{trt.__version__}")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        self.last_publish_time = self.get_clock().now()

        # Batch vis publishers
        self.batched_camera_message_publisher = self.create_publisher(
            BatchDetection, self.batch_inference_topic, 10
        )
        self.eve_batched_camera_message_publisher = self.create_publisher(
            EveBatchDetection, self.eve_batch_inference_topic, 10
        )

        # Nuscenes Publishers
        if self.nuscenes:
            self.nuscenes_camera_names = [
                "/CAM_FRONT_LEFT",
                "/CAM_FRONT",
                "/CAM_FRONT_RIGHT",
            ]
            self.batch_vis_publishers = [
                self.create_publisher(CompressedImage, f"{nuscenes}/viz", 10)
                for nuscenes in self.nuscenes_camera_names
            ]
            self.batch_detection_publishers = [
                self.create_publisher(Detection2DArray, f"{nuscenes}/detections", 10)
                for nuscenes in self.nuscenes_camera_names
            ]
            for nuscenes in self.nuscenes_camera_names:
                self.get_logger().info(
                    f"Successfully created node listening on camera topic: {nuscenes}..."
                )
        else:
            # Eve Publishers
            self.eve_camera_names = [
                self.left_camera_topic,
                self.center_camera_topic,
                self.camera_topic,
            ]
            self.eve_batch_vis_publishers = [
                self.create_publisher(Image, f"{eve}/eve_batch_viz", 10)
                for eve in self.eve_camera_names
            ]
            self.eve_batch_detection_publishers = [
                self.create_publisher(
                    Detection2DArray, f"{eve}/eve_batch_detections", 10
                )
                for eve in self.eve_camera_names
            ]
            for camera_names in self.eve_camera_names:
                self.get_logger().info(
                    f"Successfully created node listening on camera topic: {camera_names}..."
                )

    def _collect_io_names(self):
        # figure out which TRT tensors are inputs vs outputs
        all_names = [
            self.tensorRT_model.get_tensor_name(i)
            for i in range(self.tensorRT_model.num_io_tensors)
        ]
        modes = [self.tensorRT_model.get_tensor_mode(n) for n in all_names]

        # split into input_names / output_names
        self.input_names = [
            n for n, m in zip(all_names, modes) if m == trt.TensorIOMode.INPUT
        ]
        self.output_names = [
            n for n, m in zip(all_names, modes) if m == trt.TensorIOMode.OUTPUT
        ]

    def _alloc_cuda_buffers(self):
        # create a cudaStream once
        status, self.stream = cudart.cudaStreamCreate()
        assert status.value == 0, "cudaStreamCreate failed"

        # host+device for inputs
        self.input_info = []
        for name in self.input_names:
            shape = tuple(self.execution_context.get_tensor_shape(name))
            dtype = trt.nptype(self.tensorRT_model.get_tensor_dtype(name))
            host_mem = np.empty(shape, dtype=dtype)

            status, dev_mem = cudart.cudaMallocAsync(host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for input {name}"

            self.input_info.append(Tensor(name, dtype, shape, host_mem, dev_mem))

        # host+device for outputs
        self.output_info = []
        for name in self.output_names:
            shape = tuple(self.execution_context.get_tensor_shape(name))
            dtype = trt.nptype(self.tensorRT_model.get_tensor_dtype(name))
            host_mem = np.empty(shape, dtype=dtype)

            status, dev_mem = cudart.cudaMallocAsync(host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for output {name}"

            self.output_info.append(Tensor(name, dtype, shape, host_mem, dev_mem))

    # Note: TensorRT engine building has been moved to
    # `trt_engine_builder.py` to separate build-time and inference logic.

    def tensorRT_inferencing(self, batch_array: np.ndarray):
        """
        batch_array: (B,C,H,W) float32
        """

        # 1) Copy batch into the single input host buffer
        inp = self.input_info[0]
        assert batch_array.nbytes == inp.cpu.nbytes, "Batch size mismatch"
        np.copyto(inp.cpu, batch_array)

        # 2) Host → Device
        ret = cudart.cudaMemcpyAsync(
            inp.gpu,
            inp.cpu.ctypes.data,
            inp.cpu.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            self.stream,
        )
        # unpack tuple if needed
        status = ret[0] if isinstance(ret, tuple) else ret
        # some bindings wrap it in an object with .value
        code = status.value if hasattr(status, "value") else status
        assert code == 0, f"H2D memcpy failed (code={code})"

        # 3) bind and run
        self.execution_context.set_tensor_address(inp.name, inp.gpu)
        for out in self.output_info:
            self.execution_context.set_tensor_address(out.name, out.gpu)

        success = self.execution_context.execute_async_v3(self.stream)
        assert success, "Inference execution failed"

        # 4) Device → Host for each output
        for out in self.output_info:
            ret = cudart.cudaMemcpyAsync(
                out.cpu.ctypes.data,
                out.gpu,
                out.cpu.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                self.stream,
            )
            status = ret[0] if isinstance(ret, tuple) else ret
            code = status.value if hasattr(status, "value") else status
            assert code == 0, f"D2H memcpy failed (code={code})"

        # 5) synchronize once
        cudart.cudaStreamSynchronize(self.stream)

        # 6) return outputs as NumPy arrays
        return tuple(out.cpu for out in self.output_info)

    def preprocess_image(self, msg, dest_buffer):
        numpy_array = np.frombuffer(msg.data, np.uint8)
        compressedImage = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)

        resized_compressedImage = cv2.resize(
            compressedImage,
            (self.input_h, self.input_w),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb_image = cv2.cvtColor(resized_compressedImage, cv2.COLOR_BGR2RGB)

        # normalize to 0-1
        normalized_image = rgb_image.astype(np.float32) / 255.0

        # transpose directly into dest_buffer
        dest_buffer[0, :, :] = normalized_image[:, :, 0]
        dest_buffer[1, :, :] = normalized_image[:, :, 1]
        dest_buffer[2, :, :] = normalized_image[:, :, 2]

    # will be called with nuscenes rosbag
    def batch_inference_callback(self, msg1, msg2, msg3):
        """
        Returns final detections array for publishing to foxglove
        - Preprocess and batch images
        - Call tensorRT and parse through detections and send for visualization
        """
        self.last_publish_time = self.get_clock().now()
        # Taking msgs from all 6 ros2 subscribers
        image_list = [msg1, msg2, msg3]
        # First time setup
        if self.batched_images_buffer is None:
            self.batched_images_buffer = np.empty(
                (len(image_list), self.input_c, self.input_h, self.input_w),
                dtype=np.float32,
            )
        # Use concurrent futures to parallelize the preprocessing step
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = []
            for i, msg in enumerate(image_list):
                futures.append(
                    executor.submit(
                        self.preprocess_image, msg, self.batched_images_buffer[i]
                    )
                )

            concurrent.futures.wait(futures)
        detections = self.tensorRT_inferencing(self.batched_images_buffer)
        decoded = self.parse_detections(detections)
        self.publish_batch_nuscenes([msg1, msg2, msg3], decoded)
        now = self.get_clock().now()
        self.get_logger().info(
            f"Speed of inference: {1 / ((now - self.last_publish_time).nanoseconds / 1e9)} fps"
        )

    @torch.no_grad()
    def parse_detections(self, detections):
        # Convert NumPy array to PyTorch tensor
        detection_tensor = torch.tensor(detections[0], dtype=torch.float32)
        # (batch_size = 3 for three cameras)
        batch_size, _, num_anchors = detection_tensor.shape

        # Reshape: [batch, num_features, num_anchors] -> [batch, num_anchors, num_features]
        image_detections = detection_tensor.permute(0, 2, 1)

        # Extract bounding boxes & class probabilities
        bboxes = image_detections[:, :, :4]  # [batch, anchors, 4]
        # [batch, anchors, num_classes]
        class_probs = image_detections[:, :, 4:]

        # Get max confidence class per anchor
        confidence, predicted_class = class_probs.max(dim=2)

        # Compute (x_min, y_min, x_max, y_max)
        x_center, y_center, width, height = bboxes.split(1, dim=2)
        x_min = x_center - width / 2
        y_min = y_center - height / 2
        x_max = x_center + width / 2
        y_max = y_center + height / 2

        # Stack detections in the format [x_min, y_min, x_max, y_max, confidence, class]
        valid_detections = torch.cat(
            (
                x_min,
                y_min,
                x_max,
                y_max,
                confidence.unsqueeze(2),
                predicted_class.unsqueeze(2),
            ),
            dim=2,
        )

        # Apply batched Non-Maximum Suppression if supported
        nms_results = _batched_nms(
            valid_detections, conf_thres=0.5, iou_thres=0.45
        )

        # Convert detections to desired format
        decoded_results = []
        for camera_idx in range(batch_size):  # Process for each camera
            camera_detections = [
                {
                    "bbox": det[:4].tolist(),
                    "confidence": det[4].item(),
                    "class": int(det[5].item()),
                }
                for det in nms_results[camera_idx]
            ]
            decoded_results.append(camera_detections)

        return decoded_results

    def publish_batch_nuscenes(self, image_list, decoded_results):
        batch_msg = BatchDetection()
        batch_msg.header.stamp = self.get_clock().now().to_msg()
        batch_msg.header.frame_id = "batch"

        for idx, img_msg in enumerate(image_list):
            # Decode only if needed
            numpy_image = np.frombuffer(img_msg.data, np.uint8)
            image = cv2.imdecode(numpy_image, cv2.IMREAD_COLOR)
            height, width = image.shape[:2]
            if self.publish_vis_topic:
                annotator = Annotator(image, line_width=2, example="Class:0")

            detection_array = Detection2DArray()
            detection_array.header.stamp = batch_msg.header.stamp
            detection_array.header.frame_id = img_msg.header.frame_id

            batch_detections = decoded_results[idx]

            if batch_detections:
                # Convert bounding boxes in one operation
                bboxes = np.array(
                    [d["bbox"] for d in batch_detections]
                )  # Shape: (N, 4)
                bboxes[:, [0, 2]] *= width / self.input_w  # Scale x-coordinates
                bboxes[:, [1, 3]] *= height / self.input_h  # Scale y-coordinates
                bboxes = bboxes.astype(int)

                confidences = [d["confidence"] for d in batch_detections]
                class_ids = [d["class"] for d in batch_detections]

                for (x_min, y_min, x_max, y_max), confidence, predicted_class in zip(
                    bboxes, confidences, class_ids
                ):
                    label = "Class: %d, Conf: %.2f" % (predicted_class, confidence)

                    detection = Detection2D()
                    detection.bbox.center.position.x = (x_min + x_max) / 2
                    detection.bbox.center.position.y = (y_min + y_max) / 2
                    detection.bbox.size_x = float(x_max - x_min)
                    detection.bbox.size_y = float(y_max - y_min)

                    detected_object = ObjectHypothesisWithPose()
                    detected_object.hypothesis.class_id = str(predicted_class)
                    detected_object.hypothesis.score = float(confidence)
                    detection.results.append(detected_object)

                    detection_array.detections.append(detection)

                    if self.publish_vis_topic:
                        annotator.box_label(
                            (x_min, y_min, x_max, y_max), label, color=(0, 100, 0)
                        )

            batch_msg.detections.append(detection_array)

            # Publish individual detection message
            self.batch_detection_publishers[idx].publish(detection_array)

            # Publish visualization if enabled
            if self.publish_vis_topic:
                annotated_image = annotator.result()
                vis_compressed_image = CompressedImage()
                vis_compressed_image.header.stamp = self.get_clock().now().to_msg()
                vis_compressed_image.header.frame_id = img_msg.header.frame_id
                vis_compressed_image.format = "jpeg"
                vis_compressed_image.data = cv2.imencode(
                    ".jpg", annotated_image, [cv2.IMWRITE_JPEG_QUALITY, 70]
                )[1].tobytes()
                self.batch_vis_publishers[idx].publish(vis_compressed_image)

        # Publish batch detection message
        self.batched_camera_message_publisher.publish(batch_msg)

    def eve_preprocess_image(self, msg, dest_buffer):
        numpy_array = np.frombuffer(msg.data, np.uint8)
        image = numpy_array.reshape((self.image_width, self.image_height, self.input_c))

        if msg.encoding == "bgr8":
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif msg.encoding == "rgb8":
            rgb_image = image
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        # Resize the image to the desired input size
        resized_rgb_image = cv2.resize(
            rgb_image, (self.input_h, self.input_w), interpolation=cv2.INTER_LINEAR
        )

        # normalize to 0-1
        normalized_image = resized_rgb_image.astype(np.float32) / 255.0

        # transpose directly into dest_buffer
        dest_buffer[0, :, :] = normalized_image[:, :, 0]
        dest_buffer[1, :, :] = normalized_image[:, :, 1]
        dest_buffer[2, :, :] = normalized_image[:, :, 2]

    def eve_batch_inference_callback(self, msg1, msg2, msg3):
        image_list = [msg1, msg2, msg3]
        # First time setup
        if self.batched_images_buffer is None:
            self.batched_images_buffer = np.empty(
                (len(image_list), self.input_c, self.input_h, self.input_w),
                dtype=np.float32,
            )
        # Use concurrent futures to parallelize the preprocessing step
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = []
            for i, msg in enumerate(image_list):
                futures.append(
                    executor.submit(
                        self.preprocess_image, msg, self.batched_images_buffer[i]
                    )
                )

            concurrent.futures.wait(futures)
        detections = self.tensorRT_inferencing(self.batched_images_buffer)
        decoded_results = self.parse_detections(detections)
        self.publish_batch_eve([msg1, msg2, msg3], decoded_results)

    def publish_batch_eve(self, image_list, decoded_results):
        batch_msg = BatchDetection()
        batch_msg.header.stamp = self.get_clock().now().to_msg()
        batch_msg.header.frame_id = "batch"

        for idx, img_msg in enumerate(image_list):
            # Decode only if needed
            numpy_image = np.frombuffer(img_msg.data, np.uint8)
            image = cv2.imdecode(numpy_image, cv2.IMREAD_COLOR)
            height, width = image.shape[:2]
            if self.publish_vis_topic:
                annotator = Annotator(image, line_width=2, example="Class:0")

            detection_array = Detection2DArray()
            detection_array.header.stamp = batch_msg.header.stamp
            detection_array.header.frame_id = img_msg.header.frame_id

            batch_detections = decoded_results[idx]

            if batch_detections:
                # Convert bounding boxes in one operation
                bboxes = np.array(
                    [d["bbox"] for d in batch_detections]
                )  # Shape: (N, 4)
                bboxes[:, [0, 2]] *= width / self.input_w  # Scale x-coordinates
                bboxes[:, [1, 3]] *= height / self.input_h  # Scale y-coordinates
                bboxes = bboxes.astype(int)

                confidences = [d["confidence"] for d in batch_detections]
                class_ids = [d["class"] for d in batch_detections]

                for (x_min, y_min, x_max, y_max), confidence, predicted_class in zip(
                    bboxes, confidences, class_ids
                ):
                    label = "Class: %d, Conf: %.2f" % (predicted_class, confidence)

                    detection = Detection2D()
                    detection.bbox.center.position.x = (x_min + x_max) / 2
                    detection.bbox.center.position.y = (y_min + y_max) / 2
                    detection.bbox.size_x = float(x_max - x_min)
                    detection.bbox.size_y = float(y_max - y_min)

                    detected_object = ObjectHypothesisWithPose()
                    detected_object.hypothesis.class_id = str(predicted_class)
                    detected_object.hypothesis.score = float(confidence)
                    detection.results.append(detected_object)

                    detection_array.detections.append(detection)

                    if self.publish_vis_topic:
                        annotator.box_label(
                            (x_min, y_min, x_max, y_max), label, color=(0, 100, 0)
                        )

            batch_msg.detections.append(detection_array)

            # Publish individual detection message
            self.eve_batch_detection_publishers[idx].publish(detection_array)

            # Publish visualization if enabled
            if self.publish_vis_topic:
                annotated_image = annotator.result()
                vis_compressed_image = CompressedImage()
                vis_compressed_image.header.stamp = self.get_clock().now().to_msg()
                vis_compressed_image.header.frame_id = img_msg.header.frame_id
                vis_compressed_image.format = "jpeg"
                vis_compressed_image.data = cv2.imencode(
                    ".jpg", annotated_image, [cv2.IMWRITE_JPEG_QUALITY, 70]
                )[1].tobytes()
                self.eve_batch_vis_publishers[idx].publish(vis_compressed_image)

        # Publish batch detection message
        self.eve_batched_camera_message_publisher.publish(batch_msg)

    def destroy_node(self):
        for tensor in self.input_info + self.output_info:
            status = cudart.cudaFreeAsync(tensor.gpu, self.stream)
            assert status.value == 0
        cudart.cudaStreamDestroy(self.stream)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_object_detection_node = CameraDetectionNode()
    rclpy.spin(camera_object_detection_node)
    camera_object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# -------------------------
# NMS fallback implementation
# -------------------------
def _box_iou(box1: torch.Tensor, box2: torch.Tensor) -> torch.Tensor:
    """Compute IoU between each box in box1 and each box in box2.
    box format: [x1, y1, x2, y2]
    Returns: [len(box1), len(box2)] tensor of IoUs
    """
    # Areas
    area1 = (box1[:, 2] - box1[:, 0]).clamp(min=0) * (box1[:, 3] - box1[:, 1]).clamp(min=0)
    area2 = (box2[:, 2] - box2[:, 0]).clamp(min=0) * (box2[:, 3] - box2[:, 1]).clamp(min=0)

    # Intersections
    lt = torch.max(box1[:, None, :2], box2[:, :2])  # [N, M, 2]
    rb = torch.min(box1[:, None, 2:], box2[:, 2:])  # [N, M, 2]
    wh = (rb - lt).clamp(min=0)  # [N, M, 2]
    inter = wh[:, :, 0] * wh[:, :, 1]  # [N, M]

    union = area1[:, None] + area2 - inter
    return inter / (union + 1e-7)


def _simple_nms(dets: torch.Tensor, iou_thres: float) -> torch.Tensor:
    """Pure PyTorch NMS for a single-class set of detections.
    dets: [N, 6] -> [x1,y1,x2,y2,score,cls]
    Returns kept detections [K, 6]
    """
    if dets.numel() == 0:
        return dets
    # Sort by score descending
    order = torch.argsort(dets[:, 4], descending=True)
    dets = dets[order]
    keep = []
    while dets.size(0):
        curr = dets[0:1]
        keep.append(curr)
        if dets.size(0) == 1:
            break
        ious = _box_iou(curr[:, :4], dets[1:, :4]).squeeze(0)
        dets = dets[1:][ious <= iou_thres]
    return torch.cat(keep, dim=0)


def _batched_nms(pred: torch.Tensor, conf_thres: float = 0.25, iou_thres: float = 0.45):
    """Batched NMS wrapper with Ultralytics compatibility.
    pred: [B, N, 6] (x1,y1,x2,y2,conf,cls)
    Returns list of length B with [K_i, 6] tensors per image.
    Uses Ultralytics' NMS if available; otherwise a simple torch fallback.
    """
    if _yolo_nms is not None:
        return _yolo_nms(pred, conf_thres=conf_thres, iou_thres=iou_thres)

    results = []
    B = pred.shape[0]
    for i in range(B):
        det = pred[i]
        if det.numel() == 0:
            results.append(torch.empty((0, 6), device=pred.device))
            continue
        # Filter by confidence
        det = det[det[:, 4] >= conf_thres]
        if det.numel() == 0:
            results.append(torch.empty((0, 6), device=pred.device))
            continue

        classes = det[:, 5].to(torch.int64)
        kept = []
        for c in classes.unique():
            dc = det[classes == c]
            kept.append(_simple_nms(dc, iou_thres))
        if kept:
            results.append(torch.cat(kept, dim=0))
        else:
            results.append(torch.empty((0, 6), device=pred.device))
    return results
