import rclpy
from rclpy.node import Node
import os
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from camera_object_detection_msgs.msg import BatchDetection, EveBatchDetection
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    ObjectHypothesisWithPose,
    Detection2D,
    Detection2DArray,
)
import concurrent.futures
from std_msgs.msg import Header
from pathlib import Path
from ultralytics import YOLO
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.data.augment import LetterBox, CenterCrop
from ultralytics.utils.ops import non_max_suppression
from ultralytics.utils.plotting import Annotator, colors

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

import torch
import onnx
import tensorrt as trt
from typing import List, Optional, Tuple, Union
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


class Model():
    def __init__(self, model_path, device):
        self.model_path = model_path
        self.model = AutoBackend(
            self.model_path, device=device, dnn=False, fp16=False)
        self.names = self.model.module.names if hasattr(
            self.model, "module") else self.model.names
        self.stride = int(self.model.stride)


class CameraDetectionNode(Node):

    def __init__(self):
        torch.zeros(1).cuda()

        super().__init__("camera_object_detection_node")
        self.get_logger().info("Creating batched camera detection node...")

        # Eve config params
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("left_camera_topic", "/camera/left/image_color")
        self.declare_parameter("center_camera_topic",
                               "/camera/center/image_color")

        # Nuscenes
        self.declare_parameter("nuscenes", False)
        self.declare_parameter("front_center_camera_topic",
                               "/CAM_FRONT/image_rect_compressed")
        self.declare_parameter("front_right_camera_topic",
                               "/CAM_FRONT_RIGHT/image_rect_compressed")
        self.declare_parameter("front_left_camera_topic",
                               "/CAM_FRONT_LEFT/image_rect_compressed")

        self.declare_parameter("back_center_camera_topic",
                               "/CAM_BACK/image_rect_compressed")
        self.declare_parameter("back_right_camera_topic",
                               "/CAM_BACK_RIGHT/image_rect_compressed")
        self.declare_parameter("back_left_camera_topic",
                               "/CAM_BACK_LEFT/image_rect_compressed")

        # tensorRT. extensions
        self.declare_parameter(
            "onnx_model_path", "/perception_models/tensorRT.onnx")
        self.declare_parameter("tensorRT_model_path",
                               "/perception_models/tensorRT.engine")
        self.declare_parameter("eve_tensorRT_model_path",
                               "/perception_models/eve.engine")
        self.declare_parameter("publish_vis_topic", "/annotated_img")
        self.declare_parameter("batch_publish_vis_topic",
                               "/batch_annotated_img")
        self.declare_parameter(
            "batch_publish_detection_topic", "/batch_detections")
        self.declare_parameter("publish_detection_topic", "/detections")
        self.declare_parameter("model_path", "/perception_models/yolov8m.pt")
        self.declare_parameter("image_size", 1024)
        self.declare_parameter("compressed", False)
        self.declare_parameter("crop_mode", "LetterBox")
        self.declare_parameter("save_detections", False)

        # Declare batch topic
        self.declare_parameter("batch_inference_topic",
                               "/batched_camera_message")
        self.declare_parameter("eve_batch_inference_topic",
                               "/eve_batched_camera_message")

        # Batch inference topic
        self.batch_inference_topic = self.get_parameter(
            "batch_inference_topic").value
        self.eve_batch_inference_topic = self.get_parameter(
            "eve_batch_inference_topic").value
        self.batch_publish_detection_topic = self.get_parameter(
            "batch_publish_detection_topic").value
        self.batch_publish_vis_topic = self.get_parameter(
            "batch_publish_vis_topic").value
        self.onnx_model_path = self.get_parameter("onnx_model_path").value
        self.tensorRT_model_path = self.get_parameter(
            "tensorRT_model_path").value
        self.eve_tensorRT_model_path = self.get_parameter(
            "eve_tensorRT_model_path").value

        # Camera topics for eve
        self.camera_topic = self.get_parameter("camera_topic").value
        self.left_camera_topic = self.get_parameter("left_camera_topic").value
        self.center_camera_topic = self.get_parameter(
            "center_camera_topic").value

        # Camera topics for nuscenes
        self.nuscenes = self.get_parameter("nuscenes").value
        self.front_center_camera_topic = self.get_parameter(
            "front_center_camera_topic").value
        self.front_right_camera_topic = self.get_parameter(
            "front_right_camera_topic").value
        self.front_left_camera_topic = self.get_parameter(
            "front_left_camera_topic").value
        self.back_center_camera_topic = self.get_parameter(
            "back_center_camera_topic").value
        self.back_right_camera_topic = self.get_parameter(
            "back_right_camera_topic").value
        self.back_left_camera_topic = self.get_parameter(
            "back_left_camera_topic").value

        # Publish topics
        self.publish_vis_topic = self.get_parameter("publish_vis_topic").value
        self.publish_detection_topic = self.get_parameter(
            "publish_detection_topic").value

        # Model Path and configs
        self.model_path = self.get_parameter("model_path").value
        self.image_size = self.get_parameter("image_size").value
        self.compressed = self.get_parameter("compressed").value
        self.crop_mode = self.get_parameter("crop_mode").value
        self.save_detections = bool(
            self.get_parameter("save_detections").value)
        self.counter = 0  # For saving detections
        if self.save_detections:
            if not os.path.exists("detections"):
                os.makedirs("detections")

        self.line_thickness = 1
        self.half = False

        # Subscription for Nuscenes
        if (self.nuscenes):
            self.front_center_camera_subscription = Subscriber(
                self, CompressedImage, self.front_center_camera_topic)
            self.front_right_camera_subscription = Subscriber(
                self, CompressedImage, self.front_right_camera_topic)
            self.front_left_camera_subscription = Subscriber(
                self, CompressedImage, self.front_left_camera_topic)
            self.back_center_camera_subscription = Subscriber(
                self, CompressedImage, self.back_center_camera_topic)
            self.back_right_camera_subscription = Subscriber(
                self, CompressedImage, self.back_right_camera_topic)
            self.back_left_camera_subscription = Subscriber(
                self, CompressedImage, self.back_left_camera_topic)
            self.ats = ApproximateTimeSynchronizer(
                [self.front_left_camera_subscription, self.front_center_camera_subscription, self.front_right_camera_subscription], queue_size=10, slop=0.1)
            self.ats.registerCallback(self.batch_inference_callback)
        else:
            # Subscription for Eve cameras
            self.right_camera_subscription = Subscriber(
                self, Image, self.camera_topic)
            self.left_camera_subscription = Subscriber(
                self, Image, self.left_camera_topic)
            self.center_camera_subscription = Subscriber(
                self, Image, self.center_camera_topic)
            self.eve_ats = ApproximateTimeSynchronizer(
                [self.right_camera_subscription, self.left_camera_subscription, self.center_camera_subscription], queue_size=10, slop=0.1)

            # Create this callback function tomorrow
            self.eve_ats.registerCallback(self.eve_batch_inference_callback)
            self.get_logger().info(f"TENSORT VERSION:{trt.__version__}")

        self.orig_image_width = None
        self.orig_image_height = None

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        # CV bridge
        self.cv_bridge = CvBridge()

        status, self.stream = cudart.cudaStreamCreate()
        assert status.value == 0, "IS NOT ZERO"

        # self.build_engine()
        self.last_publish_time = self.get_clock().now()

        # Batch vis publishers
        self.batched_camera_message_publisher = self.create_publisher(
            BatchDetection, self.batch_inference_topic, 10)
        self.eve_batched_camera_message_publisher = self.create_publisher(
            EveBatchDetection, self.eve_batch_inference_topic, 10)
        self.num_cameras = 3  # Adjust this based on the number of cameras

       # Nuscenes Publishers
        if (self.nuscenes):
            self.nuscenes_camera_names = [
                "/CAM_FRONT_LEFT", "/CAM_FRONT", "/CAM_FRONT_RIGHT"]
            self.batch_vis_publishers = [
                self.create_publisher(CompressedImage, f"{nuscenes}/viz", 10)
                for nuscenes in self.nuscenes_camera_names
            ]
            self.batch_detection_publishers = [
                self.create_publisher(
                    Detection2DArray, f"{nuscenes}/detections", 10)
                for nuscenes in self.nuscenes_camera_names
            ]
            for nuscenes in self.nuscenes_camera_names:
                self.get_logger().info(
                    f"Successfully created node listening on camera topic: {nuscenes}...")
        else:
            # Eve Publishers
            self.eve_camera_names = [self.left_camera_topic,
                                     self.center_camera_topic, self.camera_topic]
            self.eve_batch_vis_publishers = [
                self.create_publisher(Image, f"{eve}/eve_batch_viz", 10)
                for eve in self.eve_camera_names
            ]
            self.eve_batch_detection_publishers = [
                self.create_publisher(
                    Detection2DArray, f"{eve}/eve_batch_detections", 10)
                for eve in self.eve_camera_names
            ]
            for camera_names in self.eve_camera_names:
                self.get_logger().info(
                    f"Successfully created node listening on camera topic: {camera_names}...")

    def build_engine(self):
        # Only calling this function when we dont have an engine file
        # Reading the onnx file in the perception models directory
        self.max_batch_size = 6
        self.channels = 3
        self.height = 640
        self.width = 640
        self.shape_input_model = [self.max_batch_size,
                                  self.channels, self.height, self.width]
        self.logger = trt.Logger(trt.Logger.VERBOSE)
        self.builder = trt.Builder(self.logger)
        self.config = self.builder.create_builder_config()
        self.cache = self.config.create_timing_cache(b"")
        self.config.set_timing_cache(self.cache, ignore_mismatch=False)
        self.total_memory = torch.cuda.get_device_properties(
            self.device).total_memory
        self.config.set_memory_pool_limit(
            trt.MemoryPoolType.WORKSPACE, self.total_memory)
        self.flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        self.network = self.builder.create_network(self.flag)
        self.parser = trt.OnnxParser(self.network, self.logger)
        with open(self.onnx_model_path, "rb") as f:
            if not self.parser.parse(f.read()):
                self.get_logger().info("ERROR: Cannot read ONNX FILE")
                for error in range(self.parser.num_errors):
                    self.get_logger().info(f"{self.parser.get_error(error)}")
        self.inputs = [self.network.get_input(
            i) for i in range(self.network.num_inputs)]
        self.outputs = [self.network.get_output(
            i) for i in range(self.network.num_outputs)]
        for input in self.inputs:
            self.get_logger().info(
                f"Model {input.name} shape:{input.shape} {input.dtype}")
        for output in self.outputs:
            self.get_logger().info(
                f"Model {output.name} shape: {output.shape} {output.dtype}")
        if self.max_batch_size > 1:
            self.profile = self.builder.create_optimization_profile()
            self.min_shape = [1, 3, 640, 640]
            self.opt_shape = [3, 3, 640, 640]
            self.max_shape = [6, 3, 640, 640]
            for input in self.inputs:
                self.profile.set_shape(
                    input.name, self.min_shape, self.opt_shape, self.max_shape)
            self.config.add_optimization_profile(self.profile)
        self.half = True
        self.int8 = False
        if self.half:
            self.config.set_flag(trt.BuilderFlag.FP16)
        elif self.int8:
            self.config.set_flag(trt.BuilderFlag.INT8)
        self.engine_bytes = self.builder.build_serialized_network(
            self.network, self.config)
        assert self.engine_bytes is not None, "Failed to create engine"
        self.get_logger().info("BUILT THE ENGINE ")
        with open(self.tensorRT_model_path, "wb") as f:
            f.write(self.engine_bytes)
        self.get_logger().info("FINISHED WRITING ")

    def initialize_engine(self, weight, batch_size, rgb, width, height):
        """
            Initializes engine file requirements 
            - takes in file path for tensorRT file, batch size, # of rgb channels, width & height
            - includes input names, output names, and setting dimensions for model input shape
        """
        self.weight = Path(weight) if isinstance(weight, str) else weight
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace='')
        with trt.Runtime(self.logger) as runtime:
            self.tensorRT_model = runtime.deserialize_cuda_engine(
                self.weight.read_bytes())
        self.execution_context = self.tensorRT_model.create_execution_context()
        if not self.execution_context:
            self.get_logger().error("Failed to create execution context")
            return 1

        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.input_tensor_name = self.tensorRT_model.get_tensor_name(0)

        self.execution_context.set_input_shape(
            self.input_tensor_name, (batch_size, rgb, width, height))
        self.inputShape = self.execution_context.get_tensor_shape(
            self.input_tensor_name)
        self.names = [self.tensorRT_model.get_tensor_name(
            i) for i in range(self.num_io_tensors)]
        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.bindings = [0] * self.num_io_tensors

        self.names = [self.tensorRT_model.get_tensor_name(
            i) for i in range(self.tensorRT_model.num_io_tensors)]

        # Collect input and output tensor names
        self.input_names = [name for name in self.names if self.tensorRT_model.get_tensor_mode(
            name) == trt.TensorIOMode.INPUT]
        self.output_names = [name for name in self.names if self.tensorRT_model.get_tensor_mode(
            name) == trt.TensorIOMode.OUTPUT]

        # Set number of inputs and outputs
        self.num_inputs = len(self.input_names)
        self.num_outputs = len(self.output_names)
        self.input_names = self.names[:self.num_inputs]
        # This line removes it
        self.output_names = [
            name for name in self.output_names if name not in self.input_names]
        return 0

    def initialize_tensors(self):
        """
            Initializes GPU from cuda to set up inferencing 
            - Assigns input names, and shape
            - Assigns output names, and shapes 
        """
        self.dynamic = True
        self.input_info = []
        self.output_info = []
        self.output_ptrs = []

        # Initializing output tensors
        for name in self.output_names:
            self.tensorRT_output_shape = self.execution_context.get_tensor_shape(
                name)
            self.outputDtype = trt.nptype(
                self.tensorRT_model.get_tensor_dtype(name))
            self.output_cpu = np.empty(
                self.tensorRT_output_shape, dtype=self.outputDtype)
            status, self.output_gpu = cudart.cudaMallocAsync(
                self.output_cpu.nbytes, self.stream)
            assert status.value == 0
            cudart.cudaMemcpyAsync(self.output_gpu, self.output_cpu.ctypes.data, self.output_cpu.nbytes,
                                   cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
            self.output_ptrs.append(self.output_gpu)
            self.output_info.append(Tensor(
                name, self.outputDtype, self.tensorRT_output_shape, self.output_cpu, self.output_gpu))

        # Initializes input tensors
        for i, name in enumerate(self.input_names):
            if self.tensorRT_model.get_tensor_name(i) == name:
                self.tensorRT_input_shape = tuple(self.inputShape)
                self.dtype = trt.nptype(
                    self.tensorRT_model.get_tensor_dtype(name))
                self.input_cpu = np.empty(
                    self.tensorRT_input_shape, self.dtype)
                status, self.input_gpu = cudart.cudaMallocAsync(
                    self.input_cpu.nbytes, self.stream)
                assert status.value == 0, "DOES NOT MATCH"
                cudart.cudaMemcpyAsync(
                    self.input_gpu, self.input_cpu.ctypes.data, self.input_cpu.nbytes,
                    cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
            self.input_info.append(Tensor(
                name, self.dtype, self.tensorRT_input_shape, self.input_cpu, self.input_gpu))
        return self.input_info, self.output_info

    def tensorRT_inferencing(self, batch_array):
        """
            Inferences through preprocessed batch images and gives data about detections
            - Returns a contigious array of shape (3,84,8400)
        """
        assert batch_array.ndim == 4
        batch_size = batch_array.shape[0]
        assert batch_size == self.input_info[0].shape[0]

        # Intializing memory, and names
        self.contiguous_inputs = [np.ascontiguousarray(batch_array)]
        for i in range(self.num_inputs):
            name = self.input_info[i].name
            cudart.cudaMemcpyAsync(
                self.input_info[i].gpu,
                self.contiguous_inputs[i].ctypes.data,
                self.contiguous_inputs[i].nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                self.stream
            )
            self.execution_context.set_tensor_address(
                name, self.input_info[i].gpu)

        self.output_gpu_ptrs = []
        self.outputs_ndarray = []
        for i in range(self.num_outputs):
            output_shape = self.execution_context.get_tensor_shape(
                self.output_info[i].name)
            # Reallocate output buffer if shape changed
            if self.output_info[i].cpu.shape != tuple(output_shape):
                self.output_info[i].cpu = np.empty(
                    output_shape, dtype=self.output_info[i].cpu.dtype)

            self.outputs_ndarray.append(self.output_info[i].cpu)
            self.output_gpu_ptrs.append(self.output_info[i].gpu)
            self.execution_context.set_tensor_address(
                self.output_info[i].name,
                self.output_info[i].gpu
            )

        # Execute inference
        status = self.execution_context.execute_async_v3(self.stream)
        assert status, "Inference execution failed"

        # Synchronize and copy results
        cudart.cudaStreamSynchronize(self.stream)
        for i, gpu_ptr in enumerate(self.output_gpu_ptrs):
            cudart.cudaMemcpyAsync(
                self.outputs_ndarray[i].ctypes.data,
                gpu_ptr,
                self.outputs_ndarray[i].nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                self.stream
            )
            cudart.cudaStreamSynchronize(self.stream)

        return tuple(self.outputs_ndarray) if len(self.outputs_ndarray) > 1 else self.outputs_ndarray[0]

    def preprocess_image(self, msg):
        numpy_array = np.frombuffer(msg.data, np.uint8)
        compressedImage = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
        original_height, original_width = compressedImage.shape[:2]
        resized_compressedImage = cv2.resize(
            compressedImage, (640, 640), interpolation=cv2.INTER_LINEAR)
        rgb_image = cv2.cvtColor(resized_compressedImage, cv2.COLOR_BGR2RGB)
        normalized_image = rgb_image / 255.0
        chw_image = np.transpose(normalized_image, (2, 0, 1))
        return chw_image.astype(np.float32)

    # will be called with nuscenes rosbag
    def batch_inference_callback(self, msg1, msg2, msg3):
        """
        Returns final detections array for publishing to foxglove
        - Preprocess and batch images 
        - Call tensorRT and parse through detections and send for visualization
        """
        # Taking msgs from all 6 ros2 subscribers
        image_list = [msg1, msg2, msg3]
        batched_list = []
        # Use concurrent futures to parallelize the preprocessing step
        with concurrent.futures.ThreadPoolExecutor() as executor:
            batched_list = list(executor.map(
                lambda msg: self.preprocess_image(msg), image_list))
        # Stack the images into a batch
        batched_images = np.stack(batched_list, axis=0)
        batch = batched_images.shape[0]
        # Initialize TensorRT engine
        # init engine and return if errored
        if self.initialize_engine(self.tensorRT_model_path, batch, 3, 640, 640):
            return
        # Initialize tensors
        self.input_info, self.output_info = self.initialize_tensors()
        detections = self.tensorRT_inferencing(batched_images)
        decoded_results = self.parse_detections(detections)
        self.publish_batch_nuscenes(image_list, decoded_results)

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
        valid_detections = torch.cat((x_min, y_min, x_max, y_max, confidence.unsqueeze(
            2), predicted_class.unsqueeze(2)), dim=2)

        # Apply batched Non-Maximum Suppression if supported
        nms_results = non_max_suppression(
            valid_detections, conf_thres=0.5, iou_thres=0.45
        )

        # Convert detections to desired format
        decoded_results = []
        for camera_idx in range(batch_size):  # Process for each camera
            camera_detections = [
                {"bbox": det[:4].tolist(), "confidence": det[4].item(),
                 "class": int(det[5].item())}
                for det in nms_results[camera_idx]
            ]
            decoded_results.append(camera_detections)

        return decoded_results

    def publish_batch_nuscenes(self, image_list, decoded_results):
        batch_msg = BatchDetection()
        batch_msg.header.stamp = self.get_clock().now().to_msg()
        batch_msg.header.frame_id = "batch"

        # Check if visualization is enabled
        should_visualize = hasattr(self, "batch_vis_publishers")

        for idx, img_msg in enumerate(image_list):
            # Decode only if needed
            if should_visualize:
                numpy_image = np.frombuffer(img_msg.data, np.uint8)
                image = cv2.imdecode(numpy_image, cv2.IMREAD_COLOR)
                height, width = image.shape[:2]
                annotator = Annotator(image, line_width=2, example="Class:0")
            else:
                height, width = 640, 640  # Assume default YOLOv8 input size

            detection_array = Detection2DArray()
            detection_array.header.stamp = batch_msg.header.stamp
            detection_array.header.frame_id = f"camera_{idx}"

            batch_detections = decoded_results[idx]

            if batch_detections:
                # Convert bounding boxes in one operation
                bboxes = np.array([d["bbox"]
                                  for d in batch_detections])  # Shape: (N, 4)
                bboxes[:, [0, 2]] *= width / 640  # Scale x-coordinates
                bboxes[:, [1, 3]] *= height / 640  # Scale y-coordinates
                bboxes = bboxes.astype(int)

                confidences = [d["confidence"] for d in batch_detections]
                class_ids = [d["class"] for d in batch_detections]

                for (x_min, y_min, x_max, y_max), confidence, predicted_class in zip(bboxes, confidences, class_ids):
                    label = "Class: %d, Conf: %.2f" % (
                        predicted_class, confidence)

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

                    if should_visualize:
                        annotator.box_label(
                            (x_min, y_min, x_max, y_max), label, color=(0, 100, 0))

            batch_msg.detections.append(detection_array)

            # Publish detection message
            self.batch_detection_publishers[idx].publish(detection_array)

            # Publish visualization if enabled
            if should_visualize:
                annotated_image = annotator.result()
                vis_compressed_image = CompressedImage()
                vis_compressed_image.header.stamp = self.get_clock().now().to_msg()
                vis_compressed_image.header.frame_id = f"camera_{idx}"
                vis_compressed_image.format = "jpeg"
                vis_compressed_image.data = cv2.imencode(
                    '.jpg', annotated_image, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.batch_vis_publishers[idx].publish(vis_compressed_image)

        # Publish batch detection message
        self.batched_camera_message_publisher.publish(batch_msg)

    def eve_batch_inference_callback(self, msg1, msg2, msg3):
        image_list = [msg1, msg2, msg3]
        batched_list = []
        for msg in image_list:
            if self.compressed:
                numpy_array = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
                original_height, original_width = cv_image.shape[:2]
            else:
                cv_image = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough")
            # can also be 1024, 1024
            resized_image = cv2.resize(cv_image, (640, 640))
            rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            normalized_image = rgb_image / 255
            chw_image = np.transpose(normalized_image, (2, 0, 1))
            float_image = chw_image.astype(np.float32)
            batched_list.append(float_image)
        batched_images = np.stack(batched_list, axis=0)
        self.get_logger().info(f"batched image shape:{batched_images.shape}")
        self.initialize_engine(self.tensorRT_model_path, 3, 3, 640, 640)
        self.input_info, self.output_info = self.initialize_tensors()
        detections = self.tensorRT_inferencing(batched_images)
        decoded_results = self.parse_detections(detections)

    def publish_batch_eve(self, image_list, decoded_results):
        batch_msg = BatchDetection()
        batch_msg.header.stamp = self.get_clock().now().to_msg()
        batch_msg.header.frame_id = "batch"

        for idx, img_msg in enumerate(image_list):
            if self.compressed:
                numpy_array = np.frombuffer(img_msg.data, np.uint8)
                image = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
                original_height, original_width = image.shape[:2]
            else:
                image = self.cv_bridge.imgmsg_to_cv2(
                    img_msg, desired_encoding="passthrough")
            height, width = image.shape[:2]
            annotator = Annotator(image, line_width=2, example="Class:0")
            detection_array = Detection2DArray()
            detection_array.header.stamp = batch_msg.header.stamp
            detection_array.header.frame_id = f"camera_{idx}"
            batch_detections = decoded_results[idx]
            for anchor_idx, detection in enumerate(batch_detections):
                # Extract values from the dictionary
                bbox = detection["bbox"]
                x_min, y_min, x_max, y_max = bbox
                confidence = detection["confidence"]
                predicted_class = detection["class"]
                x_min = int(x_min * width / 640)
                x_max = int(x_max * width / 640)
                y_min = int(y_min * height / 640)
                y_max = int(y_max * height / 640)
                # self.get_logger().info(f"Camera {idx}: bbox: {x_min, y_min, x_max, y_max}, conf: {confidence}")
                label = f"Class: {predicted_class}, Conf: {confidence:.2f}"
                annotator.box_label(
                    (x_min, y_min, x_max, y_max), label, color=(0, 255, 0))
                detection = Detection2D()
                detection.bbox.center.position.x = (x_min + x_max) / 2
                detection.bbox.center.position.y = (y_min + y_max) / 2
                detection.bbox.size_x = float(x_max - x_min)
                detection.bbox.size_y = float(y_max - y_min)

                detected_object = ObjectHypothesisWithPose()
                detected_object.hypothesis.class_id = str(int(predicted_class))
                detected_object.hypothesis.score = float(confidence)
                detection.results.append(detected_object)
                detection_array.detections.append(detection)
            batch_msg.detections.append(detection_array)
            annotated_image = annotator.result()
            vis_image = Image()
            vis_image.header.stamp = self.get_clock().now().to_msg()
            vis_image.header.frame_id = f"camera_{idx}"
            vis_image.format = "jpeg"
            vis_image.data = self.cv_bridge.cv2_to_imgmsg(
                annotated_image, encoding="bgr8")
            self.batch_vis_publishers[idx].publish(vis_image)

            # Publish Detection2DArray
            self.batch_detection_publishers[idx].publish(detection_array)

    # Publish batch detection message
        self.batched_camera_message_publisher.publish(batch_msg)


def main(args=None):
    rclpy.init(args=args)

    camera_object_detection_node = CameraDetectionNode()
    rclpy.spin(camera_object_detection_node)
    camera_object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
