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
        super().__init__("camera_object_detection_node")
        self.get_logger().info("Creating batched camera detection node...")

        # Eve config params
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("left_camera_topic", "/camera/left/image_color")
        self.declare_parameter("center_camera_topic",
                               "/camera/center/image_color")
        self.declare_parameter("image_width", 1600)
        self.declare_parameter("image_height", 900)

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
        self.declare_parameter("publish_vis_topic", False)
        self.declare_parameter(
            "batch_publish_detection_topic", "/batch_detections")
        self.declare_parameter("model_path", "/perception_models/yolov8m.pt")
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

        # Bool publish vis topic
        self.publish_vis_topic = bool(
            self.get_parameter("publish_vis_topic").value)

        # Model Path and configs
        self.model_path = self.get_parameter("model_path").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
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

        # NOTE: old changes from team/tracking branch, delete if unneeded
        '''
        self.subscription = self.create_subscription(
            Image if not self.compressed else CompressedImage,
            self.camera_topic,
            self.image_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
        )
        '''

        self.batch_size = getattr(self, "num_cameras", 3)
        self.input_c = 3
        self.input_h = 640
        self.input_w = 640
        self.batched_images_buffer = None

        # Start logger
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace='')

        # Initialize TensorRT model
        self.weight = Path(self.tensorRT_model_path) if isinstance(
            self.tensorRT_model_path, str) else self.tensorRT_model_path
        with trt.Runtime(self.logger) as runtime:
            self.tensorRT_model = runtime.deserialize_cuda_engine(
                self.weight.read_bytes())
        self.execution_context = self.tensorRT_model.create_execution_context()
        if not self.execution_context:
            self.get_logger().error("Failed to create execution context")
            return 1

        input_name = self.tensorRT_model.get_tensor_name(
            0)  # usually the first I/O is your input
        self.execution_context.set_input_shape(
            input_name,
            (self.batch_size, self.input_c, self.input_h, self.input_w)
        )

        # Allocate GPU memory for input and output tensors
        self._collect_io_names()
        self._alloc_cuda_buffers()

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

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        self.last_publish_time = self.get_clock().now()

        # Batch vis publishers
        self.batched_camera_message_publisher = self.create_publisher(
            BatchDetection, self.batch_inference_topic, 10)
        self.eve_batched_camera_message_publisher = self.create_publisher(
            EveBatchDetection, self.eve_batch_inference_topic, 10)

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

    def _collect_io_names(self):
        # figure out which TRT tensors are inputs vs outputs
        all_names = [self.tensorRT_model.get_tensor_name(i)
                     for i in range(self.tensorRT_model.num_io_tensors)]
        modes = [self.tensorRT_model.get_tensor_mode(n)
                 for n in all_names]

        # split into input_names / output_names
        self.input_names = [n for n, m in zip(all_names, modes)
                            if m == trt.TensorIOMode.INPUT]
        self.output_names = [n for n, m in zip(all_names, modes)
                             if m == trt.TensorIOMode.OUTPUT]

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

            status, dev_mem = cudart.cudaMallocAsync(
                host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for input {name}"

            self.input_info.append(
                Tensor(name, dtype, shape, host_mem, dev_mem))

        # host+device for outputs
        self.output_info = []
        for name in self.output_names:
            shape = tuple(self.execution_context.get_tensor_shape(name))
            dtype = trt.nptype(self.tensorRT_model.get_tensor_dtype(name))
            host_mem = np.empty(shape, dtype=dtype)

            status, dev_mem = cudart.cudaMallocAsync(
                host_mem.nbytes, self.stream)
            assert status.value == 0, f"cudaMallocAsync failed for output {name}"

            self.output_info.append(
                Tensor(name, dtype, shape, host_mem, dev_mem))

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
            self.stream
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
                self.stream
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
            compressedImage, (self.input_h, self.input_w), interpolation=cv2.INTER_LINEAR)
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
                dtype=np.float32
            )
        # Use concurrent futures to parallelize the preprocessing step
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = []
            for i, msg in enumerate(image_list):
                futures.append(executor.submit(
                    self.preprocess_image, msg, self.batched_images_buffer[i]))

            concurrent.futures.wait(futures)
        detections = self.tensorRT_inferencing(self.batched_images_buffer)
        decoded = self.parse_detections(detections)
        self.publish_batch_nuscenes([msg1, msg2, msg3], decoded)
        now = self.get_clock().now()
        self.get_logger().info(
            f"Speed of inference: {1/((now - self.last_publish_time).nanoseconds / 1e9)} fps")

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
                bboxes = np.array([d["bbox"]
                                  for d in batch_detections])  # Shape: (N, 4)
                bboxes[:, [0, 2]] *= width / \
                    self.input_w  # Scale x-coordinates
                bboxes[:, [1, 3]] *= height / \
                    self.input_h  # Scale y-coordinates
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

                    if self.publish_vis_topic:
                        annotator.box_label(
                            (x_min, y_min, x_max, y_max), label, color=(0, 100, 0))

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
                    '.jpg', annotated_image, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.batch_vis_publishers[idx].publish(vis_compressed_image)

        # Publish batch detection message
        self.batched_camera_message_publisher.publish(batch_msg)

    def eve_preprocess_image(self, msg, dest_buffer):
        numpy_array = np.frombuffer(msg.data, np.uint8)
        image = numpy_array.reshape(
            (self.image_width, self.image_height, self.input_c))

        if msg.encoding == 'bgr8':
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif msg.encoding == 'rgb8':
            rgb_image = image
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        # Resize the image to the desired input size
        resized_rgb_image = cv2.resize(
            rgb_image, (self.input_h, self.input_w), interpolation=cv2.INTER_LINEAR)

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
                dtype=np.float32
            )
        # Use concurrent futures to parallelize the preprocessing step
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = []
            for i, msg in enumerate(image_list):
                futures.append(executor.submit(
                    self.preprocess_image, msg, self.batched_images_buffer[i]))

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
                bboxes = np.array([d["bbox"]
                                  for d in batch_detections])  # Shape: (N, 4)
                bboxes[:, [0, 2]] *= width / \
                    self.input_w  # Scale x-coordinates
                bboxes[:, [1, 3]] *= height / \
                    self.input_h  # Scale y-coordinates
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

                    if self.publish_vis_topic:
                        annotator.box_label(
                            (x_min, y_min, x_max, y_max), label, color=(0, 100, 0))

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
                    '.jpg', annotated_image, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.eve_batch_vis_publishers[idx].publish(
                    vis_compressed_image)

        # Publish batch detection message
        self.eve_batched_camera_message_publisher.publish(batch_msg)

    def destroy_node(self):
        for tensor in self.input_info + self.output_info:
            status = cudart.cudaFreeAsync(tensor.gpu, self.stream)
            assert status.value == 0
        cudart.cudaStreamDestroy(self.stream)
        super().destroy_node()

    # NOTE: old changes from team/tracking branch, delete if unneeded
    '''
    def crop_image(self, cv_image):
        if self.crop_mode == "LetterBox":
            img = LetterBox(self.image_size, stride=self.stride)(image=cv_image)
        elif self.crop_mode == "CenterCrop":
            img = CenterCrop(self.image_size)(cv_image)
        else:
            raise Exception("Invalid crop mode, please choose either 'LetterBox' or 'CenterCrop'!")

        return img

    def convert_bboxes_to_orig_frame(self, bbox):
        """
        Converts bounding box coordinates from the scaled image frame back to the original image frame.

        This function takes into account the original image dimensions and the scaling method used
        (either "LetterBox" or "CenterCrop") to accurately map the bounding box coordinates back to
        their original positions in the original image.

        Parameters:
        bbox (list): A list containing the bounding box coordinates in the format [x1, y1, w1, h1]
                    in the scaled image frame.

        Returns:
        list: A list containing the bounding box coordinates in the format [x1, y1, w1, h1]
            in the original image frame.

        """
        width_scale = self.orig_image_width / self.image_size
        height_scale = self.orig_image_height / self.image_size
        if self.crop_mode == "LetterBox":
            translation = (self.image_size - self.orig_image_height / width_scale) / 2
            return [
                bbox[0] * width_scale,
                (bbox[1] - translation) * width_scale,
                bbox[2] * width_scale,
                bbox[3] * width_scale,
            ]
        elif self.crop_mode == "CenterCrop":
            translation = (self.orig_image_width / height_scale - self.image_size) / 2
            return [
                (bbox[0] + translation) * height_scale,
                bbox[1] * height_scale,
                bbox[2] * height_scale,
                bbox[3] * height_scale,
            ]

    def crop_and_convert_to_tensor(self, cv_image):
        """
        Preprocess the image by resizing, padding and rearranging the dimensions.

        Parameters:
            cv_image: A numpy or cv2 image of shape (w,h,3)

        Returns:
            torch.Tensor image for model input of shape (1,3,w,h)
        """
        img = self.crop_image(cv_image)
        # self.get_logger().info(f"croped image: {img}")

        # Convert
        img = img.transpose(2, 0, 1)

        # Further conversion
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        img = img.unsqueeze(0)

        return img

    def postprocess_detections(self, detections, annotator):
        """
        Post-process draws bouningboxes on camera image.

        Parameters:
            detections: A list of dict with the format
                {
                    "label": str,
                    "bbox": [float],
                    "conf": float
                }
            annotator: A ultralytics.yolo.utils.plotting.Annotator for the current image

        Returns:
            processed_detections: filtered detections
            annotator_img: image with bounding boxes drawn on
        """
        processed_detections = detections

        for det in detections:
            label = f'{det["label"]} {det["conf"]:.2f}'
            x1, y1, w1, h1 = det["bbox"]
            xyxy = [x1, y1, x1 + w1, y1 + h1]
            annotator.box_label(xyxy, label, color=colors(1, True)) 

        annotator_img = annotator.result()
        return (processed_detections, annotator_img)

    def publish_vis(self, annotated_img, msg, feed):
        # Publish visualizations
        imgmsg = self.cv_bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        self.vis_publisher.publish(imgmsg)

    def publish_detections(self, detections, msg, feed):
        # Publish detections to an detectionList message
        detection2darray = Detection2DArray()

        # fill header for detection list
        detection2darray.header.stamp = msg.header.stamp
        detection2darray.header.frame_id = msg.header.frame_id
        # populate detection list
        if detections is not None and len(detections):
            for detection in detections:
                detection2d = Detection2D()
                detection2d.header.stamp = msg.header.stamp
                detection2d.header.frame_id = msg.header.frame_id
                detected_object = ObjectHypothesisWithPose()
                detected_object.hypothesis.class_id = detection["label"]
                detected_object.hypothesis.score = detection["conf"]
                detection2d.results.append(detected_object)
                detection2d.bbox.center.position.x = detection["bbox"][0]
                detection2d.bbox.center.position.y = detection["bbox"][1]
                detection2d.bbox.size_x = detection["bbox"][2]
                detection2d.bbox.size_y = detection["bbox"][3]

                # append detection to detection list
                detection2darray.detections.append(detection2d)

        self.detection_publisher.publish(detection2darray)

    # def image_callback(self, msg):
    #     self.get_logger().debug("Received image")
    #     if self.orig_image_width is None:
    #         self.orig_image_width = msg.width
    #         self.orig_image_height = msg.height
    #         self.get_logger().info(f"height: {self.orig_image_height}")
    #         self.get_logger().info(f"width: {self.orig_image_width}")

    #     images = [msg]  # msg is a single sensor image
    #     startTime = time.time()
    #     for image in images:

    #         # convert ros Image to cv::Mat
    #         if self.compressed:
    #             np_arr = np.frombuffer(msg.data, np.uint8)
    #             cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         else:
    #             try:
    #                 cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    #             except CvBridgeError as e:
    #                 self.get_logger().error(str(e))
    #                 return

    #         # preprocess image and run through prediction
    #         img = self.crop_and_convert_to_tensor(cv_image)
    #         pred = self.model(img)

    #         # nms function used same as yolov8 detect.py
    #         pred = non_max_suppression(pred)
    #         detections = []
    #         for i, det in enumerate(pred):  # per image
    #             if len(det):
    #                 # Write results
    #                 for *xyxy, conf, cls in reversed(det):
    #                     label = self.names[int(cls)]

    #                     bbox = [
    #                         xyxy[0],
    #                         xyxy[1],
    #                         xyxy[2] - xyxy[0],
    #                         xyxy[3] - xyxy[1],
    #                     ]
    #                     bbox = [b.item() for b in bbox]
    #                     bbox = self.convert_bboxes_to_orig_frame(bbox)

    #                     detections.append(
    #                         {
    #                             "label": label,
    #                             "conf": conf.item(),
    #                             "bbox": bbox,
    #                         }
    #                     )
    #                     self.get_logger().debug(f"{label}: {bbox}")

    #         annotator = Annotator(
    #             cv_image,
    #             line_width=self.line_thickness,
    #             example=str(self.names),
    #         )
    #         (detections, annotated_img) = self.postprocess_detections(detections, annotator)

    #         # Currently we support a single camera so we pass an empty string
    #         feed = ""
    #         self.publish_vis(annotated_img, msg, feed)
    #         self.publish_detections(detections, msg, feed)

    #         if self.save_detections:
    #             cv2.imwrite(f"detections/{self.counter}.jpg", annotated_img)
    #             self.counter += 1

    #     self.get_logger().info(
    #         f"Finished in: {time.time() - startTime}, {1/(time.time() - startTime)} Hz"
    #     )


    def image_callback(self, msg):
        self.get_logger().debug("Received image")

        images = [msg]  # msg is a single sensor image
        startTime = time.time()
        for image in images:

            # Check if the image is compressed or not
            if self.compressed:
                # Decode the compressed image
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Set original image dimensions if not already set
                if self.orig_image_width is None:
                    self.orig_image_height, self.orig_image_width = cv_image.shape[:2]

            else:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    
                    # Set original image dimensions if not already set
                    if self.orig_image_width is None:
                        self.orig_image_width = msg.width
                        self.orig_image_height = msg.height

                except CvBridgeError as e:
                    self.get_logger().error(str(e))
                    return

            # preprocess image and run through prediction
            img = self.crop_and_convert_to_tensor(cv_image)
            pred = self.model(img)
            

            # nms function used same as yolov8 detect.py
            pred = non_max_suppression(pred)
            detections = []
            for i, det in enumerate(pred):  # per image
                if len(det):
                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        label = self.names[int(cls)]

                        bbox = [
                            xyxy[0],
                            xyxy[1],
                            xyxy[2] - xyxy[0],
                            xyxy[3] - xyxy[1],
                        ]
                        bbox = [b.item() for b in bbox]
                        bbox = self.convert_bboxes_to_orig_frame(bbox)

                        detections.append(
                            {
                                "label": label,
                                "conf": conf.item(),
                                "bbox": bbox,
                            }
                        )
                        self.get_logger().debug(f"{label}: {bbox}")

            annotator = Annotator(
                cv_image,
                line_width=self.line_thickness,
                example=str(self.names),
            )
            (detections, annotated_img) = self.postprocess_detections(detections, annotator)

            # Currently we support a single camera so we pass an empty string
            feed = ""
            self.publish_vis(annotated_img, msg, feed)
            self.publish_detections(detections, msg, feed)

            if self.save_detections:
                cv2.imwrite(f"detections/{self.counter}.jpg", annotated_img)
                self.counter += 1

            self.get_logger().info(
                f"Finished in: {time.time() - startTime}, {1/(time.time() - startTime)} Hz"
            )
    '''


def main(args=None):
    rclpy.init(args=args)

    camera_object_detection_node = CameraDetectionNode()
    rclpy.spin(camera_object_detection_node)
    camera_object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
