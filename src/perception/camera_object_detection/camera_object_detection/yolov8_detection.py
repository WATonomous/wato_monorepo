import rclpy
from rclpy.node import Node
import os
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from camera_object_detection_msgs.msg import BatchDetection 
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    ObjectHypothesisWithPose,
    Detection2D,
    Detection2DArray,
)

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
    name:str
    dtype:np.dtype
    shape:Tuple
    cpu:ndarray
    gpu:int


class CameraDetectionNode(Node):

    def __init__(self):
        torch.zeros(1).cuda()

        super().__init__("camera_object_detection_node")
        self.get_logger().info("Creating batched camera detection node...")
        

        #Eve config params 
        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("left_camera_topic", "/camera/left/image_color")
        self.declare_parameter("center_camera_topic", "/camera/center/image_color")

        #Nuscenes 
        self.declare_parameter("front_center_camera_topic", "/CAM_FRONT/image_rect_compressed")
        self.declare_parameter("front_right_camera_topic", "/CAM_FRONT_RIGHT/image_rect_compressed")
        self.declare_parameter("front_left_camera_topic", "/CAM_FRONT_LEFT/image_rect_compressed")

        self.declare_parameter("back_center_camera_topic", "/CAM_BACK/image_rect_compressed")
        self.declare_parameter("back_right_camera_topic", "/CAM_BACK_RIGHT/image_rect_compressed")
        self.declare_parameter("back_left_camera_topic", "/CAM_BACK_LEFT/image_rect_compressed")

        self.declare_parameter("onnx_model_path", "/perception_models/tensorRT.onnx") #tensorRT. extensions
        self.declare_parameter("tensorRT_model_path", "/perception_models/tensorRT.engine")
        self.declare_parameter("eve_tensorRT_model_path", "/perception_models/eve.engine")
        self.declare_parameter("publish_vis_topic", "/annotated_img")
        self.declare_parameter("batch_publish_vis_topic", "/batch_annotated_img")
        self.declare_parameter("batch_publish_detection_topic", "/batch_detections")
        self.declare_parameter("publish_detection_topic", "/detections")
        self.declare_parameter("model_path", "/perception_models/yolov8m.pt")
       
        self.declare_parameter("image_size", 1024)
        self.declare_parameter("compressed", False)
        self.declare_parameter("crop_mode", "LetterBox")
        self.declare_parameter("save_detections", False)
        
        #Declare batch topic  
        self.declare_parameter("batch_inference_topic", "/batched_camera_message")
       
       
        
       #Batch inference topic 
        self.batch_inference_topic = self.get_parameter("batch_inference_topic").value
        self.batch_publish_detection_topic = self.get_parameter("batch_publish_detection_topic").value
        self.batch_publish_vis_topic = self.get_parameter("batch_publish_vis_topic").value
        self.onnx_model_path = self.get_parameter("onnx_model_path").value
        
        self.tensorRT_model_path = self.get_parameter("tensorRT_model_path").value
        self.eve_tensorRT_model_path = self.get_parameter("eve_tensorRT_model_path").value
        #Camera topics for eve 
        self.camera_topic = self.get_parameter("camera_topic").value
        self.left_camera_topic = self.get_parameter("left_camera_topic").value
        self.center_camera_topic = self.get_parameter("center_camera_topic").value

        #Camera topics for nuscenes 
        self.front_center_camera_topic  = self.get_parameter("front_center_camera_topic").value
        self.front_right_camera_topic = self.get_parameter("front_right_camera_topic").value
        self.front_left_camera_topic = self.get_parameter("front_left_camera_topic").value

        self.back_center_camera_topic = self.get_parameter("back_center_camera_topic").value
        self.back_right_camera_topic = self.get_parameter("back_right_camera_topic").value
        self.back_left_camera_topic  = self.get_parameter("back_left_camera_topic").value


    
        
        self.publish_vis_topic = self.get_parameter("publish_vis_topic").value
        self.publish_detection_topic = self.get_parameter("publish_detection_topic").value
        self.model_path = self.get_parameter("model_path").value
      
        self.image_size = self.get_parameter("image_size").value
        self.compressed = self.get_parameter("compressed").value
        self.crop_mode = self.get_parameter("crop_mode").value
        self.save_detections = bool(self.get_parameter("save_detections").value)
        self.counter = 0  # For saving detections
        if self.save_detections:
            if not os.path.exists("detections"):
                os.makedirs("detections")

        self.line_thickness = 1
        self.half = False

        

        self.subscription = self.create_subscription(
            Image if not self.compressed else CompressedImage,
            self.camera_topic,
            self.image_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        #Subscription for Nuscenes
        self.front_center_camera_subscription = Subscriber(self, CompressedImage, self.front_center_camera_topic)
        self.front_right_camera_subscription  = Subscriber(self, CompressedImage, self.front_right_camera_topic)
        self.front_left_camera_subscription = Subscriber(self, CompressedImage, self.front_left_camera_topic)
        self.back_center_camera_subscription = Subscriber(self, CompressedImage, self.back_center_camera_topic)
        self.back_right_camera_subscription = Subscriber(self, CompressedImage, self.back_right_camera_topic)
        self.back_left_camera_subscription = Subscriber(self, CompressedImage, self.back_left_camera_topic)
        self.ats = ApproximateTimeSynchronizer([self.front_center_camera_subscription, self.back_center_camera_subscription, self.front_right_camera_subscription, self.back_right_camera_subscription, self.front_left_camera_subscription, self.back_left_camera_subscription], queue_size=10, slop=0.5)
        self.ats.registerCallback(self.batch_inference_callback)

        #Subscription for Eve cameras
        self.right_camera_subscription = Subscriber(self, Image, self.camera_topic)
        self.left_camera_subscription = Subscriber(self, Image, self.left_camera_topic)
        self.center_camera_subscription = Subscriber(self, Image, self.center_camera_topic)
        self.eve_ats = ApproximateTimeSynchronizer([self.right_camera_subscription, self.left_camera_subscription,self.center_camera_subscription], queue_size=10, slop=0.1)
        
        #Create this callback function tomorrow
        self.eve_ats.registerCallback(self.eve_batch_inference_callback) 
        self.get_logger().info(f"TENSORT VERSION:{trt.__version__}")

        self.orig_image_width = None
        self.orig_image_height = None

        # set devicepublish_visDete
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        # CV bridge
        self.cv_bridge = CvBridge()
      
        status, self.stream = cudart.cudaStreamCreate()
        assert status.value == 0, "IS NOT ZERO"
       
        #self.build_engine()
        # self.initialize_engine(self.tensorRT_model_path, 3,3,1024,1024)
        # self.input_info, self.output_info =  self.initialize_tensors()
        self.model = AutoBackend(self.model_path, device=self.device, dnn=False, fp16=False)
        self.names = self.model.module.names if hasattr(self.model, "module") else self.model.names
        self.stride = int(self.model.stride)

        # setup vis publishers
        #Batch vis publishers
        self.batched_camera_message_publisher  = self.create_publisher(BatchDetection,self.batch_inference_topic, 10)
        self.batch_detection_publisher = self.create_publisher(Detection2DArray, self.batch_publish_detection_topic, 10)
        self.batch_vis_publisher = self.create_publisher(CompressedImage, self.batch_publish_vis_topic, 10)
    
        #vis publishers
        self.vis_publisher = self.create_publisher(Image, self.publish_vis_topic, 10)
        self.detection_publisher = self.create_publisher(
            Detection2DArray, self.publish_detection_topic, 10
        )
        self.get_logger().info( 
            f"Successfully created node listening on camera topic: {self.camera_topic}..."
        )
 
    def build_engine(self):
    #Only calling this function when we dont have an engine file
    #Reading the onnx file in the perception models directory
        self.max_batch_size = 3
        self.channels = 3
        self.height = 1024
        self.width = 1024
        self.shape_input_model = [self.max_batch_size, self.channels, self.height, self.width]
        self.logger = trt.Logger(trt.Logger.VERBOSE)
        self.builder = trt.Builder(self.logger)
        self.config  = self.builder.create_builder_config()
        self.cache = self.config.create_timing_cache(b"")
        self.config.set_timing_cache(self.cache, ignore_mismatch=False)
        self.total_memory = torch.cuda.get_device_properties(self.device).total_memory
        self.config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, self.total_memory)
        self.flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        self.network = self.builder.create_network(self.flag)
        self.parser = trt.OnnxParser(self.network, self.logger)
        with open(self.onnx_model_path, "rb") as f:
            if not self.parser.parse(f.read()):
                self.get_logger().info("ERROR: Cannot read ONNX FILE")
                for error in range(self.parser.num_errors):
                    self.get_logger().info(f"{self.parser.get_error(error)}")
        self.inputs = [self.network.get_input(i) for i in range(self.network.num_inputs)]
        self.outputs = [self.network.get_output(i) for i in range(self.network.num_outputs)]
        for input in self.inputs:
         self.get_logger().info(f"Model {input.name} shape:{input.shape} {input.dtype}")
        for output in self.outputs:
         self.get_logger().info(f"Model {output.name} shape: {output.shape} {output.dtype}")  
        if self.max_batch_size > 1:
            self.profile  = self.builder.create_optimization_profile()
            self.min_shape = [1, self.channels, self.height, self.width]                 # Minimum batch size of 1
            self.opt_shape = [int(self.max_batch_size / 2), self.channels, self.height, self.width]  # Optimal batch size (half of max, so 3)
            self.max_shape = self.shape_input_model

            for input in self.inputs:
                self.profile.set_shape(input.name, self.min_shape, self.opt_shape, self.max_shape)
            self.config.add_optimization_profile(self.profile)
        self.half = True
        self.int8 = False
        if self.half:
            self.config.set_flag(trt.BuilderFlag.FP16)
        elif self.int8:
            self.config.set_flag(trt.BuilderFlag.INT8)
        self.engine_bytes = self.builder.build_serialized_network(self.network, self.config)
        assert self.engine_bytes is not None, "Failed to create engine"
        self.get_logger().info("BUILT THE ENGINE ")
        with open(self.tensorRT_model_path, "wb") as f:
            f.write(self.engine_bytes)
        self.get_logger().info("FINISHED WRITING ")
 
    def initialize_engine(self, weight, batch_size, rgb, width, height):
        self.weight = Path(weight) if isinstance(weight, str) else weight
        self.logger  = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace ='')
        with trt.Runtime(self.logger) as runtime:
            self.tensorRT_model = runtime.deserialize_cuda_engine(self.weight.read_bytes())
            #self.get_logger().info(f"TENSORRT Model:{self.weight}")
        self.execution_context = self.tensorRT_model.create_execution_context()  
        
          
        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.input_tensor_name = self.tensorRT_model.get_tensor_name(0)
      
        self.execution_context.set_input_shape(self.input_tensor_name, (batch_size, rgb, width, height))
        self.inputShape = self.execution_context.get_tensor_shape(self.input_tensor_name)
        #self.get_logger().info(f"tensor Shape:{self.inputShape}")
        self.names = [self.tensorRT_model.get_tensor_name(i) for i in range(self.num_io_tensors)]
        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.bindings = [0] * self.num_io_tensors
        self.num_inputs  = 0
        self.num_outputs = 0
        self.input_names = []
        self.output_names = []
        for i in range(self.num_io_tensors):
              self.tensor_name  = self.tensorRT_model.get_tensor_name(i)
              if self.tensorRT_model.get_tensor_mode(self.tensor_name) == trt.TensorIOMode.INPUT:
                self.input_names.append(self.tensor_name)
                self.num_inputs += 1
              elif self.tensorRT_model.get_tensor_mode(self.tensor_name) == trt.TensorIOMode.OUTPUT:
                self.output_names.append(self.tensor_name)
                self.num_outputs += 1 
        self.num_inputs = len(self.input_names)
        self.num_outputs = len(self.output_names)    
        self.input_names = self.names[:self.num_inputs]
        #self.output_names = self.names[:self.num_outputs] #This includes the input name in output
        self.output_names = [name for name in self.output_names if name not in self.input_names] # This line removes it
        #self.get_logger().info(f"INPUT NAMES:: {self.input_names}")
        #self.get_logger().info(f"OUTPUT NAMES:: {self.output_names}")
       
    
    def initialize_tensors(self):
        self.dynamic = True 
        self.input_info = []
        self.output_info = []
        self.output_ptrs = []
      
       
        for name in self.output_names:
            #self.get_logger().info(f"NAMES of outputs:{name}")
          
            self.tensorRT_output_shape = self.execution_context.get_tensor_shape(name)
            #self.get_logger().info(f"Tensor shape:{self.tensorRT_output_shape}")
            self.outputDtype  = trt.nptype(self.tensorRT_model.get_tensor_dtype(name))
            self.output_cpu = np.empty(self.tensorRT_output_shape, dtype = self.outputDtype)
            status, self.output_gpu = cudart.cudaMallocAsync(self.output_cpu.nbytes, self.stream)
            assert status.value == 0
            cudart.cudaMemcpyAsync(self.output_gpu, self.output_cpu.ctypes.data, self.output_cpu.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
            self.output_ptrs.append(self.output_gpu)
            self.output_info.append(Tensor(name, self.outputDtype, self.tensorRT_output_shape, self.output_cpu, self.output_gpu))
        
        for i, name in enumerate(self.input_names):
            if self.tensorRT_model.get_tensor_name(i) == name: 
             self.tensorRT_input_shape = tuple(self.inputShape) 
             #self.get_logger().info(f"Final Input Shape:{self.tensorRT_input_shape}")
             self.dtype = trt.nptype(self.tensorRT_model.get_tensor_dtype(name))
             self.input_cpu  = np.empty(self.tensorRT_input_shape, self.dtype)
             status, self.input_gpu = cudart.cudaMallocAsync(self.input_cpu.nbytes, self.stream)
             assert status.value == 0, "DOES NOT MATCH"
             cudart.cudaMemcpyAsync(
                    self.input_gpu, self.input_cpu.ctypes.data, self.input_cpu.nbytes,
                    cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
             self.input_info.append(Tensor(name, self.dtype, self.tensorRT_input_shape, self.input_cpu, self.input_gpu))
             
        return self.input_info, self.output_info
    
    def tensorRT_inferencing(self, batch_array):
        assert batch_array.ndim == 4
        batch_size = batch_array.shape[0]
        assert batch_size == self.input_info[0].shape[0]
        #self.get_logger().info(f"Batch Array Stats: shape={batch_array.shape}, dtype={batch_array.dtype}, range=({batch_array.min()}, {batch_array.max()})")
        #self.inputTensors = [batch_array]
        self.contiguous_inputs = [np.ascontiguousarray(i) for i in batch_array]
        #self.get_logger().info(f"Contiguous Inputs: dtype={self.contiguous_inputs[0].dtype}, shape={self.contiguous_inputs[0].shape}")
        #for i, input_tensor in enumerate(self.contiguous_inputs):
            #self.get_logger().info(f"Input {i}: shape={input_tensor.shape}, dtype={input_tensor.dtype}, range=({input_tensor.min()}, {input_tensor.max()})")
        #for i, input_gpu in enumerate(self.input_info):
                #self.get_logger().info(f"Input {i} GPU address: {input_gpu.gpu}")

        for i in range(self.num_inputs):
                name = self.input_info[i].name
                cudart.cudaMemcpyAsync(self.input_info[i].gpu, self.contiguous_inputs[i].ctypes.data,
                            self.contiguous_inputs[i].nbytes, cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
                self.execution_context.set_tensor_address(name, self.input_info[i].gpu)

        self.output_gpu_ptrs:List[int] = []
        self.outputs_ndarray:List[ndarray] = []
        for i in range(self.num_outputs):
                j = i + self.num_inputs  
                cpu = self.output_info[i].cpu
                gpu = self.output_info[i].gpu
                self.outputs_ndarray.append(cpu)
                self.output_gpu_ptrs.append(gpu)
                self.execution_context.set_tensor_address(self.output_info[i].name, self.output_info[i].gpu)

        
        self.execution_context.execute_async_v3(self.stream)
        cudart.cudaStreamSynchronize(self.stream)
        for i, o in enumerate(self.output_gpu_ptrs):
             cudart.cudaMemcpyAsync(
            self.outputs_ndarray[i].ctypes.data, o, self.outputs_ndarray[i].nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream
                )
      
        return self.outputs_ndarray

    # will be called with eve rosbag 
    def eve_batch_inference_callback(self, msg1,msg2,msg3):
        imageList = [msg1,msg2,msg3]
        eve_batched_list = []
        for msg in imageList:
            numpy_array = np.frombuffer(msg.data, np.uint8)
            compressedImage = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
            resized_compressedImage  = cv2.resize(compressedImage, (1024, 1024))
            rgb_image = cv2.color(resized_compressedImage, cv2.COLOR_BGR2RGB)
            normalized_image = rgb_image / 255.0
            chw_image = np.transpose(normalized_image, (2,0,1))
            float_image = chw_image.astype(np.float32)
            tensor_image = torch.from_numpy(float_image).to('cuda')
            batched_list.append(tensor_image)
        batched_list = [tensor.cpu().numpy() for tensor in batched_list]
        batched_images = np.stack(batched_list, axis=0)
        self.initialize_engine(self.eve_tensorRT_model_path, 3,3,1024,1024)
        self.input_info, self.output_info = self.initialize_tensors()
        detections = self.tensorRT_inferencing(batched_images)
        batch_size = detections[0].shape[0]
        anchors = detections[0].shape[2]
        results_dict = {i: [] for i in range(batch_size)}
        for image_idx in range(batch_size):
            # self.get_logger().info(f"[{time.time()}] Processing image index: {image_idx}")
            image_detections = detections[0][image_idx]
        
            if image_idx not in results_dict:
                results_dict[image_idx] = []

            for anchor_idx in range(anchors):
                x_center = image_detections[0][anchor_idx]
                y_center = image_detections[1][anchor_idx]
                width = image_detections[2][anchor_idx]
                height = image_detections[3][anchor_idx]

                confidence = image_detections[4][anchor_idx]
                class_probs = [image_detections[class_idx][anchor_idx] for class_idx in range(5,18)]
                predicted_class = np.argmax(class_probs)
                predicted_prob = class_probs[predicted_class]

                if confidence > 0.45:
                
                    #Convert from chw to tlbr  
                    x_min, y_min, x_max, y_max = self.convert_to_xyxy(x_center, y_center, width, height, 640, 640)
                    self.get_logger().info(f"Added bounding box for image {image_idx}: {x_min, y_min, x_max, y_max}, class:{predicted_class}")
                    results_dict[image_idx].append([x_min, y_min, x_max, y_max, confidence, predicted_class]) 
            results_dict[image_idx] = self.nms(results_dict[image_idx], confidence_threshold = 0.55, iou_threshold=0.5)


    # will be called with nuscenes rosbag
    def batch_inference_callback(self,msg1,msg2,msg3, msg4,msg5,msg6):
        #Taking msgs from all 6 ros2 subscribers
        image_list =  [msg1,msg2,msg3,msg4,msg5,msg6]
        batched_list = []
        #Preprocessing
        for msg in image_list:  
            numpy_array = np.frombuffer(msg.data, np.uint8)
            compressedImage  = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
            original_height, original_width = compressedImage.shape[:2]
            #self.get_logger().info(f"original Height:{original_height} original width: {original_width}")
            resized_compressedImage = cv2.resize(compressedImage,(640, 640))
            rgb_image = cv2.cvtColor(resized_compressedImage, cv2.COLOR_BGR2RGB)
            normalized_image = rgb_image / 255.0
            chw_image = np.transpose(normalized_image, (2,0,1))
            float_image = chw_image.astype(np.float32)
            # tensor_image = torch.from_numpy(float_image).to('cuda')
            batched_list.append(float_image)
        # batched_list = [tensor.cpu().numpy() for tensor in batched_list]          
        batched_images = np.stack(batched_list, axis=0)
        self.initialize_engine(self.tensorRT_model_path, 6,3,640,640)
        self.input_info, self.output_info =  self.initialize_tensors()
        detections = self.tensorRT_inferencing(batched_images)
        """ 
            - Detection[0] represents the first output name, 'outputs' with shape(6,84,8400) -> 6 representing 6 images 
            - Detection[0][0] represents the first image can range from 0 to 5, and the shape would be (84,8400) -> 84 representing 84 detection components  for bounding boxes, classes and confidence scores
            - Detection[0][0][0] represnets the first detection components ranging from 0 to 83 
            - 8400 represents 8400 anchors 

        """
       
        batch_size = detections[0].shape[0]
        results_dict = {i: [] for i in range(batch_size)}
        for image_idx in range(batch_size):
           # Extract detections for the current image
            image_detections = detections[0][image_idx]

            # Extract individual components
            x_centers = image_detections[0]
            y_centers = image_detections[1]
            widths = image_detections[2]
            heights = image_detections[3]
            confidences = image_detections[4]
            class_probs = image_detections[5:18]  # Shape: (13, 8400)

            # Filter by confidence threshold
            valid_indices = confidences > 0.3
            if not np.any(valid_indices):
                continue  # Skip image if no detections pass confidence threshold

            # Filter relevant detections
            x_centers = x_centers[valid_indices]
            y_centers = y_centers[valid_indices]
            widths = widths[valid_indices]
            heights = heights[valid_indices]
            confidences = confidences[valid_indices]
            class_probs = class_probs[:, valid_indices]

            # Predict classes and probabilities
            predicted_classes = np.argmax(class_probs, axis=0)
            predicted_probs = class_probs[predicted_classes, range(class_probs.shape[1])]

            # Convert boxes to (x_min, y_min, x_max, y_max)
            x_mins = x_centers - widths / 2
            y_mins = y_centers - heights / 2
            x_maxs = x_centers + widths / 2
            y_maxs = y_centers + heights / 2

            # Combine detections into final format
            filtered_detections = np.stack([x_mins, y_mins, x_maxs, y_maxs, confidences, predicted_classes], axis=-1)
            #self.get_logger().info(f"Detections:{filtered_detections}")
            
            # Apply NMS
            results_dict[image_idx] = self.nms(filtered_detections.tolist(), confidence_threshold=0.55, iou_threshold=0.5)

#Log final results if needed
        for image_idx, detections in results_dict.items():
            #self.get_logger().info(f"Image {image_idx}: {len(detections)} detections")
            for detection in detections:
                x_min, y_min, x_max, y_max, confidence, predicted_class = detection
                self.get_logger().info(
                    f"  Bounding Box: [{x_min:.2f}, {y_min:.2f}, {x_max:.2f}, {y_max:.2f}], "
                    f"Confidence: {confidence:.2f}, Class: {predicted_class}"
                )

        self.publish_batch(image_list, results_dict)
          # To store final results after NMS
   
    def convert_to_xyxy(self, x_center, y_center, width, height, imageHeight, imageWidth):
        half_width = width /2
        half_height = height /2 
        x_min = x_center - half_width
        y_min = y_center - half_height
        x_max = x_center + half_width
        y_max = y_center + half_height
        return x_min, y_min, x_max, y_max     

    def nms(self,bboxes, confidence_threshold= 0.45,iou_threshold = 0.5 ):
        #Confidence threshold
        bboxes_thresholded = [bbox for bbox in bboxes if bbox[4] > confidence_threshold]
        bboxes_sorted = sorted(bboxes_thresholded, key=lambda x: x[4], reverse=True)
        bbox_list_new = []
        while bboxes_sorted:
            current_box = bboxes_sorted.pop(0)
            bbox_list_new.append(current_box)
        # Filter boxes with IoU below the threshold
            bboxes_sorted = [
                box for box in bboxes_sorted if self.get_iou(current_box[:4], box[:4]) < iou_threshold
                ]
        return bbox_list_new

    def get_iou(self, box1, box2):
        x1, y1, x2, y2 = box1
        x3, y3, x4, y4 = box2

    # Intersection coordinates
        x_inter1 = max(x1, x3)
        y_inter1 = max(y1, y3)
        x_inter2 = min(x2, x4)
        y_inter2 = min(y2, y4)

        # Width and height of the intersection
        width_inter = abs(x_inter2 - x_inter1)
        height_inter = abs(y_inter2 - y_inter1)

        # If there is no intersection
        if x_inter2 < x_inter1 or y_inter2 < y_inter1:
            return 0.0

        # Area of intersection
        area_inter = width_inter * height_inter

        # Areas of the two boxes
        width_box1 = abs(x2 - x1)
        height_box1 = abs(y2 - y1)
        area_box1 = width_box1 * height_box1

        width_box2 = abs(x4 - x3)
        height_box2 = abs(y4 - y3)
        area_box2 = width_box2 * height_box2

        # Union area
        area_union = area_box1 + area_box2 - area_inter

        # Intersection over Union
        iou = area_inter / area_union

        return iou
    
    
    def publish_batch(self, image_list, results_dict):
        batch_msg = BatchDetection()
        batch_msg.header.stamp = self.get_clock().now().to_msg()
        batch_msg.header.frame_id = "batch"

        for idx, img_msg in enumerate(image_list):
            numpy_image = np.frombuffer(img_msg.data, np.uint8)
            image = cv2.imdecode(numpy_image, cv2.IMREAD_COLOR)
            height, width = image.shape[:2]
            detection_array = Detection2DArray()
            detection_array = Detection2DArray()
            detection_array.header.stamp = batch_msg.header.stamp
            detection_array.header.frame_id = f"camera_{idx}"
            detections = results_dict[idx]
            for bbox in results_dict[idx]:
                x_min, y_min, x_max, y_max , confidence, predicted_class = bbox
                x_min = int(x_min * width / 640)
                x_max = int(x_max * width / 640)
                y_min = int(y_min * height / 640)
                y_max = int(y_max * height / 640)

                label = f"Class: {predicted_class}, Conf: {confidence:.2f}"
                cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
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

            # Add detection array to batch message
            batch_msg.detections.append(detection_array)

            # Publish visualized image
            vis_compressed_image = CompressedImage()
            vis_compressed_image.header.stamp = self.get_clock().now().to_msg()
            vis_compressed_image.header.frame_id = f"camera_{idx}"
            vis_compressed_image.format = "jpeg"
            vis_compressed_image.data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
            self.batch_vis_publisher.publish(vis_compressed_image)

        # Publish Detection2DArray
            self.batch_detection_publisher.publish(detection_array)

    # Publish batch detection message
        self.batched_camera_message_publisher.publish(batch_msg)
                
    #Converting to tensor for batching

    def draw_bounding_boxes(self,img_msg, detections):
        numpy_image = np.frombuffer(img_msg.data, np.uint8)
        image = cv2.imdecode(numpy_image, cv2.IMREAD_COLOR)
        for bbox in detections:
            x_min, y_min, x_max, y_max, confidence, predicted_class = bbox
            label = f"Class: {predicted_class}, Conf: {confidence:.2f}"
            #self.get_logger().info(f"{label}")
            cv2.rectangle(image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.putText(image, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return image
    def batch_convert_to_tensor(self, cv_image):
        img  = cv_image.transpose(2,0,1)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        return img

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

        # Convert
        img = cv_image.transpose(2, 0, 1)

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

    def image_callback(self, msg):
        self.get_logger().debug("Received image")
        if self.orig_image_width is None:
            self.orig_image_width = msg.width
            self.orig_image_height = msg.height

        images = [msg]  # msg is a single sensor image
        startTime = time.time()
        for image in images:

            # convert ros Image to cv::Mat
            if self.compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
                    return

            # preprocess image and run through prediction
            img = self.crop_and_convert_to_tensor(cv_image)
            pred = self.model(img)

            # nms function used same as yolov8 detect.py
            pred = non_max_suppression(pred) #Eliminates overlapping bounding boxes
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


def main(args=None):
    rclpy.init(args=args)

    camera_object_detection_node = CameraDetectionNode()
    rclpy.spin(camera_object_detection_node)
    camera_object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
