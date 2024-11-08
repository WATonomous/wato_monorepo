import rclpy
from rclpy.node import Node
import os
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    ObjectHypothesisWithPose,
    Detection2D,
    Detection2DArray,
)
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

        self.declare_parameter("onnx_model_path", "/perception_models/yolov8m.onnx")
        self.declare_parameter("tensorRT_model_path", "/perception_models/yolov8m.engine")
        self.declare_parameter("publish_vis_topic", "/annotated_img")
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
        self.onnx_model_path = self.get_parameter("onnx_model_path").value
        
        self.tensorRT_model_path = self.get_parameter("tensorRT_model_path").value

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


        #Parameter for TensorRT Model
        
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
        self.ats = ApproximateTimeSynchronizer([self.front_center_camera_subscription, self.back_center_camera_subscription, self.front_right_camera_subscription, self.back_right_camera_subscription, self.front_left_camera_subscription, self.back_left_camera_subscription], queue_size=10, slop=0.1)
        self.ats.registerCallback(self.batch_callback)

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
        self.initialize_engine(self.tensorRT_model_path)
        self.model = AutoBackend(self.model_path, device=self.device, dnn=False, fp16=False)
        self.names = self.model.module.names if hasattr(self.model, "module") else self.model.names
        self.stride = int(self.model.stride)

        # setup vis publishers
        self.batched_camera_message_publisher  = self.create_publisher(Image,self.batch_inference_topic, 10)
        self.vis_publisher = self.create_publisher(Image, self.publish_vis_topic, 10)
        self.detection_publisher = self.create_publisher(
            Detection2DArray, self.publish_detection_topic, 10
        )
        self.get_logger().info(
            f"Successfully created node listening on camera topic: {self.camera_topic}..."
        )
 
    def build_engine(self):
    #Reading the onnx file in the perception models directory
        self.max_batch_size = 6
        self.channels = 3
        self.height = 640
        self.width = 640
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

    def initialize_engine(self, weight):
        self.weight = Path(weight) if isinstance(weight, str) else weight
        self.logger  = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, namespace ='')
        with trt.Runtime(self.logger) as runtime:
            self.tensorRT_model = runtime.deserialize_cuda_engine(self.weight.read_bytes())
            self.get_logger().info(f"TENSORRT Model:{self.weight}")
        self.execution_context = self.tensorRT_model.create_execution_context()  
        
          
        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.input_tensor_name = self.tensorRT_model.get_tensor_name(0)
        self.execution_context.set_input_shape(self.input_tensor_name, [6, 3, 1080, 1920])
        self.inputShape = self.execution_context.get_tensor_shape(self.input_tensor_name)
        self.get_logger().info(f"INPUT SHAPE:{self.inputShape}")
        self.names = [self.tensorRT_model.get_tensor_name(i) for i in range(self.num_io_tensors)]
        self.num_io_tensors = self.tensorRT_model.num_io_tensors
        self.bindings: List[int] = [0] * self.num_io_tensors
        self.num_inputs = 0
        self.num_outputs  = 0
        self.input_names = []
        self.output_names = []

        for i in range(self.num_io_tensors):
              self.tensor_name  = self.tensorRT_model.get_tensor_name(i)
              if self.tensorRT_model.get_tensor_mode(self.tensor_name) == trt.TensorIOMode.INPUT:
                self.input_names.append(self.tensor_name)
                self.num_inputs += 1
              else:
                self.output_names.append(self.tensor_name)
                self.num_outputs += 1           
  
    def batch_callback(self,msg1,msg2,msg3, msg4,msg5,msg6):
        image_list =  [msg1,msg2,msg3,msg4,msg5,msg6]
        batched_list = []
        for msg in image_list:  
            # Need to convert to numpy array and then make it have 3 color channels 
            numpy_array = np.frombuffer(msg.data, np.uint8)
            compressedImage  = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
            resized_compressedImage = cv2.resize(compressedImage,(1920, 1080))
            preprocessedImage = self.batch_convert_to_tensor(resized_compressedImage)
            batched_list.append(preprocessedImage)
            # self.get_logger().info(f"Resized Image Shape:{resized_compressedImage.shape}")a
           
        
        batch_tensor = torch.stack(batched_list)
        self.get_logger().info(f"batch tensor shape: {batch_tensor.shape}")

        #Now need to pass it to yolov8 tensorRT model
        # predictions = self.model(batch_tensor)
       
        

    #Converting to tensor for batching
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
