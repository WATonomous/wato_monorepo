import rclpy
from rclpy.node import Node
import os

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    ObjectHypothesisWithPose,
    Detection2D,
    Detection2DArray,
)

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


class CameraDetectionNode(Node):

    def __init__(self):
        torch.zeros(1).cuda()

        super().__init__("camera_object_detection_node")
        self.get_logger().info("Creating camera detection node...")

        self.declare_parameter("camera_topic", "/camera/right/image_color")
        self.declare_parameter("publish_vis_topic", "/annotated_img")
        self.declare_parameter("publish_detection_topic", "/detections")
        self.declare_parameter("model_path", "/perception_models/yolov8s.pt")
        self.declare_parameter("image_size", 1024)
        self.declare_parameter("compressed", False)
        self.declare_parameter("crop_mode", "LetterBox")
        self.declare_parameter("save_detections", False)

        self.camera_topic = self.get_parameter("camera_topic").value
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
        self.orig_image_width = None
        self.orig_image_height = None

        self.orig_image_width = None
        self.orig_image_height = None

        # set device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        # CV bridge
        self.cv_bridge = CvBridge()

        # load yolov8 model
        self.model = AutoBackend(self.model_path, device=self.device, dnn=False, fp16=False)

        self.names = self.model.module.names if hasattr(self.model, "module") else self.model.names

        self.stride = int(self.model.stride)

        # setup vis publishers
        self.vis_publisher = self.create_publisher(Image, self.publish_vis_topic, 10)
        self.detection_publisher = self.create_publisher(
            Detection2DArray, self.publish_detection_topic, 10
        )

        self.get_logger().info(
            f"Successfully created node listening on camera topic: {self.camera_topic}..."
        )

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


def main(args=None):
    rclpy.init(args=args)

    camera_object_detection_node = CameraDetectionNode()
    rclpy.spin(camera_object_detection_node)
    camera_object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
