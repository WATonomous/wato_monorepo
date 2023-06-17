import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from yolov5.models.common import DetectMultiBackend
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import check_img_size, scale_segments, non_max_suppression
from utils.plots import Annotator, colors

import os
from common_msgs.msg import Obstacle, ObstacleList

from easydict import EasyDict

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import torch

class CameraDetectionNode(Node):

    def __init__(self):
        super().__init__('minimal_param_node')
        
        self.declare_parameter('camera_topic', '/camera/right/image_color')
        self.declare_parameter('publish_vis_topic', '/annotated_img')
        self.declare_parameter('publish_obstacle_topic', '/obstacles')
        self.declare_parameter('model_path', '/perception_models/yolov5s.pt')
        self.declare_parameter('image_size', 480)
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_vis_topic = self.get_parameter('publish_vis_topic').value
        self.publish_obstacle_topic = self.get_parameter('publish_obstacle_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.image_size = self.get_parameter('image_size').value

        self.line_thickness = 1
        self.half = False
        self.augment = False

        super().__init__('camera_detection_node')
        self.get_logger().info("Creating node...")
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            10)

        # set device
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        # CV bridge
        self.cv_bridge = CvBridge()

        # load YOLOv5 model
        self.model = DetectMultiBackend(
            self.model_path, device=self.device, dnn=False, fp16=False
        )

        self.names = (
            self.model.module.names
            if hasattr(self.model, "module")
            else self.model.names
        )

        self.stride = int(self.model.stride)

        # setup vis publishers
        self.vis_publisher = self.create_publisher(Image, self.publish_vis_topic, 10)
        self.obstacle_publisher = self.create_publisher(ObstacleList, self.publish_obstacle_topic, 10)

        self.get_logger().info(f"Successfully created node listening on camera topic: {self.camera_topic}...")

    def preprocess_image(self, cv_image):
        """
        Preprocess the image by resizing, padding and rearranging the dimensions
        Parameters: 
            cv_image: A numpy or cv2 image of shape (w,h,3)
        Returns:
            torch.Tensor image for model input of shape (1,3,w,h)
        """
        # Padded resize
        img = cv_image
        img = letterbox(cv_image, self.image_size, stride=self.stride)[0]

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
        Post-process draws bouningboxes on camera image

        Parameters: 
            detections: A list of dict with the format 
                {
                    "label": str,
                    "bbox": [float],
                    "conf": float
                }
            annotator: A yolov5.utils.general.Annotator for the current image
        Returns:
            processed_detections: filtered detections
            annotator_img: image with bounding boxes drawn on
        """
        processed_detections = []

        for det in detections:
            label = f'{det["label"]} {det["conf"]:.2f}'
            x1, y1, w1, h1 = det["bbox"]
            xyxy = [x1, y1, x1 + w1, y1 + h1]
            annotator.box_label(xyxy, label, color=colors(1, True))

        annotator_img = annotator.result()
        return (processed_detections, annotator_img)

    def publish_vis(self, annotated_img, feed):
        # Publish visualizations
        imgmsg = self.cv_bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        imgmsg.header.frame_id = "camera_{}_link".format(feed)
        self.vis_publisher.publish(imgmsg)

        # TODO: Support reading stereo (left and right images)
        # if feed == "left":
        #     self.left_vis_publisher.publish(imgmsg)
        # elif feed == "right":
        #     self.right_vis_publisher.publish(imgmsg)

    def publish_detections(self, detections, msg, feed):
        # Publish detections to an ObstacleList message

        obstacle_list = ObstacleList()

        # fill header for obstacle list
        obstacle_list.header.frame_id = msg.header.frame_id

        # populate obstacle list
        if detections is not None and len(detections):
            for detection in detections:
                obstacle = Obstacle()

                # fill header for obstacle
                obstacle.header.frame_id = msg.header.frame_id

                # populate obstacle
                obstacle.confidence = detection["conf"]
                obstacle.label = self.obstacle_dict[detection["label"]]
                obstacle.pose.pose.position.x = detection["bbox"][0]
                obstacle.pose.pose.position.y = detection["bbox"][1]
                obstacle.width_along_x_axis = detection["bbox"][2]
                obstacle.height_along_y_axis = detection["bbox"][3]

                # append obstacle to obstacle list
                obstacle_list.obstacles.append(obstacle)

        self.obstacle_publisher.publish(obstacle_list)
        return

    def listener_callback(self, msg):

        self.get_logger().info('Received image')
        images = [msg]  # msg is a single sensor image
        for image in images:

            # convert ros Image to cv::Mat
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(
                    image, desired_encoding="passthrough")
            except CvBridgeError as e:
                self.get_logger().error(str(e))
                return

            # preprocess image and run through prediction
            img = self.preprocess_image(cv_image)
            processed_cv_image = letterbox(
                cv_image, self.image_size, stride=self.stride)[0]
            pred = self.model(img)

            # nms function used same as yolov5 detect.py
            pred = non_max_suppression(pred)
            detections = []
            for i, det in enumerate(pred):  # per image
                if len(det):
                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)
                        label = self.names[int(cls)]

                        bbox = [xyxy[0], xyxy[1], xyxy[2] -
                                xyxy[0], xyxy[3] - xyxy[1]]
                        bbox = [b.item() for b in bbox]

                        detections.append(
                            {
                                "label": label,
                                "conf": conf.item(),
                                "bbox": bbox,
                            }
                        )
                        self.get_logger().info(f"{label}: {bbox}")

            annotator = Annotator(
                processed_cv_image, line_width=self.line_thickness, example=str(
                    self.names)
            )
            detections, annotated_img = self.postprocess_detections(
                detections, annotator)

            feed = ""  # Currently we support a single camera so we pass an empty string
            self.publish_vis(annotated_img, feed)
            self.publish_detections(detections, msg, feed)


def main(args=None):
    rclpy.init(args=args)

    camera_detection_node = CameraDetectionNode()
    rclpy.spin(camera_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
