import numpy as np
import cv2
import os
from mmseg.apis import MMSegInferencer
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import logging


class SemanticSegmentation(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_node')
        self.declare_parameter('pub_image', True)
        self.declare_parameter('pub_masks', True)
        self.declare_parameter('compressed', True)
        self.declare_parameter('config', "model/segformer_mit-b2_8xb1-160k_cityscapes-1024x1024.py")
        self.declare_parameter('resource_path', "")
        self.declare_parameter(
            'checkpoint', "model/segformer_mit-b2_8x1_1024x1024_160k_cityscapes_20211207_134205-6096669a.pth")
        self.declare_parameter('MODEL_IMAGE_H', 1024)
        self.declare_parameter('MODEL_IMAGE_W', 1024)

        self.config = os.path.join(self.get_parameter(
            'resource_path').value, self.get_parameter('config').value)
        self.checkpoint = os.path.join(self.get_parameter(
            'resource_path').value, self.get_parameter('checkpoint').value)
        self.compressed = self.get_parameter('compressed').value
        self.modelH = self.get_parameter('MODEL_IMAGE_H').value
        self.modelW = self.get_parameter('MODEL_IMAGE_W').value
        
        self.image_subscription = self.create_subscription(
            Image if not self.compressed else CompressedImage,
            "/CAM_FRONT/image_rect_compressed",
            self.listener_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=0,
            ),
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/left/segmentations',
            10
        )
        # self.palette = np.array(self.palette, dtype=np.uint8)
        self.model = MMSegInferencer(self.config, self.checkpoint,
                                     dataset_name="cityscapes", device='cuda:0')
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        images = [msg]  # msg is a single sensor image
        for image in images:
            # convert ros Image to cv::Mat
            if self.compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                image = cv2.resize(cv_image, (self.modelW, self.modelH))
            else:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                    image = cv2.resize(cv_image, (self.modelW, self.modelH))
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
                    return
        with torch.no_grad():
            out_img = self.model(image, show=False)['predictions']

        out_array = np.array(out_img, np.uint8)
        mask_output = self.bridge.cv2_to_imgmsg(out_array)
        self.image_publisher.publish(mask_output)


def main(args=None):

    rclpy.init(args=args)
    semantic_segmentation_node = SemanticSegmentation()

    rclpy.spin(semantic_segmentation_node)

    semantic_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
