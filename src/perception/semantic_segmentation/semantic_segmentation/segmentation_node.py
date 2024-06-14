import numpy as np
import cv2
import os
from mmseg.apis import MMSegInferencer
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from rich.logging import RichHandler
import logging
from ament_index_python.packages import get_package_share_directory
# Configure the root logger
logging.basicConfig(level=logging.INFO, handlers=[RichHandler(level=logging.WARNING)])


# ALGO_VERSION = os.getenv("MODEL_NAME")

# if not ALGO_VERSION:
#     ALGO_VERSION = 'nvidia/segformer-b2-finetuned-ade-512-512'

package_name = 'semantic_segmentation'
package_share_directory = get_package_share_directory(package_name)
CONFIG = os.path.join(package_share_directory, 'resource', 'model',
                    'segformer_mit-b2_8xb1-160k_cityscapes-1024x1024.py')
CHECKPOINT = os.path.join(package_share_directory, 'resource', 'model',
                'segformer_mit-b2_8x1_1024x1024_160k_cityscapes_20211207_134205-6096669a.pth')
IMAGE_H = 900
IMAGE_W = 1600

# Adjust logging
logging.getLogger('mmseg.apis').setLevel(logging.WARNING)
logging.getLogger('mmengine').setLevel(logging.WARNING)

"""Cityscapes palette that maps each class to RGB values."""
COLOR_PALLETE = [
    [128, 64, 128],  # road
    [244, 35, 232],  # sidewalk
    [70, 70, 70],    # building
    [102, 102, 156],  # wall
    [190, 153, 153],  # fence
    [153, 153, 153],  # pole
    [250, 170, 30],  # traffic light
    [220, 220, 0],   # traffic sign
    [107, 142, 35],  # vegetation
    [152, 251, 152],  # terrain
    [0, 130, 180],   # sky
    [220, 20, 60],   # person
    [255, 0, 0],     # rider
    [0, 0, 142],     # car
    [0, 0, 70],      # truck
    [0, 60, 100],    # bus
    [0, 80, 100],    # train
    [0, 0, 230],     # motorcycle
    [119, 11, 32],   # bicycle
    [0, 0, 0]        # ignore/unlabeled
]


class SemanticSegmentation(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_node')
        self.declare_parameter('pub_image', True)
        self.declare_parameter('pub_masks', True)
        self.declare_parameter('compressed', True)
        self.declare_parameter('config', "model/segformer_mit-b2_8xb1-160k_cityscapes-1024x1024.py")
        self.declare_parameter(
            'checkpoint', "model/segformer_mit-b2_8x1_1024x1024_160k_cityscapes_20211207_134205-6096669a.pth")
        
        self.compressed = self.get_parameter('compressed').value
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
        self.palette = np.array(COLOR_PALLETE, dtype=np.uint8)
        self.model = MMSegInferencer(CONFIG, CHECKPOINT, dataset_name="cityscapes", device='cuda:0')
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        images = [msg]  # msg is a single sensor image
        for image in images:
            # convert ros Image to cv::Mat
            if self.compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                image = cv2.resize(cv_image, (1024, 1024))
            else:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                    image = cv2.resize(cv_image, (1024, 1024))
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
                    return
        with torch.no_grad():
            out_img = self.model(image, show=False, )['predictions']   
        # logits = torch.tensor(
        # out_img, dtype=torch.float32).unsqueeze(0).unsqueeze(0)
        # Add batch and channel dimensions
        # upsampled_logits = torch.nn.functional.interpolate(logits,
        #                    size=(IMAGE_H, IMAGE_W),  # (height, width)
        #                                                    mode='bilinear',
        #                                                    align_corners=False)
        # upsampled_logits = upsampled_logits.squeeze().numpy().astype(np.uint8)

        #     color_seg[out_img == label, :] = color
        color_seg = self.palette[out_img]
        # img = np_image * 0.5 + color_seg * 0.5
        # img_output = bridge.cv2_to_imgmsg(img)
        # print(f'Publishing Segmentation')
        # self.image_publisher.publish(img_output)
        color_seg = cv2.resize(color_seg, (IMAGE_W, IMAGE_H))
        mask_output = self.bridge.cv2_to_imgmsg(color_seg)
        self.image_publisher.publish(mask_output)



def main(args=None):

    rclpy.init(args=args)
    semantic_segmentation_node = SemanticSegmentation()

    rclpy.spin(semantic_segmentation_node)

    semantic_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
