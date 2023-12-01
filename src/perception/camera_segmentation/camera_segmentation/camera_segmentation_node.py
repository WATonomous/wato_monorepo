import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

import detectron2
from detectron2.utils.logger import setup_logger

# Import libraries
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import imutils
import os
import subprocess
from huggingface_hub import hf_hub_download

# Import detectron2 utilities
from detectron2.config import get_cfg
from detectron2.projects.deeplab import add_deeplab_config
from detectron2.data import MetadataCatalog
from .demo.defaults import DefaultPredictor
from .demo.visualizer import Visualizer, ColorMode

from .config import (
    add_oneformer_config,
    add_common_config,
    add_swin_config,
    add_dinat_config,
    add_convnext_config,
)

# Node initialization; setting up ROS2 parameters and the oneformer model
class CameraSegmentationNode(Node):

    def __init__(self):
        super().__init__('node')
        # log initialization
        self.get_logger().info('Initializing node...')

        # Fetch parameters from yaml file
        self.declare_parameter('camera_topic', '/CAM_FRONT/image_rect_compressed')
        self.declare_parameter('publish_topic', '/camera_segmentation')
        # camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_topic = '/CAM_FRONT/image_rect_compressed'
        self.compressed = True
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value


        # Create ROS2 publisher and subscriber
        self.publisher_ = self.create_publisher(Image if not self.compressed else CompressedImage, publish_topic, 10)
        self.subscriber_ = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.i = 0

        torch.zeros(1).cuda()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            self.get_logger().info("Using GPU for inference")
        else:
            self.get_logger().info("Using CPU for inference")

        self.SWIN_CFG_DICT = {"cityscapes": "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/configs/cityscapes/oneformer_swin_large_IN21k_384_bs16_90k.yaml",}

        self.DINAT_CFG_DICT = {"cityscapes": "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/configs/cityscapes/oneformer_dinat_large_bs16_90k.yaml",}

        self.TASK_INFER = {"panoptic": self.panoptic_run, 
            "instance": self.instance_run, 
            "semantic": self.semantic_run}
        
        self.predictor, self.metadata = self.setup_modules("cityscapes", "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/models/250_16_dinat_l_oneformer_cityscapes_90k.pth", False)
        self.cv_bridge = CvBridge()

        print("Done initializing node")
        
    def image_callback(self, msg):
        self.get_logger().info('Got callback')
        
        # Convert ROS2 image into cv2
        if self.compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough")
            except CvBridgeError as e:
                self.get_logger().error(str(e))
                return
        
        # Resize cv2 image for oneformer
        cv_image = imutils.resize(cv_image, width=512)

        # Do inference on the image using the model
        out = self.TASK_INFER["panoptic"](cv_image, self.predictor, self.metadata).get_image()

        # Convert cv2 image back to a ROS2 image
        img_msg = self.cv_bridge.cv2_to_imgmsg(out, "bgr8")
        self.get_logger().info("Ran inference")
        # Publish ROS2 image
        self.publisher_.publish(img_msg)
        self.get_logger().info('Publishing image: "%s"' % msg.data)
        self.i += 1

    def setup_cfg(self, dataset, model_path, use_swin):
        # load config from file and command-line arguments
        cfg = get_cfg()
        add_deeplab_config(cfg)
        add_common_config(cfg)
        add_swin_config(cfg)
        add_dinat_config(cfg)
        add_convnext_config(cfg)
        add_oneformer_config(cfg)
        if use_swin:
            cfg_path = self.SWIN_CFG_DICT[dataset]
        else:
            cfg_path = self.DINAT_CFG_DICT[dataset]
            cfg.merge_from_file(cfg_path)
            if torch.cuda.is_available():
                cfg.MODEL.DEVICE = 'cuda'
            else:
                cfg.MODEL.DEVICE = 'cpu'
            cfg.MODEL.WEIGHTS = model_path
            cfg.freeze()
        return cfg
    
    def setup_modules(self, dataset, model_path, use_swin):
        cfg = self.setup_cfg(dataset, model_path, use_swin)
        predictor = DefaultPredictor(cfg)
        metadata = MetadataCatalog.get(
            cfg.DATASETS.TEST_PANOPTIC[0] if len(cfg.DATASETS.TEST_PANOPTIC) else "__unused"
        )
        if 'cityscapes_fine_sem_seg_val' in cfg.DATASETS.TEST_PANOPTIC[0]:
            from cityscapesscripts.helpers.labels import labels
            stuff_colors = [k.color for k in labels if k.trainId != 255]
            metadata = metadata.set(stuff_colors=stuff_colors)
        
        return predictor, metadata
    
    def panoptic_run(self, img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "panoptic")
        panoptic_seg, segments_info = predictions["panoptic_seg"]
        out = visualizer.draw_panoptic_seg_predictions(
        panoptic_seg.to(self.device), segments_info, alpha=0.5
    )
        return out

    def instance_run(self, img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "instance")
        instances = predictions["instances"].to(self.device)
        out = visualizer.draw_instance_predictions(predictions=instances, alpha=0.5)
        return out
        
    def semantic_run(self, img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "semantic")
        out = visualizer.draw_sem_seg(
            predictions["sem_seg"].argmax(dim=0).to(self.device), alpha=0.5
        )
        return out

def main(args=None):
    setup_logger()
    setup_logger(name="oneformer")

    rclpy.init(args=args)
    node = CameraSegmentationNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()