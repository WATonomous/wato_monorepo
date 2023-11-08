import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

class CameraSegmentationNode(Node):

    def __init__(self):
        super().__init__('node')
        # log initialization
        self.get_logger().info('Initializing node...')
        # Fetch parameters from yaml file
        self.declare_parameter('camera_topic', '/camera_topic')
        self.declare_parameter('publish_topic', '/camera_segmentation')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Image, publish_topic, 10)
        self.subscriber_ = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.i = 0

    def image_callback(self):
        msg = Image()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    print("Hello World")

    import detectron2
    from detectron2.utils.logger import setup_logger
    setup_logger()
    setup_logger(name="oneformer")

    # Import libraries
    import numpy as np
    import cv2
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

    torch.zeros(1).cuda()
    cpu_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    SWIN_CFG_DICT = {"cityscapes": "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/configs/cityscapes/oneformer_swin_large_IN21k_384_bs16_90k.yaml",}

    DINAT_CFG_DICT = {"cityscapes": "/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/configs/cityscapes/oneformer_dinat_large_bs16_90k.yaml",}

    def setup_cfg(dataset, model_path, use_swin):
        # load config from file and command-line arguments
        cfg = get_cfg()
        add_deeplab_config(cfg)
        add_common_config(cfg)
        add_swin_config(cfg)
        add_dinat_config(cfg)
        add_convnext_config(cfg)
        add_oneformer_config(cfg)
        if use_swin:
            cfg_path = SWIN_CFG_DICT[dataset]
        else:
            cfg_path = DINAT_CFG_DICT[dataset]
            cfg.merge_from_file(cfg_path)
            cfg.MODEL.DEVICE = 'cuda'
            cfg.MODEL.WEIGHTS = model_path
            cfg.freeze()
        return cfg

    def setup_modules(dataset, model_path, use_swin):
        cfg = setup_cfg(dataset, model_path, use_swin)
        predictor = DefaultPredictor(cfg)
        metadata = MetadataCatalog.get(
            cfg.DATASETS.TEST_PANOPTIC[0] if len(cfg.DATASETS.TEST_PANOPTIC) else "__unused"
        )
        if 'cityscapes_fine_sem_seg_val' in cfg.DATASETS.TEST_PANOPTIC[0]:
            from cityscapesscripts.helpers.labels import labels
            stuff_colors = [k.color for k in labels if k.trainId != 255]
            metadata = metadata.set(stuff_colors=stuff_colors)
        
        return predictor, metadata
    
    def panoptic_run(img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "panoptic")
        panoptic_seg, segments_info = predictions["panoptic_seg"]
        out = visualizer.draw_panoptic_seg_predictions(
        panoptic_seg.to(cpu_device), segments_info, alpha=0.5
    )
        return out

    def instance_run(img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "instance")
        instances = predictions["instances"].to(cpu_device)
        out = visualizer.draw_instance_predictions(predictions=instances, alpha=0.5)
        return out

    def semantic_run(img, predictor, metadata):
        visualizer = Visualizer(img[:, :, ::-1], metadata=metadata, instance_mode=ColorMode.IMAGE)
        predictions = predictor(img, "semantic")
        out = visualizer.draw_sem_seg(
            predictions["sem_seg"].argmax(dim=0).to(cpu_device), alpha=0.5
        )
        return out

    TASK_INFER = {"panoptic": panoptic_run, 
                "instance": instance_run, 
                "semantic": semantic_run}

    predictor, metadata = setup_modules("cityscapes", "250_16_dinat_l_oneformer_cityscapes_90k.pth", False)

    img = cv2.imread("/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/samples/test.png")
    img = imutils.resize(img, width=512)
    task = "panoptic"
    out = TASK_INFER[task](img, predictor, metadata).get_image()
    cv2.imwrite("/home/docker/ament_ws/src/camera_segmentation/camera_segmentation/samples/test_results.png", out)

    print("At the end")

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