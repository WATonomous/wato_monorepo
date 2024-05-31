import numpy as np
import cv2
import os
from transformers import AutoFeatureExtractor, SegformerForSemanticSegmentation
import torch
from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ALGO_VERSION = os.getenv("MODEL_NAME")

if not ALGO_VERSION:
    ALGO_VERSION = 'nvidia/segformer-b2-finetuned-ade-1024-1024'


def ade_palette():
    """ADE20K palette that maps each class to RGB values."""
    return [[120, 120, 120], [180, 120, 120], [6, 230, 230], [80, 50, 50],
            [4, 200, 3], [120, 120, 80], [140, 140, 140], [204, 5, 255],
            [230, 230, 230], [4, 250, 7], [224, 5, 255], [235, 255, 7],
            [150, 5, 61], [120, 120, 70], [8, 255, 51], [255, 6, 82],
            [143, 255, 140], [204, 255, 4], [255, 51, 7], [204, 70, 3],
            [0, 102, 200], [61, 230, 250], [255, 6, 51], [11, 102, 255],
            [255, 7, 71], [255, 9, 224], [9, 7, 230], [220, 220, 220],
            [255, 9, 92], [112, 9, 255], [8, 255, 214], [7, 255, 224],
            [255, 184, 6], [10, 255, 71], [255, 41, 10], [7, 255, 255],
            [224, 255, 8], [102, 8, 255], [255, 61, 6], [255, 194, 7],
            [255, 122, 8], [0, 255, 20], [255, 8, 41], [255, 5, 153],
            [6, 51, 255], [235, 12, 255], [160, 150, 20], [0, 163, 255],
            [140, 140, 140], [250, 10, 15], [20, 255, 0], [31, 255, 0],
            [255, 31, 0], [255, 224, 0], [153, 255, 0], [0, 0, 255],
            [255, 71, 0], [0, 235, 255], [0, 173, 255], [31, 0, 255],
            [11, 200, 200], [255, 82, 0], [0, 255, 245], [0, 61, 255],
            [0, 255, 112], [0, 255, 133], [255, 0, 0], [255, 163, 0],
            [255, 102, 0], [194, 255, 0], [0, 143, 255], [51, 255, 0],
            [0, 82, 255], [0, 255, 41], [0, 255, 173], [10, 0, 255],
            [173, 255, 0], [0, 255, 153], [255, 92, 0], [255, 0, 255],
            [255, 0, 245], [255, 0, 102], [255, 173, 0], [255, 0, 20],
            [255, 184, 184], [0, 31, 255], [0, 255, 61], [0, 71, 255],
            [255, 0, 204], [0, 255, 194], [0, 255, 82], [0, 10, 255],
            [0, 112, 255], [51, 0, 255], [0, 194, 255], [0, 122, 255],
            [0, 255, 163], [255, 153, 0], [0, 255, 10], [255, 112, 0],
            [143, 255, 0], [82, 0, 255], [163, 255, 0], [255, 235, 0],
            [8, 184, 170], [133, 0, 255], [0, 255, 92], [184, 0, 255],
            [255, 0, 31], [0, 184, 255], [0, 214, 255], [255, 0, 112],
            [92, 255, 0], [0, 224, 255], [112, 224, 255], [70, 184, 160],
            [163, 0, 255], [153, 0, 255], [71, 255, 0], [255, 0, 163],
            [255, 204, 0], [255, 0, 143], [0, 255, 235], [133, 255, 0],
            [255, 0, 235], [245, 0, 255], [255, 0, 122], [255, 245, 0],
            [10, 190, 212], [214, 255, 0], [0, 204, 255], [20, 0, 255],
            [255, 255, 0], [0, 153, 255], [0, 41, 255], [0, 255, 204],
            [41, 0, 255], [41, 255, 0], [173, 0, 255], [0, 245, 255],
            [71, 0, 255], [122, 0, 255], [0, 255, 184], [0, 92, 255],
            [184, 255, 0], [0, 133, 255], [255, 214, 0], [25, 194, 194],
            [102, 255, 0], [92, 0, 255]]


def predict(image: Image):
    feature_extractor = AutoFeatureExtractor.from_pretrained(ALGO_VERSION)
    model = SegformerForSemanticSegmentation.from_pretrained(ALGO_VERSION)

    inputs = feature_extractor(image, return_tensors="pt")

    with torch.no_grad():
        output = model(**inputs)
    
    labels = model.config.id2labels
    return labels, output.logits


class SemanticSegmentation(Node):
    def __init__(self):
        super().__init__('semantic_segmentation_node')
        self.declare_parameter('pub_image', True)
        self.declare_parameter('pub_pixels', True)
        self.declare_parameter('pub_detections', True)
        self.declare_parameter('pub_masks', True)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/right/image_color',
            self.listener_callback,
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/segformer_seg/image',
            1
        )
    
        self.pixels_publisher = self.create_publisher(
            Image,
            '/segformer_seg/pixels',
            1
        )

        self.detection_publisher = self.create_publisher(
            String,
            '/segformer_seg/detections',
            1
        )

        self.mask_publisher = self.create_publisher(
            Image,
            '/segformer_seg/mask',
            1
        )

    def listener_callback(self, msg: Image):
        bridge = CvBridge()
        cv_image: np.ndarray = bridge.imgmsg_to_cv2(msg)
        np_image = cv_image.astype(np.uint8)
        converted_image = PilImage.fromarray(np_image, 'RGB')
        labels, logits = predict(converted_image)
        print(f'Predicted Segmentation')
        w, h = converted_image.size

        upsampled_logits = torch.nn.functional.interpolate(logits,
                                                           size=(h, w),  # (height, width)
                                                           mode='bilinear',
                                                           align_corners=False)

        # Second, apply argmax on the class dimension
        seg = upsampled_logits.argmax(dim=1)[0]
        np_seg = seg.numpy().astype(np.uint8)
        color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8)  # height, width, 3
        palette = np.array(ade_palette())
        for label, color in enumerate(palette):
            color_seg[seg == label, :] = color

        # Convert to BGR
        color_seg = color_seg[..., ::-1]

        if self.get_parameter('pub_pixels').value:
            pixel_output = bridge.cv2_to_imgmsg(np_seg, encoding="mono8")
            self.pixels_publisher.publish(pixel_output)

        if self.get_parameter('pub_image').value:
            img = np.uint8(cv_image) * 0.5 + color_seg * 0.5
            img_output = bridge.cv2_to_imgmsg(img)
            self.image_publisher.publish(img_output)

        if self.get_parameter('pub_masks').value:
            mask_output = bridge.cv2_to_imgmsg(color_seg)
            self.mask_publisher.publish(mask_output)

        if self.get_parameter('pub_detections').value:
            classes = torch.unique(seg).tolist()
            results = []
            for label in classes:
                results.append(labels[label])

            msg = String()
            msg.data = ' '.join(results)
            self.detection_publisher.publish(msg)

        


def main(args=None):
    print('segformer_seg Started')

    rclpy.init(args=args)

    semantic_segmentation_node = SemanticSegmentation()

    rclpy.spin(semantic_segmentation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    semantic_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()