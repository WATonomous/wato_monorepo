#!/usr/bin/env python3
"""
Test publisher node for anomaly_detection package.
Publishes a single image on the camera topic to trigger AnomalyProcessor.camera_callback.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class TestImagePublisher(Node):
    def __init__(self, image_path: str, topic: str = '/camera/right/image_color'):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        # Load the test image once
        if not os.path.exists(image_path):
            self.get_logger().error(f"Test image not found: {image_path}")
            rclpy.shutdown()
            return
        self.image = cv2.imread(image_path)
        if self.image is None:
            self.get_logger().error(f"Failed to load image at {image_path}")
            rclpy.shutdown()
            return

        # Publish immediately and then shutdown
        self.publish_and_exit()

    def publish_and_exit(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info('Published test image, shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    import sys
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} /path/to/test_image.jpg")
        return
    image_path = sys.argv[1]

    rclpy.init(args=args)
    node = TestImagePublisher(image_path)

if __name__ == '__main__':
    main()
