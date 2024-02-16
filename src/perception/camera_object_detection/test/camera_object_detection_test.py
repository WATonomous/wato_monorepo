import unittest
from sensor_msgs.msg import Image
import time
from camera_object_detection.yolov8_detection import CameraDetectionNode
import rclpy

class TestCameraDetectionNode(unittest.TestCase):

    def test_image_processing(self):
        rclpy.init()

        node = CameraDetectionNode()

        # Create a dummy image message
        dummy_image = Image()
        dummy_image.height = 480
        dummy_image.width = 640
        dummy_image.encoding = 'rgb8'
        dummy_image.step = dummy_image.width * 3
        dummy_image.data = [255] * (dummy_image.step * dummy_image.height)

        node.image_callback(dummy_image)
        # Delay to allow the callback to be called
        time.sleep(1)
        # Check if the publish method was called with a bounding box
        # node.create_publisher().publish.assert_called()
        node.destroy_node()
        rclpy.shutdown()

    def listener_callback(self, msg):
        self.msg_received = True
        self.received_msg = msg


if __name__ == '__main__':
    unittest.main()
