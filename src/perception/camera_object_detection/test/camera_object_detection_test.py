import unittest
import numpy
from unittest.mock import patch, MagicMock
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera_object_detection.yolov8_detection import CameraDetectionNode

class TestCameraDetectionNode(unittest.TestCase):

  @patch.object(Node, '__init__')
  @patch('rclpy.node.Node.get_logger')
  def test_image_processing(self,
                            get_logger_mock,
                            node_init_mock):
      node_init_mock.return_value = None  # Mock the Node's __init__ method


      node = CameraDetectionNode()
      # Mock the publish method to capture its input
      node.create_publisher = MagicMock()
      node.create_publisher().publish = MagicMock()

      # Create a dummy image message
      dummy_image = Image()
      dummy_image.height = 480
      dummy_image.width = 640
      dummy_image.encoding = 'rgb8'
      dummy_image.step = dummy_image.width * 3
      dummy_image.data = [255] * (dummy_image.step * dummy_image.height)

      node.image_callback(dummy_image)
      
      # Check if the publish method was called with a bounding box
      node.create_publisher().publish.assert_called() # changed publish to publish_detections !

if __name__ == '__main__':
    unittest.main()

