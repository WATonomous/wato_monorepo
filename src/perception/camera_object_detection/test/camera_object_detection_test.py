import os
import unittest
import rclpy
from rcl_interfaces.msg import Log
import launch_testing
import launch
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import pytest
from sensor_msgs.msg import Image
from camera_object_detection.yolov8_detection import CameraDetectionNode

@pytest.mark.launch_test
def generate_test_description():
    # Get launch file and set up for starting the test
    pkg_share = get_package_share_directory('camera_object_detection')
    launch_dir = os.path.join(pkg_share, 'launch')
    camera_detection_launch = os.path.join(launch_dir, 'camera_object_detection_test.launch.py')

    return launch.LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', camera_detection_launch],
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
        # Delay the test until the ROS nodes are up and running
        launch_testing.actions.ReadyToTest()
    ])


class TestCameraObjectDetectionLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = CameraDetectionNode() # create a CameraDetectionNode to test
        cls.logs_received = []
        cls.images_received = []

        # Subscribe to /rosout where get_logger messages are outputted
        cls.subscription = cls.node.create_subscription(
            Log,
            '/rosout',
            cls.log_callback,
            10
        )

        # Subscribe to the /annotated_img publisher of CameraDetectionNode node
        cls.subscription_images = cls.node.create_subscription(
            Image,
            '/annotated_img',
            cls.image_callback,
            10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def log_callback(cls, msg: Log):
        # Append the log message to the class's list of received messages
        cls.logs_received.append(msg)

    @classmethod
    def image_callback(cls, msg):
        cls.images_received.append(msg)

    def test_camera_detection_node_logging(self):
        # create a dummy image to test
        dummy_image = Image()
        dummy_image.height = 480
        dummy_image.width = 640
        dummy_image.encoding = 'rgb8'
        dummy_image.step = dummy_image.width * 3
        dummy_image.data = [255] * (dummy_image.step * dummy_image.height)
        dummy_image.header.stamp = self.node.get_clock().now().to_msg()

        # Test the image_callback function of the CameraDetectionNode node
        self.node.image_callback(dummy_image)
        
        # Spin some time to collect logs
        end_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=10)
        while rclpy.ok() and self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check if the expected log messages were received
        expected_log_messages = ["Successfully created node listening on camera topic:", "Finished in:"] # From CameraDetectionNode logger
        for expected_msg in expected_log_messages:
            message_logged = any(
                expected_msg in log.msg and log.name == "camera_object_detection_node"
                for log in self.logs_received
            )
            self.assertTrue(message_logged, f"Expected log message '{expected_msg}' not found in the logs.")

        self.assertTrue(len(self.images_received) > 0, "No visualization images were published.") # Check that given an image, the node is publishing

if __name__ == '__main__':
    unittest.main()
