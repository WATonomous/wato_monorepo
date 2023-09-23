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