import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2 world! %d' % self.count
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.count += 1

def main():
    rclpy.init()
    test_node = TestNode()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
