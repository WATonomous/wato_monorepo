#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from common_msgs.msg import Object  

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Object, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Object()                                         
        msg.id = self.i                                     
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.id) 
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()