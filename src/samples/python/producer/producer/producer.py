import math
import time

import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered
from rcl_interfaces.msg import ParameterType, ParameterDescriptor


class Producer(Node):

    def __init__(self):
        super().__init__('python_producer')
        # Declare and get the parameters
        self.declare_parameter('pos_x', 0.0),
        self.declare_parameter('pos_y', 0.0),
        self.declare_parameter('pos_z', 0.0),
        self.declare_parameter('velocity', 0.0)

        # For parameters, we need to explicitely declare its type for Python to know
        # what to do with it
        self.__pos_x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.__pos_y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.__pos_z = self.get_parameter('pos_z').get_parameter_value().double_value

        self.__velocity = self.get_parameter('velocity').get_parameter_value().double_value

        # Initialize ROS2 constructs
        queue_size = 10
        self.publisher_ = self.create_publisher(Unfiltered, '/unfiltered_topic', queue_size)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.__publish_position)
        
    def update_position(self):
        self.__pos_x += self.__velocity / math.sqrt(3)
        self.__pos_y += self.__velocity / math.sqrt(3)
        self.__pos_z += self.__velocity / math.sqrt(3)

    def serialize_data(self):
        return "x:" + str(self.__pos_x) + ";y:" + str(self.__pos_y) + ";z:" + str(self.__pos_z) + ";"

    def __publish_position(self):
        self.update_position()
        msg = Unfiltered()

        msg.data = self.serialize_data()
        msg.valid = True
        msg.timestamp = int(time.time() * 1000)

        self.get_logger().info(f'Publishing: {msg.data}')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    python_producer = Producer()

    rclpy.spin(python_producer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_producer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()