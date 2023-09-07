# Copyright 2023 WATonomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered
from producer.producer_core import ProducerCore


class ProducerNode(Node):

    def __init__(self):
        super().__init__('python_producer')
        # Declare and get the parameters
        self.declare_parameter('pos_x', 0.0)
        self.declare_parameter('pos_y', 0.0)
        self.declare_parameter('pos_z', 0.0)
        self.declare_parameter('velocity', 0.0)

        # For parameters, we need to explicitely declare its type for Python to know
        # what to do with it
        pos_x = self.get_parameter('pos_x').get_parameter_value().double_value
        pos_y = self.get_parameter('pos_y').get_parameter_value().double_value
        pos_z = self.get_parameter('pos_z').get_parameter_value().double_value
        velocity = self.get_parameter('velocity').get_parameter_value().double_value

        # Initialize producer core logic for serialization
        self.__producer = ProducerCore(pos_x, pos_y, pos_z, velocity)

        # Initialize ROS2 constructs
        queue_size = 10
        self.publisher_ = self.create_publisher(Unfiltered, '/unfiltered_topic', queue_size)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.__publish_position)

    def __publish_position(self):
        self.__producer.update_position()
        msg = Unfiltered()

        msg.data = self.__producer.serialize_data()
        msg.valid = True
        msg.timestamp = int(time.time() * 1000)

        self.get_logger().info(f'Publishing: {msg.data}')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    python_producer = ProducerNode()

    rclpy.spin(python_producer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_producer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
