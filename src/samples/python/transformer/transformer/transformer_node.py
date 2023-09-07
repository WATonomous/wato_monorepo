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

import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, Filtered, FilteredArray
from transformer.transformer_core import TransformerCore


class Transformer(Node):

    def __init__(self):
        super().__init__('python_transformer')
        # Declare and get the parameters
        self.declare_parameter('version', 1)
        self.declare_parameter('compression_method', 0)
        self.declare_parameter('buffer_capacity', 5)

        self.__buffer_capacity = self.get_parameter('buffer_capacity') \
            .get_parameter_value().integer_value

        # Initialize Transformer Core Logic for Deserialization
        self.__transformer = TransformerCore()

        # Initialize ROS2 Constructs
        self.publisher_ = self.create_publisher(FilteredArray, '/filtered_topic', 10)
        self.subscription = self.create_subscription(Unfiltered, '/unfiltered_topic',
                                                     self.unfiltered_callback, 10)

        self.__filtered_array_packets = []

    def unfiltered_callback(self, msg):
        if not self.check_msg_validity(msg):
            self.get_logger().info('INVALID MSG')
            return

        # Init message object
        filtered_msg = Filtered()

        # Populate message object
        filtered_msg.pos_x, filtered_msg.pos_y, filtered_msg.pos_z = self.__transformer \
            .deserialize_data(msg.data)
        filtered_msg.timestamp = msg.timestamp
        filtered_msg.metadata.version = self.get_parameter('version') \
            .get_parameter_value().integer_value
        filtered_msg.metadata.compression_method = self.get_parameter('compression_method') \
            .get_parameter_value().integer_value

        # We send off a list of Filtered Messages (a message made of messages!)
        if self.populate_packet(filtered_msg):
            return

        # If we reach the buffer capacity, publish the filtered packets
        filtered_array_msg = FilteredArray()
        filtered_array_msg.packets = self.__filtered_array_packets

        self.get_logger().info('Buffer Capacity Reached. PUBLISHING...')
        self.publisher_.publish(filtered_array_msg)

        # clear packets for next round of messages
        self.__filtered_array_packets.clear()

    def populate_packet(self, filtered_msg):
        self.__filtered_array_packets.append(filtered_msg)
        return len(self.__filtered_array_packets) <= self.__buffer_capacity

    def check_msg_validity(self, msg):
        return msg.valid


def main(args=None):
    rclpy.init(args=args)

    python_transformer = Transformer()

    rclpy.spin(python_transformer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
