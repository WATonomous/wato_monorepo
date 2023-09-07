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
import time
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, FilteredArray
from aggregator.aggregator_core import AggregatorCore


class Aggregator(Node):

    def __init__(self):
        super().__init__('python_aggregator')

        # Initialize Core Aggregator Logic (freq updates)
        node_start_time = time.time()
        self.__aggregator = AggregatorCore(node_start_time)

        # Initialize ROS2 Constructs
        self.unfiltered_subcriber = self.create_subscription(Unfiltered,
                                                             '/unfiltered_topic',
                                                             self.unfiltered_callback,
                                                             10)
        self.filtered_subscriber = self.create_subscription(FilteredArray,
                                                            '/filtered_topic',
                                                            self.filtered_callback,
                                                            10)

    def unfiltered_callback(self, msg):
        self.__aggregator.update_raw_freq()
        self.print_freqs()

    def filtered_callback(self, msg):
        self.__aggregator.update_filtered_freq()
        self.print_freqs()

    def print_freqs(self):
        self.get_logger().info('Number of unfiltered messages:' +
                               str(self.__aggregator.num_unfiltered_msgs))
        self.get_logger().info('Number of filtered messages:' +
                               str(self.__aggregator.num_filtered_msgs))

        self.get_logger().info('Producer Frequency:' + str(self.__aggregator.raw_freq))
        self.get_logger().info('Transformer Frequency:' + str(self.__aggregator.filtered_freq))


def main(args=None):
    rclpy.init(args=args)

    python_aggregator = Aggregator()

    rclpy.spin(python_aggregator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    python_aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
