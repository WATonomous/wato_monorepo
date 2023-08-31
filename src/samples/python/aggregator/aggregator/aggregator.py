import rclpy
import time
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, FilteredArray


class Aggregator(Node):

    def __init__(self):
        super().__init__('python_aggregator')

        self.num_unfiltered_msgs = 0
        self.num_filtered_msgs = 0

        self.producer_f = 0
        self.transformer_f = 0

        self.node_start_time = time.time()

        self.unfiltered_subcriber = self.create_subscription(Unfiltered,
                                                             '/unfiltered_topic',
                                                             self.unfiltered_callback,
                                                             10)
        self.filtered_subscriber = self.create_subscription(FilteredArray,
                                                            '/filtered_topic',
                                                            self.filtered_callback,
                                                            10)

    def unfiltered_callback(self, msg):
        self.num_unfiltered_msgs += 1
        self.producer_f = self.calc_freq(self.num_unfiltered_msgs)
        self.print_freqs()

    def filtered_callback(self, msg):
        self.num_filtered_msgs += 1
        self.transformer_f = self.calc_freq(self.num_filtered_msgs)
        self.print_freqs()

    def calc_freq(self, count):
        return count / (time.time() - self.node_start_time)

    def print_freqs(self):
        self.get_logger().info('Number of unfiltered messages:' + str(self.num_unfiltered_msgs))
        self.get_logger().info('Number of filtered messages:' + str(self.num_filtered_msgs))

        self.get_logger().info('Producer Frequency:' + str(self.producer_f))
        self.get_logger().info('Transformer Frequency:' + str(self.transformer_f))


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
