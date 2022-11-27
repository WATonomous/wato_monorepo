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

        self.subscription = self.create_subscription(Unfiltered, 'producer', self.producer_freq, 10)
        self.subscription = self.create_subscription(FilteredArray, 'transformer',self.transformer_freq, 10)
        self.subscription

    def producer_freq(self, msg):
        self.num_unfiltered_msgs += 1
        self.producer_f = self.num_unfiltered_msgs / (time.time() - self.node_start_time)
        self.print_freqs()

    def transformer_freq(self, msg):
        self.num_filtered_msgs += 1
        self.transformer_f = self.num_filtered_msgs / (time.time() - self.node_start_time)
        self.print_freqs()

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