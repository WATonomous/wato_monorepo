import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, Filtered, FilteredArray

class Transformer(Node):

    def __init__(self):
        super().__init__('python_transformer')
        # Declare and get the parameters
        self.declare_parameter('version', 1)
        self.declare_parameter('compression_method', 0)
        self.declare_parameter('buffer_capacity', 5)

        self.__buffer_capacity = self.get_parameter('buffer_capacity').get_parameter_value().integer_value

        # Initialize ROS2 Constructs
        self.publisher_ = self.create_publisher(FilteredArray, '/filtered_topic', 10)
        self.subscription = self.create_subscription(Unfiltered, '/unfiltered_topic', self.unfiltered_callback, 10)

        self.__filtered_array_packets = []

    def unfiltered_callback(self, msg):
        if not self.check_msg_validity(msg):
            self.get_logger().info('INVALID MSG')
            return
        
        filtered_msg = self.deserialize_data(msg)
        filtered_msg.timestamp = msg.timestamp
        filtered_msg.metadata.version = self.get_parameter('version').get_parameter_value().integer_value
        filtered_msg.metadata.compression_method = self.get_parameter('compression_method').get_parameter_value().integer_value
        
        self.__filtered_array_packets.append(filtered_msg)
        
        if len(self.__filtered_array_packets) <= self.__buffer_capacity:
            return

        # If we reach the buffer capacity, publish the filtered packets
        filtered_array_msg = FilteredArray()
        filtered_array_msg.packets = self.__filtered_array_packets

        self.get_logger().info('Buffer Capacity Reached. PUBLISHING...')
        self.publisher_.publish(filtered_array_msg)

        self.__filtered_array_packets.clear()       


    def check_msg_validity(self, msg):
        return msg.valid

    def deserialize_data(self, msg):
        filtered_msg = Filtered()

        unfiltered_array = msg.data
        unfiltered_array.split(";")

        filtered_msg.pos_x = float(unfiltered_array[unfiltered_array.find("x") + 2])
        filtered_msg.pos_y = float(unfiltered_array[unfiltered_array.find("y") + 2])
        filtered_msg.pos_z = float(unfiltered_array[unfiltered_array.find("z") + 2])

        return filtered_msg


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