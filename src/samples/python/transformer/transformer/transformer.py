import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, Filtered, FilteredArray

class Transformer(Node):

    def __init__(self):
        super().__init__('python_transformer')
        self.publisher_ = self.create_publisher(FilteredArray, '/transformer', 10)
        self.subscription = self.create_subscription(Unfiltered, '/producer', self.check_msg_validity, 10)
        self.subscription

        self.FilteredArray = []


        self.declare_parameter('message_length', 5)
        self.message_length = self.get_parameter('message_length').value


    def check_msg_validity(self, msg):
        if msg.valid:
            self.deserialize_data(msg)
        else:
            self.get_logger().info('INVALID MSG')

    def deserialize_data(self, msg):
        filtered_msg = Filtered()

        unfiltered_array = msg.data
        unfiltered_array.split(";")

        filtered_msg.pos_x = float(unfiltered_array[unfiltered_array.find("x") + 2]) #the plus 2 here is just sloppy code by me to get to the right index ;o
        filtered_msg.pos_y = float(unfiltered_array[unfiltered_array.find("y") + 2])
        filtered_msg.pos_z = float(unfiltered_array[unfiltered_array.find("z") + 2])

        self.publish_filtered_array(filtered_msg)


    def publish_filtered_array(self, msg):
        filtered_array = FilteredArray()


        if len(self.FilteredArray) < self.message_length:
            self.FilteredArray.append(msg)

        elif len(self.FilteredArray) == self.message_length:
            filtered_array.packets = self.FilteredArray
            self.publisher_.publish(filtered_array)
            self.FilteredArray.clear()

        else:
            self.FilteredArray.clear()

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