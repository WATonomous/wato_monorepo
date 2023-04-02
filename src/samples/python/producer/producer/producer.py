import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered


class Producer(Node):

    def __init__(self):
        super().__init__('python_producer')
        self.publisher_ = self.create_publisher(Unfiltered, '/producer', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.serialize_data)

        # Declare and get the parameters
        self.declare_parameter('velocity', 0.1)
        self.declare_parameter('pos_x', 0.0)
        self.declare_parameter('pos_y', 0.0)
        self.declare_parameter('pos_z', 0.0)

        self.pos_x = self.get_parameter('pos_x').value
        self.pos_y = self.get_parameter('pos_y').value
        self.pos_z = self.get_parameter('pos_z').value

    def set_velocity(self, velocity):
        velocity_ = velocity
        return velocity_

    def produce_data(self):
        self.pos_x += self.set_velocity(self.get_parameter('velocity').value)
        self.pos_y += self.set_velocity(self.get_parameter('velocity').value)
        self.pos_z += self.set_velocity(self.get_parameter('velocity').value)

    def serialize_data(self):
        self.produce_data()
        msg = Unfiltered()

        msg.data = "x:" + str(self.pos_x) + ";y:" + str(self.pos_y) + ";z:" + str(self.pos_z) + ";"
        msg.valid = True

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