import rclpy
from rclpy.node import Node

from ros_msgs.msg import Unfiltered


class Producer(Node):

    def __init__(self):
        super().__init__('python_producer')
        self.publisher_ = self.create_publisher(Unfiltered, 'producer', 10)

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0

    def set_velocity(self, velocity):
        velocity_ = velocity
        return velocity_

    def produce(self):
        msg = Unfiltered()
        msg.data.pos_x += self.set_velocity(2)
        msg.data.pos_y += self.set_velocity(2)
        msg.data.pos_z += self.set_velocity(2)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    producer = Producer()

    rclpy.spin(Producer)

    producer.produce()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    producer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()