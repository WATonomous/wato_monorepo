import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry # For odometry
from sensor_msgs.msg import NavSatFix # For GNSS

class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/carla/ego_vehicle/gnss',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        with open("/home/docker/ament_ws/src/carla_sample_node/logs/gnss_log.txt", 'a+') as file:
            file.write(str(msg.latitude) + ", " + str(msg.longitude) + "\n")

def main(args=None):
    rclpy.init(args=args)

    datalogger = Datalogger()

    rclpy.spin(datalogger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    datalogger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()