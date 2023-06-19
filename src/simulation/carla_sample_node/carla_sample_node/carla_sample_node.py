import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry # For odometry
from sensor_msgs.msg import NavSatFix # For GNSS

from std_msgs.msg import Bool

import os

class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
        self.containerId = os.getenv('LOG_FILE', 'log')
        self.gnssSubscription = self.create_subscription(
            NavSatFix,
            '/carla/ego_vehicle/gnss',
            self.gnss_callback,
            10
        )
        self.gnssSubscription  # prevent unused variable warning
        self.autopilotPublisher = self.create_publisher(
            Bool,
            '/carla/ego_vehicle/enable_autopilot',
            10
        )
        self.timer = self.create_timer(10, self.timer_callback) # Publish autopilot message every 10 seconds
    def gnss_callback(self, msg):
        with open("/home/docker/ament_ws/src/carla_sample_node/logs/gnss_" + self.containerId + ".txt", 'a+') as file:
            file.write(str(msg.latitude) + ", " + str(msg.longitude) + "\n")
    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.autopilotPublisher.publish(msg)

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