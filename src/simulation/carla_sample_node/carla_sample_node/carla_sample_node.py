##### IMPORTANT ####
"""
Uncomment the code under the main function (at the bottom of this file) to have the node actually run
"""


import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry # For odometry
from sensor_msgs.msg import NavSatFix # For GNSS

from std_msgs.msg import Bool

import os

class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
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
        # with open("/home/docker/ament_ws/src/carla_sample_node/logs/gnss_" + self.containerId + ".txt", 'a+') as file:
        #     file.write(str(msg.latitude) + ", " + str(msg.longitude) + "\n")
        self.get_logger().info(str(msg.latitude) + ", " + str(msg.longitude)) # print to screen
    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.autopilotPublisher.publish(msg)

def main(args=None):
    # Uncomment the below lines to actually run the sample node
    rclpy.init(args=args)
    
    datalogger = Datalogger()

    rclpy.spin(datalogger)

    datalogger.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()