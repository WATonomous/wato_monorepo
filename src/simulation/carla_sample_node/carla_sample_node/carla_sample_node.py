import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry  # For odometry
from sensor_msgs.msg import NavSatFix  # For GNSS

from std_msgs.msg import Bool

import os


class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
        self.declare_parameter("publish_autopilot", False)
        self.gnssSubscription = self.create_subscription(
            NavSatFix,
            '/carla/ego_vehicle/gnss',
            self.gnss_callback,
            10
        )
        if self.get_parameter("publish_autopilot").value:
            self.autopilotPublisher = self.create_publisher(
                Bool,
                '/carla/ego_vehicle/enable_autopilot',
                10
            )
            # Publish autopilot message every 10 seconds
            self.timer = self.create_timer(10, self.timer_callback)

    def gnss_callback(self, msg):
        # with open("/home/docker/ament_ws/src/carla_sample_node/logs/gnss_" + self.containerId + ".txt", 'a+') as file:
        #     file.write(str(msg.latitude) + ", " + str(msg.longitude) + "\n")
        self.get_logger().info(str(msg.latitude) + ", " +
                               str(msg.longitude))  # print to screen

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.autopilotPublisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    datalogger = Datalogger()

    rclpy.spin(datalogger)

    datalogger.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
