import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaTrafficLightInfoList
from geometry_msgs.msg import Pose, Vector3

import os


class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
        self.get_logger().info("TRAFFIC LIGHT NODE STARTED")
        
        self.trafficLightSubscription = self.create_subscription(
            CarlaTrafficLightInfoList,
            '/carla/traffic_lights/info',
            self.trafficLightCallback,
            10
        )
        
        # Publish traffic message every 10 seconds
        self.timer = self.create_timer(10, self.timer_callback)

    def trafficLightCallback(self, msg):
        self.get_logger().info(f"RECEVIED {len(msg.traffic_lights)} LIGHTS:")
        for light in msg.traffic_lights:
            # light [CarlaTrafficLightInfo]
            
            self.get_logger().info(str(light.trigger_volume))

    def timer_callback(self):
        self.get_logger().info("test timer")


def main(args=None):
    rclpy.init(args=args)

    datalogger = Datalogger()

    rclpy.spin(datalogger)

    datalogger.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
