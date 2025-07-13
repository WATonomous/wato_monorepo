#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import numpy as np

class GPSCarlaConverter(Node):
    def __init__(self):
        super().__init__('gps_to_xy')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/filtered',
            self.gps_callback,
            10)

        self.publisher = self.create_publisher(
            PointStamped,
            '/gps/filtered_xy',
            10)

        self.get_logger().info("Carla GPS converter node started.")

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        gps = np.array([lat, lon])

        # carla leaderboard conversion constants
        gps = (gps - np.array([0.0, 0.0])) * np.array([111324.60662786, 111319.490945])
        carla_xy = np.array([gps[1], gps[0]])

        point = PointStamped()
        point.header = msg.header
        point.point.x = carla_xy[0]
        point.point.y = carla_xy[1]
        point.point.z = msg.altitude

        self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = GPSCarlaConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()