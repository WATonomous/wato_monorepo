#!/usr/bin/env python3
"""
Publishes URDF description with volatile QoS for visualization in Foxglove.
Avoids transient_local QoS which is not supported by iceoryx.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import sys


class URDFPublisher(Node):
    def __init__(self, urdf_content):
        super().__init__('urdf_publisher')

        # Create QoS profile with volatile durability (compatible with iceoryx)
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher = self.create_publisher(
            String,
            '/robot_description_viz',
            qos
        )

        self.urdf_content = urdf_content

        # Publish periodically at 1 Hz for visualization tools
        self.timer = self.create_timer(1.0, self.publish_urdf)

        self.get_logger().info('URDF publisher started, publishing to /robot_description_viz')

    def publish_urdf(self):
        msg = String()
        msg.data = self.urdf_content
        self.publisher.publish(msg)


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: urdf_publisher.py <path_to_urdf>")
        sys.exit(1)

    urdf_path = sys.argv[1]

    try:
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        sys.exit(1)

    rclpy.init(args=args)
    node = URDFPublisher(urdf_content)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
