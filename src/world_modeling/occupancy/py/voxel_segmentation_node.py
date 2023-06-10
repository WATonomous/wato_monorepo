#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from world_modeling_msgs.msg import VoxelGrid

class VoxelSegmentationNode(Node):

    def __init__(self):
        super().__init__('voxel_segmentation_node')
        self.publisher_ = self.create_publisher(VoxelGrid, 'voxel_grid_test', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = VoxelGrid()
        msg.size_x = self.i
        msg.size_y = self.i+1
        msg.size_z = self.i+2

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing Voxel Grid: %f %f %f' % (msg.size_x, msg.size_y, msg.size_z))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    voxel_segmentation_node = VoxelSegmentationNode()

    rclpy.spin(voxel_segmentation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voxel_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
