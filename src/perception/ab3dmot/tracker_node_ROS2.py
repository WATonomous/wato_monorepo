#!/usr/bin/env python3

import os
import time
import numpy as np

# AB3DMOT
from core.ab3dmot import AB3DMOT
from core.utils.config import cfg, cfg_from_yaml_file, log_config_to_file
import core.utils.ros_utils as ros_utils
import core.utils.geometry_utils as geometry_utils

# ROS 2 packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from common_msgs.msg import TrafficSignList
from common_msgs.msg import TrafficSign
from common_msgs.msg import ObstacleList
from common_msgs.msg import Obstacle
from common_msgs.msg import TrackedObstacleList
from common_msgs.msg import TrackedObstacle
from common_msgs.msg import TrackedObstacleState

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('msg_queue_size', None),
                ('velocity_prediction_horizon', None),
                ('prediction_resolution', None),
                ('prediction_length', None),
                ('max_lidar_box_dim', None),
                ('min_lidar_box_dim', None),
                ('lidar_distance_threshold', None),
                ('default_distance_threshold', None),
                ('max_age', None),
                ('min_hits', None),
                ('traffic_sign_class_names', None),
                ('obstacle_class_names', None),
                ('velocity_filter_constant', None),
                ('config_path', None),
                ('publish_frequency', None)
            ]
        )

        self.queue_size = self.get_parameter('msg_queue_size').value
        self.velocity_prediction_horizon = self.get_parameter('velocity_prediction_horizon').value
        self.prediction_resolution = self.get_parameter('prediction_resolution').value
        self.prediction_length = self.get_parameter('prediction_length').value
        self.max_lidar_box_dim = self.get_parameter('max_lidar_box_dim').value
        self.min_lidar_box_dim = self.get_parameter('min_lidar_box_dim').value
        self.lidar_distance_threshold = self.get_parameter('lidar_distance_threshold').value
        self.default_distance_threshold = self.get_parameter('default_distance_threshold').value
        self.max_age = self.get_parameter('max_age').value
        self.min_hits = self.get_parameter('min_hits').value
        self.traffic_sign_classes = self.get_parameter('~traffic_sign_class_names').value
        self.obstacle_classes = self.get_parameter('~obstacle_class_names').value
        self.velocity_filter_constant = self.get_parameter('velocity_filter_constant').value

        self.reference_frame = 'odom'
        self.mot_tracker = AB3DMOT(max_age=self.max_age,
                                   min_hits=self.min_hits,
                                   tracking_name="N/A")
        self.velocites = {}

        # Subscribers / Publishers
        obstacles_topic = '/obstacles_3d'
        self.obstacles_sub = self.create_subscription(
            ObstacleList,
            obstacles_topic,
            self.obstacle_callback,
            self.queue_size)
        
        traffic_sign_topic = '/traffic_signs_3d'
        self.traffic_sign_sub = self.create_subscription(
            TrafficSignList,
            traffic_sign_topic,
            self.traffic_sign_callback,
            self.queue_size)
        
        lidar_obstacles_topic = '/object_detection'
        self.lidar_obstacles_sub = self.create_subscription(
            ObstacleList,
            lidar_obstacles_topic,
            self.lidar_obstacle_callback,
            self.queue_size)
        
        tracked_obstacles_topic = '/tracked_obstacles'
        self.tracked_obstacles_publisher = self.create_publisher(
            TrackedObstacleList,
            tracked_obstacles_topic,
            self.queue_size)
        
        tracked_signs_topic = '/tracked_signs'
        self.tracked_signs_publisher = self.create_publisher(
            TrafficSignList,
            tracked_signs_topic,
            self.queue_size)

    def reset(self):
        self.mot_tracker = AB3DMOT(max_age=self.max_age,
                                   min_hits=self.min_hits,
                                   tracking_name="N/A")

    def track(self, detections, informations, frame_id, timestamp, update_only=False, distance_threshold=None):
        pass
        # Implement track method

    def obstacle_callback(self, msg):
        pass
        # Implement obstacle_callback method

    def traffic_sign_callback(self, msg):
        pass
        # Implement traffic_sign_callback method

    def lidar_obstacle_callback(self, msg):
        pass
        # Implement lidar_obstacle_callback method

    def publish_tracks(self, dt):
        pass
        # Implement publish_tracks method

def main(args=None):
    rclpy.init(args=args)
    tracker_node = TrackerNode()

    frequency = tracker_node.get_parameter("publish_frequency").value
    ros_rate = tracker_node.create_rate(frequency)
    
    try:
        while rclpy.ok():
            try:
                tracker_node.publish_tracks(1 / frequency)
            except Exception as e:
                tracker_node.get_logger().error("Error: Exception caught while processing a frame", e.message)
            try:
                ros_rate.sleep()
            except rclpy.exceptions.ROSTimeMovedBackwardsException:
                tracker_node.reset()
                pass
    except KeyboardInterrupt:
        tracker_node.get_logger().info('Keyboard interrupt, shutting down...')

    tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
