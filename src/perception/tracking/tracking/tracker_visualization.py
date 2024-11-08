import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from tracking_msgs.msg import Obstacle as ObstacleMsg
from tracking_msgs.msg import TrackedObstacle as TrackedObstacleMsg
from tracking_msgs.msg import TrackedObstacleList as TrackedObstacleListMsg

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        # Publishers/Subscribers
        self.tracker_subscriber = self.create_subscription(TrackedObstacleListMsg, 'tracked_obstacles', self.tracker_callback, 10)
        
        self.publisher = self.create_publisher(MarkerArray, 'tracked_visualizer', 10)

    def tracker_callback(self, trackedListMsg):
        
        marker_msg_array = MarkerArray()
        
        for tracked_obstacle in trackedListMsg.tracked_obstacles:
            marker = Marker()
            marker.type = Marker.CUBE
            marker.header = tracked_obstacle.header
            marker.id = tracked_obstacle.obstacle.object_id
            
            # set location
            marker.pose = tracked_obstacle.obstacle.pose.pose
            
            # set size
            marker.scale.x = tracked_obstacle.obstacle.width_along_x_axis
            marker.scale.y = tracked_obstacle.obstacle.height_along_y_axis
            marker.scale.z = tracked_obstacle.obstacle.depth_along_z_axis
                        
            marker_msg_array.markers.append(marker)
        
        self.publisher.publish(marker_msg_array)
        self.get_logger().info(f"Published {len(marker_msg_array.markers)} tracked obstable bounding boxes")
        for marker in marker_msg_array.markers:
            self.get_logger().info(f"object {marker.id} at: {marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    publisher = VisualizerNode()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()