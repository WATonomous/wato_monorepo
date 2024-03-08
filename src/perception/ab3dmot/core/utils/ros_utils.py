#!/usr/bin/env python3

import numpy as np
import tf2_ros
import tf2_geometry_msgs
import copy
import rclpy
from .geometry_utils import project_bbox_2d, ClippingError
from geometry_msgs.msg import Pose, Quaternion
from common_msgs.msg import Obstacle, TrafficSign

def project_objects_2d(obstacles, to_camera_transform, intrinsic_name="/camera/right/intrinsic"):
    """
    Inputs:
    - obstacles: List[Obstacle], describing 3D cuboids
    Outputs:
    - obstacles_2d: List[Obstacle], describing 2D bounding boxes
    """
    if not obstacles:
        return []

    try:
        intrinsic_matrix = rclpy.get_param(intrinsic_name)
    except:
        rclpy.get_logger().debug("Can't find intrinsic matrix: {}".format(intrinsic_name))
        return []
    intrinsic_matrix = np.array(intrinsic_matrix.split(','), dtype=np.float64).reshape(3, -1)

    obstacles_2d = []
    for obstacle in obstacles:
        bbox = obstacle_to_bbox(obstacle)
        try:
            bbox_2d = project_bbox_2d(bbox, to_camera_transform, intrinsic_matrix)
        except ClippingError:
            continue
        obstacle_2d = copy.deepcopy(obstacle)
        obstacle_2d.pose.position.x = bbox_2d[0]
        obstacle_2d.pose.position.y = bbox_2d[1]
        obstacle_2d.pose.position.z = 0
        obstacle_2d.width_along_x_axis = bbox_2d[2]
        obstacle_2d.height_along_y_axis = bbox_2d[3]
        obstacle_2d.depth_along_z_axis = 0
        obstacles_2d.append(obstacle_2d)
    return obstacles_2d

def yaw_from_quaternion_msg(quaternion):
    (r, p, y) = tf.transformations.euler_from_quaternion([quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()])
    return y

def obstacle_to_bbox(obstacle):
    x = obstacle.pose.position.x
    y = obstacle.pose.position.y
    z = obstacle.pose.position.z
    rz = yaw_from_quaternion_msg(obstacle.pose.orientation)
    w = obstacle.width_along_x_axis
    l = obstacle.height_along_y_axis
    h = obstacle.depth_along_z_axis
    return [x, y, z, rz, w, l, h]

def bbox_to_obstacle(bbox, unique_id, label):
    obstacle = Obstacle()
    obstacle.label = label
    obstacle.object_id = unique_id

    obstacle.pose.position.x = bbox[0]
    obstacle.pose.position.y = bbox[1]
    obstacle.pose.position.z = bbox[2]

    quaternion = tf.transformations.quaternion_from_euler(0, 0, bbox[3])
    obstacle.pose.orientation.x = quaternion[0]
    obstacle.pose.orientation.y = quaternion[1]
    obstacle.pose.orientation.z = quaternion[2]
    obstacle.pose.orientation.w = quaternion[3]

    obstacle.width_along_x_axis = bbox[4]
    obstacle.height_along_y_axis = bbox[5]
    obstacle.depth_along_z_axis = bbox[6]

    obstacle.twist.linear.x = bbox[7]
    obstacle.twist.linear.y = bbox[8]
    obstacle.twist.linear.z = bbox[9]
    if len(bbox) > 10:
        obstacle.twist.angular = [0, 0, bbox[10]]

    return obstacle

def bbox_to_traffic_sign(bbox, unique_id, label):
    traffic_sign_message = TrafficSign()
    traffic_sign_message.id = unique_id
    traffic_sign_message.traffic_sign_type = label

    traffic_sign_message.pose.position.x = bbox[0]
    traffic_sign_message.pose.position.y = bbox[1]
    traffic_sign_message.pose.position.z = bbox[2]

    quaternion = tf.transformations.quaternion_from_euler(0, 0, bbox[3])
    traffic_sign_message.pose.orientation.x = quaternion[0]
    traffic_sign_message.pose.orientation.y = quaternion[1]
    traffic_sign_message.pose.orientation.z = quaternion[2]
    traffic_sign_message.pose.orientation.w = quaternion[3]

    traffic_sign_message.dimensions.x = bbox[4]
    traffic_sign_message.dimensions.y = bbox[5]
    traffic_sign_message.dimensions.z = bbox[6]

    return traffic_sign_message

def traffic_sign_to_bbox(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    rz = yaw_from_quaternion_msg(msg.pose.orientation)
    w = msg.dimensions.x
    l = msg.dimensions.y
    h = msg.dimensions.z
    return [x, y, z, rz, w, l, h]

def project_bbox_2d(bbox, to_camera_transform, intrinsic_matrix):
    # Implement project_bbox_2d method
    pass

class ClippingError(Exception):
    pass

def main():
    rclpy.init()
    node = rclpy.create_node('projector')

    to_camera_transform = Pose()
    intrinsic_name = "/camera/right/intrinsic"
    obstacles = []  # Replace with actual list of obstacles
    obstacles_2d = project_objects_2d(obstacles, to_camera_transform, intrinsic_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
