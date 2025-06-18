#! /usr/bin/env python
# import numpy as np
import tf_transformations as tr
# import copy

# from tracking_msgs.msg import TrackedDetection3D
from tracking_msgs.msg import Obstacle as ObstacleMsg

'''

Function to find the rotation around the z axis from a quaternion which is a 4 component
representation of orientation (x,y,z,w)

Quaternions are not intuitive for humans to interpret directly so we conver them into Euler
angles roll (roation around the x axis),

pitch (rotation around the y axis), yaw (rotation around the z-axis)

'''


def yaw_from_quaternion_msg(quaternion):
    (r, p, y) = tr.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return y


def obstacle_to_bbox(obstacle):
    # obstacle is a BoundingBox3D
    x = obstacle.center.position.x
    y = obstacle.center.position.y
    z = obstacle.center.position.z
    rz = yaw_from_quaternion_msg(obstacle.center.orientation)
    w = obstacle.size.x
    l = obstacle.size.y  # noqa: E741
    h = obstacle.size.z
    return [x, y, z, rz, w, l, h]


def marker_to_bbox(marker):
    x = marker.pose.position.x
    y = marker.pose.position.y
    z = marker.pose.position.z
    rz = yaw_from_quaternion_msg(marker.pose.orientation)
    w = marker.scale.x
    l = marker.scale.y  # noqa: E741
    h = marker.scale.z
    return [x, y, z, rz, w, l, h]


def bbox_to_obstacle(bbox, unique_id, label, confidence_score):
    # bbox: [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot?] rot_y_dot is optional
    # Assigns tracked obstacle values to appropriate data fields

    obstacle = ObstacleMsg()
    obstacle.label = label
    obstacle.confidence = confidence_score
    obstacle.object_id = unique_id

    obstacle.pose.pose.position.x = bbox[0]
    obstacle.pose.pose.position.y = bbox[1]
    obstacle.pose.pose.position.z = bbox[2]

    quaternion = tr.quaternion_from_euler(0, 0, bbox[3])
    obstacle.pose.pose.orientation.x = quaternion[0]
    obstacle.pose.pose.orientation.y = quaternion[1]
    obstacle.pose.pose.orientation.z = quaternion[2]
    obstacle.pose.pose.orientation.w = quaternion[3]

    obstacle.width_along_x_axis = bbox[4]
    obstacle.height_along_y_axis = bbox[5]
    obstacle.depth_along_z_axis = bbox[6]

    obstacle.twist.twist.linear.x = bbox[7]
    obstacle.twist.twist.linear.y = bbox[8]
    obstacle.twist.twist.linear.z = bbox[9]
    if len(bbox) > 10:
        obstacle.twist.twist.angular = [0, 0, bbox[10]]

    return obstacle


def bbox_to_traffic_sign(bbox, unique_id, label):
    # Placeholder for traffic sign conversion
    return


def traffic_sign_to_bbox(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    rz = yaw_from_quaternion_msg(msg.pose.orientation)
    w = msg.dimensions.x
    l = msg.dimensions.y  # noqa: E741
    h = msg.dimensions.z
    return [x, y, z, rz, w, l, h]
