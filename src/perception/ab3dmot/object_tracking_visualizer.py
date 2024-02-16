#! /usr/bin/env python

import roslib
import numpy as np
import core.utils.ros_utils as utils
import core.utils.geometry_utils as geo_utils
import tf2_ros
import tf.transformations as tr
import message_filters

from rospy.numpy_msg import numpy_msg
import rospy
import sensor_msgs.msg as sm
import visualization_msgs.msg as vm
import tf
from geometry_msgs.msg import PointStamped, Vector3Stamped
import copy
import cv2
import ros_numpy

from common_msgs.msg import TrafficSignList as TrafficSignListMsg
from common_msgs.msg import TrafficSign as TrafficSignMsg
from common_msgs.msg import Obstacle as ObstacleMsg
from common_msgs.msg import ObstacleList as ObstacleListMsg
from common_msgs.msg import TrackedObstacle as TrackedObstacleMsg
from common_msgs.msg import TrackedObstacleList as TrackedObstacleListMsg
from common_msgs.msg import TrackedObstacleState as TrackedObstacleStateMsg


# Marker base namespace
base_ns = "tracked_objects"
sign_ns = "tracked_signs"

# Marker lifetime constants (second)
marker_duration        = 0.4
history_duration       = 0.5

# Marker offset constants (m)
box_id_marker_offset_z = 2
vel2d_marker_offset_z  = 0

# Marker scale constants
vel2d_scale            = 0.5
vel2d_speed_scale      = 0.3

# Marker cleanup variables
marker_id_list = set()
sign_marker_id_list = set()

def removeMarkerBox(marray, m_ns, m_id):
    # remove box marker
    mbox                 = vm.Marker()
    mbox.ns              = m_ns + "_box"
    mbox.id              = m_id
    mbox.type            = vm.Marker.CUBE
    mbox.action          = vm.Marker.DELETE
    marray.markers.append(mbox)
    # remove box id marker
    mboxid                 = vm.Marker()
    mboxid.ns              = m_ns + "_id"
    mboxid.id              = m_id
    mboxid.type            = vm.Marker.TEXT_VIEW_FACING
    mboxid.action          = vm.Marker.DELETE
    marray.markers.append(mboxid)

    # remove vel 2d marker
    mvel2d                 = vm.Marker()
    mvel2d.ns              = m_ns + "_vel"
    mvel2d.id              = m_id
    mvel2d.type            = vm.Marker.ARROW
    mvel2d.action          = vm.Marker.DELETE
    marray.markers.append(mvel2d)

def removeMarkerPrediction(marray, m_ns, m_id):
    # remove predictions
    m                 = vm.Marker()
    m.ns              = m_ns + "_pred"
    m.id              = m_id
    m.type            = vm.Marker.LINE_STRIP
    m.action          = vm.Marker.DELETE
    marray.markers.append(m)

def removeMarkerObsHistory(marray, m_ns, m_id):

    # remove observation history
    m                 = vm.Marker()
    m.ns              = m_ns + "_hist"
    m.id              = m_id
    m.type            = vm.Marker.LINE_STRIP
    m.action          = vm.Marker.DELETE
    marray.markers.append(m)

def addMarkerBox(marray, m_frame, m_timestamp, m_ns, m_id, m_lifetime, \
                 tracked_object):

    global box_id_marker_offset_z
    global vel2d_marker_offset_z
    global vel2d_scale
    global vel2d_speed_scale

    ### BOX MARKER
    # box fundamental properties
    mbox                 = vm.Marker()
    mbox.header.frame_id = m_frame
    mbox.header.stamp    = m_timestamp
    mbox.ns              = m_ns + "_box"
    mbox.id              = m_id
    mbox.type            = vm.Marker.CUBE
    mbox.action          = vm.Marker.ADD
    mbox.lifetime        = rospy.Duration(m_lifetime)
    mbox.frame_locked    = True

    # setup pose, dimensions and color of box
    mbox.pose    = copy.deepcopy(tracked_object.pose.pose)
    mbox.scale.x = tracked_object.width_along_x_axis
    mbox.scale.y = tracked_object.height_along_y_axis
    mbox.scale.z = tracked_object.depth_along_z_axis
    mbox.color.r = 1.0
    mbox.color.g = 1.0
    mbox.color.b = 0.0
    mbox.color.a = 1.0

    # add box marker to marker array
    marray.markers.append(mbox)

    ### BOX ID MARKER
    # box id fundamental properties
    mboxid                 = vm.Marker()
    mboxid.header.frame_id = m_frame
    mboxid.header.stamp    = m_timestamp
    mboxid.ns              = m_ns + "_id"
    mboxid.id              = m_id
    mboxid.type            = vm.Marker.TEXT_VIEW_FACING
    mboxid.action          = vm.Marker.ADD
    mboxid.lifetime        = rospy.Duration(m_lifetime)
    mboxid.frame_locked    = True

    # setup pose, dimensions and color of the box id
    mboxid.pose             = copy.deepcopy(tracked_object.pose.pose)
    mboxid.pose.position.z += box_id_marker_offset_z
    mboxid.scale.x = 1.0
    mboxid.scale.y = 1.0
    mboxid.scale.z = 1.0
    mboxid.color.r = 1.0
    mboxid.color.g = 0.0
    mboxid.color.b = 0.0
    mboxid.color.a = 1.0

    # box id text field
    mboxid.text = "{}_{}".format(tracked_object.label[:3], str(tracked_object.object_id))

    # add box id marker to marker array
    marray.markers.append(mboxid)

    ### BOX VEL 2D ARROW MARKER
    # box vel 2d fundamental properties
    mvel2d                 = vm.Marker()
    mvel2d.header.frame_id = m_frame
    mvel2d.header.stamp    = m_timestamp
    mvel2d.ns              = m_ns + "_vel"
    mvel2d.id              = m_id
    mvel2d.type            = vm.Marker.ARROW
    mvel2d.action          = vm.Marker.ADD
    mvel2d.lifetime        = rospy.Duration(m_lifetime)
    mvel2d.frame_locked    = True

    # compute speed and magnitude of arrow
    obj_speed_2d = np.sqrt(np.power(tracked_object.twist.twist.linear.x, 2.0) + \
                           np.power(tracked_object.twist.twist.linear.y, 2.0))
    mvel2d.scale.x = tracked_object.width_along_x_axis / 2.0 + \
                     vel2d_speed_scale * obj_speed_2d
    mvel2d.scale.y = vel2d_scale
    mvel2d.scale.z = vel2d_scale

    # position
    mvel2d.pose             = copy.deepcopy(tracked_object.pose.pose)
    mvel2d.pose.position.z += vel2d_marker_offset_z

    # orientation
    yaw_2d = np.arctan2(tracked_object.twist.twist.linear.y, \
                        tracked_object.twist.twist.linear.x)
    mvel2d_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_2d)
    mvel2d.pose.orientation.x = mvel2d_quat[0]
    mvel2d.pose.orientation.y = mvel2d_quat[1]
    mvel2d.pose.orientation.z = mvel2d_quat[2]
    mvel2d.pose.orientation.w = mvel2d_quat[3]

    # color
    mvel2d.color.r = 1.0
    mvel2d.color.g = 0.5
    mvel2d.color.b = 0.3
    mvel2d.color.a = 1.0

    # add box id marker to marker array
    marray.markers.append(mvel2d)


def addSignMarkerBox(marray, m_frame, m_timestamp, m_ns, m_id, m_lifetime, \
                 tracked_sign):

    global box_id_marker_offset_z
    global vel2d_marker_offset_z
    global vel2d_scale
    global vel2d_speed_scale

    ### BOX MARKER
    # box fundamental properties
    mbox                 = vm.Marker()
    mbox.header.frame_id = m_frame
    mbox.header.stamp    = m_timestamp
    mbox.ns              = m_ns + "_box"
    mbox.id              = m_id
    mbox.type            = vm.Marker.CUBE
    mbox.action          = vm.Marker.ADD
    mbox.lifetime        = rospy.Duration(m_lifetime)
    mbox.frame_locked    = True

    # setup pose, dimensions and color of box
    mbox.pose    = copy.deepcopy(tracked_sign.pose)
    mbox.scale.x = tracked_sign.dimensions.x
    mbox.scale.y = tracked_sign.dimensions.y
    mbox.scale.z = tracked_sign.dimensions.z
    mbox.color.r = 1.0
    mbox.color.g = 1.0
    mbox.color.b = 0.0
    mbox.color.a = 1.0

    # add box marker to marker array
    marray.markers.append(mbox)

    ### BOX ID MARKER
    # box id fundamental properties
    mboxid                 = vm.Marker()
    mboxid.header.frame_id = m_frame
    mboxid.header.stamp    = m_timestamp
    mboxid.ns              = m_ns + "_id"
    mboxid.id              = m_id
    mboxid.type            = vm.Marker.TEXT_VIEW_FACING
    mboxid.action          = vm.Marker.ADD
    mboxid.lifetime        = rospy.Duration(m_lifetime)
    mboxid.frame_locked    = True

    # setup pose, dimensions and color of the box id
    mboxid.pose             = copy.deepcopy(tracked_sign.pose)
    mboxid.pose.position.z += box_id_marker_offset_z
    mboxid.scale.x = 1.0
    mboxid.scale.y = 1.0
    mboxid.scale.z = 1.0
    mboxid.color.r = 1.0
    mboxid.color.g = 0.0
    mboxid.color.b = 0.0
    mboxid.color.a = 1.0

    # box id text field
    mboxid.text = "{}_{}".format(tracked_sign.traffic_sign_type, str(tracked_sign.id))

    # add box id marker to marker array
    marray.markers.append(mboxid)

def addMarkerPrediction(marray, m_frame, m_timestamp, m_ns, m_id, m_lifetime, \
                        predictions):

    # check to see if there are predictions
    if len(predictions) <= 0:
        return

    # fundamental properties
    m                 = vm.Marker()
    m.header.frame_id = m_frame
    m.header.stamp    = m_timestamp
    m.ns              = m_ns + "_pred"
    m.id              = m_id
    m.type            = vm.Marker.LINE_STRIP
    m.action          = vm.Marker.ADD
    m.lifetime        = rospy.Duration(m_lifetime)
    m.frame_locked    = True
    # default pose to 0
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1

    # prediction line properties
    m.color.r = 0.0
    m.color.g = 0.7
    m.color.b = 0.2
    m.color.a = 1.0
    m.scale.x = 0.5

    # populate line_strip with prediction points
    m.points = []
    for pred in predictions:
        m.points.append(pred.pose.position)

    # append prediction marker to marker array
    marray.markers.append(m)

def addMarkerObsHistory(marray, m_frame, m_timestamp, m_ns, m_id, m_lifetime, \
                        observation_hist):

    # check to see if there are observation histories
    if len(observation_hist) <= 0:
        return

    # fundamental properties
    m                 = vm.Marker()
    m.header.frame_id = m_frame
    m.header.stamp    = m_timestamp
    m.ns              = m_ns + "_hist"
    m.id              = m_id
    m.type            = vm.Marker.LINE_STRIP
    m.action          = vm.Marker.ADD
    m.lifetime        = rospy.Duration(m_lifetime)
    m.frame_locked    = True
    #  default pose to 0
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1

    # observation history line properties
    m.color.r = 0.0
    m.color.g = 0.3
    m.color.b = 1.0
    m.color.a = 1.0
    m.scale.x = 0.5

    # populate line_strip with previous observation points
    m.points = []
    for obs in observation_hist:
        m.points.append(obs.pose.position)

    # append observation history marker to marker array
    marray.markers.append(m)

def receive_tracked_boxes(tracked_bb_msg):

    global marker_duration
    global history_duration
    global marker_id_list
    mark_array = vm.MarkerArray()
    pred_array = vm.MarkerArray()
    hist_array = vm.MarkerArray()
    current_marker_id_list = set()

    for tracked_obstacle in tracked_bb_msg.tracked_obstacles:

        # check if there are objects to display (at least 1 observed object)
        if len(tracked_obstacle.observation_history) > 0:

            # populate box markers
            addMarkerBox(mark_array, \
                         tracked_bb_msg.header.frame_id, \
                         tracked_bb_msg.header.stamp, \
                         base_ns, \
                         tracked_obstacle.obstacle.object_id, \
                         marker_duration, \
                         tracked_obstacle.obstacle)

            # populate prediction markers
            addMarkerPrediction(pred_array, \
                                tracked_bb_msg.header.frame_id, \
                                tracked_bb_msg.header.stamp, \
                                base_ns, \
                                tracked_obstacle.obstacle.object_id, \
                                marker_duration, \
                                tracked_obstacle.predicted_states)

            # populate observation markers
            addMarkerObsHistory(hist_array, \
                                tracked_bb_msg.header.frame_id, \
                                tracked_bb_msg.header.stamp, \
                                base_ns, \
                                tracked_obstacle.obstacle.object_id, \
                                history_duration, \
                                tracked_obstacle.observation_history)

            # append new id to list
            current_marker_id_list.add(tracked_obstacle.obstacle.object_id)

    # compare the two lists and cleanup any markers with stale ids
    stale_marker_ids = marker_id_list - current_marker_id_list
    for stale_id in stale_marker_ids:

        # keep observations alive (remove box and predictions)
        removeMarkerBox(mark_array, base_ns, stale_id)
        removeMarkerPrediction(pred_array, base_ns, stale_id)
        removeMarkerObsHistory(hist_array, base_ns, stale_id)

    # update new list
    marker_id_list = copy.deepcopy(current_marker_id_list)

    # publish the marker arrays
    ma_pub.publish(mark_array)
    history_marker_pub.publish(hist_array)
    pred_marker_pub.publish(pred_array)


def map_class_to_rgb(label: str):
    """
    Maps classes to colours for viz purposes
    """
    if label == "CYCLIST":
        return (255, 255, 0)

    elif label == "PEDISTRIAN":  # TODO this typo was left in for compatibility
        return (255, 0, 255)

    elif label == "UNKNOWN":
        return (255, 255, 255)

    elif label == "CAR":
        return (255, 0, 0)

def draw_tracks_on_camera_image(tracks: TrackedObstacleListMsg, camera_image: sm.Image):

    font = cv2.FONT_HERSHEY_SIMPLEX
    np_image = ros_numpy.numpify(camera_image)


    if len(tracks.tracked_obstacles) == 0:
        # nothing to draw
        camera_tracks_pub.publish(camera_image)
        return
    frame_id = tracks.tracked_obstacles[0].header.frame_id

    if not frame_id:
        frame_id = 'odom'

    # get the camera transform
    to_camera_transform = tf2_buffer.lookup_transform("camera_right_link", frame_id,
                                                      rospy.Time(0), rospy.Duration(0.1)).transform
    for track in tracks.tracked_obstacles:
        two_d_obstacles = utils.project_objects_2d([track.obstacle],
                                                       to_camera_transform=to_camera_transform)
        if len(two_d_obstacles) == 0:
            continue  # we have a track, but it's out of the camera frame
        else:
            assert len(two_d_obstacles) == 1

        obstacle = two_d_obstacles[0]

        conf = obstacle.confidence
        ID = obstacle.object_id
        colour = map_class_to_rgb(obstacle.label)

        # openCV uses (x, y) where top left is the origin
        header_text = f"c: {conf}, ID: {ID}"
        top_left = (int(obstacle.pose.pose.position.x), int(obstacle.pose.pose.position.y))
        bottom_right = (top_left[0] + int(obstacle.width_along_x_axis),
                        top_left[1] + int(obstacle.height_along_y_axis))

        box_thickness = 10
        np_image = cv2.rectangle(np_image, top_left, bottom_right, colour, thickness=box_thickness)
        # background to make the text readable
        text_box_top_right = (top_left[0] + (13 * len(header_text)), top_left[1] - 25)
        text_box_bottom_left = (top_left[0] - box_thickness//2,
                                top_left[1] + box_thickness//2)
        np_image = cv2.rectangle(np_image, text_box_bottom_left,
                                 text_box_top_right, colour, thickness=-1)
        np_image = cv2.putText(np_image, str(header_text), top_left, font, fontScale=0.8,
                               color=(20, 20, 20), thickness=3)

    image = ros_numpy.msgify(sm.Image, np_image, camera_image.encoding)
    camera_tracks_pub.publish(image)


def receive_tracked_signs(tracked_signs_msg):

    global marker_duration
    global sign_marker_id_list
    mark_array = vm.MarkerArray()
    pred_array = vm.MarkerArray()
    hist_array = vm.MarkerArray()
    current_marker_id_list = set()

    for tracked_sign in tracked_signs_msg.traffic_signs:

        # populate box markers
        addSignMarkerBox(mark_array, \
                        tracked_sign.header.frame_id, \
                        tracked_sign.header.stamp, \
                        sign_ns, \
                        tracked_sign.id, \
                        marker_duration, \
                        tracked_sign)

        # append new id to list
        current_marker_id_list.add(tracked_sign.id)

    # compare the two lists and cleanup any markers with stale ids
    stale_marker_ids = sign_marker_id_list - current_marker_id_list
    for stale_id in stale_marker_ids:
        # keep observations alive (remove box and predictions)
        removeMarkerBox(mark_array, sign_ns, stale_id)

    # update new list
    sign_marker_id_list = copy.deepcopy(current_marker_id_list)

    # publish the marker arrays
    ma_pub.publish(mark_array)

def setup():

    global ma_pub, history_marker_pub, listener, pred_marker_pub, camera_tracks_pub

    listener = tf.TransformListener()
    ma_pub = rospy.Publisher('/tracked_vis/observation', vm.MarkerArray, queue_size=1)
    history_marker_pub = rospy.Publisher('/tracked_vis/history', vm.MarkerArray, queue_size = 1)
    pred_marker_pub = rospy.Publisher('/tracked_vis/predictions', vm.MarkerArray, queue_size = 1)
    camera_tracks_pub = rospy.Publisher('/tracked_vis/camera_tracks', sm.Image, queue_size = 1)  # publish an image
    bb3d_sub = rospy.Subscriber('/tracked_obstacles', TrackedObstacleListMsg, callback = receive_tracked_boxes, queue_size = 1, tcp_nodelay=False)

    bb3d = message_filters.Subscriber('/tracked_obstacles', TrackedObstacleListMsg, queue_size = 1, tcp_nodelay=False)
    camera_sub = message_filters.Subscriber('/camera/right/image_color', sm.Image, queue_size = 1)
    signs3d = rospy.Subscriber('/tracked_signs', TrafficSignListMsg, callback = receive_tracked_signs, queue_size = 1, tcp_nodelay=False)
    merged_camera_detections = message_filters.ApproximateTimeSynchronizer([bb3d, camera_sub], queue_size=2, slop=0.1)  # last param is time tolerance in seconds, "close enough"
    merged_camera_detections.registerCallback(draw_tracks_on_camera_image)


if __name__ == '__main__':
    rospy.init_node('object_tracking_visualizer')

    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    try:
        setup()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass