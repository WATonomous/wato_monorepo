#!/usr/bin/python3

import os
import time
import numpy as np

# AB3DMOT
from core.ab3dmot import AB3DMOT
from core.utils.config import cfg, cfg_from_yaml_file, log_config_to_file
import core.utils.ros_utils as ros_utils
import core.utils.geometry_utils as geometry_utils

# ROS packages
import rospy
import message_filters
import tf2_ros
import tf.transformations as tr
import ros_numpy

from std_msgs.msg import Bool
from std_msgs.msg import Header

from common_msgs.msg import TrafficSignList as TrafficSignListMsg
from common_msgs.msg import TrafficSign as TrafficSignMsg
from common_msgs.msg import Obstacle as ObstacleMsg
from common_msgs.msg import ObstacleList as ObstacleListMsg
from common_msgs.msg import TrackedObstacle as TrackedObstacleMsg
from common_msgs.msg import TrackedObstacleList as TrackedObstacleListMsg
from common_msgs.msg import TrackedObstacleState as TrackedObstacleStateMsg

class TrackerNode:

	def __init__(self):
		self.queue_size = rospy.get_param("msg_queue_size")
		self.velocity_prediction_horizon = rospy.get_param("velocity_prediction_horizon")
		self.prediction_resolution = rospy.get_param("prediction_resolution")
		self.prediction_length = rospy.get_param("prediction_length")
		self.max_lidar_box_dim = rospy.get_param("max_lidar_box_dim")
		self.min_lidar_box_dim = rospy.get_param("min_lidar_box_dim")
		# association euclidian distance thresholds
		self.lidar_distance_threshold = rospy.get_param("lidar_distance_threshold")
		self.default_distance_threshold = rospy.get_param("default_distance_threshold")
		self.max_age = rospy.get_param("max_age")
		self.min_hits = rospy.get_param("min_hits")

		self.reference_frame='odom'

		self.traffic_sign_classes = rospy.get_param("~traffic_sign_class_names")
		self.obstacle_classes = rospy.get_param("~obstacle_class_names")

		# Initialize Tracker
		self.mot_tracker = AB3DMOT(max_age=self.max_age, 	# max age in seconds of a track before deletion
								   min_hits=self.min_hits,	# min number of detections before publishing
								   tracking_name="N/A") 	# default tracking age

		# Velocities for each track
		self.velocites = {}
		# low pass filter constant for velocity
		self.velocity_filter_constant = rospy.get_param("velocity_filter_constant")

		# tf2 listener
		self.tf2_buffer = tf2_ros.Buffer()
		tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		# Subscribers / Publishers
		obstacles_topic = '/obstacles_3d'
		self.obstacles_sub = rospy.Subscriber(
				obstacles_topic, ObstacleListMsg, self.obstacle_callback, queue_size=self.queue_size)

		traffic_sign_topic = '/traffic_signs_3d'
		self.obstacles_sub = rospy.Subscriber(
				traffic_sign_topic, TrafficSignListMsg, self.traffic_sign_callback, queue_size=self.queue_size)

		lidar_obstacles_topic = '/object_detection'  # non classified objects from euclidean clustering
		self.lidar_obstacles_sub = rospy.Subscriber(
				lidar_obstacles_topic, ObstacleListMsg, self.lidar_obstacle_callback, queue_size=self.queue_size)

		tracked_obstacles_topic = '/tracked_obstacles'
		self.tracked_obstacles_publisher = rospy.Publisher(
				tracked_obstacles_topic, TrackedObstacleListMsg, queue_size=self.queue_size)

		tracked_signs_topic = '/tracked_signs'
		self.tracked_signs_publisher = rospy.Publisher(
				tracked_signs_topic, TrafficSignListMsg, queue_size=self.queue_size)

	def reset(self):
		self.mot_tracker = AB3DMOT(max_age=self.max_age, 	# max age in seconds of a track before deletion
								   min_hits=self.min_hits,	# min number of detections before publishing
								   tracking_name="N/A") 	# default tracking age

	def track(self, detections, informations, frame_id, timestamp, update_only=False, distance_threshold=None):
		"""
		Update all trackers.
		Input:
			detections: [[x,y,z,rz,w,l,h]...] list of list of doubles in the Odom frame
			informations: [[String, Double]...] list of list of class labels with confidence
			frame_id: String, frame name
			timstamp: rospy.Time detections timestamp
			update_only: Bool, only update tracks don't create new ones
			distance_threshold: euclidian distance threshold for association
		"""
		if distance_threshold == None:
			distance_threshold = self.default_distance_threshold

#		frame_id = "base_link"
		detections = np.array(detections)
		infos = np.array(informations, dtype=object)

		if (frame_id != self.reference_frame and frame_id != ""):
			try:
				bbox_to_reference_transform = self.tf2_buffer.lookup_transform(self.reference_frame, frame_id, rospy.Time(0), rospy.Duration(0.1)).transform
			except:
				rospy.logerr("Failed to look up transform from {} to {}".format(self.reference_frame, frame_id))
				return

			detections = geometry_utils.transform_boxes(detections, bbox_to_reference_transform)

		self.mot_tracker.update(detections, infos, timestamp.to_sec(), update_only, distance_threshold)

	def obstacle_callback(self, obstacles_msg):

		start_time = time.time()

		timestamp = obstacles_msg.header.stamp
		frame_id = obstacles_msg.header.frame_id

		detections = [] 		# [x, y, z, rot_y, l, w, h]
		informations = []		# [class label, confidence score]

		for box in obstacles_msg.obstacles:
			detections.append(ros_utils.obstacle_to_bbox(box))
			informations.append([box.label, box.confidence])

		self.track(detections, informations, frame_id, timestamp)

		# rospy.loginfo("Obstacle Update Time: {}".format(time.time() - start_time))

	def traffic_sign_callback(self, sign_list_msg):

		start_time = time.time()

		timestamp = sign_list_msg.header.stamp
		frame_id = sign_list_msg.header.frame_id

		detections = [] 		# [x, y, z, rot_y, l, w, h]
		informations = []		# [class label, confidence score]

		for sign in sign_list_msg.traffic_signs:
			detections.append(ros_utils.traffic_sign_to_bbox(sign))
			informations.append([sign.traffic_sign_type, sign.confidence])

		self.track(detections, informations, frame_id, timestamp)

		# rospy.loginfo("Traffic Sign Update Time: {}".format(time.time() - start_time))

	def lidar_obstacle_callback(self, obstacles_msg):

		start_time = time.time()

		timestamp = obstacles_msg.header.stamp
		frame_id = obstacles_msg.header.frame_id

		detections = [] 		# [x, y, z, rot_y, l, w, h]
		informations = []		# [class label, confidence score]

		for box in obstacles_msg.obstacles:
			bbox = ros_utils.obstacle_to_bbox(box)
			lwh = np.array(bbox[-3:])
			if (lwh < self.max_lidar_box_dim).all() and (lwh > self.min_lidar_box_dim).any():  # filter out boxes too big or too small
				detections.append(bbox)
				informations.append(['UNKNOWN', 0.5])
		self.track(detections, informations, frame_id, timestamp, update_only=True, distance_threshold=self.lidar_distance_threshold)

		# rospy.loginfo("Obstacle Update Time: {}".format(time.time() - start_time))

	def publish_tracks(self, dt):
		"""
		Publishes the tracks as obj_tracked. Also publishes the node status message
		dt: time since last update (in seconds)
		"""
		tracked_obstacle_list = TrackedObstacleListMsg()
		tracked_obstacle_list.header.frame_id = self.reference_frame
		tracked_obstacle_list.header.stamp = rospy.Time.now()
		tracked_obstacle_list.tracked_obstacles = []

		traffic_sign_list = TrafficSignListMsg()
		traffic_sign_list.header.frame_id = self.reference_frame
		traffic_sign_list.header.stamp = rospy.Time.now()
		traffic_sign_list.traffic_signs = []

		for kf_track in self.mot_tracker.trackers:
			tracking_name = kf_track.tracking_name
			if tracking_name in self.traffic_sign_classes:
				traffic_sign_message = self._create_traffic_sign_message(kf_track, tracking_name)
				traffic_sign_list.traffic_signs.append(traffic_sign_message)
			else:
				x = kf_track.get_state()
				tracked_obstacle_message = self._create_tracked_obstacle_message(kf_track, tracking_name, dt)
				tracked_obstacle_list.tracked_obstacles.append(tracked_obstacle_message)
		self.tracked_obstacles_publisher.publish(tracked_obstacle_list)
		self.tracked_signs_publisher.publish(traffic_sign_list)

#		rospy.loginfo("Pub Time: {}".format(time.time() - start_time))

	def _create_traffic_sign_message(self, kf_track, tracking_name):
		# [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
		bbox = kf_track.get_state()

		traffic_sign_message = ros_utils.bbox_to_traffic_sign(bbox, kf_track.id, tracking_name)
		traffic_sign_message.header.frame_id = self.reference_frame

		return traffic_sign_message

	def _create_tracked_obstacle_message(self, kf_track, tracking_name, dt):
		"""
		Helper that creates the TrackedObstacle message from KalmanBoxTracker (python class)
		Args:
			kf_track: KalmanBoxTracker (Python class) object that represents the track
			tracking_name: String
			dt: time since last publish (in seconds)

		Returns: TrackedObstacle (anm_msgs)
		"""

		tracked_obstacle_message = TrackedObstacleMsg()
		# [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
		bbox = kf_track.get_state()
		tracked_obstacle_message.obstacle = ros_utils.bbox_to_obstacle(bbox, kf_track.id, tracking_name)
		tracked_obstacle_message.obstacle.header.frame_id = self.reference_frame
		tracked_obstacle_message.header.frame_id = self.reference_frame
		#tracked_obstacle_message.score = kf_track.track_score

		if len(kf_track.history) == 0:
			rospy.logerr("No History for id {}".format(kf_track.id))
			return tracked_obstacle_message

		if kf_track.id not in self.velocites:
			self.velocites[kf_track.id] = np.array([0.0, 0.0])

		# Estimate velocity using KF track history
		latest_timestamp = kf_track.history[-1][0]
		for timestamp, history in reversed(kf_track.history):
			dt = abs(latest_timestamp - timestamp)
			if dt >= self.velocity_prediction_horizon:
				# Ignore z velocity.
				new_velocity = np.array([(bbox[0] - history[0]) / dt, (bbox[1] - history[1]) / dt])
				# Use low pass filter
				alpha = dt / self.velocity_filter_constant
				self.velocites[kf_track.id] += alpha * (new_velocity - self.velocites[kf_track.id])
				break

		velocity = self.velocites[kf_track.id]

		# Set velocity
		tracked_obstacle_message.obstacle.twist.twist.linear.x = velocity[0]
		tracked_obstacle_message.obstacle.twist.twist.linear.y = velocity[1]

		# Create observation history messages
		for timestamp, history in kf_track.history:
			tracked_obstacle_message.observation_history.append(self.create_tracked_object_state_msg(rospy.Time.from_sec(timestamp), history))

		last_known_obs = tracked_obstacle_message.observation_history[-1]

		# Add the current observation
		tracked_obstacle_message.predicted_states.append(last_known_obs)

		current_time = last_known_obs.header.stamp
		max_pred_time = current_time + rospy.Duration.from_sec(self.prediction_length)
		delta_time = 0

		# Initialize - first prediction is actual observation
		pred_time = current_time

		while (pred_time < max_pred_time):
			# rospy.loginfo('dt:{0}'.format(delta_time))
			delta_time +=  self.prediction_resolution

			pred_time += rospy.Duration.from_sec(self.prediction_resolution)

			pred_obs = TrackedObstacleStateMsg()
			pred_obs.pose.position.x = last_known_obs.pose.position.x + (delta_time)*velocity[0]
			pred_obs.pose.position.y = last_known_obs.pose.position.y + (delta_time)*velocity[1]

			# zero velocity on Up direction
			pred_obs.pose.position.z = last_known_obs.pose.position.z

			pred_obs.pose.orientation = last_known_obs.pose.orientation
			pred_obs.velocity = last_known_obs.velocity
			pred_obs.header.stamp = pred_time
			pred_obs.header.frame_id = self.reference_frame

			tracked_obstacle_message.predicted_states.append(pred_obs)

		return tracked_obstacle_message

	def create_tracked_object_state_msg(self, timestamp, bbox):
		"""Helper that creates creates the TrackedObstacleState message from a KF tracked state
		Args:
			timestamp: ROS timestamp
			bbox:  [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
		Returns: TrackedObstacleState (anm_msgs)
		"""
		tracked_object_state = TrackedObstacleStateMsg()
		tracked_object_state.header.frame_id = self.reference_frame
		tracked_object_state.header.stamp = timestamp
		# [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
		tracked_object_state.pose.position.x = bbox[0]
		tracked_object_state.pose.position.y = bbox[1]
		tracked_object_state.pose.position.z = bbox[2]
		q = tr.quaternion_from_euler(0, 0, bbox[3])
		tracked_object_state.pose.orientation.x = q[0]
		tracked_object_state.pose.orientation.y = q[1]
		tracked_object_state.pose.orientation.z = q[2]
		tracked_object_state.pose.orientation.w = q[3]

		tracked_object_state.velocity.linear.x = bbox[7]
		tracked_object_state.velocity.linear.y = bbox[8]
		tracked_object_state.velocity.linear.z = bbox[9]
		if (len(bbox) == 11):
			tracked_object_state.velocity.angular.z = bbox[10]

		return tracked_object_state

def main():
	rospy.init_node('tracker_node', anonymous=True)

	# Required parameters
	node_dir = os.path.dirname(os.path.realpath(__file__))
	root_dir = os.path.split(node_dir)[0]
	top_dir = os.path.split(root_dir)[0]

	config_path = rospy.get_param('config_path')
	cfg_from_yaml_file(config_path, cfg)

	cfg.TRAFFIC_SIGN_CLASSES = rospy.get_param("~traffic_sign_class_names")
	cfg.OBSTACLE_CLASSES = rospy.get_param("~obstacle_class_names")

	tracker_node = TrackerNode()

	# Set node rate
	frequency = rospy.get_param("publish_frequency")
	ros_rate = rospy.Rate(frequency)

	try:
		while not rospy.is_shutdown():
			try:
				tracker_node.publish_tracks(1 / frequency)
			except Exception as e:
				rospy.logerr("Error: Exception caught while processing a frame", e.message)
			try:
				ros_rate.sleep()
			# This exception is raised when playing rosbags.
			except rospy.exceptions.ROSTimeMovedBackwardsException:
				tracker_node.reset()
				pass

	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down")


if __name__ == "__main__":
	main()