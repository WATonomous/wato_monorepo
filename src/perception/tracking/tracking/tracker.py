#!/usr/bin/python3
import time
import numpy as np

from vision_msgs.msg import Detection3DArray
from tracking_msgs.msg import TrackedDetection3D, TrackedDetection3DArray

from .core.ab3dmot import AB3DMOT
from .core.utils import ros_utils
from .core.utils import geometry_utils
from .core.utils.config import cfg, cfg_from_yaml_file, log_config_to_file

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import copy

class TrackerNode(Node):
    def __init__(self):

        super().__init__("tracker_node")
        self.get_logger().info("Creating object tracker node...")

        self.declare_parameter("camera_detections_topic", "/augmented_camera_detections")
        self.declare_parameter("lidar_detections_topic", "/lidar_detections")
        self.declare_parameter("tracked_detections_topic", "/tracked_detections")
        self.declare_parameter("velocity_filter_constant", 5)
        self.declare_parameter("velocity_prediction_horizon", 0.5)
        self.declare_parameter("prediction_resolution", 0.5)
        self.declare_parameter("prediction_length", 4.0)
        self.declare_parameter("max_lidar_box_dim", 2)
        self.declare_parameter("min_lidar_box_dim", 0.5)
        self.declare_parameter("lidar_distance_threshold", 1)
        self.declare_parameter("default_distance_threshold", 3)
        self.declare_parameter("max_age", 5)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("reference_frame", "odom")
        self.declare_parameter("~traffic_sign_class_names")
        self.declare_parameter("~obstacle_class_names")
        self.declare_parameter("frequency", 10)
 
        self.declare_parameter("config_path", "/home/bolty/ament_ws/src/tracking/config/mahalanobis.yaml")


        self.camera_detections_topic = self.get_parameter("camera_detections_topic").value
        self.lidar_detections_topic = self.get_parameter("lidar_detections_topic").value
        self.tracked_detections_topic = self.get_parameter("tracked_detections_topic").value

        self.velocity_prediction_horizon = self.get_parameter("velocity_prediction_horizon").value
        self.prediction_resolution = self.get_parameter("prediction_resolution").value
        self.prediction_length = self.get_parameter("prediction_length").value
        self.max_lidar_box_dim = self.get_parameter("max_lidar_box_dim").value
        self.min_lidar_box_dim = self.get_parameter("min_lidar_box_dim").value
        self.lidar_distance_threshold = self.get_parameter("lidar_distance_threshold").value
        self.default_distance_threshold = self.get_parameter("default_distance_threshold").value
        self.max_age = self.get_parameter("max_age").value
        self.min_hits = self.get_parameter("min_hits").value
        self.reference_frame = self.get_parameter("reference_frame").value
        cfg.TRAFFIC_SIGN_CLASSES = self.get_parameter("~traffic_sign_class_names").value
        cfg.OBSTACLE_CLASSES = self.get_parameter("~obstacle_class_names").value
        self.config_path = self.get_parameter("config_path").value
        self.frequency = self.get_parameter("frequency").value
        self.dt = 1.0 / self.frequency

        cfg_from_yaml_file(self.config_path, cfg)


        # Initialize Tracker
        self.mot_tracker = AB3DMOT(max_age=self.max_age, 	# max age in seconds of a track before deletion
                                   min_hits=self.min_hits,	# min number of detections before publishing
                                   tracking_name="N/A") 	# default tracking age

        # Velocities for each track
        self.velocites = {}
        # low pass filter constant for velocity
        self.velocity_filter_constant = self.get_parameter("velocity_filter_constant")

        # tf2 listener
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        # Subscribers / Publishers
        self.obstacles_sub = self.create_subscription(
                 Detection3DArray, self.camera_detections_topic, self.obstacle_callback, 10)

        self.lidar_obstacles_sub = self.create_subscription(
                Detection3DArray, self.lidar_detections_topic, self.lidar_obstacle_callback, 10)

        self.tracked_obstacles_publisher = self.create_publisher(
                 TrackedDetection3DArray, self.tracked_detections_topic, 10)

        self.tracked_obstacles_timer = self.create_timer(0.1, self.publish_tracks, 10)

        # tracked_signs_topic = '/tracked_signs'
        # self.tracked_signs_publisher = self.create_publisher(
        # 		tracked_signs_topic, TrafficSignListMsg, queue_size=10)

    def reset(self):
        self.mot_tracker = AB3DMOT(max_age=self.max_age, 	# max age in seconds of a track before deletion
                                   min_hits=self.min_hits,	# min number of detections before publishing
                                   tracking_name="N/A") 	# default tracking age

    def project_objects_2d(self, obstacles, to_camera_transform, intrinsic_name="/camera/right/intrinsic"):
        """
        Inputs:
        - obstacles: List[Obstacle], describing 3D cuboids
        Outputs:
        - obstacles_2d: List[Obstacle], describing 2D bounding boxes
        """
        if not obstacles:
            return []

        # Retrieve intrinsic matrix
        try:
            intrinsic_matrix = self.get_parameter(intrinsic_name)
        except:
            self.get_logger().error("Can't find intrinsic matrix: {}".format(intrinsic_name))
            return []
        intrinsic_matrix = np.array(intrinsic_matrix.split(','), dtype=np.float64).reshape(3, -1)

        # Project obstacles to 2d
        obstacles_2d = []
        for obstacle in obstacles:
            bbox = ros_utils.obstacle_to_bbox(obstacle)
            try:
                bbox_2d = geometry_utils.project_bbox_2d(bbox, to_camera_transform, intrinsic_matrix)
            except geometry_utils.ClippingError:
                continue
            obstacle_2d = copy.deepcopy(obstacle)
            obstacle_2d.pose.pose.position.x = bbox_2d[0]
            obstacle_2d.pose.pose.position.y = bbox_2d[1]
            obstacle_2d.pose.pose.position.z = 0
            obstacle_2d.width_along_x_axis = bbox_2d[2]
            obstacle_2d.height_along_y_axis = bbox_2d[3]
            obstacle_2d.depth_along_z_axis = 0
            obstacles_2d.append(obstacle_2d)
        return obstacles_2d

    def track(self, detections, informations, frame_id, timestamp, update_only=False, distance_threshold=None):
        """
        Update all trackers.
        Input:
            detections: [[x,y,z,rz,w,l,h]...] list of list of doubles in the Odom frame
            informations: [[String, Double]...] list of list of class labels with confidence
            frame_id: String, frame name
            timstamp: rclpy.time detections timestamp
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
                bbox_to_reference_transform = self.tf2_buffer.lookup_transform(self.reference_frame, frame_id, rclpy.time.Time(), rclpy.time.Duration(0.1)).transform
            except:
                self.get_logger().error("Failed to look up transform from {} to {}".format(self.reference_frame, frame_id))
                return

            detections = geometry_utils.transform_boxes(detections, bbox_to_reference_transform)

        self.mot_tracker.update(detections, infos, timestamp.to_sec(), update_only, distance_threshold)

    def obstacle_callback(self, detections_msg):
        start_time = time.time()

        timestamp = detections_msg.header.stamp
        frame_id = detections_msg.header.frame_id

        detections = [] 		# [x, y, z, rot_y, l, w, h]
        informations = []		# [class label, confidence score]

        for detection in detections_msg.detections:
            detections.append(ros_utils.obstacle_to_bbox(detection.bbox))
            informations.append([detection.results[0].hypothesis.class_id, detection.results[0].hypothesis.score])

        self.track(detections, informations, frame_id, timestamp)

        # self.get_logger().info("Obstacle Update Time: {}".format(time.time() - start_time))

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

        # self.get_logger().info("Traffic Sign Update Time: {}".format(time.time() - start_time))

    def lidar_obstacle_callback(self, detections_msg):

        start_time = time.time()

        timestamp = detections_msg.header.stamp
        frame_id = detections_msg.header.frame_id

        detections = [] 		# [x, y, z, rot_y, l, w, h]
        informations = []		# [class label, confidence score]

        for box in detections_msg.obstacles:
            bbox = ros_utils.obstacle_to_bbox(box)
            lwh = np.array(bbox[-3:])
            if (lwh < self.max_lidar_box_dim).all() and (lwh > self.min_lidar_box_dim).any():  # filter out boxes too big or too small
                detections.append(bbox)
                informations.append(['UNKNOWN', 0.5])
        self.track(detections, informations, frame_id, timestamp, update_only=True, distance_threshold=self.lidar_distance_threshold)

        # self.get_logger().info("Obstacle Update Time: {}".format(time.time() - start_time))

    def publish_tracks(self):
        """
        Publishes the tracks as obj_tracked. Also publishes the node status message
        dt: time since last update (in seconds)
        """
        tracked_obstacle_list = TrackedDetection3DArray()
        # tracked_obstacle_list.header.frame_id = self.reference_frame
        # tracked_obstacle_list.header.stamp = rclpy.time.Time()
        # tracked_obstacle_list.tracked_obstacles = []

        # traffic_sign_list = TrafficSignListMsg()
        # traffic_sign_list.header.frame_id = self.reference_frame
        # traffic_sign_list.header.stamp = rclpy.time.Time()
        # traffic_sign_list.traffic_signs = []

        # for kf_track in self.mot_tracker.trackers:
        #     tracking_name = kf_track.tracking_name
        #     if tracking_name in self.traffic_sign_classes:
        #         traffic_sign_message = self._create_traffic_sign_message(kf_track, tracking_name)
        #         traffic_sign_list.traffic_signs.append(traffic_sign_message)
        #     else:
        #         x = kf_track.get_state()
        #         tracked_obstacle_message = self._create_tracked_obstacle_message(kf_track, tracking_name, self.dt)
        #         tracked_obstacle_list.tracked_obstacles.append(tracked_obstacle_message)
        # self.tracked_obstacles_publisher.publish(tracked_obstacle_list)
        # self.tracked_signs_publisher.publish(traffic_sign_list)

#		self.get_logger().info("Pub Time: {}".format(time.time() - start_time))

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

        tracked_obstacle_message = TrackedDetection3D()
        # [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
        bbox = kf_track.get_state()
        tracked_obstacle_message.obstacle = ros_utils.bbox_to_obstacle(bbox, kf_track.id, tracking_name)
        tracked_obstacle_message.obstacle.header.frame_id = self.reference_frame
        tracked_obstacle_message.header.frame_id = self.reference_frame
        #tracked_obstacle_message.score = kf_track.track_score

        if len(kf_track.history) == 0:
            self.get_logger().error("No History for id {}".format(kf_track.id))
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
            tracked_obstacle_message.observation_history.append(self.create_tracked_object_state_msg(rclpy.time.from_sec(timestamp), history))

        last_known_obs = tracked_obstacle_message.observation_history[-1]

        # Add the current observation
        tracked_obstacle_message.predicted_states.append(last_known_obs)

        current_time = last_known_obs.header.stamp
        max_pred_time = current_time + rclpy.time.Duration.from_sec(self.prediction_length)
        delta_time = 0

        # Initialize - first prediction is actual observation
        pred_time = current_time

        while (pred_time < max_pred_time):
            # self.get_logger().info('dt:{0}'.format(delta_time))
            delta_time +=  self.prediction_resolution

            pred_time += rclpy.time.Duration.from_sec(self.prediction_resolution)

            pred_obs = TrackedDetection3DArray()
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
        tracked_object_state = TrackedDetection3DArray()
        tracked_object_state.header.frame_id = self.reference_frame
        tracked_object_state.header.stamp = timestamp
        # [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
        tracked_object_state.pose.position.x = bbox[0]
        tracked_object_state.pose.position.y = bbox[1]
        tracked_object_state.pose.position.z = bbox[2]
        q = tr.quaternionerror(0, 0, bbox[3])
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

def main(args=None):
    rclpy.init(args=args)
    tracker_node = TrackerNode()
    rclpy.spin(tracker_node)
    tracker_node.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
