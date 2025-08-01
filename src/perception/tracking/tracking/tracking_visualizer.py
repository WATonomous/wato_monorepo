import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
# Update this if your custom message name differs
from tracking_msgs.msg import TrackedObstacleList
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os

from tracking.core.utils.ros_utils import tracked_obstacle_to_bbox3d, marker_to_bbox3d

import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from collections import deque
from multiprocessing import Lock
from scipy.spatial.transform import Rotation
from random import randint

mutex = Lock()


class ObjectTrackingVisualizer(Node):
    def __init__(self):
        super().__init__("object_tracking_visualizer")

        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("publish_viz_topic", "/annotated_3d_det_img")
        self.declare_parameter("det_3d_topic", "/detections")
        self.declare_parameter("marker_topic", "/markers/annotations")
        self.declare_parameter("track_topic", "/tracked_obstacles")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame", "/camera")
        self.declare_parameter("lidar_frame", "/lidar")
        self.declare_parameter("global_frame", "/map")
        self.declare_parameter("sub_det3d", True)
        self.declare_parameter("viz_tracks", True)
        self.declare_parameter("viz_dets", True)
        self.declare_parameter("pub_markers", True)
        self.declare_parameter("pub_images", True)
        self.declare_parameter("skip_missed_frames", False)
        self.declare_parameter("box_line_width", 5)

        self.image_topic = self.get_parameter("image_topic").value
        self.publish_viz_topic = self.get_parameter("publish_viz_topic").value
        self.det_3d_topic = self.get_parameter("det_3d_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.track_topic = self.get_parameter("track_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.global_frame = self.get_parameter("global_frame").value
        self.sub_det3d = self.get_parameter("sub_det3d").value
        self.viz_tracks = self.get_parameter("viz_tracks").value
        self.viz_dets = self.get_parameter("viz_dets").value
        self.pub_markers = self.get_parameter("pub_markers").value
        self.pub_images = self.get_parameter("pub_images").value
        self.skip_missed_frames = self.get_parameter("skip_missed_frames").value
        self.box_line_width = self.get_parameter("box_line_width").value

        # Subscribing to the tracked obstacles topic
        self.tracked_obstacles_sub = self.create_subscription(
            TrackedObstacleList,  # Message type of the tracked obstacles
            self.track_topic,  # Topic name
            self.tracked_obstacles_callback,
            10  # Queue size
        )
        
        if self.sub_det3d:
            self.detection_subscriber = self.create_subscription(
                Detection3DArray, self.det_3d_topic, self.detection_callback, 10
            )
        else:
            self.detection_subscriber = self.create_subscription(
                MarkerArray, self.marker_topic, self.detection_callback, 10
            )

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "/tracked_objects_markers", 10)
        # Subscribing to the camera image feed
        self.image_sub = self.create_subscription(
            CompressedImage, "/CAM_FRONT/image_rect_compressed", self.image_callback, 10
        )

        # Camera info for 3D to 2D projection
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Publisher for the annotated image
        self.image_pub = self.create_publisher(Image, "/annotated_image", 10)
        # Namespace for the markers
        self.marker_namespace = "tracked_objects"
        # For image handling
        self.cv_bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.unprocessed_images = deque()
        self.unprocessed_dets = deque()
        self.unprocessed_tracked_obstacles = deque()
        self.image_width = None
        self.image_height = None
        self.camera_info = None
        self.transform = None
        self.latest_image = None
        self.latest_dets = None
        self.latest_tracked_obstacles = None
        self.latest_time = None
        self.get_logger().info(f"AAAAAAAAAAAA: {self.track_topic}")
        self.imgs_saved = 0

    def det_timestamp(self, det):
        if self.sub_det3d:
            return det.header.stamp
        else:
            return det.markers[0].header.stamp

    def time_diff(self, t1, t2):
        t1 = rclpy.time.Time.from_msg(t1)
        t2 = rclpy.time.Time.from_msg(t2)
        if t1 > t2:
            return t1 - t2
        else:
            return t2 - t1

    def get_closest_by_time(self, time):
        self.unprocessed_images = sorted(self.unprocessed_images, key=lambda x: self.time_diff(x.header.stamp, time))
        self.latest_image = self.unprocessed_images[0]
        if self.viz_tracks:
            self.unprocessed_dets = sorted(self.unprocessed_dets, key=lambda x: self.time_diff(self.det_timestamp(x), time))
            self.latest_dets = self.unprocessed_dets[0]
            self.unprocessed_dets = deque()
        self.latest_time = self.latest_image.header.stamp
        self.get_logger().info(f"CHOSEN TIME: {self.latest_time}")
        self.unprocessed_images = deque()
        return

    def image_callback(self, msg: CompressedImage):
        """Store and convert the latest compressed image frame."""
        self.get_logger().info(f"GOT IMAGE: {msg.header.stamp}")
        try:
            # Convert the compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert the OpenCV image to a ROS Image message
            with mutex:
                self.latest_image = 0
                imsg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                imsg.header.stamp = msg.header.stamp
                # if self.skip_missed_frames:
                #     self.latest_image = imsg
                #     self.latest_time = msg.header.stamp
                # else:
                self.unprocessed_images.append(imsg)
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {e}")

    def tracked_obstacles_callback(self, msg: TrackedObstacleList):
        """Process the tracked obstacles to generate markers and update the image."""
        # msg.tracked_obstacles = [msg.tracked_obstacles[3]]
        # msg.tracked_obstacles[0].obstacle.pose.pose.position.x = 400.0
        # msg.tracked_obstacles[0].obstacle.pose.pose.position.y = 1100.0
        # msg.tracked_obstacles[0].obstacle.pose.pose.position.z = 2.0
        with mutex:
            self.latest_tracked_obstacles = 0
            if self.skip_missed_frames:
                self.latest_tracked_obstacles = msg
            else:
                self.unprocessed_tracked_obstacles.append(msg)
        self.get_logger().info("GOT TRACKS")
        if self.viz_tracks and self.unprocessed_images and self.unprocessed_dets:
            # Generate 3D visualization markers for bounding boxes
            self.generate_markers(msg)

            # Get transform from global -> camera
            self.get_closest_by_time(msg.header.stamp)
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.global_frame,
                    self.latest_time)
                q = self.transform.transform.rotation
                r = Rotation.from_quat([q.x, q.y, q.z, q.w])
                self.get_logger().info(f"TRANS: {self.transform.transform.translation}, {r.as_euler('xyz', degrees=True)}")
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform from {self.global_frame} to {self.camera_frame}: {ex}')
                return

            self.get_logger().info("TRYING DRAW")
            self.try_draw()
        
        # Annotate the camera image with bounding boxes and IDs
        # self.annotate_image_with_bounding_boxes()

    def detection_callback(self, msg):
        with mutex:
            self.latest_dets = 0
            if self.skip_missed_frames and not self.viz_tracks:
                self.latest_dets = msg
            else:
                self.unprocessed_dets.append(msg)
        self.get_logger().info("GOT DETS")
        if not self.viz_tracks and self.unprocessed_images:
            # Get transform from global -> camera
            self.get_closest_by_time(self.det_timestamp(msg))
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.global_frame,
                    self.latest_time)
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform from {self.global_frame} to {self.camera_frame}: {ex}')
                return

            self.try_draw()

    def camera_info_callback(self, msg):
        # msg.k[5] = msg.height/2
        self.image_width = msg.width
        self.image_height = msg.height
        self.camera_info = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"GOT CAMERA INFO... {self.camera_info}")
        self.destroy_subscription(self.camera_info_subscription)

    def generate_markers(self, msg: TrackedObstacleList):
        """Generate markers for visualizing bounding boxes in 3D."""
        marker_array = MarkerArray()
        for tracked_obstacle in msg.tracked_obstacles:
            obj_id = tracked_obstacle.obstacle.object_id
            pose = tracked_obstacle.obstacle.pose.pose
            dimensions = tracked_obstacle.obstacle
            # Create a CUBE marker for the bounding box
            box_marker = Marker()
            box_marker.header.frame_id = msg.header.frame_id
            box_marker.header.stamp = self.get_clock().now().to_msg()
            box_marker.ns = f"{self.marker_namespace}_box"
            box_marker.id = obj_id
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose = pose
            box_marker.scale.x = dimensions.width_along_x_axis
            box_marker.scale.y = dimensions.height_along_y_axis
            box_marker.scale.z = dimensions.depth_along_z_axis
            box_marker.color.r = 1.0
            box_marker.color.g = 1.0
            box_marker.color.b = 0.0
            box_marker.color.a = 0.8  # Transparency
            box_marker.lifetime = rclpy.time.Duration(seconds=0.1).to_msg()
            # Add the box marker to the array
            marker_array.markers.append(box_marker)
            # Create a TEXT_VIEW_FACING marker for the object ID
            text_marker = Marker()
            text_marker.header.frame_id = msg.header.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"{self.marker_namespace}_id"
            text_marker.id = obj_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose
            # Position text above the box
            text_marker.pose.position.z += dimensions.depth_along_z_axis + 0.5
            text_marker.scale.z = 0.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID: {obj_id}"
            box_marker.lifetime = rclpy.time.Duration(seconds=3).to_msg()
            # Add the text marker to the array
            marker_array.markers.append(text_marker)
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def project_3d_to_2d(self, bbox):
        center = np.array(
            [bbox.center.position.x, bbox.center.position.y, bbox.center.position.z])
        rot = Rotation.from_quat([bbox.center.orientation.x, bbox.center.orientation.y,
                                bbox.center.orientation.z, bbox.center.orientation.w])
        half_size = np.array([bbox.size.x, bbox.size.y, bbox.size.z])/2

        # get all 8 corners
        vert = [center + rot.apply(np.multiply(half_size, np.array([-1, 1, 1]))),
                center + rot.apply(np.multiply(half_size, np.array([-1, -1, 1]))),
                center + rot.apply(np.multiply(half_size, np.array([-1, -1, -1]))),
                center + rot.apply(np.multiply(half_size, np.array([-1, 1, -1]))),
                center + rot.apply(np.multiply(half_size, np.array([1, 1, 1]))),
                center + rot.apply(np.multiply(half_size, np.array([1, -1, 1]))),
                center + rot.apply(np.multiply(half_size, np.array([1, -1, -1]))),
                center + rot.apply(np.multiply(half_size, np.array([1, 1, -1]))),
                ]

        verts_2d = []

        c_msg = Pose()
        c_msg.position.x = center[0]
        c_msg.position.y = center[1]
        c_msg.position.z = center[2]
        c_trans = tf2_geometry_msgs.do_transform_pose(c_msg, self.transform)

        if c_trans.position.z < 0:
            return None

        # project each 3d vert to 2d
        for v in vert:
            # convert v into a pos2d message
            v_msg = Pose()
            v_msg.position.x = v[0]
            v_msg.position.y = v[1]
            v_msg.position.z = v[2]

            # global to camera frame
            v_trans = tf2_geometry_msgs.do_transform_pose(v_msg, self.transform)
            v_trans = np.array([v_trans.position.x, v_trans.position.y, v_trans.position.z])
            
            # project 3d camera frame to 2d camera plane
            v_2d = self.camera_info @ v_trans
            v_2d = np.array([int(v_2d[0] / v_2d[2]), int(v_2d[1] / v_2d[2])])
            if v_trans[2] < 0:
                v_2d[0] = self.image_width - v_2d[0]
                v_2d[1] = self.image_height - v_2d[1]

            verts_2d.append(v_2d)
            #self.get_logger().info(f"FFFF: {v}, {v_trans}, {v_2d}")

            # draw vertex onto image
            # image = cv2.circle(image, v_2d, 5, color, thickness=-1)
        
        return verts_2d

    def draw_dets(self, image, det_msg):
        if self.sub_det3d:
            det_arr = det_msg.detections
        else:
            det_arr = det_msg.markers

        for det in det_arr:
            if self.sub_det3d:
                bb = det.bbox
            else:
                bb = marker_to_bbox3d(det)

            verts_2d = self.project_3d_to_2d(bb)

            if verts_2d is None:
                continue

            color = (255, 0, 0)#(randint(0, 255), randint(0, 255), randint(0, 255))

            # draw edges
            for i in range(4):
                # face 1
                image = cv2.line(image, verts_2d[i], verts_2d[(i+1) % 4], color, self.box_line_width)
                # face 2
                image = cv2.line(image, verts_2d[i+4], verts_2d[(i+1) % 4 + 4], color, self.box_line_width)
                # connect faces
                image = cv2.line(image, verts_2d[i], verts_2d[i+4], color, self.box_line_width)

        return image

    def draw_tracks(self, image, track_msg):
        for trk_msg in track_msg.tracked_obstacles:
            verts_2d = self.project_3d_to_2d(tracked_obstacle_to_bbox3d(trk_msg))

            if verts_2d is None:
                continue

            self.get_logger().info(f"{trk_msg.obstacle.pose.pose.position.x}, {trk_msg.obstacle.pose.pose.position.y}, {trk_msg.obstacle.pose.pose.position.z}, {verts_2d}")

            color = (0, 0, 255)  # Red for track boxes

            # draw edges
            for i in range(4):
                # face 1
                image = cv2.line(image, verts_2d[i], verts_2d[(i+1) % 4], color, self.box_line_width)
                # face 2
                image = cv2.line(image, verts_2d[i+4], verts_2d[(i+1) % 4 + 4], color, self.box_line_width)
                # connect faces
                image = cv2.line(image, verts_2d[i], verts_2d[i+4], color, self.box_line_width)

        return image

    def try_draw(self):
        if (
            self.latest_dets is None and self.viz_dets
            or self.latest_tracked_obstacles is None and self.viz_tracks
            or self.latest_image is None
            or self.transform is None
            or self.camera_info is None
        ):
            return

        with mutex:
            if self.skip_missed_frames:
                image_msg = self.latest_image
                det_msg = self.latest_dets
                track_msg = self.latest_tracked_obstacles
                self.get_logger().info(f"CCCC: {len(self.unprocessed_images)}, {len(self.unprocessed_dets)}, {len(self.unprocessed_tracked_obstacles)}")
            else:
                image_msg = self.unprocessed_images.popleft()
                if self.viz_dets:
                    det_msg = self.unprocessed_dets.popleft()
                if self.viz_tracks:
                    track_msg = self.unprocessed_tracked_obstacles.popleft()
                self.get_logger().info(f"BBBB: {len(self.unprocessed_images)}, {len(self.unprocessed_dets)}, {len(self.unprocessed_tracked_obstacles)}")

        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().info(str(e))
            return

        self.get_logger().info(f"PROCESSING IMAGE{' + DET3D' if self.viz_dets else ''}{' + TRACK' if self.viz_tracks else ''}")

        if self.viz_dets:
            self.get_logger().info(f"{len(det_msg.detections if self.sub_det3d else det_msg.markers)} DET3DS DRAWN")
            image = self.draw_dets(image, det_msg)
        if self.viz_tracks:
            self.get_logger().info(f"{len(track_msg.tracked_obstacles)} TRACKS DRAWN")
            image = self.draw_tracks(image, track_msg)

        self.publish_img(image, image_msg)

    # def annotate_image_with_bounding_boxes(self):
    #     """Annotate the latest image with bounding boxes and IDs."""
    #     if self.latest_image is None or self.latest_tracked_obstacles is None:
    #         return  # Wait until both the image and tracked obstacles are available
    #     # Convert the ROS image message to an OpenCV image
    #     cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
    #     for tracked_obstacle in self.latest_tracked_obstacles.tracked_obstacles:
    #         obj_id = tracked_obstacle.obstacle.object_id
    #         pose = tracked_obstacle.obstacle.pose.pose
    #         dimensions = tracked_obstacle.obstacle
    #         # self.get_logger().info(
    #         #     f"Obstacle ID {tracked_obstacle.obstacle.object_id}: "
    #         #     f"Pose ({pose.position.x}, {pose.position.y}, {pose.position.z}), "
    #         #     f"Dimensions (WxH: {dimensions.width_along_x_axis}x"
    #         #     f"{dimensions.height_along_y_axis})"
    #         # )
    #         # Assuming you have projection logic for 3D to 2D (replace this with actual projection)
    #         # Placeholder coordinates for now:
    #         top_left = (int(pose.position.x), int(pose.position.y))
    #         bottom_right = (
    #             int(pose.position.x + dimensions.width_along_x_axis),
    #             int(pose.position.y + dimensions.height_along_y_axis),
    #         )
    #         self.get_logger().info(
    #             f"Bounding box: Top-left {top_left}, Bottom-right {bottom_right}"
    #         )
    #         # Draw the bounding box
    #         cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
    #         # Annotate with the ID
    #         text = f"ID: {obj_id}"
    #         cv2.putText(cv_image, text, (top_left[0], top_left[1] - 10),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    #     # Convert the annotated image back to a ROS image message
    #     annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    #     # Publish the annotated image
    #     self.image_pub.publish(annotated_image_msg)

    def publish_img(self, cv_img, msg):
        imgmsg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        # save_dir = os.sep + os.path.join('home', 'bolty', 'ament_ws', 'src', 'tracking', 'tracking', 'images_check')
        # cv2.imwrite(os.path.join(save_dir, f"image{self.imgs_saved}.png"), cv_img)
        self.image_pub.publish(imgmsg)
        self.imgs_saved += 1
        self.get_logger().info("IMAGE PUBBED")


def main(args=None):
    rclpy.init(args=args)
    visualizer = ObjectTrackingVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
