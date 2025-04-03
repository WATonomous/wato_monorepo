import rclpy
from rclpy.node import Node
import os

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import time
from collections import deque
from multiprocessing import Lock
from scipy.spatial.transform import Rotation
from random import randint

mutex = Lock()


class DrawBasicDetections(Node):
    def __init__(self):
        super().__init__("tracking_viz_node")
        self.get_logger().info("Creating tracking viz node...")

        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("publish_viz_topic", "/annotated_3d_det_img")
        self.declare_parameter("det_3d_topic", "/det_3d_topic")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame", "/camera")
        self.declare_parameter("lidar_frame", "/lidar")

        self.image_topic = self.get_parameter("image_topic").value
        self.publish_viz_topic = self.get_parameter("publish_viz_topic").value
        self.det_3d_topic = self.get_parameter("det_3d_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value

        # subscribes to images & 3D dets
        self.unprocessed_images = deque()
        self.unprocessed_dets = deque()
        self.camera_info = None
        self.transform = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self.det_3d_subscription = self.create_subscription(
            Detection3DArray,
            self.det_3d_topic,
            self.det_3d_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self.viz_publisher = self.create_publisher(Image, self.publish_viz_topic, 10)

    def image_callback(self, msg):
        with mutex:
            self.unprocessed_images.append(msg)
        self.try_draw()

    def det_3d_callback(self, msg):
        with mutex:
            self.unprocessed_dets.append(msg)

        # get transform from lidar -> camera
        if self.transform is None:
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.lidar_frame,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform from {self.lidar_frame} to {self.camera_frame}: {ex}')
                return

        self.try_draw()

    def camera_info_callback(self, msg):
        self.camera_info = np.array(msg.p).reshape(3, 4)

        self.get_logger().info(f"GOT CAMERA INFO... {self.camera_info}")

        self.destroy_subscription(self.camera_info_subscription)

    def try_draw(self):
        if not self.unprocessed_images or not self.unprocessed_dets or self.transform is None or self.camera_info is None:
            return

        with mutex:
            image_msg = self.unprocessed_images.popleft()
            det_3d_msg = self.unprocessed_dets.popleft()

        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        self.get_logger().info(f"PROCESSING IMAGE + DET3D...")

        for det_msg in det_3d_msg.detections:
            bbox = det_msg.bbox

            center = np.array(
                [bbox.center.position.x, bbox.center.position.y, bbox.center.position.z])
            rot = Rotation.from_quat([bbox.center.orientation.x, bbox.center.orientation.y,
                                     bbox.center.orientation.z, bbox.center.orientation.w])
            size = np.array([bbox.size.x, bbox.size.y, bbox.size.z])

            # get all 8 corners
            vert = [center + rot.apply(np.multiply(size, np.array([-1, 1, 1]))),
                    center + rot.apply(np.multiply(size, np.array([-1, -1, 1]))),
                    center + rot.apply(np.multiply(size, np.array([-1, -1, -1]))),
                    center + rot.apply(np.multiply(size, np.array([-1, 1, -1]))),
                    center + rot.apply(np.multiply(size, np.array([1, 1, 1]))),
                    center + rot.apply(np.multiply(size, np.array([1, -1, 1]))),
                    center + rot.apply(np.multiply(size, np.array([1, -1, -1]))),
                    center + rot.apply(np.multiply(size, np.array([1, 1, -1]))),
                    ]

            color = (randint(0, 255), randint(0, 255), randint(0, 255))
            verts_2d = []

            # project each 3d vert to 2d
            for v in vert:
                # convert v into a pos2d message
                v_msg = Pose()
                v_msg.position.x = v[0]
                v_msg.position.y = v[1]
                v_msg.position.z = v[2]

                # lidar to camera frame
                v_trans = tf2_geometry_msgs.do_transform_pose(v_msg, self.transform)
                v_trans = np.array([v_trans.position.x, v_trans.position.y, v_trans.position.z, 1])

                # project 3d camera frame to 2d camera plane
                v_2d = self.camera_info @ v_trans
                v_2d = np.array([int(v_2d[0] / v_2d[2]), int(v_2d[1] / v_2d[2])])
                verts_2d.append(v_2d)

                # draw vertex onto image
                # image = cv2.circle(image, v_2d, 5, color, thickness=-1)

            # draw edges
            for i in range(4):
                image = cv2.line(image, verts_2d[i], verts_2d[(i+1) % 4], color, 10)  # face 1
                image = cv2.line(image, verts_2d[i+4], verts_2d[(i+1) % 4 + 4], color, 10)  # face 2
                image = cv2.line(image, verts_2d[i], verts_2d[i+4], color, 10)  # connect faces

        self.publish_viz(image, image_msg)

    def publish_viz(self, cv_img, msg):
        imgmsg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        self.viz_publisher.publish(imgmsg)


def main(args=None):
    rclpy.init(args=args)

    tracking_viz_node = DrawBasicDetections()
    rclpy.spin(tracking_viz_node)
    tracking_viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
