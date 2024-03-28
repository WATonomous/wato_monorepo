import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge

import cv2
import numpy as np

import calibrate

class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")

        self.camera_topic = "/flir_camera/image_raw"
        
        self.count = 0
        self.capture_rate = 10

        self.objpoints = [] # 3d points in real world
        self.imgpoints = [] # corresponsing 2d points in camera frame
        self.K = np.zeros((3, 3))
        self.D = np.zeros((4, 1))

        self.cv_bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),  
        )

        # setup publishers for the instrinsics & distortion coeffs
        self.undistorted_publisher = self.create_publisher(
            Image, "/undistorted_img", 10)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, "/camera_info_calc", 10)
        
        # temp - for testing
        self.pub = self.create_publisher(Image, "/flir_camera/image_raw", 10)
        temp_msg = Image()
        temp = cv2.imread('calibrate.jpg')
        self.publish_img(temp, temp_msg, self.pub)

    def publish_img(self, img, msg, pub):
        imgmsg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        imgmsg.header.stamp = msg.header.stamp
        imgmsg.header.frame_id = msg.header.frame_id
        pub.publish(imgmsg)

    def publish_intrin(self):
        msg = CameraInfo()
        msg.k = self.K

        print(self.D)
        msg.d = np.array(self.D, dtype=np.float64)
        self.camera_info_publisher.publish(msg)

    def image_callback(self, msg):
        # process every nth image
        if self.count % self.capture_rate == 0:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                        msg, desired_encoding="passthrough")

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BAYER_BG2BGR)
            objpoints, imgpoints, _ = calibrate.find_corners(cv_image)

            # update intrinsics value (if possible)
            if len(objpoints) > 0:
                self.objpoints.extend(objpoints)
                self.imgpoints.extend(imgpoints)

                self.K, self.D = calibrate.solve_intrinsics(self.objpoints, self.imgpoints, cv_image, 'fisheye')

                self.publish_intrin()

            # publish undistorted image
            undistorted_cv_image = calibrate.undistort_image(self.K, self.D, cv_image, 'fisheye')
            self.publish_img(undistorted_cv_image, msg, self.undistorted_publisher)

        self.count += 1
        print("got image")

def main(args=None):
    rclpy.init(args=args)

    calibration_node = CalibrationNode()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
