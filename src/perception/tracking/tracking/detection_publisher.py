# import time
# import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from geometry_msgs.msg import Point, Quaternion, Vector3


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        # Publishers/Subscribers
        self.publisher = self.create_publisher(Detection3DArray, 'detections', 10)

        self.publish_first_detection_message()
        # Publish second message after 5 seconds
        self.create_timer(5, self.publish_second_detection_message)

    def publish_first_detection_message(self):
        msg = Detection3DArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Create the first Detection3D message
        detection1 = Detection3D()
        hypothesis1 = ObjectHypothesisWithPose()
        hypothesis1.hypothesis.class_id = "car"
        hypothesis1.hypothesis.score = 0.9
        bbox1 = BoundingBox3D()
        bbox1.center.position = Point(x=20.0, y=15.0, z=50.0)  # Significantly different position
        # Quaternion for yaw 90 degrees (π/2)
        bbox1.center.orientation = Quaternion(x=0.0, y=0.0, z=0.7071, w=0.7071)
        bbox1.size = Vector3(x=10.0, y=5.0, z=5.0)  # Significantly different size
        detection1.results.append(hypothesis1)
        detection1.bbox = bbox1

        # Create the second Detection3D message
        detection2 = Detection3D()
        hypothesis2 = ObjectHypothesisWithPose()
        hypothesis2.hypothesis.class_id = "truck"
        hypothesis2.hypothesis.score = 0.8
        bbox2 = BoundingBox3D()
        bbox2.center.position = Point(x=2.4745, y=1.6008, z=15.2377)
        bbox2.center.orientation = Quaternion(
            x=0.0, y=0.0, z=0.24740395925452294, w=0.9689124217106447
        )  # Quaternion for yaw 0.5
        bbox2.size = Vector3(x=4.1846, y=1.6609, z=1.5694)
        detection2.results.append(hypothesis2)
        detection2.bbox = bbox2

        # Add both detections to the array
        msg.detections.append(detection1)
        msg.detections.append(detection2)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info("Published first detection message with 2 detections")

    def publish_second_detection_message(self):
        msg = Detection3DArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Create a new Detection3D message
        detection = Detection3D()
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "bike"
        hypothesis.hypothesis.score = 0.95
        bbox = BoundingBox3D()
        bbox.center.position = Point(x=5.0, y=3.0, z=2.0)
        bbox.center.orientation = Quaternion(
            x=0.0, y=0.0, z=0.479425538604203, w=0.8775825618903728
        )  # Quaternion for yaw 1.0
        bbox.size = Vector3(x=2.0, y=1.0, z=1.0)
        detection.results.append(hypothesis)
        detection.bbox = bbox

        # Add the detection to the array
        msg.detections.append(detection)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info("Published second detection message with 1 new detection")

        # Shutdown the node after publishing the second message
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    publisher = TrackerNode()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
