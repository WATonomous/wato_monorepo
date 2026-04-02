#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class FixedEgoOdometryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fixed_ego_odometry")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_topic", "liso/odometry")
        self.declare_parameter("odom_topic", "transform/odometry")
        self.declare_parameter("slam_topic", "slam/odometry")
        self.declare_parameter("pose_topic", "fixed_ego_pose")

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.roll = float(self.get_parameter("roll").value)
        self.pitch = float(self.get_parameter("pitch").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        map_topic = str(self.get_parameter("map_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        slam_topic = str(self.get_parameter("slam_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)

        self.map_pub = self.create_publisher(Odometry, map_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.slam_pub = self.create_publisher(Odometry, slam_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)

        period = 1.0 / max(self.rate_hz, 1e-3)
        self.timer = self.create_timer(period, self.publish_messages)

        self.get_logger().info(
            f"Publishing fixed ego pose at x={self.x:.3f} y={self.y:.3f} z={self.z:.3f} "
            f"yaw={self.yaw:.3f} rad"
        )
        self.get_logger().info(f"Listening for pose updates on '{pose_topic}'")

    def pose_callback(self, msg: PoseStamped) -> None:
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.z = float(msg.pose.position.z)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        quat_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if quat_norm > 1e-6:
            self.roll, self.pitch, self.yaw = quaternion_to_euler(qx, qy, qz, qw)

        self.get_logger().info(
            f"Updated ego pose to x={self.x:.3f} y={self.y:.3f} z={self.z:.3f} "
            f"yaw={self.yaw:.3f} rad"
        )
        self.publish_messages()

    def make_msg(self, frame_id: str) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.child_frame_id = self.base_frame

        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        return msg

    def publish_tf(self) -> None:
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.translation.z = self.z
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)

    def publish_messages(self) -> None:
        self.publish_tf()
        self.map_pub.publish(self.make_msg(self.map_frame))
        self.odom_pub.publish(self.make_msg(self.odom_frame))
        self.slam_pub.publish(self.make_msg(self.map_frame))


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main() -> None:
    rclpy.init()
    node = FixedEgoOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
