# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Localization lifecycle node for CARLA - publishes TF from map -> odom -> base_link."""

from typing import Optional
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from carla_common import (
    connect_carla,
    find_ego_vehicle,
    euler_to_quaternion,
    carla_to_ros_position,
    carla_to_ros_rotation,
)


class LocalizationNode(LifecycleNode):
    """Lifecycle node for publishing TF transforms from CARLA ground truth."""

    def __init__(self, node_name="carla_localization"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter(
            "carla_host", "localhost",
            ParameterDescriptor(description="CARLA server hostname"))
        self.declare_parameter(
            "carla_port", 2000,
            ParameterDescriptor(description="CARLA server port"))
        self.declare_parameter(
            "carla_timeout", 10.0,
            ParameterDescriptor(description="Connection timeout in seconds"))
        self.declare_parameter(
            "role_name", "ego_vehicle",
            ParameterDescriptor(description="Role name of the ego vehicle to track"))

        # TF frame parameters
        self.declare_parameter(
            "map_frame", "map",
            ParameterDescriptor(description="Name of the map frame"))
        self.declare_parameter(
            "odom_frame", "odom",
            ParameterDescriptor(description="Name of the odom frame"))
        self.declare_parameter(
            "base_link_frame", "base_link",
            ParameterDescriptor(description="Name of the base_link frame"))
        self.declare_parameter(
            "publish_rate", 50.0,
            ParameterDescriptor(description="TF publish rate in Hz"))

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None

        # ROS interfaces
        self.tf_broadcaster: Optional[TransformBroadcaster] = None
        self.publish_timer = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value
        role_name = self.get_parameter("role_name").value

        try:
            self.carla_client = connect_carla(host, port, timeout)
            self.get_logger().info(f"Connected to CARLA at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        try:
            world = self.carla_client.get_world()
            world.wait_for_tick()
            self.ego_vehicle = find_ego_vehicle(world, role_name)
            if not self.ego_vehicle:
                self.get_logger().error(
                    f'No vehicle with role_name "{role_name}" found'
                )
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f"Found ego vehicle: {self.ego_vehicle.type_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to find ego vehicle: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Create publish timer
        publish_rate = self.get_parameter("publish_rate").value
        timer_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(timer_period, self.publish_tf_callback)

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Stop publish timer
        if self.publish_timer:
            self.destroy_timer(self.publish_timer)
            self.publish_timer = None

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Release resources
        self.tf_broadcaster = None
        self.ego_vehicle = None
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def publish_tf_callback(self):
        """Publish TF transforms from CARLA ground truth."""
        if not self.ego_vehicle or not self.tf_broadcaster:
            return

        try:
            # Get vehicle transform from CARLA
            carla_transform = self.ego_vehicle.get_transform()

            # Get frame names
            map_frame = self.get_parameter("map_frame").value
            odom_frame = self.get_parameter("odom_frame").value
            base_link_frame = self.get_parameter("base_link_frame").value

            # Current timestamp
            now = self.get_clock().now().to_msg()

            # Convert CARLA transform to ROS coordinates
            x, y, z = carla_to_ros_position(
                carla_transform.location.x,
                carla_transform.location.y,
                carla_transform.location.z,
            )
            roll, pitch, yaw = carla_to_ros_rotation(
                carla_transform.rotation.roll,
                carla_transform.rotation.pitch,
                carla_transform.rotation.yaw,
            )
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            # Publish map -> odom (identity transform, no drift in simulation)
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = now
            map_to_odom.header.frame_id = map_frame
            map_to_odom.child_frame_id = odom_frame
            map_to_odom.transform.translation.x = 0.0
            map_to_odom.transform.translation.y = 0.0
            map_to_odom.transform.translation.z = 0.0
            map_to_odom.transform.rotation.x = 0.0
            map_to_odom.transform.rotation.y = 0.0
            map_to_odom.transform.rotation.z = 0.0
            map_to_odom.transform.rotation.w = 1.0

            # Publish odom -> base_link (vehicle pose)
            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = now
            odom_to_base.header.frame_id = odom_frame
            odom_to_base.child_frame_id = base_link_frame
            odom_to_base.transform.translation.x = x
            odom_to_base.transform.translation.y = y
            odom_to_base.transform.translation.z = z
            odom_to_base.transform.rotation.x = qx
            odom_to_base.transform.rotation.y = qy
            odom_to_base.transform.rotation.z = qz
            odom_to_base.transform.rotation.w = qw

            # Broadcast both transforms
            self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

        except Exception as e:
            self.get_logger().error(f"Error publishing TF: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
