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
"""Teleop control lifecycle node for CARLA - connects to Foxglove teleop panel."""

from typing import Optional
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from carla_common import connect_carla, find_ego_vehicle

import carla


class TeleopNode(LifecycleNode):
    """Lifecycle node for teleop control via Twist messages (Foxglove compatible)."""

    def __init__(self, node_name="carla_teleop"):
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
            ParameterDescriptor(description="Role name of the ego vehicle to control"))

        # Control parameters
        self.declare_parameter(
            "max_speed", 10.0,
            ParameterDescriptor(description="Max speed for normalizing linear.x input (m/s)"))
        self.declare_parameter(
            "max_steering", 1.0,
            ParameterDescriptor(description="Max angular.z value for normalizing steering input"))
        self.declare_parameter(
            "throttle_scale", 1.0,
            ParameterDescriptor(description="Scale factor applied to throttle output"))
        self.declare_parameter(
            "steering_scale", 1.0,
            ParameterDescriptor(description="Scale factor applied to steering output"))
        self.declare_parameter(
            "command_timeout", 0.5,
            ParameterDescriptor(description="Stop vehicle if no command received within this time (seconds)"))
        self.declare_parameter(
            "control_rate", 50.0,
            ParameterDescriptor(description="Control loop frequency in Hz"))

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None
        self.last_command_time: Optional[Time] = None
        self.last_twist: Optional[Twist] = None
        self.autonomy_enabled: bool = False

        # ROS interfaces
        self.twist_subscription = None
        self.control_timer = None
        self.autonomy_service = None

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

        # Create subscription for Twist messages (Foxglove teleop panel)
        self.twist_subscription = self.create_subscription(
            Twist, "~/cmd_vel", self.twist_callback, 10
        )

        # Create service for enabling/disabling autonomy mode
        self.autonomy_service = self.create_service(
            SetBool, "~/set_autonomy", self.set_autonomy_callback
        )

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Reset state
        self.last_command_time = None
        self.last_twist = None
        self.autonomy_enabled = False

        # Ensure CARLA autopilot is disabled on activation
        if self.ego_vehicle:
            try:
                self.ego_vehicle.set_autopilot(False)
            except Exception as e:
                self.get_logger().warn(f"Failed to disable autopilot: {e}")

        control_rate = self.get_parameter("control_rate").value
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_timer_callback)

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Stop control timer
        if self.control_timer:
            self.destroy_timer(self.control_timer)
            self.control_timer = None

        # Disable autopilot and stop vehicle
        if self.ego_vehicle:
            try:
                self.ego_vehicle.set_autopilot(False)
                self.autonomy_enabled = False
            except Exception as e:
                self.get_logger().warn(f"Failed to disable autopilot: {e}")

            try:
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.steer = 0.0
                self.ego_vehicle.apply_control(control)
            except Exception as e:
                self.get_logger().error(f"Failed to stop vehicle: {e}")

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Destroy subscription
        if self.twist_subscription:
            self.destroy_subscription(self.twist_subscription)
            self.twist_subscription = None

        # Destroy service
        if self.autonomy_service:
            self.destroy_service(self.autonomy_service)
            self.autonomy_service = None

        # Release CARLA resources
        self.ego_vehicle = None
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def twist_callback(self, msg: Twist):
        """Handle incoming Twist commands from Foxglove teleop panel."""
        self.last_command_time = self.get_clock().now()
        self.last_twist = msg

    def set_autonomy_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Handle autonomy mode enable/disable requests."""
        if not self.ego_vehicle:
            response.success = False
            response.message = "No ego vehicle available"
            return response

        try:
            if request.data:
                # Enable autonomy - activate CARLA autopilot
                self.ego_vehicle.set_autopilot(True)
                self.autonomy_enabled = True
                response.success = True
                response.message = (
                    "Autonomy enabled - CARLA autopilot active, teleop disabled"
                )
                self.get_logger().info("Autonomy mode ENABLED - teleop disabled")
            else:
                # Disable autonomy - deactivate CARLA autopilot
                self.ego_vehicle.set_autopilot(False)
                self.autonomy_enabled = False
                # Immediately apply stop control to take back control from Traffic Manager
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.steer = 0.0
                self.ego_vehicle.apply_control(control)
                response.success = True
                response.message = "Autonomy disabled - teleop enabled"
                self.get_logger().info("Autonomy mode DISABLED - teleop enabled")
        except Exception as e:
            response.success = False
            response.message = f"Failed to set autopilot: {e}"
            self.get_logger().error(f"Failed to set autopilot: {e}")

        return response

    def control_timer_callback(self):
        """Apply control to vehicle (called at fixed rate)."""
        if not self.ego_vehicle:
            return

        # Skip teleop control when autonomy is enabled
        if self.autonomy_enabled:
            return

        # Check for command timeout
        timeout = self.get_parameter("command_timeout").value
        if self.last_command_time is None:
            self._apply_stop_control()
            return

        time_since_command = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9
        if time_since_command > timeout:
            self._apply_stop_control()
            return

        if self.last_twist is None:
            self._apply_stop_control()
            return

        # Apply Twist command
        try:
            twist = self.last_twist
            max_speed = self.get_parameter("max_speed").value
            max_steering = self.get_parameter("max_steering").value
            throttle_scale = self.get_parameter("throttle_scale").value
            steering_scale = self.get_parameter("steering_scale").value

            control = carla.VehicleControl()

            # Convert linear.x to throttle/brake
            # Positive linear.x = forward (throttle)
            # Negative linear.x = reverse or brake
            normalized_speed = twist.linear.x / max_speed if max_speed > 0 else 0.0
            normalized_speed = max(-1.0, min(1.0, normalized_speed))

            if normalized_speed >= 0:
                control.throttle = min(1.0, normalized_speed * throttle_scale)
                control.brake = 0.0
                control.reverse = False
            else:
                # Negative = reverse
                control.throttle = min(1.0, abs(normalized_speed) * throttle_scale)
                control.brake = 0.0
                control.reverse = True

            # Convert angular.z to steering
            # Positive angular.z = left turn (negative steer in CARLA)
            # Negative angular.z = right turn (positive steer in CARLA)
            normalized_steer = (
                -twist.angular.z / max_steering if max_steering > 0 else 0.0
            )
            steer_value = normalized_steer * steering_scale
            control.steer = max(-1.0, min(1.0, steer_value))

            self.ego_vehicle.apply_control(control)

        except Exception as e:
            self.get_logger().error(f"Error applying control: {e}")

    def _apply_stop_control(self):
        """Apply stop control to vehicle."""
        try:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            self.ego_vehicle.apply_control(control)
        except Exception as e:
            self.get_logger().error(f"Error applying stop control: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
