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
"""Ackermann control lifecycle node for CARLA vehicles."""

import re
import time
from typing import Optional

import carla
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from carla_common import connect_carla, find_ego_vehicle
from carla_msgs.srv import BindControl
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class AckermannControlNode(LifecycleNode):
    """Lifecycle node for Ackermann vehicle control."""

    def __init__(self, node_name="ackermann_control"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter(
            "carla_host",
            "localhost",
            ParameterDescriptor(description="CARLA server hostname"),
        )
        self.declare_parameter(
            "carla_port", 2000, ParameterDescriptor(description="CARLA server port")
        )
        self.declare_parameter(
            "carla_timeout",
            10.0,
            ParameterDescriptor(description="Connection timeout in seconds"),
        )
        self.declare_parameter(
            "role_name",
            "ego_vehicle",
            ParameterDescriptor(description="Role name of the ego vehicle to control"),
        )

        # Control parameters
        self.declare_parameter(
            "command_timeout",
            0.5,
            ParameterDescriptor(
                description="Stop vehicle if no command received within this time (seconds)"
            ),
        )
        self.declare_parameter(
            "control_rate",
            50.0,
            ParameterDescriptor(description="Control loop frequency in Hz"),
        )

        self.declare_parameter("require_control_gate", False)
        self.control_enabled = not self.get_parameter("require_control_gate").value
        self.bind_control_service = None
        self.command_generation = 0
        self.command_topic = "~/command"
        self.control_gate_service = None
        self.last_command_wall = None
        self.command_epoch_ns = 0

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.ego_vehicle: Optional["carla.Vehicle"] = None
        self.last_command_time: Optional[Time] = None
        self.last_command: Optional["AckermannDriveStamped"] = None

        # ROS interfaces
        self.command_subscription = None
        self.control_timer = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

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

        self._subscribe_commands("~/command")

        if self.get_parameter("require_control_gate").value:
            self.bind_control_service = self.create_service(
                BindControl, "~/bind_control", self.bind_control_callback
            )
            self.control_gate_service = self.create_service(
                SetBool, "~/set_enabled", self.control_gate_service_callback
            )

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        self.last_command_time = None
        self.last_command = None

        control_rate = self.get_parameter("control_rate").value
        self.control_timer = self.create_timer(
            1.0 / control_rate, self.control_timer_callback
        )

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        if self.control_timer:
            self.destroy_timer(self.control_timer)
            self.control_timer = None

        # Stop vehicle
        if self.ego_vehicle:
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

        self.command_generation += 1
        if self.command_subscription:
            self.destroy_subscription(self.command_subscription)
            self.command_subscription = None

        if self.control_gate_service:
            self.destroy_service(self.control_gate_service)
            self.control_gate_service = None
        if self.bind_control_service:
            self.destroy_service(self.bind_control_service)
            self.bind_control_service = None
        self.control_enabled = not self.get_parameter("require_control_gate").value
        self.last_command = None
        self.last_command_time = None
        self.last_command_wall = None
        self.ego_vehicle = None
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def _subscribe_commands(self, topic):
        """Replace the source and invalidate callbacks queued by its predecessor."""
        self.command_generation += 1
        generation = self.command_generation
        if self.command_subscription:
            self.destroy_subscription(self.command_subscription)
        self.command_subscription = self.create_subscription(
            AckermannDriveStamped,
            topic,
            lambda msg: self._command_for_generation(msg, generation),
            10,
        )
        self.command_topic = topic
        self.last_command = self.last_command_time = self.last_command_wall = None

    def _command_for_generation(self, message, generation):
        if generation == self.command_generation:
            self.command_callback(message)

    def bind_control_callback(self, request, response):
        """Bind a unique attempt topic only after the gate has been closed."""
        if self.control_enabled:
            response.success = False
            response.message = "Block commands before binding an attempt."
            return response
        if not re.fullmatch(
            r"/sim/attempts/[A-Za-z0-9_]+/command", request.command_topic
        ):
            response.success = False
            response.message = "Expected /sim/attempts/ATTEMPT_ID/command."
            return response
        self._subscribe_commands(request.command_topic)
        response.success = True
        response.message = self.command_topic
        return response

    def control_gate_callback(self, msg: Bool):
        """Fence queued commands at every pause, hold and attempt boundary."""
        self.control_enabled = msg.data
        if msg.data:
            self.command_epoch_ns = self.get_clock().now().nanoseconds
        self.last_command = None
        self.last_command_time = None
        self.last_command_wall = None

    def control_gate_service_callback(self, request, response):
        """Acknowledge command fencing before the world resumes ticking."""
        self.control_gate_callback(Bool(data=request.data))
        if not request.data and self.ego_vehicle:
            self._apply_stop_control()
        response.success = True
        response.message = "control enabled" if request.data else "commands cleared"
        return response

    def command_callback(self, msg: AckermannDriveStamped):
        """Handle incoming Ackermann drive commands."""
        if not self.control_enabled:
            return
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp, clock_type=now.clock_type)
        age = (now - stamp).nanoseconds / 1e9
        if self.get_parameter("require_control_gate").value and (
            stamp.nanoseconds <= self.command_epoch_ns
            or age < -0.001
            or age > self.get_parameter("command_timeout").value
        ):
            return
        self.last_command_time = now
        self.last_command_wall = time.monotonic()
        self.last_command = msg.drive

    def control_timer_callback(self):
        """Apply control to vehicle."""
        if not self.ego_vehicle or not self.control_enabled:
            return

        timeout = self.get_parameter("command_timeout").value
        if self.last_command_time is None or self.last_command is None:
            self._apply_stop_control()
            return

        time_since_command = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9
        if (
            time_since_command < 0.0
            or time_since_command > timeout
            or (
                self.last_command_wall is None
                or time.monotonic() - self.last_command_wall > timeout
            )
        ):
            self._apply_stop_control()
            return

        try:
            cmd = self.last_command

            # ROS Ackermann steering is left-positive; CARLA is right-positive.
            # Convert at the bridge boundary so ROS controllers keep their
            # standard convention. Steering velocity is an unsigned rate limit.
            ackermann = carla.VehicleAckermannControl()
            ackermann.steer = -cmd.steering_angle
            ackermann.steer_speed = cmd.steering_angle_velocity
            ackermann.speed = cmd.speed
            ackermann.acceleration = cmd.acceleration
            ackermann.jerk = cmd.jerk

            self.ego_vehicle.apply_ackermann_control(ackermann)

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
    node = AckermannControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
