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
"""Tests for Ackermann control node."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from ackermann_msgs.msg import AckermannDrive
from carla_control.ackermann_control_node import AckermannControlNode
from rclpy.time import Time


class TestAckermannControlConversion(unittest.TestCase):
    """Exercise the bridge callback without starting ROS or contacting CARLA."""

    def make_node(self, command):
        return SimpleNamespace(
            ego_vehicle=MagicMock(),
            control_enabled=True,
            last_command=command,
            last_command_time=Time(seconds=10.0),
            last_command_wall=99.9,
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.1)),
            get_parameter=lambda _: SimpleNamespace(value=0.5),
            get_logger=MagicMock(),
            _apply_stop_control=MagicMock(),
        )

    def test_ros_turn_direction_is_converted_once(self):
        for steering, speed in ((0.2, 3.0), (-0.2, 3.0), (0.0, 3.0), (0.2, -1.0)):
            with self.subTest(steering=steering, speed=speed):
                command = AckermannDrive(
                    steering_angle=steering,
                    steering_angle_velocity=0.4,
                    speed=speed,
                    acceleration=1.5,
                    jerk=0.7,
                )
                node = self.make_node(command)
                with patch.object(time, "monotonic", return_value=100.0):
                    AckermannControlNode.control_timer_callback(node)

                node.ego_vehicle.apply_ackermann_control.assert_called_once()
                applied = node.ego_vehicle.apply_ackermann_control.call_args.args[0]
                self.assertAlmostEqual(applied.steer, -command.steering_angle)
                self.assertAlmostEqual(
                    applied.steer_speed, command.steering_angle_velocity
                )
                self.assertAlmostEqual(applied.speed, command.speed)
                self.assertAlmostEqual(applied.acceleration, command.acceleration)
                self.assertAlmostEqual(applied.jerk, command.jerk)
                node._apply_stop_control.assert_not_called()
                node.get_logger.assert_not_called()

    def test_closed_gate_does_not_apply_a_command(self):
        node = self.make_node(AckermannDrive(steering_angle=0.2, speed=3.0))
        node.control_enabled = False

        AckermannControlNode.control_timer_callback(node)

        node.ego_vehicle.apply_ackermann_control.assert_not_called()

    def test_stale_command_still_stops(self):
        node = self.make_node(AckermannDrive(steering_angle=0.2, speed=3.0))
        with patch.object(time, "monotonic", return_value=101.0):
            AckermannControlNode.control_timer_callback(node)

        node._apply_stop_control.assert_called_once()
        node.ego_vehicle.apply_ackermann_control.assert_not_called()
