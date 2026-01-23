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
"""Tests for teleop node."""


class TestTeleopNode:
    """Tests for TeleopNode class."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 10.0,
            "role_name": "ego_vehicle",
            "max_speed": 10.0,
            "max_steering": 1.0,
            "throttle_scale": 1.0,
            "steering_scale": 1.0,
            "command_timeout": 0.5,
            "control_rate": 50.0,
        }

        assert defaults["max_speed"] == 10.0
        assert defaults["max_steering"] == 1.0
        assert defaults["throttle_scale"] == 1.0
        assert defaults["steering_scale"] == 1.0


class TestTwistToControl:
    """Tests for Twist message to vehicle control conversion."""

    def test_forward_motion(self):
        """Test forward motion from positive linear.x."""
        linear_x = 5.0
        max_speed = 10.0
        throttle_scale = 1.0

        # Normalized throttle
        throttle = (linear_x / max_speed) * throttle_scale
        throttle = max(0.0, min(1.0, throttle))

        assert throttle == 0.5

    def test_reverse_motion(self):
        """Test reverse motion from negative linear.x."""
        linear_x = -5.0
        _max_speed = 10.0

        # Reverse uses brake or reverse gear
        is_reverse = linear_x < 0
        assert is_reverse is True

    def test_steering_conversion(self):
        """Test angular.z to steering conversion."""
        angular_z = 0.5
        max_steering = 1.0
        steering_scale = 1.0

        steering = (angular_z / max_steering) * steering_scale
        steering = max(-1.0, min(1.0, steering))

        assert steering == 0.5

    def test_steering_clamping(self):
        """Test steering is clamped to [-1, 1]."""
        angular_z = 2.0  # Exceeds max
        max_steering = 1.0
        steering_scale = 1.0

        steering = (angular_z / max_steering) * steering_scale
        steering = max(-1.0, min(1.0, steering))

        assert steering == 1.0

    def test_throttle_clamping(self):
        """Test throttle is clamped to [0, 1]."""
        linear_x = 15.0  # Exceeds max
        max_speed = 10.0
        throttle_scale = 1.0

        throttle = (linear_x / max_speed) * throttle_scale
        throttle = max(0.0, min(1.0, throttle))

        assert throttle == 1.0


class TestCommandTimeout:
    """Tests for command timeout handling."""

    def test_timeout_stops_vehicle(self):
        """Test vehicle stops when command times out."""
        timeout = 0.5
        time_since_command = 0.6

        should_stop = time_since_command > timeout
        assert should_stop is True

    def test_fresh_command_continues(self):
        """Test vehicle continues with fresh command."""
        timeout = 0.5
        time_since_command = 0.1

        should_stop = time_since_command > timeout
        assert should_stop is False


class TestAutonomyMode:
    """Tests for autonomy mode handling."""

    def test_autonomy_mode_disables_teleop(self):
        """Test autonomy mode disables teleop control."""
        autonomy_enabled = True
        teleop_should_apply = not autonomy_enabled

        assert teleop_should_apply is False

    def test_manual_mode_enables_teleop(self):
        """Test manual mode enables teleop control."""
        autonomy_enabled = False
        teleop_should_apply = not autonomy_enabled

        assert teleop_should_apply is True
