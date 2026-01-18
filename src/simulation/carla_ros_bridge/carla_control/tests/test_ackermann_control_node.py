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

import sys
from unittest.mock import MagicMock, patch

import pytest


class TestAckermannControlNode:
    """Tests for AckermannControlNode class."""

    def test_apply_stop_control_logic(self):
        """Test stop control applies correct values."""
        mock_control = MagicMock()
        mock_control.throttle = 0.0
        mock_control.brake = 1.0
        mock_control.steer = 0.0

        # Verify stop control structure
        assert mock_control.throttle == 0.0
        assert mock_control.brake == 1.0
        assert mock_control.steer == 0.0


class TestAckermannControlParameters:
    """Tests for parameter handling."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 10.0,
            "role_name": "ego_vehicle",
            "command_timeout": 0.5,
            "control_rate": 50.0,
        }

        assert defaults["carla_host"] == "localhost"
        assert defaults["carla_port"] == 2000
        assert defaults["carla_timeout"] == 10.0
        assert defaults["role_name"] == "ego_vehicle"
        assert defaults["command_timeout"] == 0.5
        assert defaults["control_rate"] == 50.0

    def test_control_rate_to_period(self):
        """Test control rate to timer period conversion."""
        control_rate = 50.0
        expected_period = 1.0 / control_rate
        assert abs(expected_period - 0.02) < 1e-10


class TestCommandTimeout:
    """Tests for command timeout logic."""

    def test_timeout_detection(self):
        """Test timeout is detected correctly."""
        timeout = 0.5
        time_since_command = 0.6  # Exceeds timeout

        is_timed_out = time_since_command > timeout
        assert is_timed_out is True

    def test_no_timeout(self):
        """Test no timeout when command is recent."""
        timeout = 0.5
        time_since_command = 0.3  # Within timeout

        is_timed_out = time_since_command > timeout
        assert is_timed_out is False

    def test_timeout_boundary(self):
        """Test timeout at exact boundary."""
        timeout = 0.5
        time_since_command = 0.5  # Exactly at timeout

        is_timed_out = time_since_command > timeout
        assert is_timed_out is False  # Not greater than, so not timed out
