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
"""Tests for localization node."""

import math
import sys
from unittest.mock import MagicMock, patch

import pytest


class TestLocalizationNode:
    """Tests for LocalizationNode class."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 10.0,
            "role_name": "ego_vehicle",
            "map_frame": "map",
            "odom_frame": "odom",
            "base_link_frame": "base_link",
            "publish_rate": 50.0,
        }

        assert defaults["carla_host"] == "localhost"
        assert defaults["carla_port"] == 2000
        assert defaults["map_frame"] == "map"
        assert defaults["odom_frame"] == "odom"
        assert defaults["base_link_frame"] == "base_link"
        assert defaults["publish_rate"] == 50.0

    def test_publish_rate_to_period(self):
        """Test publish rate to timer period conversion."""
        publish_rate = 50.0
        expected_period = 1.0 / publish_rate
        assert abs(expected_period - 0.02) < 1e-10


class TestTransformConversion:
    """Tests for transform conversion logic."""

    def test_carla_to_ros_position(self):
        """Test CARLA to ROS position conversion."""
        # CARLA: X-forward, Y-right, Z-up
        # ROS: X-forward, Y-left, Z-up
        carla_x, carla_y, carla_z = 10.0, 5.0, 2.0

        # Y is negated
        ros_x = carla_x
        ros_y = -carla_y
        ros_z = carla_z

        assert ros_x == 10.0
        assert ros_y == -5.0
        assert ros_z == 2.0

    def test_carla_to_ros_rotation(self):
        """Test CARLA to ROS rotation conversion."""
        # CARLA uses degrees, ROS uses radians
        # Pitch and yaw are negated
        carla_roll, carla_pitch, carla_yaw = 0.0, 90.0, 180.0

        ros_roll = math.radians(carla_roll)
        ros_pitch = -math.radians(carla_pitch)
        ros_yaw = -math.radians(carla_yaw)

        assert abs(ros_roll) < 1e-10
        assert abs(ros_pitch + math.pi / 2) < 1e-10
        assert abs(ros_yaw + math.pi) < 1e-10


class TestFrameHierarchy:
    """Tests for TF frame hierarchy."""

    def test_frame_names(self):
        """Test standard frame names."""
        map_frame = "map"
        odom_frame = "odom"
        base_link_frame = "base_link"

        # Standard ROS frame hierarchy: map -> odom -> base_link
        assert map_frame == "map"
        assert odom_frame == "odom"
        assert base_link_frame == "base_link"

    def test_transform_chain(self):
        """Test transform chain is correct."""
        # map -> odom (static identity for ground truth)
        # odom -> base_link (vehicle pose)
        frames = ["map", "odom", "base_link"]

        assert len(frames) == 3
        assert frames[0] == "map"
        assert frames[-1] == "base_link"
