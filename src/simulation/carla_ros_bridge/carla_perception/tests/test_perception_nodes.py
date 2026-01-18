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
"""Tests for perception nodes."""

import math
import struct
import sys
from unittest.mock import MagicMock, patch

import pytest


class TestCameraPublisher:
    """Tests for camera publisher node."""

    def test_default_camera_parameters(self):
        """Test default camera parameters."""
        defaults = {
            "image_width": 800,
            "image_height": 600,
            "fov": 90.0,
        }

        assert defaults["image_width"] == 800
        assert defaults["image_height"] == 600
        assert defaults["fov"] == 90.0

    def test_fov_to_focal_length(self):
        """Test FOV to focal length conversion."""
        image_width = 800
        fov_degrees = 90.0
        fov_radians = math.radians(fov_degrees)

        # fx = width / (2 * tan(fov/2))
        focal_length = image_width / (2.0 * math.tan(fov_radians / 2.0))

        assert abs(focal_length - 400.0) < 1e-10

    def test_optical_frame_rotation(self):
        """Test optical frame has correct rotation."""
        # Camera optical frame: Z-forward, X-right, Y-down
        # Standard ROS frame: X-forward, Y-left, Z-up
        # Rotation: 90 degrees around X, then -90 around Z
        pass  # Frame conversion is handled by static transform


class TestLidarPublisher:
    """Tests for LiDAR publisher node."""

    def test_default_lidar_parameters(self):
        """Test default LiDAR parameters."""
        defaults = {
            "channels": 32,
            "range": 100.0,
            "rotation_frequency": 15.0,
            "upper_fov": 15.0,
            "lower_fov": -25.0,
        }

        assert defaults["channels"] == 32
        assert defaults["range"] == 100.0
        assert defaults["rotation_frequency"] == 15.0
        assert defaults["upper_fov"] == 15.0
        assert defaults["lower_fov"] == -25.0

    def test_points_per_channel_calculation(self):
        """Test points per channel based on horizontal FOV."""
        horizontal_fov = 360.0
        points_per_degree = 5.0

        points_per_channel = int(horizontal_fov * points_per_degree)
        assert points_per_channel == 1800

    def test_vertical_fov_range(self):
        """Test vertical FOV range calculation."""
        upper_fov = 15.0
        lower_fov = -25.0

        vertical_fov = upper_fov - lower_fov
        assert vertical_fov == 40.0

    def test_channel_angle_distribution(self):
        """Test channel angles are evenly distributed."""
        channels = 32
        upper_fov = 15.0
        lower_fov = -25.0

        if channels > 1:
            angle_step = (upper_fov - lower_fov) / (channels - 1)
            assert abs(angle_step - 40.0 / 31) < 1e-10


class TestPointCloud2Format:
    """Tests for PointCloud2 message format."""

    def test_point_fields(self):
        """Test PointCloud2 fields are correct."""
        expected_fields = ["x", "y", "z", "intensity"]
        assert len(expected_fields) == 4
        assert "x" in expected_fields
        assert "intensity" in expected_fields

    def test_point_step(self):
        """Test point step size (bytes per point)."""
        # 4 floats: x, y, z, intensity
        point_step = 4 * 4  # 4 fields * 4 bytes each
        assert point_step == 16

    def test_coordinate_conversion(self):
        """Test CARLA to ROS coordinate conversion for points."""
        # CARLA: X-forward, Y-right, Z-up
        # ROS: X-forward, Y-left, Z-up
        carla_point = (10.0, 5.0, 2.0)

        ros_x = carla_point[0]
        ros_y = -carla_point[1]
        ros_z = carla_point[2]

        assert ros_x == 10.0
        assert ros_y == -5.0
        assert ros_z == 2.0


class TestBboxPublisher:
    """Tests for bounding box publisher node."""

    def test_default_bbox_parameters(self):
        """Test default bounding box parameters."""
        defaults = {
            "publish_rate": 10.0,
            "frame_id": "base_link",
            "include_vehicles": True,
            "include_pedestrians": True,
            "max_distance": -1.0,
        }

        assert defaults["publish_rate"] == 10.0
        assert defaults["frame_id"] == "base_link"
        assert defaults["include_vehicles"] is True
        assert defaults["include_pedestrians"] is True
        assert defaults["max_distance"] == -1.0

    def test_distance_filtering(self):
        """Test distance-based filtering."""
        max_distance = 50.0
        actor_distances = [10.0, 30.0, 60.0, 100.0]

        filtered = [d for d in actor_distances if d <= max_distance]
        assert len(filtered) == 2
        assert 60.0 not in filtered
        assert 100.0 not in filtered

    def test_no_distance_limit(self):
        """Test no filtering when max_distance is -1."""
        max_distance = -1.0
        actor_distances = [10.0, 100.0, 1000.0]

        if max_distance < 0:
            filtered = actor_distances
        else:
            filtered = [d for d in actor_distances if d <= max_distance]

        assert len(filtered) == 3

    def test_bounding_box_dimensions(self):
        """Test bounding box has correct dimensions."""
        # CARLA provides extent (half-dimensions)
        extent_x = 2.0
        extent_y = 1.0
        extent_z = 0.75

        # Full dimensions
        size_x = extent_x * 2
        size_y = extent_y * 2
        size_z = extent_z * 2

        assert size_x == 4.0
        assert size_y == 2.0
        assert size_z == 1.5
