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
"""Tests for transform utilities."""

import math

from carla_common.transforms import (
    euler_to_quaternion,
    quaternion_to_euler,
    carla_to_ros_position,
    carla_to_ros_rotation,
)


class TestEulerToQuaternion:
    """Tests for euler_to_quaternion function."""

    def test_identity(self):
        """Test zero rotation produces identity quaternion."""
        qx, qy, qz, qw = euler_to_quaternion(0, 0, 0)
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz) < 1e-10
        assert abs(qw - 1.0) < 1e-10

    def test_yaw_90_degrees(self):
        """Test 90 degree yaw rotation."""
        qx, qy, qz, qw = euler_to_quaternion(0, 0, math.pi / 2)
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz - math.sqrt(2) / 2) < 1e-10
        assert abs(qw - math.sqrt(2) / 2) < 1e-10

    def test_roll_90_degrees(self):
        """Test 90 degree roll rotation."""
        qx, qy, qz, qw = euler_to_quaternion(math.pi / 2, 0, 0)
        assert abs(qx - math.sqrt(2) / 2) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz) < 1e-10
        assert abs(qw - math.sqrt(2) / 2) < 1e-10

    def test_pitch_90_degrees(self):
        """Test 90 degree pitch rotation."""
        qx, qy, qz, qw = euler_to_quaternion(0, math.pi / 2, 0)
        assert abs(qx) < 1e-10
        assert abs(qy - math.sqrt(2) / 2) < 1e-10
        assert abs(qz) < 1e-10
        assert abs(qw - math.sqrt(2) / 2) < 1e-10

    def test_unit_quaternion(self):
        """Test that output is a unit quaternion."""
        for roll in [0, math.pi / 4, math.pi / 2]:
            for pitch in [0, math.pi / 4, math.pi / 2]:
                for yaw in [0, math.pi / 4, math.pi / 2]:
                    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                    magnitude = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
                    assert abs(magnitude - 1.0) < 1e-10


class TestQuaternionToEuler:
    """Tests for quaternion_to_euler function."""

    def test_identity(self):
        """Test identity quaternion produces zero rotation."""
        roll, pitch, yaw = quaternion_to_euler(0, 0, 0, 1)
        assert abs(roll) < 1e-10
        assert abs(pitch) < 1e-10
        assert abs(yaw) < 1e-10

    def test_roundtrip(self):
        """Test euler -> quaternion -> euler roundtrip."""
        test_cases = [
            (0, 0, 0),
            (0.1, 0.2, 0.3),
            (math.pi / 4, 0, 0),
            (0, math.pi / 4, 0),
            (0, 0, math.pi / 4),
        ]
        for original_roll, original_pitch, original_yaw in test_cases:
            qx, qy, qz, qw = euler_to_quaternion(
                original_roll, original_pitch, original_yaw
            )
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
            assert abs(roll - original_roll) < 1e-10
            assert abs(pitch - original_pitch) < 1e-10
            assert abs(yaw - original_yaw) < 1e-10

    def test_gimbal_lock(self):
        """Test behavior near gimbal lock (pitch = +-90 degrees)."""
        qx, qy, qz, qw = euler_to_quaternion(0, math.pi / 2, 0)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        assert abs(pitch - math.pi / 2) < 1e-10


class TestCarlaToRosPosition:
    """Tests for carla_to_ros_position function."""

    def test_origin(self):
        """Test origin stays at origin."""
        x, y, z = carla_to_ros_position(0, 0, 0)
        assert x == 0
        assert y == 0
        assert z == 0

    def test_y_negation(self):
        """Test that Y axis is negated."""
        x, y, z = carla_to_ros_position(1, 2, 3)
        assert x == 1
        assert y == -2
        assert z == 3

    def test_negative_values(self):
        """Test with negative values."""
        x, y, z = carla_to_ros_position(-1, -2, -3)
        assert x == -1
        assert y == 2
        assert z == -3


class TestCarlaToRosRotation:
    """Tests for carla_to_ros_rotation function."""

    def test_zero_rotation(self):
        """Test zero rotation stays zero."""
        roll, pitch, yaw = carla_to_ros_rotation(0, 0, 0)
        assert roll == 0
        assert pitch == 0
        assert yaw == 0

    def test_degree_to_radian_conversion(self):
        """Test degrees are converted to radians."""
        roll, pitch, yaw = carla_to_ros_rotation(180, 0, 0)
        assert abs(roll - math.pi) < 1e-10
        assert pitch == 0
        assert yaw == 0

    def test_pitch_yaw_negation(self):
        """Test that pitch and yaw are negated."""
        roll, pitch, yaw = carla_to_ros_rotation(90, 90, 90)
        assert abs(roll - math.pi / 2) < 1e-10
        assert abs(pitch + math.pi / 2) < 1e-10
        assert abs(yaw + math.pi / 2) < 1e-10
