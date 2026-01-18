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
"""Transform utilities for coordinate conversions."""

import math
from typing import Tuple


def euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion.

    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)

    Returns:
        Tuple of (qx, qy, qz, qw)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def quaternion_to_euler(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles.

    Args:
        qx: Quaternion x component
        qy: Quaternion y component
        qz: Quaternion z component
        qw: Quaternion w component

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def carla_to_ros_position(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Convert CARLA position (left-handed) to ROS position (right-handed).

    CARLA: X-forward, Y-right, Z-up
    ROS:   X-forward, Y-left, Z-up

    Args:
        x: CARLA X coordinate
        y: CARLA Y coordinate
        z: CARLA Z coordinate

    Returns:
        Tuple of (x, y, z) in ROS coordinates
    """
    return x, -y, z


def carla_to_ros_rotation(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> Tuple[float, float, float]:
    """
    Convert CARLA rotation (degrees, left-handed) to ROS rotation (radians, right-handed).

    Args:
        roll_deg: CARLA roll in degrees
        pitch_deg: CARLA pitch in degrees
        yaw_deg: CARLA yaw in degrees

    Returns:
        Tuple of (roll, pitch, yaw) in radians for ROS
    """
    return (
        math.radians(roll_deg),
        -math.radians(pitch_deg),
        -math.radians(yaw_deg),
    )
