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
"""Rigid-transform helpers (numpy only). Quaternions are (x, y, z, w) like ROS and cuVSLAM."""

import numpy as np


def quat_to_matrix(q) -> np.ndarray:
    """Convert a unit quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w) with w >= 0."""
    r = np.asarray(r, dtype=float)
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        ]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
            (r[2, 1] - r[1, 2]) / s,
        ]
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
            (r[0, 2] - r[2, 0]) / s,
        ]
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [
            (r[0, 2] + r[2, 0]) / s,
            (r[1, 2] + r[2, 1]) / s,
            0.25 * s,
            (r[1, 0] - r[0, 1]) / s,
        ]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def make_transform(translation, quat) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a translation and a quaternion (x, y, z, w)."""
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quat)
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def split_transform(t: np.ndarray):
    """Split a 4x4 transform into (translation[3], quaternion[4] x, y, z, w)."""
    return t[:3, 3].copy(), matrix_to_quat(t[:3, :3])


def invert(t: np.ndarray) -> np.ndarray:
    """Invert a 4x4 rigid transform."""
    inv = np.eye(4)
    inv[:3, :3] = t[:3, :3].T
    inv[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return inv


def rotation_angle(r: np.ndarray) -> float:
    """Return the angle (rad) of a 3x3 rotation matrix."""
    return float(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
