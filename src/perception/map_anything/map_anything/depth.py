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
"""Sparse depth maps from 3D points, e.g. lidar, as a geometric input to MapAnything (ROS-free).

MapAnything takes an optional per-view depth map (z in the camera frame, metres) next to the image
and intrinsics; pixels without a measurement are 0. Points must already be in the camera's optical
frame at the image's time (the client moves lidar points there).
"""

from typing import Tuple

import numpy as np


def sparse_depth(
    points: np.ndarray,
    intrinsics: np.ndarray,
    shape: Tuple[int, int],
    min_depth: float = 0.5,
    max_depth: float = 80.0,
) -> np.ndarray:
    """Depth map (h, w) float32 of points (N, 3) in the camera frame; 0 where no point falls.

    Each point goes to the pixel it projects into; where several do, the nearest wins (points behind
    a surface seen from a slightly different viewpoint, the lidar's, are hidden).
    """
    h, w = shape
    depth = np.zeros((h, w), np.float32)
    if not len(points):
        return depth
    points = np.asarray(points, dtype=np.float64)
    z = points[:, 2]
    ok = np.isfinite(points).all(axis=1) & (z > min_depth) & (z < max_depth)
    points, z = points[ok], z[ok]
    u = np.floor(intrinsics[0, 0] * points[:, 0] / z + intrinsics[0, 2]).astype(np.int64)
    v = np.floor(intrinsics[1, 1] * points[:, 1] / z + intrinsics[1, 2]).astype(np.int64)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[inside], v[inside], z[inside]
    order = np.argsort(-z)  # farthest first, so the nearest is written last
    depth[v[order], u[order]] = z[order]
    return depth


def xyz_from_cloud(data: bytes, point_step: int, count: int, offsets=(0, 4, 8)) -> np.ndarray:
    """(N, 3) float32 x, y, z from PointCloud2 data with float32 fields at the given offsets."""
    raw = np.frombuffer(data, np.uint8).reshape(count, point_step)
    out = np.empty((count, 3), np.float32)
    for k, off in enumerate(offsets):
        out[:, k] = raw[:, off : off + 4].copy().view("<f4")[:, 0]
    return out
