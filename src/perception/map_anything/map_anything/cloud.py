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
"""Dense MapAnything geometry as a coloured point cloud in the rig frame (ROS-free, GPU-free).

The model predicts every pixel's 3D point in one arbitrary world frame (the first view's). Each
camera's points are brought into its own camera frame with the pose the model predicted for that
camera, then onto the rig with the camera's known extrinsic. The model's placement of the cameras
relative to each other is not used, the same choice as ``rig.relative_rig_pose_per_camera``: it is
the least reliable part of its output when cameras overlap little.
"""

from typing import Sequence, Tuple

import numpy as np

from visual_odometry.geometry import invert


def voxel_downsample(
    points: np.ndarray, colors: np.ndarray, size: float
) -> Tuple[np.ndarray, np.ndarray]:
    """One point per voxel of edge ``size`` (the first one), keeping its colour. ``size <= 0``: all."""
    if size <= 0 or not len(points):
        return points, colors
    keys = np.floor(points / size).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    keep.sort()
    return points[keep], colors[keep]


def rig_cloud(
    world_points: Sequence[np.ndarray],
    valid: Sequence[np.ndarray],
    rgb: Sequence[np.ndarray],
    world_from_cameras: Sequence[np.ndarray],
    rig_from_cameras: Sequence[np.ndarray],
    max_range: float = 0.0,
    voxel: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Points of several cameras of one instant in the rig frame, with their colours.

    Args:
        world_points: per camera, (h, w, 3) points in the model's world frame.
        valid: per camera, (h, w) mask of pixels to keep (not sky, not a depth edge).
        rgb: per camera, (h, w, 3) uint8 colours.
        world_from_cameras: per camera, the 4x4 pose the model predicted for it.
        rig_from_cameras: per camera, its 4x4 extrinsic on the rig.
        max_range: drop points farther than this from their camera (m); 0 keeps all.
        voxel: keep one point per voxel of this size (m); 0 keeps all.

    Returns:
        (N, 3) float32 points in the rig frame and (N, 3) uint8 colours.
    """
    if not (
        len(world_points)
        == len(valid)
        == len(rgb)
        == len(world_from_cameras)
        == len(rig_from_cameras)
    ):
        raise ValueError("need points, mask, colours, pose and extrinsic per camera")
    points, colors = [], []
    for pts, mask, col, world_from_camera, rig_from_camera in zip(
        world_points, valid, rgb, world_from_cameras, rig_from_cameras
    ):
        camera_from_world = invert(np.asarray(world_from_camera, dtype=float))
        in_camera = (
            pts[mask].astype(np.float64) @ camera_from_world[:3, :3].T
            + camera_from_world[:3, 3]
        )
        keep = np.isfinite(in_camera).all(axis=1)
        if max_range > 0:
            keep &= np.linalg.norm(in_camera, axis=1) < max_range
        rig_from_camera = np.asarray(rig_from_camera, dtype=float)
        points.append(
            in_camera[keep] @ rig_from_camera[:3, :3].T + rig_from_camera[:3, 3]
        )
        colors.append(col[mask][keep])
    if not points:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
    out_points, out_colors = voxel_downsample(
        np.concatenate(points), np.concatenate(colors), voxel
    )
    return out_points.astype(np.float32), out_colors.astype(np.uint8)


def pack_xyzrgb(points: np.ndarray, colors: np.ndarray) -> bytes:
    """Points as the data of a PointCloud2 with float32 x, y, z and rgb packed in a float32.

    The layout RViz, Foxglove and PCL's PointXYZRGB read: 16 bytes per point, rgb = (r << 16) |
    (g << 8) | b in the low three bytes.
    """
    data = np.empty(len(points), dtype=[("xyz", "<f4", 3), ("rgb", "<u4")])
    data["xyz"] = points
    c = colors.astype(np.uint32)
    data["rgb"] = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
    return data.tobytes()
