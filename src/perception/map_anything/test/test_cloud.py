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
import numpy as np
from scipy.spatial.transform import Rotation

from map_anything.cloud import pack_xyzrgb, rig_cloud, voxel_downsample


def transform(rotvec, translation):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    t[:3, 3] = translation
    return t


OPTICAL = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])
FRONT = transform([0, 0, 0], [0.2, 0.0, 1.5]) @ OPTICAL
LEFT = transform([0, 0, np.pi / 2], [0.0, 0.2, 1.5]) @ OPTICAL


def camera_view(rig_points, rig_from_camera, world_from_rig):
    """What a perfect model would output for points seen by one camera: (h=1, w, 3) world points."""
    world = rig_points @ world_from_rig[:3, :3].T + world_from_rig[:3, 3]
    return world[None], world_from_rig @ rig_from_camera


def test_points_return_to_rig_frame():
    world_from_rig = transform([0.1, -0.2, 0.7], [10.0, -3.0, 2.0])
    front_pts = np.array([[8.0, 0.5, 1.0], [12.0, -1.0, 0.2]])
    left_pts = np.array([[0.3, 6.0, 1.2]])
    front_world, front_pose = camera_view(front_pts, FRONT, world_from_rig)
    left_world, left_pose = camera_view(left_pts, LEFT, world_from_rig)
    colors = [
        np.array([[[255, 0, 0], [0, 255, 0]]], np.uint8),
        np.array([[[0, 0, 255]]], np.uint8),
    ]
    points, out_colors = rig_cloud(
        [front_world, left_world],
        [np.ones((1, 2), bool), np.ones((1, 1), bool)],
        colors,
        [front_pose, left_pose],
        [FRONT, LEFT],
    )
    np.testing.assert_allclose(points, np.vstack([front_pts, left_pts]), atol=1e-5)
    np.testing.assert_array_equal(out_colors, [[255, 0, 0], [0, 255, 0], [0, 0, 255]])


def test_model_placement_of_cameras_is_not_used():
    # The model puts the left camera 3 m off where the front camera implies; its own points stay
    # consistent with its own pose, so they must still land where they are on the rig.
    world_from_rig = np.eye(4)
    left_pts = np.array([[0.3, 6.0, 1.2]])
    left_world, left_pose = camera_view(left_pts, LEFT, world_from_rig)
    shift = transform([0, 0, 0.3], [3.0, 0.0, 0.0])
    left_world = left_world @ shift[:3, :3].T + shift[:3, 3]
    points, _ = rig_cloud(
        [left_world],
        [np.ones((1, 1), bool)],
        [np.zeros((1, 1, 3), np.uint8)],
        [shift @ left_pose],
        [LEFT],
    )
    np.testing.assert_allclose(points, left_pts, atol=1e-5)


def test_mask_and_range():
    world_from_rig = np.eye(4)
    pts = np.array([[5.0, 0.0, 1.5], [50.0, 0.0, 1.5], [6.0, 1.0, 1.5]])
    world, pose = camera_view(pts, FRONT, world_from_rig)
    points, _ = rig_cloud(
        [world],
        [np.array([[True, True, False]])],
        [np.zeros((1, 3, 3), np.uint8)],
        [pose],
        [FRONT],
        max_range=20.0,
    )
    np.testing.assert_allclose(points, pts[:1], atol=1e-5)


def test_voxel_keeps_one_point_per_voxel():
    pts = np.array([[0.01, 0.01, 0.0], [0.02, 0.03, 0.0], [0.5, 0.0, 0.0]])
    cols = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]], np.uint8)
    out, out_cols = voxel_downsample(pts, cols, 0.2)
    np.testing.assert_allclose(out, pts[[0, 2]])
    np.testing.assert_array_equal(out_cols, cols[[0, 2]])


def test_pack_layout():
    data = pack_xyzrgb(
        np.array([[1.0, 2.0, 3.0]], np.float32), np.array([[10, 20, 30]], np.uint8)
    )
    assert len(data) == 16
    xyz = np.frombuffer(data[:12], "<f4")
    rgb = np.frombuffer(data[12:], "<u4")[0]
    np.testing.assert_allclose(xyz, [1, 2, 3])
    assert rgb == (10 << 16) | (20 << 8) | 30
