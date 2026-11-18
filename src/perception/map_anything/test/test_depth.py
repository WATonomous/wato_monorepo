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

from map_anything.depth import sparse_depth, xyz_from_cloud

K = np.array([[100.0, 0, 50.0], [0, 100.0, 40.0], [0, 0, 1]])


def test_projects_to_the_right_pixel():
    depth = sparse_depth(np.array([[1.0, -0.5, 10.0]]), K, (80, 100))
    v, u = np.argwhere(depth > 0)[0]
    assert (u, v) == (60, 35)
    assert depth[v, u] == 10.0
    assert (depth > 0).sum() == 1


def test_nearest_point_wins():
    pts = np.array([[0.0, 0.0, 20.0], [0.0, 0.0, 5.0], [0.0, 0.0, 12.0]])
    depth = sparse_depth(pts, K, (80, 100))
    assert depth[40, 50] == 5.0


def test_range_and_frame():
    pts = np.array(
        [
            [0.0, 0.0, 0.2],  # too close
            [0.0, 0.0, 100.0],  # too far
            [0.0, 0.0, -5.0],  # behind
            [100.0, 0.0, 1.0],  # outside the image
            [np.nan, 0.0, 5.0],
        ]
    )
    assert not sparse_depth(pts, K, (80, 100)).any()


def test_xyz_from_cloud():
    data = np.zeros(2, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("pad", "u1", 6)])
    data["x"], data["y"], data["z"] = [1, 4], [2, 5], [3, 6]
    xyz = xyz_from_cloud(data.tobytes(), data.dtype.itemsize, 2)
    np.testing.assert_allclose(xyz, [[1, 2, 3], [4, 5, 6]])
