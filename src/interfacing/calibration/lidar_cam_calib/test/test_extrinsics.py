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
import pytest

from lidar_cam_calib import extrinsics as ex

URDF = """<robot name="t">
  <link name="base_link"/><link name="lidar_cc"/><link name="camera_pano_nn"/>
  <joint name="a" type="fixed"><parent link="base_link"/><child link="lidar_cc"/>
    <origin xyz="0 0 0.35" rpy="-0.006167 0.042198 0.10472"/></joint>
  <joint name="b" type="fixed"><parent link="base_link"/><child link="camera_pano_nn"/>
    <origin xyz="0.240676 0 0.217" rpy="-1.5707963 0 -1.5707963"/></joint>
</robot>"""


def test_vec7_and_rpy_round_trips():
    T = ex.T_from_xyz_rpy([0.1, -0.2, 0.3], [0.1, -0.4, 2.5])
    np.testing.assert_allclose(ex.T_from_vec7(ex.vec7_from_T(T)), T, atol=1e-12)
    xyz, rpy = ex.xyz_rpy_from_T(T)
    np.testing.assert_allclose(ex.T_from_xyz_rpy(xyz, rpy), T, atol=1e-12)


def test_urdf_lookup_matches_hand_computed_nominal_guess():
    g = ex.FrameGraph.from_urdf_xml(URDF)
    T = g.lookup("lidar_cc", "camera_pano_nn")
    # Same numbers as the nominal pano_nn guess computed from the real xacro
    np.testing.assert_allclose(T[:3, 3], [0.2448, -0.0244, -0.1229], atol=1e-4)
    q = ex.vec7_from_T(T)[3:]
    q_ref = np.array([0.46046, -0.51703, 0.53376, -0.48555])
    assert min(np.abs(q - q_ref).max(), np.abs(q + q_ref).max()) < 1e-4
    np.testing.assert_allclose(
        g.lookup("camera_pano_nn", "lidar_cc") @ T, np.eye(4), atol=1e-12
    )


def test_missing_frame_raises():
    with pytest.raises(KeyError):
        ex.FrameGraph.from_urdf_xml(URDF).lookup("lidar_cc", "nowhere")


def test_mean_difference_and_perturb():
    T = ex.T_from_xyz_rpy([1, 2, 3], [0.1, 0.2, 0.3])
    rng = np.random.default_rng(1)
    assert ex.difference(T, ex.perturb(T, 0.0, 0.0, rng)) == pytest.approx(
        (0.0, 0.0), abs=1e-9
    )
    P = ex.perturb(T, 0.03, 1.0, rng)
    dt, dr = ex.difference(T, P)
    assert 0 < dt < 0.2 and 0 < dr < 6
    M = ex.mean_transform([T, T, T])
    assert ex.difference(T, M) == pytest.approx((0.0, 0.0), abs=1e-9)
