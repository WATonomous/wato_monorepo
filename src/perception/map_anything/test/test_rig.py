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
from scipy.spatial.transform import Rotation

from map_anything.model import centred_view
from map_anything.rig import relative_rig_pose, relative_rig_pose_per_camera, rig_pose
from visual_odometry.geometry import invert


def transform(rotvec, translation):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    t[:3, 3] = translation
    return t


# A front-facing and a left-facing camera (optical frames) on a rig with x forward, z up.
FRONT = transform([0, 0, 0], [0.2, 0.0, 1.5]) @ np.array(
    [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]]
)
LEFT = transform([0, 0, np.pi / 2], [0.0, 0.2, 1.5]) @ np.array(
    [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]]
)
EXTRINSICS = [FRONT, LEFT]
# The rig drives 5 m forward while turning 10 degrees left.
MOTION = transform([0, 0, np.radians(10)], [5.0, 0.4, 0.0])


def camera_poses(world_from_rig):
    """What a perfect model would predict: every camera's pose, in an arbitrary world frame."""
    world = transform(
        [0.1, -0.2, 0.3], [3.0, -1.0, 2.0]
    )  # the model's frame is not the rig's
    return [world @ world_from_rig @ extrinsic for extrinsic in EXTRINSICS]


def test_rig_pose_recovers_the_rig_from_consistent_cameras():
    world_from_rig = transform([0, 0, 0.5], [1.0, 2.0, 0.0])
    world = transform([0.1, -0.2, 0.3], [3.0, -1.0, 2.0])
    estimate = rig_pose(camera_poses(world_from_rig), EXTRINSICS)
    np.testing.assert_allclose(
        estimate.world_from_rig, world @ world_from_rig, atol=1e-9
    )
    assert estimate.trans_spread == pytest.approx(0.0, abs=1e-9)
    assert estimate.cameras == 2


def test_both_fits_recover_the_rig_motion():
    a, b = camera_poses(np.eye(4)), camera_poses(MOTION)
    joint = relative_rig_pose(a, EXTRINSICS, b, EXTRINSICS)
    per_camera = relative_rig_pose_per_camera(a, b, EXTRINSICS)
    np.testing.assert_allclose(joint.rig_a_from_rig_b, MOTION, atol=1e-9)
    np.testing.assert_allclose(per_camera.rig_a_from_rig_b, MOTION, atol=1e-9)
    assert per_camera.trans_spread == pytest.approx(0.0, abs=1e-9)
    assert per_camera.rot_spread == pytest.approx(0.0, abs=1e-6)


def test_a_misplaced_camera_shows_in_the_joint_spread_but_not_in_the_rotation():
    """A model that puts the left camera 3 m away from where it is (at both instants alike)."""
    shift = transform([0, 0, 0], [0.0, 3.0, 0.0])
    a, b = camera_poses(np.eye(4)), camera_poses(MOTION)
    a[1], b[1] = a[1] @ shift, b[1] @ shift
    per_camera = relative_rig_pose_per_camera(a, b, EXTRINSICS)
    joint = relative_rig_pose(a, EXTRINSICS, b, EXTRINSICS)
    # the rotation is still exact for both; only the joint fit's spread reports the inconsistency
    np.testing.assert_allclose(
        per_camera.rig_a_from_rig_b[:3, :3], MOTION[:3, :3], atol=1e-9
    )
    assert joint.trans_spread > 1.0


def test_spread_reports_disagreeing_cameras():
    a, b = camera_poses(np.eye(4)), camera_poses(MOTION)
    b[1] = b[1] @ transform(
        [0, 0, 0], [0.0, 0.0, 1.0]
    )  # the left camera says 1 m further
    estimate = relative_rig_pose_per_camera(a, b, EXTRINSICS)
    assert estimate.trans_spread == pytest.approx(
        0.5, abs=1e-6
    )  # two estimates 1 m apart


def test_fits_need_matching_inputs():
    a = camera_poses(np.eye(4))
    with pytest.raises(ValueError):
        relative_rig_pose_per_camera(a, a[:1], EXTRINSICS)
    with pytest.raises(ValueError):
        rig_pose([], [])


def test_centred_view_puts_the_principal_point_in_the_middle():
    rgb = np.zeros((1024, 1280, 3), dtype=np.uint8)
    intrinsics = np.array([[1021.7, 0, 645.3], [0, 1027.3, 535.9], [0, 0, 1.0]])
    view = centred_view(rgb, intrinsics, cut_bottom=110)
    height, width = view.rgb.shape[:2]
    assert height == 2 * round(1024 - 110 - 535.9)  # limited by the cut at the bottom
    assert abs(view.intrinsics[0, 2] - width / 2) <= 1.0
    assert abs(view.intrinsics[1, 2] - height / 2) <= 1.0
    assert view.intrinsics[0, 0] == intrinsics[0, 0]  # focal lengths untouched
    # a pixel keeps its ray: the principal point still looks at the same image content
    assert rgb.shape[0] - 110 >= round(535.9 - view.intrinsics[1, 2]) + height


def test_centred_view_rejects_a_principal_point_outside_what_remains():
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    intrinsics = np.array([[100.0, 0, 50.0], [0, 100.0, 80.0], [0, 0, 1.0]])
    with pytest.raises(ValueError):
        centred_view(rgb, intrinsics, cut_bottom=30)


def test_invert_is_consistent_with_the_test_helper():
    np.testing.assert_allclose(invert(MOTION) @ MOTION, np.eye(4), atol=1e-12)
