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
"""VoCore and odometry message tests against a fake cuvslam module (no GPU needed)."""

import sys
import types

import numpy as np
import pytest

from visual_odometry.geometry import make_transform, quat_to_matrix
from visual_odometry.vo_core import CameraSpec, TrackResult


class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _fake_cuvslam(poses):
    """A stand-in for the cuvslam module that returns the given world_from_rig poses in order."""
    mod = types.ModuleType("cuvslam")
    calls = {"track": [], "rig": None}

    class Tracker:
        OdometryConfig = lambda **kw: _Obj(**kw)  # noqa: E731
        OdometryMode = _Obj(Multicamera="multicamera")
        MulticameraMode = _Obj(
            Performance="perf", Precision="precision", Moderate="moderate"
        )

        def __init__(self, rig, cfg):
            calls["rig"], calls["cfg"] = rig, cfg

        def track(self, stamp, images, masks=None):
            calls["track"].append((stamp, images, masks))
            pose = poses[len(calls["track"]) - 1]
            if pose is None:
                return _Obj(world_from_rig=None), None
            t, q = pose
            cov = np.diag([1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6]).ravel()
            return _Obj(
                world_from_rig=_Obj(
                    pose=_Obj(translation=t, rotation=q), covariance_xyz_rpy=cov
                )
            ), None

        def get_last_landmarks(self):
            return [1, 2, 3]

        def get_last_observations(self, i):
            return [1] * (i + 1)

    mod.Tracker = Tracker
    mod.Rig = lambda: _Obj()
    mod.Camera = lambda: _Obj()
    mod.Pose = lambda rotation, translation: _Obj(
        rotation=rotation, translation=translation
    )
    mod.Distortion = type(
        "Distortion",
        (),
        {"Model": _Obj(Pinhole=0), "__init__": lambda self, m, p: None},
    )
    mod.set_verbosity = lambda v: None
    return mod, calls


@pytest.fixture
def fake(monkeypatch):
    def install(poses):
        mod, calls = _fake_cuvslam(poses)
        monkeypatch.setitem(sys.modules, "cuvslam", mod)
        return calls

    return install


def _specs():
    t_nn = make_transform([0.241, 0, 0.217], [0.5, -0.5, 0.5, -0.5])
    p = [1021.7, 0, 645.3, 0, 0, 1027.3, 535.9, 0, 0, 0, 1, 0]
    return [
        CameraSpec.from_projection("nn", 1280, 1024, p, t_nn),
        CameraSpec.from_projection("nw", 1280, 1024, p, t_nn),
    ]


def test_rig_and_config_are_built_from_specs(fake):
    from visual_odometry.vo_core import VoCore

    calls = fake([])
    VoCore(_specs(), multicam_mode="moderate", rectified_stereo_camera=True)
    cam = calls["rig"].cameras[0]
    assert list(cam.focal) == pytest.approx([1021.7, 1027.3])
    np.testing.assert_allclose(
        quat_to_matrix(cam.rig_from_camera.rotation),
        quat_to_matrix([0.5, -0.5, 0.5, -0.5]),
    )
    assert (
        calls["cfg"].multicam_mode == "moderate"
        and calls["cfg"].rectified_stereo_camera is True
    )
    with pytest.raises(ValueError):
        VoCore(_specs(), multicam_mode="fast")


def test_track_results_masks_and_stamp_order(fake):
    from visual_odometry.vo_core import VoCore

    calls = fake([([1.0, 0, 0], [0, 0, 0, 1]), None])
    mask = np.zeros((1024, 1280), np.uint8)
    mask[-10:] = 255
    core = VoCore(_specs(), masks=[mask, mask])
    img = np.zeros((1024, 1280), np.uint8)
    res = core.track(100, [img, None])
    assert (
        res.valid and res.world_from_rig[0, 3] == 1.0 and res.covariance.shape == (6, 6)
    )
    assert res.num_landmarks == 3 and res.num_observations == [1, 2]
    _, images, masks = calls["track"][0]
    # missing camera -> empty
    assert images[1].size == 0 and masks[1].size == 0 and masks[0] is mask
    assert not core.track(200, [img, img]).valid
    with pytest.raises(ValueError):
        core.track(200, [img, img])  # not strictly increasing


def test_odometry_builder_twist_and_reset():
    from visual_odometry.ros_io import OdometryBuilder

    b = OdometryBuilder("vo_odom", "base_link", max_twist_dt=0.2)
    yaw = 0.1
    r0 = TrackResult(stamp_ns=0, world_from_rig=np.eye(4))
    r1 = TrackResult(
        stamp_ns=50_000_000,
        world_from_rig=make_transform(
            [0.5, 0, 0], [0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]
        ),
    )
    assert b.build(r0).twist.twist.linear.x == 0.0
    msg = b.build(r1)
    assert (msg.header.frame_id, msg.child_frame_id) == ("vo_odom", "base_link")
    assert msg.header.stamp.nanosec == 50_000_000
    assert msg.twist.twist.linear.x == pytest.approx(10.0)
    assert msg.twist.twist.angular.z == pytest.approx(2.0)
    assert b.build(TrackResult(stamp_ns=100_000_000)) is None  # lost
    r3 = TrackResult(stamp_ns=150_000_000, world_from_rig=np.eye(4))
    assert b.build(r3).twist.twist.linear.x == 0.0  # no differencing across the loss
