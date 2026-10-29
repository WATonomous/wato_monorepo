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
"""Tests for virtual stereo rectification and pair self-calibration (synthetic pano pair)."""

import numpy as np
import pytest

from visual_odometry.geometry import make_transform
from visual_odometry.pair_calibration import calibrate_pair, rotate_about_axes
from visual_odometry.rectify import (
    RigPreprocessor,
    pair_frame,
    rectified_disparity,
    rectify_pair,
)
from visual_odometry.vo_core import CameraSpec

P = [1021.7, 0.0, 645.3, 0.0, 0.0, 1027.3, 535.9, 0.0, 0.0, 0.0, 1.0, 0.0]
# Nominal nn / nw poses in base_link (eve_roof_mount.xacro: radius 0.240676 m, 45 deg steps)
T_NN = make_transform([0.240676, 0.0, 0.217], [0.5, -0.5, 0.5, -0.5])
T_NW = make_transform(
    [0.170184, 0.170184, 0.217], [-0.65328148, 0.27059805, -0.27059805, 0.65328148]
)


def _spec(name, t, border=(0, 0, 0, 0)):
    return CameraSpec.from_projection(name, 1280, 1024, P, t, border=border)


def _project(spec, pts_rig):
    """Project rig-frame points into a camera; returns pixels and a visibility mask."""
    t = np.linalg.inv(spec.rig_from_camera)
    pc = pts_rig @ t[:3, :3].T + t[:3, 3]
    uv = np.column_stack(
        [
            spec.fx * pc[:, 0] / pc[:, 2] + spec.cx,
            spec.fy * pc[:, 1] / pc[:, 2] + spec.cy,
        ]
    )
    ok = (
        (pc[:, 2] > 0.5)
        & (uv[:, 0] >= 0)
        & (uv[:, 0] < spec.width)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] < spec.height)
    )
    return uv, ok


def _scene(n=4000, seed=1):
    """Points 3..80 m away, around the nn/nw overlap (22.5 deg left of forward), plus far points."""
    rng = np.random.default_rng(seed)
    yaw = np.radians(rng.uniform(10, 35, n))
    rng_m = np.concatenate([rng.uniform(3, 80, n - 200), np.full(200, 5000.0)])
    z = rng.uniform(-3, 8, n)
    return np.column_stack([rng_m * np.cos(yaw), rng_m * np.sin(yaw), z])


def _matches(a, b, pts):
    ua, oka = _project(a, pts)
    ub, okb = _project(b, pts)
    ok = oka & okb
    return ua[ok], ub[ok]


def test_from_projection_downscale_keeps_pixel_centres():
    s = CameraSpec.from_projection(
        "c", 1280, 1024, P, np.eye(4), downscale=2, border=(0, 110, 0, 0)
    )
    assert (s.width, s.height, s.border) == (640, 512, (0, 55, 0, 0))
    assert s.fx == pytest.approx(P[0] / 2)
    assert s.cx == pytest.approx((P[2] + 0.5) / 2 - 0.5)


def test_pair_frame_is_bisector_with_baseline_along_x():
    frame = pair_frame(_spec("nn", T_NN), _spec("nw", T_NW))
    assert frame.left_index == 1  # nw is left of nn
    assert frame.baseline == pytest.approx(0.184, abs=1e-3)
    z = frame.r_rig_rect[:, 2]
    assert np.degrees(np.arctan2(z[1], z[0])) == pytest.approx(22.5, abs=0.1)
    np.testing.assert_allclose(
        frame.r_rig_rect.T @ frame.r_rig_rect, np.eye(3), atol=1e-12
    )


def test_rectified_pair_rows_align_and_disparity_matches_depth():
    a, b = _spec("nw", T_NW), _spec("nn", T_NN)
    pts = _scene()
    ua, ub = _matches(a, b, pts)
    dx, dy = rectified_disparity(a, b, ua, ub)
    assert np.abs(dy).max() < 1e-6
    assert dx.min() > -1e-6  # everything in front: positive disparity
    rect = rectify_pair(a, b)
    assert (
        rect.left.spec.name == "nw/rect_left"
        and rect.right.spec.name == "nn/rect_right"
    )
    rel = (
        np.linalg.inv(rect.left.spec.rig_from_camera) @ rect.right.spec.rig_from_camera
    )
    np.testing.assert_allclose(rel[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(rel[:3, 3], [0.184, 0, 0], atol=1e-3)


def test_masks_cover_pixels_without_source_data():
    rect = rectify_pair(
        _spec("nw", T_NW, border=(0, 110, 0, 0)),
        _spec("nn", T_NN, border=(0, 110, 0, 0)),
    )
    for view in (rect.left, rect.right):
        img = view.apply(np.full((1024, 1280), 200, np.uint8))
        # unmasked pixels all come from the source image
        assert np.all(img[view.mask == 0] > 0)
        assert 0.2 < (view.mask > 0).mean() < 0.8


def test_rig_preprocessor_orders_pairs_left_right():
    nn, nw = _spec("nn", T_NN), _spec("nw", T_NW)
    pre = RigPreprocessor([nn, nw], [["nw", "nn"]])
    assert [s.name for s in pre.specs] == ["nw/rect_left", "nn/rect_right"]
    assert pre.sources == [1, 0]
    out = pre.apply([np.zeros((1024, 1280), np.uint8), None])
    assert out[0] is None and out[1].shape == (1024, 1280)
    with pytest.raises(ValueError):
        RigPreprocessor([nn, nw], [["nw", "ss"]])


def test_calibrate_pair_recovers_rotation_error():
    nn, nw_true = _spec("nn", T_NN), _spec("nw", T_NW)
    ua, ub = _matches(nn, nw_true, _scene())
    # The configured nw rotation is off by ~1.5 deg (roll/pitch-like, as measured on the car)
    axes = pair_frame(nn, nw_true).r_rig_rect
    nw_cfg = rotate_about_axes(nw_true, axes, np.radians([0.8, 0.5, -0.7]))
    dx0, dy0 = rectified_disparity(nn, nw_cfg, ua, ub)
    assert np.median(np.abs(dy0)) > 5.0
    fit = calibrate_pair(nn, nw_cfg, ua, ub, far_percentile=2.0, far_disparity_px=0.0)
    dx, dy = rectified_disparity(nn, fit.corrected, ua, ub)
    assert np.median(np.abs(dy)) < 0.05
    # With the far points (5 km) pinned to ~0 px the along-epipolar rotation is recovered too
    err = fit.corrected.rig_from_camera[:3, :3].T @ nw_true.rig_from_camera[:3, :3]
    assert np.degrees(np.arccos(np.clip((np.trace(err) - 1) / 2, -1, 1))) < 0.05
    np.testing.assert_allclose(
        fit.corrected.rig_from_camera[:3, 3], nw_true.rig_from_camera[:3, 3]
    )
