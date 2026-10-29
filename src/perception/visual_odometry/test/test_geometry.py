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
"""Tests for the rigid-transform helpers, the static TF tree and the frame grouper."""

import numpy as np
import pytest

from visual_odometry.geometry import (
    invert,
    make_transform,
    matrix_to_quat,
    quat_to_matrix,
    rotation_angle,
)
from visual_odometry.grouping import FrameGrouper
from visual_odometry.tf_tree import StaticTfTree

# base_link -> camera_pano_nn from the car's /tf_static (an optical frame looking forward)
Q_NN = [0.5, -0.5, 0.5, -0.5]


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q = -q if q[3] < 0 else q
        np.testing.assert_allclose(matrix_to_quat(quat_to_matrix(q)), q, atol=1e-9)


def test_optical_frame_axes():
    r = quat_to_matrix(Q_NN)
    # optical z = forward
    np.testing.assert_allclose(r @ [0, 0, 1], [1, 0, 0], atol=1e-9)
    # optical x = right
    np.testing.assert_allclose(r @ [1, 0, 0], [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, -1], atol=1e-9)  # optical y = down


def test_invert_and_angle():
    t = make_transform([1, 2, 3], [0, 0, np.sin(0.25), np.cos(0.25)])
    np.testing.assert_allclose(invert(t) @ t, np.eye(4), atol=1e-12)
    assert rotation_angle(t[:3, :3]) == pytest.approx(0.5)


def test_tf_tree_lookup_matches_chain():
    tree = StaticTfTree()
    t_fp_bl = make_transform([0, 0, 1.76], [0, 0, 0, 1])
    t_bl_nn = make_transform([0.241, 0, 0.217], Q_NN)
    t_bl_lidar = make_transform([0, 0, 0.35], [0, 0, np.sin(0.05), np.cos(0.05)])
    tree.add("base_footprint", "base_link", t_fp_bl)
    tree.add("base_link", "camera_pano_nn", t_bl_nn)
    tree.add("/base_link", "lidar_cc", t_bl_lidar)
    np.testing.assert_allclose(
        tree.lookup("base_footprint", "camera_pano_nn"), t_fp_bl @ t_bl_nn
    )
    np.testing.assert_allclose(
        tree.lookup("lidar_cc", "camera_pano_nn"),
        invert(t_bl_lidar) @ t_bl_nn,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        tree.lookup("camera_pano_nn", "camera_pano_nn"), np.eye(4)
    )
    with pytest.raises(LookupError):
        tree.lookup("map", "camera_pano_nn")


def _ms(x):
    return int(x * 1e6)


def test_grouper_matches_nearest_within_slop():
    g = FrameGrouper(["nn", "nw", "ne"], slop_ns=_ms(10))
    out = []
    # nw is +3 ms, ne is +21 ms (outside the 10 ms slop) relative to nn, 50 ms period
    for k in range(4):
        t = _ms(50 * k)
        out += g.add("nn", t, f"nn{k}")
        out += g.add("nw", t + _ms(3), f"nw{k}")
        out += g.add("ne", t + _ms(21), f"ne{k}")
    assert [s.stamp_ns for s in out] == [_ms(0), _ms(50), _ms(100), _ms(150)]
    assert [f[1] for f in out[1].frames[:2]] == ["nn1", "nw1"]
    assert out[1].frames[2] is None and not out[1].complete
    assert g.offset_stats()["nw"]["mean_ms"] == pytest.approx(3.0)


def test_grouper_waits_for_late_camera_and_times_out():
    g = FrameGrouper(["nn", "nw"], slop_ns=_ms(10), max_wait_ns=_ms(300))
    assert g.add("nn", _ms(0), "a") == []  # nw has nothing at/after 0 yet
    assert g.add("nn", _ms(50), "b") == []
    # nw at 52 decides both: 0 has no match, 50 matches 52
    sets = g.add("nw", _ms(52), "x")
    assert [(s.stamp_ns, s.complete) for s in sets] == [
        (_ms(0), False),
        (_ms(50), True),
    ]
    # nw stops publishing: reference frames are finalized once they are max_wait old
    assert g.add("nn", _ms(100), "c") == []
    sets = g.add("nn", _ms(450), "d")
    assert [(s.stamp_ns, s.complete) for s in sets] == [(_ms(100), False)]


def test_grouper_ignores_duplicate_stamps():
    g = FrameGrouper(["nn"], slop_ns=_ms(10))
    assert len(g.add("nn", _ms(0), "a")) == 1
    assert g.add("nn", _ms(0), "a") == []


def test_grouper_restarts_after_time_jumps_backwards():
    g = FrameGrouper(["nn"], slop_ns=_ms(10), max_wait_ns=_ms(300))
    assert len(g.add("nn", _ms(5000), "a")) == 1
    assert g.add("nn", _ms(4900), "stale") == []  # within max_wait: stale, dropped
    # bag looped
    assert [s.stamp_ns for s in g.add("nn", _ms(100), "loop")] == [_ms(100)]
