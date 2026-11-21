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
"""Bag round trip, toolbox folder handling, and GICP motion on synthetic data."""

import json

import numpy as np
import pytest
import rosbag2_py
from conftest import make_cloud
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from lidar_cam_calib import bag_io, toolbox
from lidar_cam_calib.config import load_config, resolve
from lidar_cam_calib.cutting import cut_segment, segment_name

LIDAR = "/lidar_cc/velodyne_points"
IMAGE = "/camera_pano_nn/image_rect_compressed"


def write_bag(path, seconds=4.0):
    w = bag_io.open_writer(path)
    for i, (name, typ) in enumerate(
        [
            (LIDAR, "sensor_msgs/msg/PointCloud2"),
            (IMAGE, "sensor_msgs/msg/CompressedImage"),
            ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ]
    ):
        w.create_topic(rosbag2_py.TopicMetadata(i, name, typ, "cdr"))
    t0 = 1_000_000_000_000
    w.write("/tf_static", serialize_message(TFMessage()), t0)
    for k in range(int(seconds * 20)):
        stamp = t0 + k * 50_000_000
        h, wd = 4, 3
        xyz = np.full((h * wd, 3), 5.0)
        xyz[0] = np.nan
        cloud = make_cloud(xyz, np.tile(np.arange(wd), h), np.zeros(h * wd), h, wd)
        w.write(LIDAR, serialize_message(cloud), stamp)
        if k % 2 == 0:
            img = CompressedImage(format="jpeg", data=b"not-a-real-jpeg")
            w.write(IMAGE, serialize_message(img), stamp)
    del w


def test_find_read_and_cut(tmp_path):
    src = tmp_path / "bags" / "drive1"
    write_bag(src)
    (tmp_path / "bags" / "loose.mcap").write_bytes(b"")  # found, but unreadable
    found = bag_io.find_bags(tmp_path / "bags")
    assert found == [src, tmp_path / "bags" / "loose.mcap"]

    info = bag_io.read_info(src)
    assert (
        info.name == "drive1"
        and info.counts[LIDAR] == 80
        and info.duration_s == pytest.approx(3.95)
    )

    out = tmp_path / "out" / "creep" / segment_name(info.name, "creep", 1.0)
    assert out.name == "drive1__creep_00001p0s"
    stats = cut_segment(info, 1.0, 2.0, out, LIDAR, [IMAGE], ["/tf_static"], 0.05)
    assert stats["messages"][LIDAR] == 21 and stats["messages"][IMAGE] == 11
    assert stats["messages"]["/tf_static"] == 1 and stats["lidar_time"] == "rows"

    cut = bag_io.read_info(out)
    assert set(cut.topics) == {LIDAR, IMAGE, "/tf_static"}
    reader = bag_io.open_reader(out, [LIDAR])
    _, data, _ = reader.read_next()
    pts = point_cloud2.read_points(deserialize_message(data, PointCloud2))
    assert len(pts) == 11 and pts["time"].max() == pytest.approx(0.0375)

    # The output folder is marked, so a later scan of a parent folder skips it
    (tmp_path / "out" / bag_io.OUTPUT_MARKER).touch()
    assert bag_io.find_bags(tmp_path) == [src, tmp_path / "bags" / "loose.mcap"]


def test_make_trial_links_files_and_sets_init(tmp_path):
    base = tmp_path / "base"
    base.mkdir()
    (base / "a.png").write_bytes(b"x")
    toolbox.write_calib(
        base,
        {
            "meta": {"bag_names": ["a", "b", "c"]},
            "results": {"T_lidar_camera": [0] * 7},
        },
    )
    trial = tmp_path / "trials" / "loo_b"
    toolbox.make_trial(base, trial, ["a", "c"], [1, 2, 3, 0, 0, 0, 1])
    assert (trial / "a.png").is_symlink() and (trial / "a.png").read_bytes() == b"x"
    calib = json.loads((trial / "calib.json").read_text())
    assert calib["meta"]["bag_names"] == ["a", "c"]
    assert calib["results"] == {"init_T_lidar_camera": [1, 2, 3, 0, 0, 0, 1]}
    assert toolbox.trial_result(trial) is None


def test_rectified_intrinsics_use_p_not_k():
    cfg = load_config()
    fx, fy, cx, cy = toolbox.rectified_intrinsics(
        resolve(cfg["intrinsics_dir"]) / "camera_pano_nn.yaml"
    )
    assert (fx, fy, cx, cy) == pytest.approx(
        (1021.703735, 1027.313965, 645.281916, 535.906307)
    )


def test_headless_prefix(monkeypatch):
    monkeypatch.setenv("DISPLAY", ":0")
    assert toolbox.headless_prefix("auto") == []
    assert toolbox.headless_prefix("never") == []


def _scene(rng):
    """Box room with pillars: structure in every direction."""
    pts = []
    for axis, val in [(0, 15.0), (0, -15.0), (1, 12.0), (1, -12.0)]:
        p = rng.uniform([-15, -12, -1.5], [15, 12, 4], (3000, 3))
        p[:, axis] = val
        pts.append(p)
    ground = rng.uniform([-15, -12, -1.8], [15, 12, -1.8], (4000, 3))
    pts.append(ground)
    for cx, cy in [(5, 3), (-6, 4), (3, -7), (-4, -5)]:
        a = rng.uniform(0, 2 * np.pi, 800)
        pts.append(
            np.stack(
                [cx + 0.4 * np.cos(a), cy + 0.4 * np.sin(a), rng.uniform(-1.8, 3, 800)],
                1,
            )
        )
    return np.vstack(pts)


def test_gicp_track_recovers_motion():
    pytest.importorskip("small_gicp")
    from lidar_cam_calib import motion

    rng = np.random.default_rng(0)
    world = _scene(rng)
    xs = [0.0, 0.0, 0.0, 0.1, 0.3, 0.6]  # still, then creeping forward
    scans = [
        (0.25 * i, world - [x, 0, 0] + rng.normal(0, 0.01, world.shape))
        for i, x in enumerate(xs)
    ]
    cfg = load_config()["motion"]["gicp"]
    track = motion.gicp_track(scans, cfg)
    assert track.reliable.all()
    np.testing.assert_allclose(track.trans, np.diff(xs), atol=0.01)
