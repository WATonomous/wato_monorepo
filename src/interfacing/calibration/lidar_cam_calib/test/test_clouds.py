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
from conftest import make_cloud
from sensor_msgs_py import point_cloud2

from lidar_cam_calib.clouds import clean_cloud, rebuild_time, xyz


def test_rebuilds_time_from_rows_and_drops_nan(organized_cloud):
    out, stats = clean_cloud(organized_cloud, scan_period=0.05)
    assert stats == {"points_in": 12, "points_out": 11, "time": "rows"}
    pts = point_cloud2.read_points(out)
    assert out.height == 1 and out.width == 11 and out.is_dense
    # 4 firings -> times 0, 0.0125, 0.025, 0.0375; point 4 (row 1) was NaN
    expected = np.repeat(np.arange(4) * 0.0125, 3)
    np.testing.assert_allclose(pts["time"], np.delete(expected, 4), atol=1e-6)
    assert np.isfinite(pts["x"]).all()
    assert [f.name for f in out.fields] == ["x", "y", "z", "intensity", "ring", "time"]


def test_keeps_a_valid_time_field():
    xyz_pts = np.array([[5.0, 0, 0], [5.0, 1, 0], [5.0, 2, 0]])
    times = np.array([-0.05, -0.02, 0.0])  # driver default: relative to the last packet
    out, stats = clean_cloud(make_cloud(xyz_pts, np.zeros(3), times, 1, 3), 0.05)
    assert stats["time"] == "original"
    np.testing.assert_allclose(point_cloud2.read_points(out)["time"], times, atol=1e-7)


def test_columns_when_each_column_is_a_firing():
    h, w = 3, 4  # rows are rings
    rings = np.repeat(np.arange(h), w)
    cloud = make_cloud(np.ones((h * w, 3)), rings, np.zeros(h * w), h, w)
    arr = np.array(point_cloud2.read_points(cloud)).reshape(-1)
    t, method = rebuild_time(arr, h, w, 0.04)
    assert method == "columns"
    np.testing.assert_allclose(t[:4], [0, 0.01, 0.02, 0.03], atol=1e-6)


def test_azimuth_for_unorganized_clouds():
    # Clockwise spin: azimuth goes 0 -> -90 -> 180 -> 90 deg over the scan
    pts = np.array([[1.0, 0, 0], [0, -1.0, 0], [-1.0, 0, 0], [0, 1.0, 0]])
    cloud = make_cloud(pts, np.zeros(4), np.zeros(4), 1, 4)
    arr = np.array(point_cloud2.read_points(cloud)).reshape(-1)
    t, method = rebuild_time(arr, 1, 4, 0.1)
    assert method == "azimuth"
    np.testing.assert_allclose(t, [0, 0.025, 0.05, 0.075], atol=1e-6)


def test_xyz_skips_nan(organized_cloud):
    assert xyz(organized_cloud).shape == (11, 3)
