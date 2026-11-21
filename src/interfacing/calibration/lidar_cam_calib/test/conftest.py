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
import sys
from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

# The package runs from the source tree; make `import lidar_cam_calib` work in tests
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def make_cloud(
    xyz: np.ndarray, rings: np.ndarray, times: np.ndarray, height: int, width: int
) -> PointCloud2:
    """Velodyne-style XYZIRT cloud (same layout as velodyne_pointcloud's)."""
    dtype = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
        ("time", "<f4"),
    ]
    arr = np.zeros(len(xyz), dtype=dtype)
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr["intensity"] = 10.0
    arr["ring"] = rings
    arr["time"] = times
    msg = PointCloud2()
    msg.height, msg.width = height, width
    offs = [0, 4, 8, 12, 16, 18]
    types = [PointField.FLOAT32] * 4 + [PointField.UINT16, PointField.FLOAT32]
    msg.fields = [
        PointField(name=n, offset=o, datatype=t, count=1)
        for (n, _), o, t in zip(dtype, offs, types)
    ]
    msg.point_step = 22
    msg.row_step = 22 * width
    msg.is_dense = False
    msg.data = arr.tobytes()
    return msg


@pytest.fixture
def organized_cloud():
    """4 firings x 3 rings, time all zero, one NaN point (as recorded on Eve)."""
    h, w = 4, 3
    xyz = np.array([[10.0 + r, float(c), 0.5 * r] for c in range(h) for r in range(w)])
    xyz[4] = np.nan
    rings = np.tile(np.arange(w), h)
    return make_cloud(xyz, rings, np.zeros(h * w), h, w)
