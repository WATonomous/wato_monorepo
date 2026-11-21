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
"""Velodyne PointCloud2 clean-up for the calibration toolbox.

The toolbox's dynamic integration (`preprocess -d`) deskews every scan with a
per-point time field and divides by the last point's time. Clouds recorded on
Eve before the transform-node fix have `time = 0` everywhere, so this module
rebuilds it. It also drops NaN points: the driver publishes organized clouds
with NaN for missing returns, and the toolbox's distance filter lets NaN through.
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

MIN_TIME_SPAN = 1e-4  # s; a span below this means the time field is unusable

_OUT_FIELDS = [
    ("x", np.float32, PointField.FLOAT32),
    ("y", np.float32, PointField.FLOAT32),
    ("z", np.float32, PointField.FLOAT32),
    ("intensity", np.float32, PointField.FLOAT32),
    ("ring", np.uint16, PointField.UINT16),
    ("time", np.float32, PointField.FLOAT32),
]


def cloud_array(msg: PointCloud2) -> np.ndarray:
    """Structured array of all points in storage order (row-major for organized clouds)."""
    return np.array(point_cloud2.read_points(msg, skip_nans=False)).reshape(-1)


def xyz(msg: PointCloud2) -> np.ndarray:
    """Finite points as an Nx3 float64 array."""
    arr = cloud_array(msg)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def rebuild_time(arr: np.ndarray, height: int, width: int, scan_period: float):
    """Per-point time in [0, scan_period) from firing order, plus the method used.

    Organized Velodyne clouds hold one firing sequence (all rings) per row, so
    the row index is proportional to time. Unorganized clouds fall back to the
    azimuth, which decreases over a scan because the sensor spins clockwise.
    """
    n = arr.shape[0]
    names = arr.dtype.names
    if height > 1 and "ring" in names:
        rings = arr["ring"].reshape(height, width)
        if len(np.unique(rings[0])) > 1:  # each row spans the rings: rows are firings
            t = np.repeat(np.arange(height) / height, width)
            return (t * scan_period).astype(np.float32), "rows"
        t = np.tile(np.arange(width) / width, height)  # each column is a firing
        return (t * scan_period).astype(np.float32), "columns"
    x, y = arr["x"].astype(np.float64), arr["y"].astype(np.float64)
    finite = np.isfinite(x) & np.isfinite(y)
    t = np.zeros(n, np.float64)
    if finite.any():
        az = np.arctan2(y, x)
        az0 = az[np.argmax(finite)]  # first valid point starts the scan
        t = np.mod(az0 - az, 2 * np.pi) / (2 * np.pi)
        t[~finite] = 0.0
    return (t * scan_period).astype(np.float32), "azimuth"


def clean_cloud(msg: PointCloud2, scan_period: float) -> tuple[PointCloud2, dict]:
    """Unorganized copy with finite points only and a usable time field.

    Returns the new message and stats: points in/out and how time was obtained
    ("original", "rows", "columns" or "azimuth").
    """
    arr = cloud_array(msg)
    names = arr.dtype.names
    if "intensity" not in names:
        raise ValueError("point cloud has no intensity field; the toolbox needs one")

    times, method = None, "original"
    if "time" in names:
        times = arr["time"].astype(np.float32)
        span = float(np.nanmax(times) - np.nanmin(times)) if times.size else 0.0
        if not span >= MIN_TIME_SPAN:
            times = None
    if times is None:
        times, method = rebuild_time(arr, msg.height, msg.width, scan_period)

    keep = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
    out = np.zeros(int(keep.sum()), dtype=[(n, d) for n, d, _ in _OUT_FIELDS])
    for name, dtype, _ in _OUT_FIELDS:
        if name == "time":
            out[name] = times[keep]
        elif name in names:
            out[name] = arr[name][keep].astype(dtype)

    fields, offset = [], 0
    for name, dtype, datatype in _OUT_FIELDS:
        fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
        offset += np.dtype(dtype).itemsize

    new = PointCloud2()
    new.header = msg.header
    new.height = 1
    new.width = out.shape[0]
    new.fields = fields
    new.is_bigendian = False
    new.point_step = out.dtype.itemsize
    new.row_step = new.point_step * new.width
    new.is_dense = True
    new.data = out.tobytes()
    return new, {
        "points_in": int(arr.shape[0]),
        "points_out": int(out.shape[0]),
        "time": method,
    }
