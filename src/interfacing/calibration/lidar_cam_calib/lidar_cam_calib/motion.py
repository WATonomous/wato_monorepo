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
"""Vehicle motion over a bag, used to find rest and creep periods.

Primary signal: LiDAR scan-to-scan GICP (small_gicp), which works on any bag
with the roof LiDAR. Optional signals, used when the bag has them:
  - IMU: confirms the car is truly still (nobody shifting inside, no rocking)
  - CAN wheel speed: cross-check, and speed when GICP is unreliable
  - /tf odometry (odom -> base_footprint): same, for older bags
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from . import bag_io
from .clouds import xyz


@dataclass
class GicpTrack:
    """Relative motion between LiDAR scans `stride` apart (one sample per pair)."""

    t: np.ndarray  # s since bag start, at the later scan
    dt: np.ndarray  # s between the two scans
    trans: np.ndarray  # m moved
    rot_deg: np.ndarray  # degrees rotated
    reliable: np.ndarray  # converged with enough inliers
    inlier_ratio: np.ndarray


@dataclass
class Timeline:
    """Fused motion on the GICP sample times."""

    t: np.ndarray
    dt: np.ndarray
    speed: np.ndarray  # m/s, NaN where unknown
    rot_rate: np.ndarray  # deg/s, NaN where unknown
    rest: np.ndarray  # bool
    reliable: np.ndarray  # bool, GICP usable at this sample
    sources: list[str] = field(default_factory=list)
    extra: dict = field(default_factory=dict)  # per-signal arrays for the motion CSV

    @property
    def distance(self) -> np.ndarray:
        step = np.where(np.isfinite(self.speed), self.speed * self.dt, 0.0)
        return np.cumsum(step)


def rotation_angle_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def gicp_track(scans, cfg: dict) -> GicpTrack:
    """Register each scan to the previous one in `scans` ((t, Nx3) pairs, already strided)."""
    import small_gicp  # imported here so the rest of the package works without it

    rows = []
    prev = None
    guess = np.eye(4)
    for t, pts in scans:
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > cfg["min_range"]) & (r < cfg["max_range"])]
        if len(pts) < cfg["min_points"]:
            if prev is not None:
                rows.append((t, t - prev[0], np.nan, np.nan, False, 0.0))
            prev, guess = None, np.eye(4)
            continue
        cloud, tree = small_gicp.preprocess_points(
            pts, downsampling_resolution=cfg["voxel"], num_threads=cfg["threads"]
        )
        if prev is not None:
            res = small_gicp.align(
                prev[1],
                cloud,
                prev[2],
                init_T_target_source=guess,
                max_correspondence_distance=cfg["max_correspondence"],
                num_threads=cfg["threads"],
            )
            T = np.asarray(res.T_target_source)
            ratio = res.num_inliers / max(cloud.size(), 1)
            ok = bool(res.converged) and ratio >= cfg["min_inlier_ratio"]
            rows.append(
                (
                    t,
                    t - prev[0],
                    float(np.linalg.norm(T[:3, 3])),
                    rotation_angle_deg(T[:3, :3]),
                    ok,
                    ratio,
                )
            )
            guess = T if ok else np.eye(4)
        prev = (t, cloud, tree)
    a = np.array(rows, dtype=float).reshape(-1, 6)
    return GicpTrack(a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4].astype(bool), a[:, 5])


def imu_stillness(t: np.ndarray, gyro: np.ndarray, acc: np.ndarray, cfg: dict):
    """Per-window stillness from IMU samples. Returns window centers, still flags, stats."""
    win = cfg["window"]
    if len(t) == 0:
        return np.array([]), np.array([], bool), np.array([]), np.array([])
    edges = np.arange(t[0], t[-1] + win, win)
    idx = np.digitize(t, edges)
    centers, still, gmean, astd = [], [], [], []
    gnorm = np.linalg.norm(gyro, axis=1)
    anorm = np.linalg.norm(acc, axis=1)
    for k in np.unique(idx):
        m = idx == k
        if m.sum() < 5:
            continue
        g, a = float(gnorm[m].mean()), float(anorm[m].std())
        centers.append(float(t[m].mean()))
        gmean.append(g)
        astd.append(a)
        still.append(g < cfg["gyro_still"] and a < cfg["accel_std_still"])
    return np.array(centers), np.array(still, bool), np.array(gmean), np.array(astd)


def _nearest(t_src: np.ndarray, v_src: np.ndarray, t_query: np.ndarray, max_gap: float):
    """Value of the nearest source sample (NaN when none within max_gap)."""
    out = np.full(t_query.shape, np.nan)
    if len(t_src) == 0:
        return out
    i = np.clip(np.searchsorted(t_src, t_query), 1, len(t_src) - 1)
    left, right = t_src[i - 1], t_src[i]
    j = np.where(np.abs(t_query - left) <= np.abs(right - t_query), i - 1, i)
    near = np.abs(t_src[j] - t_query) <= max_gap
    out[near] = v_src[j][near]
    return out


def read_signals(bag: bag_io.BagInfo, cfg: dict) -> dict:
    """One pass over the bag: strided LiDAR scans, IMU, CAN speed and /tf odometry."""
    m = cfg["motion"]
    lidar = cfg["lidar"]["topic"]
    imu_topic = m["imu"]["topic"]
    can_topic = m["can_speed_topic"]
    tf_parent, tf_child = m["tf_odom"]["parent"], m["tf_odom"]["child"]
    topics = [lidar] + [x for x in (imu_topic, can_topic, "/tf") if x in bag.topics]
    reader = bag_io.open_reader(bag.path, topics)
    t0 = bag.start_ns

    scans, n_lidar = [], 0
    imu_t, gyro, acc = [], [], []
    can_t, can_v = [], []
    tf_t, tf_p = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = (stamp - t0) * 1e-9
        if topic == lidar:
            if n_lidar % m["gicp"]["stride"] == 0:
                scans.append((t, xyz(deserialize_message(data, PointCloud2))))
            n_lidar += 1
        elif topic == imu_topic:
            msg = deserialize_message(data, Imu)
            imu_t.append(t)
            w, a = msg.angular_velocity, msg.linear_acceleration
            gyro.append((w.x, w.y, w.z))
            acc.append((a.x, a.y, a.z))
        elif topic == can_topic:
            can_t.append(t)
            can_v.append(abs(deserialize_message(data, Float64).data))
        elif topic == "/tf":
            for tf in deserialize_message(data, TFMessage).transforms:
                if tf.header.frame_id == tf_parent and tf.child_frame_id == tf_child:
                    p = tf.transform.translation
                    tf_t.append(t)
                    tf_p.append((p.x, p.y, p.z))
    return {
        "scans": scans,
        "imu": (
            np.array(imu_t),
            np.array(gyro).reshape(-1, 3),
            np.array(acc).reshape(-1, 3),
        ),
        "can": (np.array(can_t), np.array(can_v)),
        "tf": (np.array(tf_t), np.array(tf_p).reshape(-1, 3)),
    }


def tf_speed(
    t: np.ndarray, p: np.ndarray, t_query: np.ndarray, dt: np.ndarray
) -> np.ndarray:
    """Speed from odometry positions over each query interval [t - dt, t]."""
    if len(t) < 2:
        return np.full(t_query.shape, np.nan)
    order = np.argsort(t)
    t, p = t[order], p[order]
    t, keep = np.unique(t, return_index=True)
    p = p[keep]
    inside = (t_query - dt >= t[0]) & (t_query <= t[-1])
    end = np.stack([np.interp(t_query, t, p[:, k]) for k in range(3)], 1)
    start = np.stack([np.interp(t_query - dt, t, p[:, k]) for k in range(3)], 1)
    out = np.linalg.norm(end - start, axis=1) / dt
    out[~inside] = np.nan
    return out


def build_timeline(signals: dict, cfg: dict) -> Timeline:
    m = cfg["motion"]
    track = gicp_track(signals["scans"], m["gicp"])
    t, dt = track.t, track.dt
    speed = np.where(track.reliable, track.trans / dt, np.nan)
    rot_rate = np.where(track.reliable, track.rot_deg / dt, np.nan)
    sources = ["gicp"]
    extra = {
        "gicp_speed": track.trans / dt,
        "gicp_rot_rate": track.rot_deg / dt,
        "gicp_inlier_ratio": track.inlier_ratio,
    }
    rest_gicp = (speed < m["rest_speed"]) & (rot_rate < m["rest_rot_rate_deg"])

    # Other signals: each one, when present, must also agree that the car is at rest
    agree = np.ones(t.shape, bool)
    fallback_speed = np.full(t.shape, np.nan)

    can_t, can_v = signals["can"]
    if len(can_t):
        sources.append("can")
        can = _nearest(can_t, can_v, t, 0.5)
        extra["can_speed"] = can
        agree &= ~(can >= m["rest_speed"])
        fallback_speed = np.where(np.isfinite(fallback_speed), fallback_speed, can)

    tf_t, tf_p = signals["tf"]
    if len(tf_t) > 1:
        sources.append("tf")
        odo = tf_speed(tf_t, tf_p, t, dt)
        extra["tf_speed"] = odo
        fallback_speed = np.where(np.isfinite(fallback_speed), fallback_speed, odo)

    imu_t, gyro, acc = signals["imu"]
    if len(imu_t):
        sources.append("imu")
        centers, still, gmean, astd = imu_stillness(imu_t, gyro, acc, m["imu"])
        imu_still = _nearest(centers, still.astype(float), t, m["imu"]["window"])
        extra["imu_gyro_mean"] = _nearest(centers, gmean, t, m["imu"]["window"])
        extra["imu_accel_std"] = _nearest(centers, astd, t, m["imu"]["window"])
        agree &= ~(imu_still == 0.0)

    # Where GICP is unreliable, fall back to CAN/odometry speed for both rest and distance
    fill = ~track.reliable & np.isfinite(fallback_speed)
    speed = np.where(fill, fallback_speed, speed)
    rest = (
        np.where(track.reliable, rest_gicp, fill & (fallback_speed < m["rest_speed"]))
        & agree
    )
    return Timeline(t, dt, speed, rot_rate, rest, track.reliable, sources, extra)
