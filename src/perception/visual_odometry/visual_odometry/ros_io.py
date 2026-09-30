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
"""Helpers shared by the live node and the offline bag runner: config, decoding, message building."""

from typing import Dict, Optional

import cv2
import numpy as np
import yaml
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry

from visual_odometry.geometry import invert, make_transform, split_transform
from visual_odometry.vo_core import TrackResult

# Parameter defaults, shared by vo_node (ROS params) and offline_vo (same YAML file).
DEFAULTS = {
    "cameras": ["camera_pano_nn", "camera_pano_nw"],
    "image_topic": "/{camera}/image_rect_compressed",
    "camera_info_topic": "/{camera}/camera_info",
    "rig_frame": "base_link",
    "odom_frame": "vo_odom",
    "odom_topic": "visual_odometry/odometry",
    "status_topic": "visual_odometry/status",
    "sync_slop_ms": 10.0,
    "max_wait_ms": 300.0,
    "allow_incomplete": False,
    "downscale": 1,
    "border": [0, 0, 0, 0],
    "stereo_pairs": [],
    "rectified_stereo_camera": True,
    "extrinsics_file": "",
    "calibration_anchors": [],
    "multicam_mode": "precision",
    "async_sba": True,
    "use_motion_model": True,
    "use_denoising": False,
    "verbosity": 0,
    "queue_size": 5,
    "max_twist_dt": 0.2,
}

_REDUCED_FLAGS = {
    1: cv2.IMREAD_GRAYSCALE,
    2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
    4: cv2.IMREAD_REDUCED_GRAYSCALE_4,
    8: cv2.IMREAD_REDUCED_GRAYSCALE_8,
}


def load_params(path: str) -> Dict:
    """Read ros__parameters from a ROS 2 params YAML (first node entry, e.g. ``/**``) over DEFAULTS."""
    params = dict(DEFAULTS)
    if not path:
        return params
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    for node_params in data.values():
        if isinstance(node_params, dict) and "ros__parameters" in node_params:
            params.update(node_params["ros__parameters"])
            break
    return params


def decode_gray(data, downscale: int = 1) -> Optional[np.ndarray]:
    """Decode a JPEG/PNG CompressedImage payload to 8-bit grayscale at 1/downscale resolution."""
    if downscale not in _REDUCED_FLAGS:
        raise ValueError(f"downscale must be one of {sorted(_REDUCED_FLAGS)}")
    return cv2.imdecode(
        np.frombuffer(bytes(data), dtype=np.uint8), _REDUCED_FLAGS[downscale]
    )


def transform_msg_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4."""
    t, q = transform.translation, transform.rotation
    return make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(ns: int) -> Time:
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


class OdometryBuilder:
    """Turns TrackResults into nav_msgs/Odometry, with a finite-difference body twist."""

    def __init__(self, odom_frame: str, rig_frame: str, max_twist_dt: float = 0.2):
        self.odom_frame = odom_frame
        self.rig_frame = rig_frame
        self.max_twist_dt = max_twist_dt
        self._prev: Optional[TrackResult] = None

    def reset(self) -> None:
        self._prev = None

    def build(self, result: TrackResult) -> Optional[Odometry]:
        if not result.valid:
            self._prev = None  # never difference across a tracking failure
            return None
        msg = Odometry()
        msg.header.stamp = ns_to_stamp(result.stamp_ns)
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.rig_frame
        translation, quat = split_transform(result.world_from_rig)
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y, p.z = translation.tolist()
        o.x, o.y, o.z, o.w = quat.tolist()
        if result.covariance is not None:
            msg.pose.covariance = result.covariance.reshape(-1).tolist()

        prev = self._prev
        if prev is not None:
            dt = (result.stamp_ns - prev.stamp_ns) * 1e-9
            if 0.0 < dt <= self.max_twist_dt:
                # motion in the previous body frame
                delta = invert(prev.world_from_rig) @ result.world_from_rig
                rot = delta[:3, :3]
                angle = np.arccos(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0))
                axis = np.array(
                    [
                        rot[2, 1] - rot[1, 2],
                        rot[0, 2] - rot[2, 0],
                        rot[1, 0] - rot[0, 1],
                    ]
                )
                omega = (
                    axis * (0.5 if angle < 1e-6 else angle / (2.0 * np.sin(angle))) / dt
                )
                v = delta[:3, 3] / dt
                lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
                lin.x, lin.y, lin.z = v.tolist()
                ang.x, ang.y, ang.z = omega.tolist()
        self._prev = result
        return msg


def build_status(
    stamp_ns: int,
    name: str,
    result: Optional[TrackResult],
    counters: Dict,
    offset_stats: Dict,
) -> DiagnosticArray:
    """DiagnosticArray with tracking state, timing, feature counts and per-camera sync offsets."""
    status = DiagnosticStatus(name=name, hardware_id="cuvslam")
    if result is None:
        status.level, status.message = DiagnosticStatus.WARN, "no frame set tracked yet"
    elif result.valid:
        status.level, status.message = DiagnosticStatus.OK, "tracking"
    else:
        status.level, status.message = DiagnosticStatus.WARN, "tracking lost"
    values = dict(counters)
    if result is not None:
        values["track_ms"] = f"{result.track_ms:.2f}"
        values["landmarks"] = result.num_landmarks
        values["observations"] = ",".join(str(n) for n in result.num_observations)
    for cam, s in offset_stats.items():
        values[f"offset_ms/{cam}"] = (
            f"{s['mean_ms']:+.2f} (max {s['max_abs_ms']:.2f}, n={s['n']})"
        )
    status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
    msg = DiagnosticArray()
    msg.header.stamp = ns_to_stamp(stamp_ns)
    msg.status = [status]
    return msg
