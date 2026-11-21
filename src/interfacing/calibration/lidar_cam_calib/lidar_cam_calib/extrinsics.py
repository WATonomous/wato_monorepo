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
"""Rigid transforms: URDF / tf_static frame graphs, toolbox vectors, statistics.

Conventions:
  T_a_b  4x4 pose of frame b in frame a (maps points from b to a).
  vec7   the toolbox's [x, y, z, qx, qy, qz, qw]; its T_lidar_camera is
         T_lidar_camera in this notation (camera optical frame in the LiDAR frame).
  rpy    URDF fixed-axis roll-pitch-yaw, R = Rz(yaw) Ry(pitch) Rx(roll).
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def T_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def xyz_rpy_from_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_euler("xyz")


def T_from_vec7(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(v[3:7]).as_matrix()
    T[:3, 3] = v[:3]
    return T


def vec7_from_T(T: np.ndarray) -> list[float]:
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return [float(x) for x in (*T[:3, 3], *q)]


def T_from_transform_msg(tr) -> np.ndarray:
    t, q = tr.translation, tr.rotation
    return T_from_vec7([t.x, t.y, t.z, q.x, q.y, q.z, q.w])


class FrameGraph:
    """Tree of fixed transforms (child -> parent) with lookups between any two frames."""

    def __init__(self) -> None:
        self.parent: dict[str, tuple[str, np.ndarray]] = {}

    def add(self, parent: str, child: str, T_parent_child: np.ndarray) -> None:
        self.parent[child] = (parent, T_parent_child)

    def _to_root(self, frame: str) -> tuple[str, np.ndarray]:
        T = np.eye(4)
        seen = set()
        while frame in self.parent:
            if frame in seen:
                raise ValueError(f"cycle in frame graph at {frame}")
            seen.add(frame)
            parent, T_pc = self.parent[frame]
            T = T_pc @ T
            frame = parent
        return frame, T

    def lookup(self, target: str, source: str) -> np.ndarray:
        """T_target_source."""
        root_t, T_rt = self._to_root(target)
        root_s, T_rs = self._to_root(source)
        if root_t != root_s:
            raise KeyError(f"no transform between {target} and {source}")
        return np.linalg.inv(T_rt) @ T_rs

    @classmethod
    def from_urdf_xml(cls, xml_text: str) -> FrameGraph:
        g = cls()
        for joint in ET.fromstring(xml_text).iter("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            xyz = [
                float(x)
                for x in (
                    origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
                ).split()
            ]
            rpy = [
                float(x)
                for x in (
                    origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
                ).split()
            ]
            g.add(parent, child, T_from_xyz_rpy(xyz, rpy))
        return g

    @classmethod
    def from_urdf(cls, path: Path) -> FrameGraph:
        """Load a .urdf, or expand a .xacro with the `xacro` tool first."""
        if path.suffix == ".xacro":
            if not shutil.which("xacro"):
                raise RuntimeError(
                    "xacro is not installed; run inside the calibration image, or pass a built .urdf"
                )
            text = subprocess.run(
                ["xacro", str(path)], check=True, capture_output=True, text=True
            ).stdout
        else:
            text = path.read_text()
        return cls.from_urdf_xml(text)

    @classmethod
    def from_tf_static(cls, messages) -> FrameGraph:
        g = cls()
        for msg in messages:
            for tf in msg.transforms:
                g.add(
                    tf.header.frame_id,
                    tf.child_frame_id,
                    T_from_transform_msg(tf.transform),
                )
        return g


def perturb(
    T: np.ndarray, sigma_t: float, sigma_r_deg: float, rng: np.random.Generator
) -> np.ndarray:
    """Random offset of the camera pose: translation in the parent frame, rotation in the camera frame."""
    out = T.copy()
    out[:3, 3] += rng.normal(0.0, sigma_t, 3)
    dR = Rotation.from_rotvec(np.radians(rng.normal(0.0, sigma_r_deg, 3))).as_matrix()
    out[:3, :3] = T[:3, :3] @ dR
    return out


def mean_transform(Ts: list[np.ndarray]) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.mean([t[:3, 3] for t in Ts], axis=0)
    T[:3, :3] = (
        Rotation.from_matrix(np.array([t[:3, :3] for t in Ts])).mean().as_matrix()
    )
    return T


def difference(T_a: np.ndarray, T_b: np.ndarray) -> tuple[float, float]:
    """(translation distance in m, rotation angle in deg) between two poses."""
    dt = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    dr = float(
        np.degrees(Rotation.from_matrix(T_a[:3, :3].T @ T_b[:3, :3]).magnitude())
    )
    return dt, dr
