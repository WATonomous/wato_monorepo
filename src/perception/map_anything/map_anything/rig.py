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
"""Rig poses from per-camera poses (ROS-free, GPU-free).

A multi-view model predicts one pose per camera image, all in one arbitrary world frame. Every
camera is rigidly mounted, so each predicted camera pose is also an estimate of where the rig was:
``world_from_rig = world_from_camera @ camera_from_rig``. The estimates of one rig (one instant) are
averaged, and how much they disagree is reported as a quality measure: it grows when the model's
geometry is inconsistent or the extrinsics are wrong.

Two ways to turn that into the rig's motion between two instants a and b:

* ``relative_rig_pose``: average the rig pose of each instant, then difference. Relies on the model
  placing the cameras of one instant correctly relative to each other, which needs overlap between
  them.
* ``relative_rig_pose_per_camera``: each camera's own motion from a to b, expressed as rig motion
  through its extrinsic, then averaged. Relies only on each camera seeing the same scene at both
  instants, and a constant time offset between unsynchronized cameras mostly cancels.
"""

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from visual_odometry.geometry import invert, rotation_angle


@dataclass
class RigPose:
    """Rig pose averaged over its cameras, with the disagreement between the cameras."""

    world_from_rig: np.ndarray  # 4x4
    trans_spread: float  # m, RMS distance of the per-camera estimates from the average
    rot_spread: float  # rad, RMS angle of the per-camera estimates from the average
    cameras: int


@dataclass
class RelativeRigPose:
    """Motion of the rig between two instants: ``rig_a_from_rig_b = T_rig_a^-1 * T_rig_b``."""

    rig_a_from_rig_b: np.ndarray  # 4x4
    trans_spread: float  # m, the two rigs' spreads combined (root sum of squares)
    rot_spread: float  # rad
    cameras: int  # the smaller camera count of the two rigs


def _average(estimates: Sequence[np.ndarray]):
    """Mean rotation / median translation of 4x4 poses, and their RMS spread around it."""
    mean = np.eye(4)
    mean[:3, :3] = (
        Rotation.from_matrix([e[:3, :3] for e in estimates]).mean().as_matrix()
    )
    mean[:3, 3] = np.median([e[:3, 3] for e in estimates], axis=0)
    trans = [np.linalg.norm(e[:3, 3] - mean[:3, 3]) for e in estimates]
    rot = [rotation_angle(mean[:3, :3].T @ e[:3, :3]) for e in estimates]
    return (
        mean,
        float(np.sqrt(np.mean(np.square(trans)))),
        float(np.sqrt(np.mean(np.square(rot)))),
    )


def rig_pose(
    world_from_cameras: Sequence[np.ndarray], rig_from_cameras: Sequence[np.ndarray]
) -> RigPose:
    """Average rig pose of one instant from its cameras' predicted poses and known extrinsics."""
    if len(world_from_cameras) != len(rig_from_cameras) or not len(world_from_cameras):
        raise ValueError("need one extrinsic per camera pose, and at least one camera")
    estimates = [
        np.asarray(world_from_camera, dtype=float) @ invert(np.asarray(rig_from_camera))
        for world_from_camera, rig_from_camera in zip(
            world_from_cameras, rig_from_cameras
        )
    ]
    mean, trans_spread, rot_spread = _average(estimates)
    return RigPose(
        world_from_rig=mean,
        trans_spread=trans_spread,
        rot_spread=rot_spread,
        cameras=len(estimates),
    )


def relative_rig_pose(
    world_from_cameras_a: Sequence[np.ndarray],
    rig_from_cameras_a: Sequence[np.ndarray],
    world_from_cameras_b: Sequence[np.ndarray],
    rig_from_cameras_b: Sequence[np.ndarray],
) -> RelativeRigPose:
    """Rig motion between instants a and b, from camera poses predicted in one common frame."""
    a = rig_pose(world_from_cameras_a, rig_from_cameras_a)
    b = rig_pose(world_from_cameras_b, rig_from_cameras_b)
    return RelativeRigPose(
        rig_a_from_rig_b=invert(a.world_from_rig) @ b.world_from_rig,
        trans_spread=float(np.hypot(a.trans_spread, b.trans_spread)),
        rot_spread=float(np.hypot(a.rot_spread, b.rot_spread)),
        cameras=min(a.cameras, b.cameras),
    )


def relative_rig_pose_per_camera(
    world_from_cameras_a: Sequence[np.ndarray],
    world_from_cameras_b: Sequence[np.ndarray],
    rig_from_cameras: Sequence[np.ndarray],
) -> RelativeRigPose:
    """Rig motion between instants a and b as the average of every camera's own motion.

    Camera i is the same physical camera at both instants. Its motion ``cam_a_from_cam_b`` is
    conjugated with its extrinsic into the motion of the rig.
    """
    if not (
        len(world_from_cameras_a) == len(world_from_cameras_b) == len(rig_from_cameras)
    ) or not len(rig_from_cameras):
        raise ValueError("need the same cameras at both instants, and at least one")
    estimates = [
        np.asarray(rig_from_camera)
        @ invert(np.asarray(world_from_a, dtype=float))
        @ np.asarray(world_from_b, dtype=float)
        @ invert(np.asarray(rig_from_camera))
        for world_from_a, world_from_b, rig_from_camera in zip(
            world_from_cameras_a, world_from_cameras_b, rig_from_cameras
        )
    ]
    mean, trans_spread, rot_spread = _average(estimates)
    return RelativeRigPose(
        rig_a_from_rig_b=mean,
        trans_spread=trans_spread,
        rot_spread=rot_spread,
        cameras=len(estimates),
    )
