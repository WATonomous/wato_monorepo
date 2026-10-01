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
"""ROS-free wrapper around cuVSLAM multicamera odometry.

The rig frame is whatever frame the camera extrinsics are expressed in (``base_link`` on the car).
cuVSLAM's world frame is the rig frame at the first tracked frame, so ``world_from_rig`` is the rig
pose relative to where tracking started. ``cuvslam`` is imported lazily so the rest of the package
(and its tests) work without a GPU.
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from visual_odometry.geometry import make_transform, split_transform

MULTICAM_MODES = ("performance", "precision", "moderate")


@dataclass
class CameraSpec:
    """Pinhole intrinsics of a rectified camera plus its pose in the rig frame."""

    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # 4x4, maps camera (OpenCV optical) coordinates into the rig frame
    rig_from_camera: np.ndarray
    # top, bottom, left, right pixels to ignore
    border: Tuple[int, int, int, int] = (
        0,
        0,
        0,
        0,
    )

    @classmethod
    def from_projection(
        cls,
        name: str,
        width: int,
        height: int,
        projection: Sequence[float],
        rig_from_camera: np.ndarray,
        downscale: int = 1,
        border: Tuple[int, int, int, int] = (0, 0, 0, 0),
    ) -> "CameraSpec":
        """Build from a CameraInfo ``P`` matrix (the intrinsics of the rectified image).

        ``downscale`` > 1 describes images decoded at 1/downscale resolution (e.g. OpenCV's
        IMREAD_REDUCED_GRAYSCALE_2), which rounds sizes up.
        """
        p = np.asarray(projection, dtype=float).reshape(3, 4)
        if downscale < 1:
            raise ValueError("downscale must be >= 1")
        s = 1.0 / downscale
        return cls(
            name=name,
            width=(int(width) + downscale - 1) // downscale,
            height=(int(height) + downscale - 1) // downscale,
            fx=p[0, 0] * s,
            fy=p[1, 1] * s,
            # Pixel centres: u' + 0.5 = (u + 0.5) / downscale
            cx=(p[0, 2] + 0.5) * s - 0.5,
            cy=(p[1, 2] + 0.5) * s - 0.5,
            rig_from_camera=np.asarray(rig_from_camera, dtype=float),
            border=tuple(int(b) // downscale for b in border),
        )


@dataclass
class TrackResult:
    """Output of one VoCore.track() call."""

    stamp_ns: int
    world_from_rig: Optional[np.ndarray] = None  # 4x4, None when tracking failed
    covariance: Optional[np.ndarray] = (
        None  # 6x6, (x, y, z, rot_x, rot_y, rot_z), row-major
    )
    num_landmarks: int = 0
    num_observations: List[int] = field(default_factory=list)
    track_ms: float = 0.0

    @property
    def valid(self) -> bool:
        return self.world_from_rig is not None


class VoCore:
    """cuVSLAM Multicamera odometry over a fixed rig."""

    def __init__(
        self,
        cameras: Sequence[CameraSpec],
        multicam_mode: str = "precision",
        async_sba: bool = True,
        use_motion_model: bool = True,
        use_denoising: bool = False,
        max_frame_delta_s: float = 1.0,
        rectified_stereo_camera: bool = False,
        masks: Optional[Sequence[np.ndarray]] = None,
        export_features: bool = True,
        verbosity: int = 0,
    ):
        import cuvslam  # noqa: PLC0415 - lazy import, needs CUDA

        if multicam_mode not in MULTICAM_MODES:
            raise ValueError(
                f"multicam_mode must be one of {MULTICAM_MODES}, got {multicam_mode!r}"
            )
        self._cuvslam = cuvslam
        self.cameras = list(cameras)
        if masks is not None and len(masks) != len(self.cameras):
            raise ValueError(f"expected {len(self.cameras)} masks, got {len(masks)}")
        # Static per-camera masks (uint8, 255 = ignore); None when nothing is masked.
        self._masks = (
            None if masks is None or not any(np.any(m) for m in masks) else list(masks)
        )
        self._export_features = export_features
        cuvslam.set_verbosity(verbosity)

        rig = cuvslam.Rig()
        rig.cameras = [self._make_camera(c) for c in self.cameras]
        self._rig = rig

        tracker_cls = cuvslam.Tracker
        self._config = tracker_cls.OdometryConfig(
            odometry_mode=tracker_cls.OdometryMode.Multicamera,
            multicam_mode=getattr(
                tracker_cls.MulticameraMode, multicam_mode.capitalize()
            ),
            rectified_stereo_camera=rectified_stereo_camera,
            async_sba=async_sba,
            use_motion_model=use_motion_model,
            use_denoising=use_denoising,
            max_frame_delta_s=max_frame_delta_s,
            enable_landmarks_export=export_features,
            enable_observations_export=export_features,
        )
        self._tracker = None
        self._last_stamp_ns: Optional[int] = None
        self.reset()

    def _make_camera(self, spec: CameraSpec):
        cam = self._cuvslam.Camera()
        cam.size = [spec.width, spec.height]
        cam.focal = [spec.fx, spec.fy]
        cam.principal = [spec.cx, spec.cy]
        cam.distortion = self._cuvslam.Distortion(
            self._cuvslam.Distortion.Model.Pinhole, []
        )
        translation, quat = split_transform(spec.rig_from_camera)
        cam.rig_from_camera = self._cuvslam.Pose(
            rotation=quat.tolist(), translation=translation.tolist()
        )
        cam.border_top, cam.border_bottom, cam.border_left, cam.border_right = (
            spec.border
        )
        return cam

    def reset(self) -> None:
        """Start a new tracking session (new world frame)."""
        self._tracker = self._cuvslam.Tracker(self._rig, self._config)
        self._last_stamp_ns = None

    def track(
        self, stamp_ns: int, images: Sequence[Optional[np.ndarray]]
    ) -> TrackResult:
        """Track one frame set. ``images`` are 8-bit grayscale, in camera order; None = missing."""
        if len(images) != len(self.cameras):
            raise ValueError(f"expected {len(self.cameras)} images, got {len(images)}")
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            raise ValueError(
                f"timestamps must strictly increase ({stamp_ns} <= {self._last_stamp_ns})"
            )
        self._last_stamp_ns = stamp_ns

        empty = np.empty((0, 0), dtype=np.uint8)
        frames = [img if img is not None else empty for img in images]
        t0 = time.perf_counter()
        if self._masks is None:
            estimate, _ = self._tracker.track(int(stamp_ns), frames)
        else:
            masks = [
                m if img is not None else empty for m, img in zip(self._masks, images)
            ]
            estimate, _ = self._tracker.track(int(stamp_ns), frames, masks)
        result = TrackResult(
            stamp_ns=int(stamp_ns), track_ms=(time.perf_counter() - t0) * 1e3
        )

        if self._export_features:
            result.num_landmarks = len(self._tracker.get_last_landmarks())
            result.num_observations = [
                len(self._tracker.get_last_observations(i))
                for i in range(len(self.cameras))
            ]

        pose_cov = estimate.world_from_rig
        if pose_cov is None:
            return result
        result.world_from_rig = make_transform(
            pose_cov.pose.translation, pose_cov.pose.rotation
        )
        cov = np.asarray(pose_cov.covariance_xyz_rpy, dtype=float)
        if cov.size == 36:
            result.covariance = cov.reshape(6, 6)
        return result
