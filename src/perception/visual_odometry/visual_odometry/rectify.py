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
"""Virtual stereo rectification of overlapping pano cameras.

Neighbouring pano cameras look 45 deg apart. cuVSLAM's cross-camera matching only works for small
relative rotations (measured on the sensors bag: landmarks up to ~20 deg of yaw, none from 30 deg), so
each overlapping pair is reprojected into two virtual cameras that share one orientation (the bisector
of the two optical axes, x along the baseline). That is a classic rectified stereo pair with the
cameras' real 0.184 m baseline. Pixels with no source data are masked for cuVSLAM.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from visual_odometry.vo_core import CameraSpec


@dataclass
class PairFrame:
    """Shared orientation of a rectified pair."""

    left_index: int  # 0 if the first camera is the left one, else 1
    # 3x3, virtual camera axes in the rig frame (OpenCV optical convention)
    r_rig_rect: np.ndarray
    baseline: float  # m, left -> right along the virtual x axis
    k_rect: np.ndarray  # 3x3 shared virtual intrinsics
    size: Tuple[int, int]  # (width, height) of the virtual images


def _k(spec: CameraSpec) -> np.ndarray:
    return np.array([[spec.fx, 0.0, spec.cx], [0.0, spec.fy, spec.cy], [0.0, 0.0, 1.0]])


def pair_frame(
    a: CameraSpec, b: CameraSpec, size: Optional[Tuple[int, int]] = None
) -> PairFrame:
    """Virtual orientation: z = bisector of the optical axes (made orthogonal to the baseline), x = left->right.

    Both virtual cameras use the same pinhole intrinsics (mean focal length, centred principal point),
    so rows are aligned if the extrinsics are right.
    """
    t_a, t_b = a.rig_from_camera, b.rig_from_camera
    z = t_a[:3, 2] + t_b[:3, 2]
    y = t_a[:3, 1] + t_b[:3, 1]
    z, y = z / np.linalg.norm(z), y / np.linalg.norm(y)
    left_index = 0 if (t_b[:3, 3] - t_a[:3, 3]) @ np.cross(y, z) > 0 else 1
    left, right = (a, b) if left_index == 0 else (b, a)

    x = right.rig_from_camera[:3, 3] - left.rig_from_camera[:3, 3]
    baseline = float(np.linalg.norm(x))
    if baseline < 1e-3:
        raise ValueError(
            f"{a.name} and {b.name} share a centre; cannot rectify a zero baseline"
        )
    x /= baseline
    z = z - (z @ x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)

    width, height = size if size is not None else (left.width, left.height)
    f = 0.5 * (left.fx + right.fx)
    k_rect = np.array(
        [[f, 0.0, (width - 1) / 2.0], [0.0, f, (height - 1) / 2.0], [0.0, 0.0, 1.0]]
    )
    return PairFrame(
        left_index, np.column_stack([x, y, z]), baseline, k_rect, (width, height)
    )


def rectify_points(src: CameraSpec, frame: PairFrame, pts: np.ndarray) -> np.ndarray:
    """Map pixel coordinates of a source camera into the pair's virtual camera."""
    r_rect_src = frame.r_rig_rect.T @ src.rig_from_camera[:3, :3]
    out = cv2.undistortPoints(
        np.asarray(pts, np.float64).reshape(-1, 1, 2),
        _k(src),
        None,
        R=r_rect_src,
        P=frame.k_rect,
    )
    return out.reshape(-1, 2)


def rectified_disparity(
    a: CameraSpec, b: CameraSpec, pts_a: np.ndarray, pts_b: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Horizontal and vertical disparity (left - right, px) of matched pixels after rectification."""
    frame = pair_frame(a, b)
    ra, rb = rectify_points(a, frame, pts_a), rectify_points(b, frame, pts_b)
    d = ra - rb if frame.left_index == 0 else rb - ra
    return d[:, 0], d[:, 1]


@dataclass
class RectifiedView:
    """One virtual camera: a source camera re-rendered into the pair's shared orientation."""

    spec: CameraSpec  # virtual intrinsics + rig_from_camera of the virtual camera
    source: CameraSpec
    map_x: np.ndarray
    map_y: np.ndarray
    mask: np.ndarray  # uint8, 255 = no source pixel (ignored by cuVSLAM), 0 = valid

    def apply(self, image: np.ndarray) -> np.ndarray:
        return cv2.remap(
            image,
            self.map_x,
            self.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )


@dataclass
class RectifiedPair:
    left: RectifiedView
    right: RectifiedView
    frame: PairFrame


def border_mask(height: int, width: int, border: Sequence[int]) -> np.ndarray:
    """uint8 image: 255 inside, 0 in the (top, bottom, left, right) border strips."""
    valid = np.full((height, width), 255, np.uint8)
    top, bottom, left, right = (int(b) for b in border)
    valid[:top, :] = 0
    valid[:, :left] = 0
    if bottom:
        valid[height - bottom :, :] = 0
    if right:
        valid[:, width - right :] = 0
    return valid


def _source_valid_mask(spec: CameraSpec) -> np.ndarray:
    return border_mask(spec.height, spec.width, spec.border)


def rectify_pair(
    a: CameraSpec,
    b: CameraSpec,
    size: Optional[Tuple[int, int]] = None,
    mask_dilate_px: int = 15,
) -> RectifiedPair:
    """Build the remap tables and masks of a rectified virtual stereo pair (poses in the rig frame)."""
    frame = pair_frame(a, b, size)
    left, right = (a, b) if frame.left_index == 0 else (b, a)
    width, height = frame.size
    kernel = (
        np.ones((mask_dilate_px, mask_dilate_px), np.uint8)
        if mask_dilate_px > 0
        else None
    )
    views = []
    for src, suffix in ((left, "rect_left"), (right, "rect_right")):
        r_rect_src = frame.r_rig_rect.T @ src.rig_from_camera[:3, :3]
        map_x, map_y = cv2.initUndistortRectifyMap(
            _k(src), None, r_rect_src, frame.k_rect, (width, height), cv2.CV_32FC1
        )
        valid = cv2.remap(
            _source_valid_mask(src),
            map_x,
            map_y,
            cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        mask = np.where(valid > 0, 0, 255).astype(np.uint8)
        if kernel is not None:
            mask = cv2.dilate(mask, kernel)
        rig_from_virtual = np.eye(4)
        rig_from_virtual[:3, :3] = frame.r_rig_rect
        rig_from_virtual[:3, 3] = src.rig_from_camera[:3, 3]
        spec = CameraSpec(
            name=f"{src.name}/{suffix}",
            width=width,
            height=height,
            fx=frame.k_rect[0, 0],
            fy=frame.k_rect[1, 1],
            cx=frame.k_rect[0, 2],
            cy=frame.k_rect[1, 2],
            rig_from_camera=rig_from_virtual,
        )
        views.append(RectifiedView(spec, src, map_x, map_y, mask))
    return RectifiedPair(left=views[0], right=views[1], frame=frame)


class RigPreprocessor:
    """Turns a physical frame set into the images/masks of the rig cuVSLAM sees.

    With no ``stereo_pairs`` the rig is the physical cameras (masked by their borders). With pairs,
    the rig is the rectified virtual cameras, two per pair, in pair order (left, right). A physical
    camera may appear in several pairs (e.g. nn in nw/nn and nn/ne).
    """

    def __init__(
        self, cameras: Sequence[CameraSpec], stereo_pairs: Sequence[Sequence[str]] = ()
    ):
        self.physical = list(cameras)
        index: Dict[str, int] = {c.name: i for i, c in enumerate(self.physical)}
        self.sources: List[int] = []  # physical camera index per rig camera
        self.specs: List[CameraSpec] = []
        self.masks: List[np.ndarray] = []
        self._views: List[Optional[RectifiedView]] = []
        self.pairs: List[RectifiedPair] = []
        if not stereo_pairs:
            for i, cam in enumerate(self.physical):
                self.sources.append(i)
                self.specs.append(cam)
                self.masks.append(
                    np.where(_source_valid_mask(cam) > 0, 0, 255).astype(np.uint8)
                )
                self._views.append(None)
            return
        for pair in stereo_pairs:
            if len(pair) != 2 or pair[0] not in index or pair[1] not in index:
                raise ValueError(
                    f"stereo pair {list(pair)} must name two of {list(index)}"
                )
            ia, ib = index[pair[0]], index[pair[1]]
            rect = rectify_pair(self.physical[ia], self.physical[ib])
            self.pairs.append(rect)
            left_src, right_src = (ia, ib) if rect.frame.left_index == 0 else (ib, ia)
            for view, src_index in ((rect.left, left_src), (rect.right, right_src)):
                self.sources.append(src_index)
                self.specs.append(view.spec)
                self.masks.append(view.mask)
                self._views.append(view)

    def apply(
        self, images: Sequence[Optional[np.ndarray]]
    ) -> List[Optional[np.ndarray]]:
        """Physical images (camera order) -> rig images (rig order). Missing images stay None."""
        out: List[Optional[np.ndarray]] = []
        for src, view in zip(self.sources, self._views):
            img = images[src]
            out.append(
                None if img is None else (img if view is None else view.apply(img))
            )
        return out
