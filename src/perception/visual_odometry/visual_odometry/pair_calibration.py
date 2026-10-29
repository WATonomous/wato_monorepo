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
"""Self-calibration of the relative rotation of an overlapping camera pair from matched pixels.

The rotation of the non-reference camera is corrected about the pair's rectified axes, keeping the
nominal translation (baseline):

* rotations about the rectified x (baseline) and z (viewing) axes change the *vertical* disparity and
  are fitted so that matched rows line up (robust Gauss-Newton);
* the rotation about the rectified y axis moves points *along* the epipolar lines. It is not observable
  from two-view matches without known depth, and any error in it biases stereo depth (VO scale). It is
  set by a heuristic: the farthest matched points (a low disparity percentile) sit at a small positive
  disparity. Refine it later against a metric reference (e.g. VO scale vs INS on a moving bag).
"""

from dataclasses import dataclass

import cv2
import numpy as np

from visual_odometry.geometry import rotation_angle
from visual_odometry.rectify import pair_frame, rectified_disparity
from visual_odometry.vo_core import CameraSpec


def _expso3(w: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(w))
    k = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if theta < 1e-12:
        return np.eye(3) + k
    k /= theta
    return np.eye(3) + np.sin(theta) * k + (1 - np.cos(theta)) * k @ k


def rotate_about_axes(
    spec: CameraSpec, axes: np.ndarray, delta: np.ndarray
) -> CameraSpec:
    """Return ``spec`` rotated by the rotation vector ``delta`` expressed in ``axes`` (3x3, rig frame)."""
    rotated = spec.rig_from_camera.copy()
    rotated[:3, :3] = (
        axes @ _expso3(np.asarray(delta, float)) @ axes.T @ spec.rig_from_camera[:3, :3]
    )
    return CameraSpec(
        spec.name,
        spec.width,
        spec.height,
        spec.fx,
        spec.fy,
        spec.cx,
        spec.cy,
        rotated,
        tuple(spec.border),
    )


@dataclass
class PairFit:
    corrected: CameraSpec  # the non-reference camera with its corrected rotation
    # correction about the rectified (x, y, z) axes of the pair, deg
    delta_deg: np.ndarray
    rotation_change_deg: float
    num_matches: int
    num_inliers: int
    vertical_before: tuple  # (median, MAD) px
    vertical_after: tuple
    far_disparity_before: float  # px at far_percentile
    far_disparity_after: float
    median_disparity_after: float


def _median_mad(x: np.ndarray) -> tuple:
    med = float(np.median(x))
    return med, float(np.median(np.abs(x - med)))


def calibrate_pair(
    reference: CameraSpec,
    other: CameraSpec,
    pts_ref: np.ndarray,
    pts_other: np.ndarray,
    far_percentile: float = 2.0,
    far_disparity_px: float = 0.5,
    inlier_px: float = 3.0,
    iterations: int = 15,
    ransac_px: float = 2.0,
    max_yaw_deg: float = 6.0,
) -> PairFit:
    """Fit the rotation of ``other`` so the rectified pair (reference, other) is row-aligned."""
    pts_ref, pts_other = np.asarray(pts_ref, float), np.asarray(pts_other, float)
    num_matches = len(pts_ref)
    # Drop mismatches (repetitive texture such as hedges) independently of the extrinsics.
    _, keep = cv2.findFundamentalMat(
        pts_ref, pts_other, cv2.FM_RANSAC, ransac_px, 0.999
    )
    if keep is None or keep.sum() < 20:
        raise RuntimeError(
            f"too few epipolar-consistent matches ({0 if keep is None else int(keep.sum())})"
        )
    keep = keep.ravel().astype(bool)
    pts_ref, pts_other = pts_ref[keep], pts_other[keep]
    axes = pair_frame(
        reference, other
    ).r_rig_rect  # parametrization axes, fixed at the nominal pair frame

    def disparity(delta):
        return rectified_disparity(
            reference, rotate_about_axes(other, axes, delta), pts_ref, pts_other
        )

    dx0, dy0 = disparity(np.zeros(3))
    before_v, before_far = _median_mad(dy0), float(np.percentile(dx0, far_percentile))

    def fit_rows(delta, inliers, gate):
        """Rotations about rectified x and z: robust (Huber) Gauss-Newton on vertical disparity."""
        for it in range(iterations):
            _, dy = disparity(delta)
            if (
                gate and it == 3
            ):  # gate gross mismatches once the misalignment is mostly removed
                inliers = np.abs(dy) < inlier_px
            r = dy[inliers]
            jac = np.zeros((inliers.sum(), 2))
            for k, axis in enumerate((0, 2)):
                step = np.zeros(3)
                step[axis] = 1e-6
                jac[:, k] = (disparity(delta + step)[1][inliers] - r) / 1e-6
            scale = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-9
            w = np.minimum(1.0, 1.345 * scale / np.maximum(np.abs(r), 1e-12))
            update = -np.linalg.solve(
                jac.T @ (w[:, None] * jac) + 1e-12 * np.eye(2), jac.T @ (w * r)
            )
            delta[[0, 2]] += update
            if np.abs(update).max() < 1e-8:
                break
        return delta, inliers

    def fit_along_epipolar(delta, inliers):
        """Rotation about rectified y by the far-point heuristic (bisection)."""

        def far_error(yaw):
            d = delta.copy()
            d[1] = yaw
            return (
                float(np.percentile(disparity(d)[0][inliers], far_percentile))
                - far_disparity_px
            )

        lo, hi = -np.radians(max_yaw_deg), np.radians(max_yaw_deg)
        f_lo, f_hi = far_error(lo), far_error(hi)
        if np.sign(f_lo) == np.sign(f_hi):
            raise RuntimeError(
                f"far-point disparity {f_lo + far_disparity_px:.1f} / {f_hi + far_disparity_px:.1f} px at "
                f"-/+{max_yaw_deg} deg does not bracket {far_disparity_px} px; check the matches"
            )
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            f_mid = far_error(mid)
            if np.sign(f_mid) == np.sign(f_lo):
                lo, f_lo = mid, f_mid
            else:
                hi = mid
        delta[1] = 0.5 * (lo + hi)
        return delta

    # The two steps interact slightly (the pair's bisector moves), so alternate them.
    delta = np.zeros(3)
    inliers = np.ones(len(pts_ref), bool)
    delta, inliers = fit_rows(delta, inliers, gate=True)
    for _ in range(3):
        _, dy = disparity(delta)
        inliers = np.abs(dy - np.median(dy)) < min(inlier_px, 1.0)
        delta = fit_along_epipolar(delta, inliers)
        delta, inliers = fit_rows(delta, inliers, gate=False)

    corrected = rotate_about_axes(other, axes, delta)
    dx, dy = disparity(delta)
    return PairFit(
        corrected=corrected,
        delta_deg=np.degrees(delta),
        rotation_change_deg=float(
            np.degrees(
                rotation_angle(
                    other.rig_from_camera[:3, :3].T @ corrected.rig_from_camera[:3, :3]
                )
            )
        ),
        num_matches=num_matches,
        num_inliers=int(inliers.sum()),
        vertical_before=before_v,
        vertical_after=_median_mad(dy[inliers]),
        far_disparity_before=before_far,
        far_disparity_after=float(np.percentile(dx[inliers], far_percentile)),
        median_disparity_after=float(np.median(dx[inliers])),
    )
