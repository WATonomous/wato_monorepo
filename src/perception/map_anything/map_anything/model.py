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
"""MapAnything (facebookresearch/map-anything) camera-pose inference (ROS-free).

A thin wrapper over the upstream API: ``preprocess_inputs`` then ``MapAnything.infer``. Each view is
an RGB image with its pinhole intrinsics; the model returns one metric camera pose per view, in the
frame of the first view, and optionally the dense geometry it predicted for every pixel. ``torch``
and ``mapanything`` are imported lazily so the rest of the package (and its tests) work without a
GPU.
"""

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

DEFAULT_CHECKPOINT = "facebook/map-anything-apache"
# A depth map with fewer measured pixels is not passed to the model (it normalizes by their mean)
MIN_DEPTH_PIXELS = 20


@dataclass
class View:
    """One camera image: RGB uint8 (H, W, 3), its 3x3 pinhole intrinsics (pixels) and, optionally, a
    metric depth map (H, W) float32 of z in the camera frame, 0 where unknown (e.g. sparse lidar)."""

    rgb: np.ndarray
    intrinsics: np.ndarray
    depth: Optional[np.ndarray] = None


def centred_view(rgb: np.ndarray, intrinsics: np.ndarray, cut_bottom: int = 0) -> View:
    """Crop an image so that its principal point is at the image centre.

    MapAnything's poses behave as if the principal point were at the image centre: with it off-centre
    by some angle, every predicted pose is tilted by that angle (measured on Boreas: cutting 470 rows
    from the bottom only moved the translation direction by 9.3 deg, the angle the centre moved). So
    the image is cropped symmetrically about the principal point, which costs a few rows and columns.

    ``cut_bottom`` rows are removed from the bottom first: parts of the vehicle (hood, roof rack) that
    move with the camera. The top is then cut to match.
    """
    height, width = rgb.shape[:2]
    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
    half_w = min(cx, width - cx)
    half_h = min(cy, height - cut_bottom - cy)
    if half_w <= 0 or half_h <= 0:
        raise ValueError("the principal point is outside the image that would remain")
    left, top = int(round(cx - half_w)), int(round(cy - half_h))
    cropped = np.array(intrinsics, dtype=float)
    cropped[0, 2], cropped[1, 2] = cx - left, cy - top
    return View(
        rgb=rgb[
            top : top + int(round(2 * half_h)), left : left + int(round(2 * half_w))
        ],
        intrinsics=cropped,
    )


@dataclass
class Prediction:
    """Camera poses (OpenCV camera-to-world, metres) in the first view's frame, one per view."""

    world_from_cameras: List[np.ndarray]
    confidences: List[float]  # mean per-pixel confidence of each view
    infer_ms: float


@dataclass
class DenseView:
    """What the model predicted for one view, at the model's input resolution (h, w)."""

    world_from_camera: np.ndarray  # 4x4, in the first view's frame
    points: np.ndarray  # (h, w, 3) float32, metres, in the first view's frame
    rgb: np.ndarray  # (h, w, 3) uint8, the image as the model saw it
    valid: np.ndarray  # (h, w) bool: not sky / ambiguous, not a depth edge
    confidence: np.ndarray  # (h, w) float32
    intrinsics: np.ndarray  # 3x3, pixels at (h, w)


class MapAnythingModel:
    """Loads a MapAnything checkpoint from the Hugging Face hub (or its local cache)."""

    def __init__(
        self,
        checkpoint: str = DEFAULT_CHECKPOINT,
        device: str = "cuda",
        amp_dtype: str = "bf16",
        memory_efficient: bool = False,
    ):
        import torch
        from mapanything.models import MapAnything
        from mapanything.utils.image import preprocess_inputs

        self._torch = torch
        self._preprocess = preprocess_inputs
        self.checkpoint = checkpoint
        self.device = device
        self.amp_dtype = amp_dtype
        self.memory_efficient = memory_efficient
        self.model = MapAnything.from_pretrained(checkpoint).to(device).eval()

    def predict(self, views: Sequence[View]) -> Prediction:
        """Poses of all views in the first view's frame. Needs at least two views."""
        if len(views) < 2:
            raise ValueError("MapAnything needs at least two views")
        start = time.perf_counter()
        # masks only affect the dense geometry outputs, which are not used here
        outputs = self._infer(views, masks=False)
        poses = [o["camera_poses"][0].double().cpu().numpy() for o in outputs]
        confidences = [float(o["conf"][0].float().mean().cpu()) for o in outputs]
        return Prediction(
            world_from_cameras=poses,
            confidences=confidences,
            infer_ms=(time.perf_counter() - start) * 1e3,
        )

    def reconstruct(self, views: Sequence[View]) -> List[DenseView]:
        """Dense geometry of all views in the first view's frame. Needs at least two views."""
        if len(views) < 2:
            raise ValueError("MapAnything needs at least two views")
        return [
            DenseView(
                world_from_camera=o["camera_poses"][0].double().cpu().numpy(),
                points=o["pts3d"][0].float().cpu().numpy(),
                rgb=(o["img_no_norm"][0].float().clamp(0, 1) * 255)
                .round()
                .byte()
                .cpu()
                .numpy(),
                valid=o["mask"][0, ..., 0].bool().cpu().numpy(),
                confidence=o["conf"][0].float().cpu().numpy(),
                intrinsics=o["intrinsics"][0].double().cpu().numpy(),
            )
            for o in self._infer(views, masks=True)
        ]

    def _infer(self, views: Sequence[View], masks: bool):
        torch = self._torch
        inputs = []
        for view in views:
            entry = {
                "img": torch.from_numpy(np.ascontiguousarray(view.rgb)),
                "intrinsics": torch.as_tensor(view.intrinsics, dtype=torch.float32),
            }
            if view.depth is not None and np.count_nonzero(view.depth) >= MIN_DEPTH_PIXELS:
                entry["depth_z"] = torch.from_numpy(np.ascontiguousarray(view.depth, dtype=np.float32))
                entry["is_metric_scale"] = torch.tensor([True])
            inputs.append(entry)
        return self.model.infer(
            self._preprocess(inputs),
            memory_efficient_inference=self.memory_efficient,
            use_amp=True,
            amp_dtype=self.amp_dtype,
            apply_mask=masks,
            mask_edges=masks,
        )
