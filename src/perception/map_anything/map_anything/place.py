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
"""Place-recognition descriptors with MegaLoc (gmberton/MegaLoc, MIT) (ROS-free).

One L2-normalised descriptor per image; the similarity of two images is the dot product of their
descriptors. Preprocessing is the one the MegaLoc authors evaluate with: ImageNet normalisation and
a resize to 322 x 322. ``torch`` is imported lazily so the rest of the package works without a GPU.
"""

from typing import Sequence

import numpy as np

_SIZE = 322
_MEAN = (0.485, 0.456, 0.406)
_STD = (0.229, 0.224, 0.225)


class PlaceDescriptor:
    """Loads MegaLoc through torch.hub (code from GitHub, weights from the Hugging Face hub)."""

    def __init__(self, device: str = "cuda"):
        import torch

        self._torch = torch
        self.device = device
        self.model = (
            torch.hub.load("gmberton/MegaLoc", "get_trained_model", trust_repo=True)
            .to(device)
            .eval()
        )
        self._mean = torch.tensor(_MEAN, device=device).view(1, 3, 1, 1)
        self._std = torch.tensor(_STD, device=device).view(1, 3, 1, 1)

    def describe(self, images: Sequence[np.ndarray]) -> np.ndarray:
        """Descriptors (N, dim) float32 of RGB uint8 images (H, W, 3)."""
        torch = self._torch
        batch = []
        for rgb in images:
            image = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device)
            image = image.permute(2, 0, 1).unsqueeze(0).float() / 255.0
            image = (image - self._mean) / self._std
            batch.append(
                torch.nn.functional.interpolate(
                    image,
                    size=(_SIZE, _SIZE),
                    mode="bilinear",
                    antialias=True,
                    align_corners=False,
                )
            )
        with torch.inference_mode():
            descriptors = self.model(torch.cat(batch))
        return descriptors.float().cpu().numpy()
