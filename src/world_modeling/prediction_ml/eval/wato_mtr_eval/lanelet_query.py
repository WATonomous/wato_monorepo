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
"""Waymo map features -> the lane centerlines a LaneletAhead query would return.

LaneletAhead returns "legally reachable lanelets ahead of ego within a
configurable radius". This keeps every lane centerline within radius_m: a
deterministic baseline that over-supplies relative to real WATO.

Pure geometry: imports neither ROS nor torch.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import scenario

LANE_TYPES = {"lane"}


@dataclasses.dataclass(frozen=True)
class QueryConfig:
    radius_m: float = 80.0


def _within_radius(
    poly: scenario.MapPolyline,
    ego_xy: np.ndarray,
    radius_m: float,
) -> bool:
    points = np.asarray(poly.points)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError(
            f"map feature {poly.feature_id} points must have shape [N,>=2], "
            f"got {points.shape}"
        )
    xy = points[:, :2]
    if not np.isfinite(xy).all():
        raise ValueError(f"map feature {poly.feature_id} contains non-finite XY points")
    if len(xy) == 0:
        return False
    return bool(np.linalg.norm(xy - ego_xy, axis=1).min() <= radius_m)


def query(
    polylines: list[scenario.MapPolyline],
    ego_xy: tuple[float, float],
    config: QueryConfig,
) -> list[scenario.MapPolyline]:
    """Return the lane centerlines within ``config.radius_m`` of ego."""
    if not math.isfinite(config.radius_m) or config.radius_m < 0.0:
        raise ValueError("radius_m must be finite and non-negative")
    ego = np.asarray(ego_xy, dtype=np.float64)
    if ego.shape != (2,) or not np.isfinite(ego).all():
        raise ValueError("ego_xy must contain exactly two finite coordinates")
    return [
        poly
        for poly in polylines
        if poly.feature_type in LANE_TYPES
        and _within_radius(poly, ego, config.radius_m)
    ]
