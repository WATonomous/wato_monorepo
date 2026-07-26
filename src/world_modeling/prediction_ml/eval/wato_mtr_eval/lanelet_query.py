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
"""Waymo map features -> the subset a LaneletAhead query would return.

LaneletAhead is a query result, not a map: "legally reachable lanelets ahead of
ego within a configurable radius ... BFS through the routing graph". Two
strategies are offered:

  radius     every lane centerline within radius_m. Deterministic upper-context
             baseline; over-supplies relative to real WATO.
  reachable  additionally requires the lane to fall in the forward region, an
             explicit APPROXIMATION of the forward-reachable filter -- not a
             Lanelet2 replica.

Reporting the delta between them prices the query's aggressiveness. Stats are
always computed for both strategies so a single run yields the comparison.

Pure geometry: imports neither ROS nor torch, so the rung-1 ablation reuses the
exact query the converter uses.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import ledger, scenario

LANE_TYPES = {"lane"}
DISCARDED_TYPES = {
    "road_line",
    "road_edge",
    "crosswalk",
    "speed_bump",
    "driveway",
}
_STRATEGIES = {"radius", "reachable"}
_POSITION_EPSILON = 1e-9


@dataclasses.dataclass(frozen=True)
class QueryConfig:
    radius_m: float = 80.0
    strategy: str = "radius"
    forward_half_angle_deg: float = 90.0


def _validated_xy(poly: scenario.MapPolyline) -> np.ndarray:
    points = np.asarray(poly.points)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError(
            f"map feature {poly.feature_id} points must have shape [N,>=2], "
            f"got {points.shape}"
        )
    xy = points[:, :2]
    if not np.isfinite(xy).all():
        raise ValueError(
            f"map feature {poly.feature_id} contains non-finite XY points"
        )
    return xy


def _within_radius(
    poly: scenario.MapPolyline,
    ego_xy: np.ndarray,
    radius_m: float,
) -> bool:
    xy = _validated_xy(poly)
    if len(xy) == 0:
        return False
    distance = np.linalg.norm(xy - ego_xy, axis=1)
    return bool(distance.min() <= radius_m)


def _in_forward_region(
    poly: scenario.MapPolyline,
    ego_xy: np.ndarray,
    ego_heading: float,
    half_angle_deg: float,
) -> bool:
    """True if any non-coincident point lies within half-angle of ego heading.

    Mirrors SceneBuilder's forward-hemisphere notion (mtr_types.hpp:47-48):
    90 degrees means the front hemisphere, including points directly beside
    ego. A point exactly at ego has no direction and cannot establish that a
    lane is forward.
    """
    rel = _validated_xy(poly) - ego_xy
    if len(rel) == 0:
        return False
    directional = np.linalg.norm(rel, axis=1) > _POSITION_EPSILON
    if not directional.any():
        return False
    rel = rel[directional]
    cos_h, sin_h = math.cos(ego_heading), math.sin(ego_heading)
    forward = rel[:, 0] * cos_h + rel[:, 1] * sin_h
    lateral = -rel[:, 0] * sin_h + rel[:, 1] * cos_h
    angle = np.abs(np.arctan2(lateral, forward))
    return bool((angle <= math.radians(half_angle_deg)).any())


def _validate_query(
    ego_xy: tuple[float, float],
    ego_heading: float,
    config: QueryConfig,
) -> np.ndarray:
    if config.strategy not in _STRATEGIES:
        raise ValueError(
            f"unknown strategy {config.strategy!r}; expected one of "
            f"{sorted(_STRATEGIES)}"
        )
    if not math.isfinite(config.radius_m) or config.radius_m < 0.0:
        raise ValueError("radius_m must be finite and non-negative")
    if (
        not math.isfinite(config.forward_half_angle_deg)
        or not 0.0 <= config.forward_half_angle_deg <= 180.0
    ):
        raise ValueError(
            "forward_half_angle_deg must be finite and within [0, 180]"
        )
    if not math.isfinite(ego_heading):
        raise ValueError("ego_heading must be finite")
    ego = np.asarray(ego_xy, dtype=np.float64)
    if ego.shape != (2,) or not np.isfinite(ego).all():
        raise ValueError("ego_xy must contain exactly two finite coordinates")
    return ego


def query(
    polylines: list[scenario.MapPolyline],
    ego_xy: tuple[float, float],
    ego_heading: float,
    config: QueryConfig,
) -> tuple[list[scenario.MapPolyline], ledger.MapStats]:
    """Return the queried lane polylines plus loss statistics."""
    ego = _validate_query(ego_xy, ego_heading, config)

    lanes = [poly for poly in polylines if poly.feature_type in LANE_TYPES]
    omitted: dict[str, int] = {}
    for poly in polylines:
        if poly.feature_type in DISCARDED_TYPES:
            omitted[poly.feature_type] = omitted.get(poly.feature_type, 0) + 1

    radius_hits = [
        poly for poly in lanes if _within_radius(poly, ego, config.radius_m)
    ]
    reachable_hits = [
        poly
        for poly in radius_hits
        if _in_forward_region(
            poly, ego, ego_heading, config.forward_half_angle_deg
        )
    ]
    kept = radius_hits if config.strategy == "radius" else reachable_hits

    return kept, ledger.MapStats(
        radius_m=config.radius_m,
        nearby_lanes=len(lanes),
        radius_only_emitted=len(radius_hits),
        reachable_emitted=len(reachable_hits),
        points_before=int(sum(len(poly.points) for poly in lanes)),
        points_after=int(sum(len(poly.points) for poly in kept)),
        omitted_by_type=omitted,
    )
