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
"""Metric computations and limit checks for SIL runs (pure Python + numpy).

Kept free of ROS so the logic is independently unit-testable. The analyzer node
extracts time-series from the recorded bag and feeds them to these functions.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np


@dataclass
class LimitResult:
    name: str
    kind: str  # "upper" or "lower"
    limit: float
    observed: float
    passed: bool
    first_violation_time: Optional[float]


def lateral_acceleration_from_steering(
    speed: Sequence[float], steering: Sequence[float], wheelbase: float
) -> np.ndarray:
    """Kinematic-bicycle lateral acceleration: a = v^2 * tan(delta) / L."""
    speed = np.asarray(speed, dtype=float)
    steering = np.asarray(steering, dtype=float)
    return speed**2 * np.tan(steering) / wheelbase


def steering_rate(steering: Sequence[float], times: Sequence[float]) -> np.ndarray:
    steering = np.asarray(steering, dtype=float)
    times = np.asarray(times, dtype=float)
    if steering.size < 2:
        return np.zeros_like(steering)
    return np.gradient(steering, times)


def yaw_rate_from_headings(yaw: Sequence[float], times: Sequence[float]) -> np.ndarray:
    yaw = np.unwrap(np.asarray(yaw, dtype=float))
    times = np.asarray(times, dtype=float)
    if yaw.size < 2:
        return np.zeros_like(yaw)
    return np.gradient(yaw, times)


def _point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Shortest distance from point P to segment AB."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab_sq = abx * abx + aby * aby
    if ab_sq == 0.0:
        return float(np.hypot(apx, apy))
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
    cx, cy = ax + t * abx, ay + t * aby
    return float(np.hypot(px - cx, py - cy))


def distance_point_to_polyline(
    px: float, py: float, route_xy: Sequence[Sequence[float]]
) -> float:
    """Shortest distance from a single point to the reference polyline."""
    route = np.asarray(route_xy, dtype=float)
    if route.shape[0] < 2:
        raise ValueError("route polyline needs at least 2 points")
    return min(
        _point_to_segment_distance(
            px, py, route[j, 0], route[j, 1], route[j + 1, 0], route[j + 1, 1]
        )
        for j in range(route.shape[0] - 1)
    )


def cross_track_error(
    xs: Sequence[float], ys: Sequence[float], route_xy: Sequence[Sequence[float]]
) -> np.ndarray:
    """Per-sample shortest distance from the ego position to the reference polyline."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    route = np.asarray(route_xy, dtype=float)
    if route.shape[0] < 2:
        raise ValueError("route polyline needs at least 2 points")

    errors = np.empty(xs.shape[0], dtype=float)
    for i in range(xs.shape[0]):
        best = np.inf
        for j in range(route.shape[0] - 1):
            d = _point_to_segment_distance(
                xs[i], ys[i],
                route[j, 0], route[j, 1],
                route[j + 1, 0], route[j + 1, 1],
            )
            best = min(best, d)
        errors[i] = best
    return errors


def min_distance_to_actors(
    ego_xy: Sequence[Sequence[float]],
    actor_xy_per_sample: Sequence[Sequence[Sequence[float]]],
) -> np.ndarray:
    """Per-sample minimum distance from ego to any actor.

    actor_xy_per_sample[i] is the list of (x, y) for all actors at sample i.
    Samples with no actors yield +inf (no proximity constraint active).
    """
    ego = np.asarray(ego_xy, dtype=float)
    out = np.empty(ego.shape[0], dtype=float)
    for i in range(ego.shape[0]):
        actors = actor_xy_per_sample[i] if i < len(actor_xy_per_sample) else []
        if not len(actors):
            out[i] = np.inf
            continue
        a = np.asarray(actors, dtype=float)
        out[i] = float(np.min(np.hypot(a[:, 0] - ego[i, 0], a[:, 1] - ego[i, 1])))
    return out


def check_limit(
    name: str,
    series: Sequence[float],
    times: Sequence[float],
    limit: float,
) -> LimitResult:
    """Compare a metric series against a named limit.

    The comparison direction is inferred from the name prefix: ``max_*`` is an upper
    bound (series must stay <= limit), ``min_*`` is a lower bound (series must stay
    >= limit). Returns the worst observed value and the first violation time, if any.
    """
    series = np.asarray(series, dtype=float)
    times = np.asarray(times, dtype=float)
    finite = series[np.isfinite(series)]

    if name.startswith("max_"):
        kind = "upper"
        violation = series > limit
        observed = float(np.max(finite)) if finite.size else 0.0
    elif name.startswith("min_"):
        kind = "lower"
        violation = series < limit
        observed = float(np.min(finite)) if finite.size else float("inf")
    else:
        raise ValueError(f"limit '{name}' must start with 'max_' or 'min_'")

    first_t: Optional[float] = None
    if violation.any() and times.size == series.size:
        first_t = float(times[np.argmax(violation)])

    return LimitResult(name, kind, limit, observed, not bool(violation.any()), first_t)


def overall_passed(results: List[LimitResult]) -> bool:
    return all(r.passed for r in results)
