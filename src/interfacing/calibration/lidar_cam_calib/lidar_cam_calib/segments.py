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
"""Rest / creep / parked segment detection on a motion timeline.

The toolbox uses only the first image of each bag and anchors the integrated
cloud at the first scan, so every segment starts with the car at rest:

  creep   rest for >= min_rest, then gentle motion until creep_max_distance,
          creep_max_duration, a speed above creep_max_speed, or a new stop.
          Used with `preprocess -d` (dynamic integration).
  static  part of a rest period, no motion. Used without `-d`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class Segment:
    kind: str  # "creep" or "static"
    start_s: float  # since bag start
    end_s: float
    rest_s: float  # seconds of rest at the start
    distance_m: float
    max_speed_mps: float
    end_reason: str
    unreliable_frac: float  # share of samples where GICP was unusable

    def to_dict(self) -> dict:
        return {
            k: (round(v, 3) if isinstance(v, float) else v)
            for k, v in asdict(self).items()
        }


def rest_runs(t: np.ndarray, dt: np.ndarray, rest: np.ndarray, min_rest: float):
    """(first index, last index, start time, end time) of rest runs lasting >= min_rest."""
    runs = []
    i, n = 0, len(t)
    while i < n:
        if not rest[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and rest[j + 1]:
            j += 1
        start, end = t[i] - dt[i], t[j]
        if end - start >= min_rest:
            runs.append((i, j, float(start), float(end)))
        i = j + 1
    return runs


def detect(timeline, cfg: dict) -> list[Segment]:
    """Find creep and static segments. cfg is the `segments` block of calib.yaml."""
    t, dt, speed, rest = timeline.t, timeline.dt, timeline.speed, timeline.rest
    unreliable = ~timeline.reliable
    out: list[Segment] = []
    for i, j, rs, re in rest_runs(t, dt, rest, cfg["min_rest"]):
        # Static: the first static_duration seconds of the rest run, minus a margin
        margin = cfg["static_margin"]
        s0, s1 = rs + margin, min(re - margin, rs + margin + cfg["static_duration"])
        if s1 - s0 >= cfg["static_min_duration"]:
            m = (t > s0) & (t <= s1)
            out.append(
                Segment(
                    "static",
                    s0,
                    s1,
                    s1 - s0,
                    0.0,
                    0.0,
                    "rest",
                    float(unreliable[m].mean()),
                )
            )

        # Creep: walk forward from the end of the rest run
        if j + 1 >= len(t):
            continue
        dist, reason, k_end = 0.0, "bag_end", len(t) - 1
        still_since = None
        for k in range(j + 1, len(t)):
            v = speed[k]
            if not np.isfinite(v):
                reason, k_end = "motion_unknown", k - 1
                break
            if v > cfg["creep_max_speed"]:
                reason, k_end = "too_fast", k - 1
                break
            dist += v * dt[k]
            if rest[k]:
                still_since = still_since if still_since is not None else k
                if t[k] - (t[still_since] - dt[still_since]) >= cfg["creep_stop_time"]:
                    reason, k_end = "stopped", still_since - 1
                    break
            else:
                still_since = None
            if dist >= cfg["creep_max_distance"]:
                reason, k_end = "distance", k
                break
            if t[k] - re >= cfg["creep_max_duration"]:
                reason, k_end = "duration", k
                break
        if k_end <= j:
            continue
        dist = float(np.nansum(speed[j + 1 : k_end + 1] * dt[j + 1 : k_end + 1]))
        if dist < cfg["creep_min_distance"]:
            continue
        start = max(rs, re - cfg["pre_rest"])
        m = (t > start) & (t <= t[k_end])
        out.append(
            Segment(
                "creep",
                float(start),
                float(t[k_end]),
                float(re - start),
                dist,
                float(np.nanmax(speed[j + 1 : k_end + 1])),
                reason,
                float(unreliable[m].mean()),
            )
        )
    return out
