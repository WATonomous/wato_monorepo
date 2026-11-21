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
from types import SimpleNamespace

import numpy as np
import pytest

from lidar_cam_calib.config import load_config
from lidar_cam_calib.segments import detect, rest_runs

CFG = load_config()["segments"]
DT = 0.25


def timeline(*pieces):
    """pieces: (seconds, speed m/s); rest wherever speed < 0.05."""
    speed = np.concatenate([np.full(int(s / DT), v) for s, v in pieces])
    t = (np.arange(len(speed)) + 1) * DT
    return SimpleNamespace(
        t=t,
        dt=np.full(len(t), DT),
        speed=speed,
        rest=speed < 0.05,
        reliable=np.ones(len(t), bool),
    )


def kinds(segs):
    return {s.kind: s for s in segs}


def test_rest_then_creep_then_fast():
    segs = kinds(detect(timeline((10, 0.0), (20, 0.5), (10, 5.0)), CFG))
    creep = segs["creep"]
    assert creep.start_s == pytest.approx(7.0)  # 3 s of rest kept before the start
    assert creep.rest_s == pytest.approx(3.0)
    assert creep.distance_m == pytest.approx(5.0, abs=0.13)
    assert creep.end_reason == "distance"
    static = segs["static"]
    assert (static.start_s, static.end_s) == pytest.approx((0.5, 9.5))


def test_fast_start_gives_no_creep():
    segs = detect(timeline((10, 0.0), (10, 3.0)), CFG)
    assert [s.kind for s in segs] == ["static"]


def test_creep_ends_at_a_new_stop():
    segs = kinds(detect(timeline((5, 0.0), (6, 0.5), (5, 0.0)), CFG))
    creep = segs["creep"]
    assert creep.end_reason == "stopped"
    assert creep.end_s == pytest.approx(11.0)
    assert creep.distance_m == pytest.approx(3.0)


def test_short_creep_is_dropped():
    segs = detect(timeline((5, 0.0), (1, 0.5), (5, 0.0)), CFG)
    assert "creep" not in [s.kind for s in segs]


def test_unknown_speed_ends_creep():
    tl = timeline((5, 0.0), (10, 0.5))
    tl.speed[int(9 / DT) :] = np.nan
    creep = kinds(detect(tl, CFG))["creep"]
    assert creep.end_reason == "motion_unknown"
    assert creep.end_s == pytest.approx(9.0)


def test_rest_runs_respects_min_rest():
    tl = timeline((2, 0.0), (1, 1.0), (4, 0.0))
    runs = rest_runs(tl.t, tl.dt, tl.rest, 3.0)
    assert [(round(a, 2), round(b, 2)) for _, _, a, b in runs] == [(3.0, 7.0)]
