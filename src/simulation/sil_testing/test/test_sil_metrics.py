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
import math

import numpy as np
import pytest

from sil_testing import sil_metrics as m


def test_lateral_accel_zero_when_straight():
    a = m.lateral_acceleration_from_steering([10.0, 5.0], [0.0, 0.0], 2.0)
    assert np.allclose(a, 0.0)


def test_lateral_accel_known_value():
    # v=10, tan(delta)=0.1, L=2 -> a = 100 * 0.1 / 2 = 5.0
    delta = math.atan(0.1)
    a = m.lateral_acceleration_from_steering([10.0], [delta], 2.0)
    assert a[0] == pytest.approx(5.0, rel=1e-6)


def test_cross_track_error_offset_from_straight_line():
    # Route along y=0; ego sits 0.5 m to the left.
    route = [[0.0, 0.0], [10.0, 0.0]]
    cte = m.cross_track_error([5.0], [0.5], route)
    assert cte[0] == pytest.approx(0.5)


def test_distance_point_to_polyline_endpoint():
    route = [[0.0, 0.0], [10.0, 0.0]]
    # Beyond the segment end, distance clamps to the endpoint.
    assert m.distance_point_to_polyline(13.0, 0.0, route) == pytest.approx(3.0)


def test_check_limit_upper_violation():
    series = [1.0, 2.0, 5.0, 3.0]
    times = [0.0, 0.1, 0.2, 0.3]
    r = m.check_limit("max_lateral_accel_mps2", series, times, 4.0)
    assert not r.passed
    assert r.observed == pytest.approx(5.0)
    assert r.first_violation_time == pytest.approx(0.2)


def test_check_limit_upper_pass():
    r = m.check_limit("max_speed_mps", [1.0, 2.0, 3.0], [0.0, 1.0, 2.0], 4.0)
    assert r.passed
    assert r.first_violation_time is None


def test_check_limit_lower_violation():
    # min distance dips below the threshold at the second sample.
    r = m.check_limit("min_distance_to_actor_m", [5.0, 1.0, 4.0], [0.0, 0.1, 0.2], 2.0)
    assert not r.passed
    assert r.observed == pytest.approx(1.0)
    assert r.first_violation_time == pytest.approx(0.1)


def test_check_limit_ignores_infinite_for_lower_bound():
    # No actors -> +inf samples should not be treated as the observed minimum.
    r = m.check_limit("min_distance_to_actor_m", [np.inf, 3.0, np.inf], [0.0, 0.1, 0.2], 2.0)
    assert r.passed
    assert r.observed == pytest.approx(3.0)


def test_steering_rate():
    rate = m.steering_rate([0.0, 0.1, 0.2], [0.0, 1.0, 2.0])
    assert np.allclose(rate, 0.1)


def test_min_distance_to_actors():
    ego = [[0.0, 0.0], [0.0, 0.0]]
    actors = [[(3.0, 4.0)], [(1.0, 0.0), (10.0, 0.0)]]
    out = m.min_distance_to_actors(ego, actors)
    assert out[0] == pytest.approx(5.0)
    assert out[1] == pytest.approx(1.0)
