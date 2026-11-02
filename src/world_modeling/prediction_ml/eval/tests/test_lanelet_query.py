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

import numpy as np
import pytest

from wato_mtr_eval import lanelet_query as lq, scenario


def _line(fid, ftype, xs, y=0.0):
    pts = np.array([[x, y, 0.0] for x in xs], dtype=np.float64)
    return scenario.MapPolyline(feature_id=fid, feature_type=ftype, points=pts)


def _map():
    return [
        _line(1, "lane", [0, 10, 20]),
        _line(2, "lane", [200, 210, 220]),
        _line(3, "lane", [-30, -20, -10]),
        _line(4, "road_edge", [0, 10, 20], y=3),
        _line(5, "crosswalk", [5, 6], y=1),
    ]


def test_only_lane_features_survive():
    kept = lq.query(_map(), (0.0, 0.0), lq.QueryConfig(radius_m=80.0))
    assert {p.feature_type for p in kept} == {"lane"}


def test_radius_filters_distant_lanes():
    kept = lq.query(_map(), (0.0, 0.0), lq.QueryConfig(radius_m=80.0))
    assert {p.feature_id for p in kept} == {1, 3}


def test_empty_lane_is_never_emitted():
    empty = scenario.MapPolyline(
        feature_id=9,
        feature_type="lane",
        points=np.empty((0, 3), dtype=np.float64),
    )
    assert lq.query([empty], (0.0, 0.0), lq.QueryConfig()) == []


def test_single_point_lane_is_valid_geometry():
    point = _line(10, "lane", [1.0])
    assert lq.query([point], (0.0, 0.0), lq.QueryConfig(radius_m=2.0)) == [point]


@pytest.mark.parametrize("radius", [-1.0, np.nan, np.inf])
def test_rejects_invalid_radius(radius):
    with pytest.raises(ValueError, match="radius_m"):
        lq.query(_map(), (0.0, 0.0), lq.QueryConfig(radius_m=radius))


def test_rejects_malformed_or_nonfinite_lane_points():
    malformed = scenario.MapPolyline(
        feature_id=11,
        feature_type="lane",
        points=np.array([1.0, 2.0]),
    )
    nonfinite = _line(12, "lane", [np.nan])

    with pytest.raises(ValueError, match="feature 11"):
        lq.query([malformed], (0.0, 0.0), lq.QueryConfig())
    with pytest.raises(ValueError, match="feature 12"):
        lq.query([nonfinite], (0.0, 0.0), lq.QueryConfig())
