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
    return scenario.MapPolyline(
        feature_id=fid, feature_type=ftype, points=pts
    )


def _map():
    return [
        _line(1, "lane", [0, 10, 20]),
        _line(2, "lane", [200, 210, 220]),
        _line(3, "lane", [-30, -20, -10]),
        _line(4, "road_edge", [0, 10, 20], y=3),
        _line(5, "crosswalk", [5, 6], y=1),
    ]


def test_only_lane_features_survive():
    kept, _ = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0)
    )
    assert {p.feature_type for p in kept} == {"lane"}


def test_radius_filters_distant_lanes():
    kept, _ = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0)
    )
    assert {p.feature_id for p in kept} == {1, 3}


def test_reachable_strategy_drops_lanes_behind_ego():
    cfg = lq.QueryConfig(radius_m=80.0, strategy="reachable")
    kept, _ = lq.query(_map(), (0.0, 0.0), 0.0, cfg)
    assert {p.feature_id for p in kept} == {1}, "lane 3 is behind ego"


def test_radius_only_keeps_more_than_reachable():
    radius, _ = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="radius")
    )
    reachable, _ = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="reachable")
    )
    assert len(radius) >= len(
        reachable
    ), "radius-only is the upper-context baseline"


def test_stats_count_omitted_semantics_by_type():
    _, stats = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0)
    )
    assert stats.omitted_by_type["road_edge"] == 1
    assert stats.omitted_by_type["crosswalk"] == 1


def test_stats_report_both_strategies_regardless_of_which_ran():
    """The radius-vs-reachable delta must be recordable from a single run."""
    _, stats = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="reachable")
    )
    assert stats.radius_only_emitted == 2
    assert stats.reachable_emitted == 1


def test_stats_count_points_before_and_after():
    _, stats = lq.query(
        _map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0)
    )
    assert stats.points_before == 3 + 3 + 3
    assert stats.points_after == 3 + 3


def test_rejects_unknown_strategy():
    with pytest.raises(ValueError, match="unknown strategy"):
        lq.query(
            _map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="magic")
        )


def test_heading_rotates_the_forward_region():
    """Facing -x, the lane behind becomes the lane ahead."""
    cfg = lq.QueryConfig(strategy="reachable")
    kept, _ = lq.query(_map(), (0.0, 0.0), np.pi, cfg)
    assert {p.feature_id for p in kept} == {3}


def test_empty_lane_is_counted_but_never_emitted():
    empty = scenario.MapPolyline(
        feature_id=9,
        feature_type="lane",
        points=np.empty((0, 3), dtype=np.float64),
    )

    kept, stats = lq.query(
        [empty], (0.0, 0.0), 0.0, lq.QueryConfig()
    )

    assert kept == []
    assert stats.nearby_lanes == 1
    assert stats.points_before == stats.points_after == 0


def test_single_point_lane_is_valid_geometry():
    point = _line(10, "lane", [1.0])

    kept, _ = lq.query(
        [point], (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=2.0)
    )

    assert kept == [point]


@pytest.mark.parametrize("radius", [-1.0, np.nan, np.inf])
def test_rejects_invalid_radius(radius):
    with pytest.raises(ValueError, match="radius_m"):
        lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=radius))


@pytest.mark.parametrize("angle", [-1.0, 181.0, np.nan, np.inf])
def test_rejects_invalid_forward_half_angle(angle):
    with pytest.raises(ValueError, match="forward_half_angle_deg"):
        lq.query(
            _map(),
            (0.0, 0.0),
            0.0,
            lq.QueryConfig(forward_half_angle_deg=angle),
        )


def test_rejects_malformed_or_nonfinite_lane_points():
    malformed = scenario.MapPolyline(
        feature_id=11,
        feature_type="lane",
        points=np.array([1.0, 2.0]),
    )
    nonfinite = _line(12, "lane", [np.nan])

    with pytest.raises(ValueError, match="feature 11"):
        lq.query([malformed], (0.0, 0.0), 0.0, lq.QueryConfig())
    with pytest.raises(ValueError, match="feature 12"):
        lq.query([nonfinite], (0.0, 0.0), 0.0, lq.QueryConfig())
