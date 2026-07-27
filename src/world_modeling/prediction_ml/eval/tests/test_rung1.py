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

from __future__ import annotations

import numpy as np
import pytest
from waymo_open_dataset.protos import scenario_pb2

from wato_mtr_eval import (
    lanelet_query as lq,
    reference_pack,
    rung1,
    scenario,
)

S = scenario.STATE_COL


def _states(x: float) -> np.ndarray:
    states = np.zeros((91, scenario.NUM_STATE_COLS), dtype=np.float64)
    states[:, S["x"]] = x + np.arange(91) * 0.1
    states[:, S["length"]] = 4.0
    states[:, S["width"]] = 2.0
    states[:, S["height"]] = 1.5
    states[:, S["vx"]] = 1.0
    states[:, S["valid"]] = 1.0
    return states


def _polyline(
    feature_id: int,
    feature_type: str,
    global_type: int,
    y: float,
) -> scenario.MapPolyline:
    points = np.asarray(
        [[0.0, y, 0.0], [10.0, y, 0.0], [20.0, y, 0.0]]
    )
    return scenario.MapPolyline(
        feature_id,
        feature_type,
        points,
        global_type=global_type,
    )


def _scenario() -> scenario.Scenario:
    return scenario.Scenario(
        scenario_id="ablation",
        timestamps_s=np.arange(91) * 0.1,
        current_time_index=10,
        sdc_track_index=0,
        tracks=[
            scenario.Track(1, "vehicle", _states(0.0)),
            scenario.Track(2, "vehicle", _states(5.0)),
        ],
        tracks_to_predict=[1],
        map_polylines=[
            _polyline(1, "lane", 2, 0.0),
            _polyline(2, "road_line", 7, 1.0),
            _polyline(3, "road_edge", 15, 2.0),
            _polyline(4, "crosswalk", 18, 3.0),
            _polyline(5, "speed_bump", 19, 4.0),
            _polyline(6, "driveway", 20, 5.0),
        ],
    )


def test_sweep_has_explicit_supported_baseline_and_wato_endpoints():
    assert rung1.SWEEP[0].name == "all_supported_typed_radius"
    assert rung1.SWEEP[0].keep_types == {
        "lane",
        "road_line",
        "road_edge",
        "crosswalk",
        "speed_bump",
        "driveway",
    }
    assert rung1.SWEEP[-1].name == "lane_only_collapsed_reachable"
    assert rung1.SWEEP[-1].keep_types == {"lane"}
    assert rung1.SWEEP[-1].collapse_global_type is True


def test_sweep_is_monotonic_and_geometry_steps_remove_one_layer():
    for previous, current in zip(rung1.SWEEP, rung1.SWEEP[1:]):
        assert current.keep_types <= previous.keep_types
        removed = previous.keep_types - current.keep_types
        if removed:
            assert len(removed) == 1, (
                f"{current.name} removes multiple layers: {removed}"
            )


def test_sweep_names_and_policy_states_are_unique():
    names = [ablation.name for ablation in rung1.SWEEP]
    assert names == [
        "all_supported_typed_radius",
        "no_driveways",
        "no_speed_bumps",
        "no_road_edges",
        "no_crosswalks",
        "no_road_lines",
        "lane_only_collapsed_radius",
        "lane_only_collapsed_reachable",
    ]
    states = [
        (
            ablation.keep_types,
            ablation.collapse_global_type,
            ablation.query_strategy,
        )
        for ablation in rung1.SWEEP
    ]
    assert len(names) == len(set(names))
    assert len(states) == len(set(states))


def test_collapsed_radius_policy_is_gate_b_default_byte_equivalent():
    scen = _scenario()
    query = lq.QueryConfig(strategy="radius")
    default = reference_pack.pack(scen, query)
    policy = next(
        item
        for item in rung1.SWEEP
        if item.name == "lane_only_collapsed_radius"
    )
    explicit = reference_pack.pack(
        scen, query, map_options=policy.map_options()
    )

    assert default.keys() == explicit.keys()
    for name in default:
        np.testing.assert_array_equal(default[name], explicit[name])


def test_typed_policy_keeps_layers_and_writes_exact_official_tokens():
    scen = _scenario()
    tensors = reference_pack.pack(
        scen,
        lq.QueryConfig(strategy="radius"),
        map_options=rung1.SWEEP[0].map_options(),
    )
    mask = tensors["map_polylines_mask"][0].astype(bool)
    tokens = tensors["map_polylines"][0, :, :, 6][mask]

    assert set(tokens.tolist()) == {2.0, 7.0, 15.0, 18.0, 19.0, 20.0}


@pytest.mark.parametrize(
    ("field", "subtype", "expected"),
    [
        ("lane", 0, -1),
        ("lane", 3, 3),
        ("road_line", 1, 6),
        ("road_line", 8, 13),
        ("road_edge", 1, 15),
        ("road_edge", 2, 16),
    ],
)
def test_scenario_parser_preserves_official_subtype_token(
    field,
    subtype,
    expected,
):
    proto = scenario_pb2.Scenario()
    feature = proto.map_features.add()
    feature.id = 1
    feature_data = getattr(feature, field)
    feature_data.type = subtype
    point = feature_data.polyline.add()
    point.x = 1.0

    parsed = scenario._polylines_from_proto(proto)

    assert parsed[0].global_type == expected


@pytest.mark.parametrize(
    ("field", "expected"),
    [("crosswalk", 18), ("speed_bump", 19), ("driveway", 20)],
)
def test_scenario_parser_preserves_fixed_polygon_token(field, expected):
    proto = scenario_pb2.Scenario()
    feature = proto.map_features.add()
    feature.id = 1
    polygon = getattr(feature, field).polygon
    point = polygon.add()
    point.x = 1.0

    parsed = scenario._polylines_from_proto(proto)

    assert parsed[0].global_type == expected


def test_lanelet_query_can_explicitly_query_non_lane_layers():
    kept, _ = lq.query(
        _scenario().map_polylines,
        (0.0, 0.0),
        0.0,
        lq.QueryConfig(radius_m=80.0),
        feature_types={"road_edge", "crosswalk"},
    )
    assert {item.feature_type for item in kept} == {
        "road_edge",
        "crosswalk",
    }


def test_map_budget_reports_capacity_evictions_by_coarse_layer():
    scen = _scenario()
    # 769 chunks, alternating lane and road edge, force one eviction.
    polylines = []
    for index in range(769):
        feature_type = "lane" if index % 2 == 0 else "road_edge"
        global_type = 2 if feature_type == "lane" else 15
        polylines.append(
            _polyline(index, feature_type, global_type, float(index) * 0.01)
        )
    scen.map_polylines[:] = polylines
    options = reference_pack.MapPackingOptions(
        {"lane", "road_edge"}, collapse_global_type=False
    )

    budget = reference_pack.map_packing_budget(
        scen, lq.QueryConfig(radius_m=1000.0), options
    )

    assert budget.candidate_chunks == 769
    assert budget.retained_target_chunks == 768
    assert sum(budget.evicted_target_chunks_by_type.values()) == 1


def test_adjacent_report_delta_is_current_minus_previous():
    first = rung1._Accumulator(
        ade=[np.asarray([1.0])],
        fde=[np.asarray([2.0])],
        agent_ids=["a"],
        scenario_ids=["s"],
    )
    second = rung1._Accumulator(
        ade=[np.asarray([1.25])],
        fde=[np.asarray([1.5])],
        agent_ids=["a"],
        scenario_ids=["s"],
    )

    reports = rung1._reports_from_accumulators(
        rung1.SWEEP[:2], [first, second], num_scenarios=1
    )

    assert reports[1].paired_minade_from_previous.mean == pytest.approx(0.25)
    assert reports[1].paired_minfde_from_previous.mean == pytest.approx(-0.5)


def test_run_rejects_cpu_before_external_work(tmp_path):
    with pytest.raises(ValueError, match="requires.*cuda"):
        rung1.run(
            tmp_path / "shard",
            tmp_path / "checkpoint",
            tmp_path,
            rung1.SWEEP[-1],
            device="cpu",
        )
