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

import json

import pytest

from wato_mtr_eval import ledger, representation as rep


def _map_stats():
    return ledger.MapStats(
        radius_m=80.0,
        nearby_lanes=37,
        radius_only_emitted=37,
        reachable_emitted=16,
        points_before=611,
        points_after=284,
        omitted_by_type={
            "road_edge": 18,
            "road_line": 24,
            "crosswalk": 3,
            "stop_sign": 1,
        },
    )


def _ledger(**overrides):
    kwargs = dict(
        scenario_id="abc123",
        num_waymo_tracks=54,
        num_type_mapped=48,
        dropped_by_type={"other": 6},
        num_candidates=48,
        selected_target_indices=[0, 1, 2, 3, 4, 5, 6, 7],
        heuristic_target_indices=[0, 1, 2, 3, 4, 90, 91, 92],
        history_valid=491,
        history_total=528,
        map_stats=_map_stats(),
    )
    kwargs.update(overrides)
    return ledger.build_ledger(**kwargs)


def test_agent_drops_are_counted_not_silent():
    lg = _ledger()
    assert lg.agents["waymo_tracks"] == 54
    assert lg.agents["type_mapped"] == 48
    assert lg.agents["dropped"] == 6
    assert lg.agents["dropped_by_type"] == {"other": 6}


def test_context_overflow_is_reported_when_agents_exceed_capacity():
    lg = _ledger(num_waymo_tracks=141, num_type_mapped=141)
    assert lg.agents["context_capacity"] == rep.CONTEXT_CAPACITY
    assert lg.agents["context_overflow"] == 141 - rep.CONTEXT_CAPACITY


def test_no_overflow_reported_when_under_capacity():
    assert _ledger().agents["context_overflow"] == 0


def test_target_selection_records_heuristic_agreement():
    """Pinned selection is what runs; the heuristic's choice is recorded free."""
    lg = _ledger()
    assert lg.targets["selected"] == 8
    assert lg.targets["pinned"] is True
    assert lg.targets["heuristic_agreement"] == 5
    assert lg.targets["heuristic_agreement_fraction"] == pytest.approx(5 / 8)


def test_history_masking_fraction():
    lg = _ledger()
    assert lg.history["valid"] == 491
    assert lg.history["masked"] == 37
    assert lg.history["masked_fraction"] == pytest.approx(37 / 528)


def test_map_section_records_what_the_query_and_packing_discarded():
    lg = _ledger().map_
    assert lg["nearby_lanes"] == 37
    assert lg["radius_only_emitted"] == 37
    assert lg["reachable_emitted"] == 16
    assert lg["dropped_by_connectivity"] == 21
    assert lg["centerline_points_before"] == 611
    assert lg["centerline_points_after"] == 284
    assert lg["omitted_semantics"]["road_edge"] == 18
    assert lg["global_type_collapse"] == "20 waymo types -> 1 wato token"


def test_output_section_records_the_horizon_clamp():
    lg = _ledger().output
    assert lg["model_steps"] == rep.NUM_FUTURE_FRAMES
    assert lg["stride"] == 2
    assert lg["emitted_steps"] == rep.DEPLOYED_STEPS
    assert lg["emitted_horizon_s"] == pytest.approx(rep.DEPLOYED_HORIZON_S)


def test_format_is_human_readable_and_names_every_section():
    text = _ledger().format()
    for section in ("AGENTS", "TARGETS", "HISTORY", "MAP", "OUTPUT"):
        assert section in text


def test_json_round_trips(tmp_path):
    path = tmp_path / "ledger.json"
    _ledger().to_json(path)
    loaded = json.loads(path.read_text())
    assert loaded["scenario_id"] == "abc123"
    assert loaded["agents"]["dropped"] == 6
