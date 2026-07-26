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

import pathlib

import pytest

pytest.importorskip(
    "waymo_open_dataset.protos.scenario_pb2",
    reason="install waymo-open-dataset to parse Scenario protos",
)

from wato_mtr_eval import representation as rep
from wato_mtr_eval import scenario

SHARD = (
    pathlib.Path(__file__).resolve().parents[5]
    / "uncompressed_scenario_validation_validation.tfrecord-00000-of-00150"
)
needs_shard = pytest.mark.skipif(
    not SHARD.exists(), reason="waymo validation shard not present"
)


@pytest.fixture(scope="module")
def first_scenario():
    return next(scenario.load_scenarios(SHARD, limit=1))


@needs_shard
def test_scenario_has_the_expected_temporal_layout(first_scenario):
    assert first_scenario.current_time_index == 10, "10 past steps + current"
    assert len(first_scenario.timestamps_s) == 91, "11 history + 80 future"
    assert (
        len(first_scenario.timestamps_s) - first_scenario.current_time_index - 1
        == rep.NUM_FUTURE_FRAMES
    )


@needs_shard
def test_tracks_carry_full_state_arrays(first_scenario):
    assert first_scenario.tracks, "scenario must contain tracks"
    for track in first_scenario.tracks:
        assert track.states.shape == (91, 10)
        assert track.object_type in {"vehicle", "pedestrian", "cyclist", "other"}


@needs_shard
def test_sdc_and_targets_are_valid_track_indices(first_scenario):
    assert 0 <= first_scenario.sdc_track_index < len(first_scenario.tracks)
    assert first_scenario.tracks_to_predict, "validation scenarios designate targets"
    assert all(
        0 <= index < len(first_scenario.tracks)
        for index in first_scenario.tracks_to_predict
    )
    assert len(first_scenario.tracks_to_predict) <= rep.TARGET_CAPACITY, (
        "more designated targets than the contract's 8 slots"
    )


@needs_shard
def test_map_polylines_are_typed_and_include_more_than_lanes(first_scenario):
    """Waymo ships the map types whose loss the rung-1 sweep will price."""
    types = {polyline.feature_type for polyline in first_scenario.map_polylines}
    assert "lane" in types
    assert types - {"lane"}, "expected road edges / crosswalks / lines beyond lanes"


@needs_shard
def test_shard_yields_the_documented_scenario_count():
    """DEVELOPING.md records 286 scenarios in this shard."""
    assert sum(1 for _ in scenario.load_scenarios(SHARD)) == 286
