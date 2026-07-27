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

from wato_mtr_eval import acceleration_counterfactual as counterfactual
from wato_mtr_eval import lanelet_query as lq
from wato_mtr_eval import reference_pack, scenario
from wato_mtr_eval import representation as rep


def _object_tensor(
    velocity: list[float],
    present: list[bool],
) -> tuple[np.ndarray, np.ndarray]:
    obj = np.zeros(
        (1, 1, rep.HISTORY_STEPS, rep.OBJ_FEATURE_DIM),
        dtype=np.float32,
    )
    mask = np.zeros((1, 1, rep.HISTORY_STEPS), dtype=np.uint8)
    obj[0, 0, :, rep.OBJ_COL["velocity"].start] = velocity
    mask[0, 0] = present
    obj[0, 0, ~mask[0, 0].astype(bool)] = 0.0
    return obj, mask


def test_official_rule_matches_gap_free_wato_acceleration():
    velocity = list(range(rep.HISTORY_STEPS))
    obj, mask = _object_tensor(velocity, [True] * rep.HISTORY_STEPS)

    actual = counterfactual.official_acceleration(obj, mask)

    np.testing.assert_array_equal(actual[0, 0, :, 0], [10.0] * rep.HISTORY_STEPS)
    np.testing.assert_array_equal(actual[..., 1], 0.0)


def test_official_rule_copies_fixed_second_slot_instead_of_searching_later():
    velocity = [1.0, 0.0, 0.0, 4.0, 5.0] + [5.0] * 6
    present = [True, False, False, True, True] + [True] * 6
    obj, mask = _object_tensor(velocity, present)

    actual = counterfactual.official_acceleration(obj, mask)

    # Slot 0 copies slot 1's zero-filled difference: (0 - 1) * 10.
    assert actual[0, 0, 0, 0] == -10.0
    # The current WATO rule instead searches forward and would copy +10 from
    # slots 3 -> 4. Official MTR does not perform that search.
    assert actual[0, 0, 3, 0] == 40.0
    assert actual[0, 0, 4, 0] == 10.0
    np.testing.assert_array_equal(actual[0, 0, 1:3], 0.0)


def test_valid_slot_after_internal_gap_differences_against_zero_fill():
    velocity = [1.0, 2.0, 0.0, 4.0] + [4.0] * 7
    present = [True, True, False, True] + [True] * 7
    obj, mask = _object_tensor(velocity, present)

    actual = counterfactual.official_acceleration(obj, mask)

    assert actual[0, 0, 3, 0] == 40.0
    assert actual[0, 0, 2, 0] == 0.0


def test_counterfactual_changes_only_acceleration_columns():
    generator = np.random.default_rng(20260727)
    obj = generator.normal(size=(2, 3, rep.HISTORY_STEPS, rep.OBJ_FEATURE_DIM)).astype(
        np.float32
    )
    mask = generator.integers(0, 2, size=(2, 3, rep.HISTORY_STEPS), dtype=np.uint8)
    obj[~mask.astype(bool)] = 0.0
    original = {
        "obj_trajs": obj,
        "obj_trajs_mask": mask,
        "map_polylines": generator.normal(size=(2, 4, 3, 9)).astype(np.float32),
        "track_index_to_predict": np.asarray([0, 1], dtype=np.int64),
    }

    actual = counterfactual.counterfactual_bundle(original)

    assert actual.keys() == original.keys()
    np.testing.assert_array_equal(actual["obj_trajs"][..., :27], obj[..., :27])
    np.testing.assert_array_equal(actual["obj_trajs_mask"], mask)
    np.testing.assert_array_equal(actual["map_polylines"], original["map_polylines"])
    np.testing.assert_array_equal(
        actual["track_index_to_predict"],
        original["track_index_to_predict"],
    )
    assert not np.shares_memory(actual["obj_trajs"], original["obj_trajs"])
    assert not np.shares_memory(actual["map_polylines"], original["map_polylines"])


def test_exact_scenario_rule_uses_nonzero_velocity_from_invalid_raw_state():
    states = np.zeros((11, scenario.NUM_STATE_COLS), dtype=np.float64)
    states[:, scenario.STATE_COL["length"]] = 4.0
    states[:, scenario.STATE_COL["width"]] = 2.0
    states[:, scenario.STATE_COL["height"]] = 1.5
    states[:, scenario.STATE_COL["valid"]] = 1.0
    states[:, scenario.STATE_COL["vx"]] = np.arange(11)
    states[5, scenario.STATE_COL["valid"]] = 0.0
    states[5, scenario.STATE_COL["vx"]] = 3.0
    states[6, scenario.STATE_COL["vx"]] = 4.0
    scen = scenario.Scenario(
        scenario_id="invalid-velocity",
        timestamps_s=np.arange(11, dtype=np.float64) * 0.1,
        current_time_index=10,
        sdc_track_index=0,
        tracks=[
            scenario.Track(1, "vehicle", states.copy()),
            scenario.Track(2, "vehicle", states.copy()),
        ],
        tracks_to_predict=[1],
        map_polylines=[],
    )
    selections = reference_pack.target_selection(scen)
    packed = reference_pack.pack(scen, lq.QueryConfig(strategy="radius"))

    zero_filled = counterfactual.deployable_scenario_bundle(
        scen,
        packed,
        selections,
    )
    raw_invalid = counterfactual.upstream_raw_invalid_scenario_bundle(
        scen,
        packed,
        selections,
    )

    target_row = int(packed["track_index_to_predict"][0])
    # Packed velocity at invalid slot 5 is zero, so zero-fill sees
    # (4 - 0) / 0.1. Raw-invalid semantics see (4 - 3) / 0.1.
    assert zero_filled["obj_trajs"][0, target_row, 6, 27] == 40.0
    assert raw_invalid["obj_trajs"][0, target_row, 6, 27] == 10.0
    difference = raw_invalid["obj_trajs"] != zero_filled["obj_trajs"]
    changed_indices = np.argwhere(difference)
    assert changed_indices.size
    assert set(changed_indices[:, -1]) <= {27, 28}
    for target_slot, context_slot, history_slot, _ in changed_indices:
        assert packed["obj_trajs_mask"][target_slot, context_slot, history_slot]
    np.testing.assert_array_equal(
        raw_invalid["obj_trajs"][..., :27],
        zero_filled["obj_trajs"][..., :27],
    )
    for key in packed.keys() - {"obj_trajs"}:
        np.testing.assert_array_equal(raw_invalid[key], zero_filled[key])


def test_deployable_rule_is_byte_identical_to_wato_on_rotated_all_valid_history():
    states = np.zeros((11, scenario.NUM_STATE_COLS), dtype=np.float64)
    states[:, scenario.STATE_COL["length"]] = 4.17
    states[:, scenario.STATE_COL["width"]] = 1.93
    states[:, scenario.STATE_COL["height"]] = 1.61
    states[:, scenario.STATE_COL["valid"]] = 1.0
    steps = np.arange(11, dtype=np.float64)
    states[:, scenario.STATE_COL["x"]] = 0.31 * steps
    states[:, scenario.STATE_COL["y"]] = -0.23 * steps
    states[:, scenario.STATE_COL["heading"]] = 0.371
    states[:, scenario.STATE_COL["vx"]] = 0.137 + 0.373 * steps
    states[:, scenario.STATE_COL["vy"]] = -0.281 + 0.197 * steps
    other_states = states.copy()
    other_states[:, scenario.STATE_COL["heading"]] = -0.419
    other_states[:, scenario.STATE_COL["vx"]] = -0.213 + 0.117 * steps
    other_states[:, scenario.STATE_COL["vy"]] = 0.337 - 0.083 * steps
    scen = scenario.Scenario(
        scenario_id="rotated-all-valid",
        timestamps_s=0.1 * steps,
        current_time_index=10,
        sdc_track_index=0,
        tracks=[
            scenario.Track(1, "vehicle", other_states),
            scenario.Track(2, "vehicle", states),
        ],
        tracks_to_predict=[1],
        map_polylines=[],
    )
    selections = reference_pack.target_selection(scen)
    packed = reference_pack.pack(scen, lq.QueryConfig(strategy="radius"))

    deployable = counterfactual.deployable_scenario_bundle(
        scen,
        packed,
        selections,
    )

    packed_acceleration = packed["obj_trajs"][..., rep.OBJ_COL["acceleration"]]
    deployable_acceleration = deployable["obj_trajs"][..., rep.OBJ_COL["acceleration"]]
    assert deployable_acceleration.tobytes() == packed_acceleration.tobytes()


def test_metric_arms_require_identical_finite_masks():
    current = np.asarray([1.0, np.nan, 2.0])
    same_mask = np.asarray([3.0, np.nan, 4.0])

    counterfactual._require_identical_finite_masks(
        "minFDE",
        current,
        same_mask,
        same_mask,
    )

    with pytest.raises(RuntimeError, match="minFDE finite masks differ"):
        counterfactual._require_identical_finite_masks(
            "minFDE",
            current,
            np.asarray([3.0, 0.0, 4.0]),
            same_mask,
        )
