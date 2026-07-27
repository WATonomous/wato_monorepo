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

import pathlib
import types

import numpy as np
import pytest

from wato_mtr_eval import representation as rep
from wato_mtr_eval import rung4

FIXTURE = (
    pathlib.Path(__file__).resolve().parents[2]
    / "test"
    / "fixtures"
    / "923a0c6bb90f4127"
)
SOURCE_NS = 1_999_900_000


def _mode(xy: np.ndarray, confidence: float = 0.5) -> rung4.RecordedMode:
    step_ns = round(rep.DEPLOYED_STEP_S * 1_000_000_000)
    return rung4.RecordedMode(
        confidence=confidence,
        source_stamp_ns=SOURCE_NS,
        pose_stamps_ns=SOURCE_NS
        + np.arange(1, rep.DEPLOYED_STEPS + 1, dtype=np.int64) * step_ns,
        xy=np.asarray(xy, dtype=np.float64),
    )


def _fixture_gt() -> tuple[np.ndarray, np.ndarray]:
    with np.load(FIXTURE / "ground_truth.npz", allow_pickle=False) as data:
        return (
            data["gt"][:, 1 : 1 + rep.DEPLOYED_STEPS * 2 : 2],
            data["valid"][:, 1 : 1 + rep.DEPLOYED_STEPS * 2 : 2],
        )


def test_score_scenario_joins_runtime_ids_to_waymo_ids_and_rung3():
    gt, _ = _fixture_gt()
    recorded = [
        rung4.RecordedObject("2", SOURCE_NS, (_mode(gt[0]),)),
        rung4.RecordedObject("__ego__", SOURCE_NS, (_mode(gt[1]),)),
    ]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    rows = rung4.score_scenario(FIXTURE, recorded, SOURCE_NS, reference)

    assert [(row.waymo_id, row.runtime_id) for row in rows] == [
        ("2", "2"),
        ("9", "__ego__"),
    ]
    assert [row.minfde for row in rows] == pytest.approx([0.0, 0.0])
    assert [row.delta_minfde for row in rows] == pytest.approx([0.0, 0.0])
    assert [row.world_seed_status for row in rows] == [
        "not_checked",
        "not_checked",
    ]


def test_score_prefers_later_more_complete_diagnostic_cache_output():
    gt, _ = _fixture_gt()
    far = gt[0] + 100.0
    recorded = [
        rung4.RecordedObject("2", SOURCE_NS, (_mode(far, 1.0),)),
        rung4.RecordedObject(
            "2",
            SOURCE_NS + 50_000_000,
            (_mode(gt[0], 0.7), _mode(far, 0.3)),
        ),
        rung4.RecordedObject("__ego__", SOURCE_NS, (_mode(gt[1]),)),
    ]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    rows = rung4.score_scenario(FIXTURE, recorded, SOURCE_NS, reference)

    assert rows[0].mode_count == 2
    assert rows[0].minfde == pytest.approx(0.0)


def test_world_seed_fallback_and_missing_are_reported_but_not_scored():
    gt, _ = _fixture_gt()
    far = gt + 100.0
    diagnostic = [
        rung4.RecordedObject("2", SOURCE_NS, (_mode(gt[0], 0.7), _mode(far[0], 0.3))),
        rung4.RecordedObject(
            "__ego__",
            SOURCE_NS,
            (_mode(gt[1], 0.7), _mode(far[1], 0.3)),
        ),
    ]
    world_seeds = [rung4.RecordedObject("2", SOURCE_NS, (_mode(far[0], 1.0),))]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    rows = rung4.score_scenario(
        FIXTURE,
        diagnostic,
        SOURCE_NS,
        reference,
        world_seeds=world_seeds,
    )

    assert [row.minfde for row in rows] == pytest.approx([0.0, 0.0])
    assert [row.world_seed_status for row in rows] == ["fallback", "missing"]


def test_single_mode_world_seed_matching_diagnostic_is_mtr_not_fallback():
    gt, _ = _fixture_gt()
    far = gt[0] + 100.0
    diagnostic = [
        rung4.RecordedObject("2", SOURCE_NS, (_mode(gt[0], 1.0),)),
        rung4.RecordedObject("__ego__", SOURCE_NS, (_mode(gt[1], 1.0),)),
    ]
    world_seeds = [
        rung4.RecordedObject("2", SOURCE_NS, (_mode(far, 1.0),)),
        *diagnostic,
    ]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    rows = rung4.score_scenario(
        FIXTURE,
        diagnostic,
        SOURCE_NS,
        reference,
        world_seeds=world_seeds,
    )

    assert [row.world_seed_status for row in rows] == ["mtr", "mtr"]


def test_missing_pinned_runtime_target_is_not_silently_dropped():
    gt, _ = _fixture_gt()
    recorded = [rung4.RecordedObject("2", SOURCE_NS, (_mode(gt[0]),))]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    with pytest.raises(ValueError, match="__ego__"):
        rung4.score_scenario(FIXTURE, recorded, SOURCE_NS, reference)


def test_wrong_prediction_cadence_is_rejected():
    gt, _ = _fixture_gt()
    bad = _mode(gt[0])
    bad.pose_stamps_ns[4] += 1
    recorded = [
        rung4.RecordedObject("2", SOURCE_NS, (bad,)),
        rung4.RecordedObject("__ego__", SOURCE_NS, (_mode(gt[1]),)),
    ]
    reference = {
        (FIXTURE.name, "2"): 0.0,
        (FIXTURE.name, "9"): 0.0,
    }

    with pytest.raises(ValueError, match="cadence"):
        rung4.score_scenario(FIXTURE, recorded, SOURCE_NS, reference)


def test_rung3_report_requires_unique_finite_engine_minfde(tmp_path):
    report = tmp_path / "rung3.json"
    report.write_text(
        '{"schema_version": 1, "rung": "rung3", "per_agent": {'
        '"wrong-key": {'
        '"scenario_id": "s", "waymo_id": "a", "engine_minfde_m": 1.0'
        "}}}",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="does not match"):
        rung4._load_rung3(report)


def test_module_import_does_not_require_ros():
    assert "rosbag2_py" not in rung4.__dict__


def test_dedicated_mtr_diagnostic_topic_is_the_reader_default():
    assert rung4.MTR_TOPIC == "/world_modeling/mtr_predictions"
    assert rung4.WORLD_SEEDS_TOPIC == "/world_modeling/world_object_seeds"


def test_diagnostic_reader_uses_detection_id_and_prediction_source_stamp():
    def stamp(value: int):
        return types.SimpleNamespace(
            sec=value // 1_000_000_000,
            nanosec=value % 1_000_000_000,
        )

    poses = [
        types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp(SOURCE_NS + 200_000_000)),
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0)),
        )
    ]
    prediction = types.SimpleNamespace(
        conf=0.75,
        header=types.SimpleNamespace(stamp=stamp(SOURCE_NS)),
        poses=poses,
    )
    obj = types.SimpleNamespace(
        detection=types.SimpleNamespace(id="__ego__"),
        predictions=[prediction],
    )
    message = types.SimpleNamespace(
        # The array publication stamp is intentionally unrelated; cached
        # entries can have different authoritative source stamps.
        header=types.SimpleNamespace(stamp=stamp(SOURCE_NS + 999)),
        objects=[obj],
    )

    recorded = rung4._normalize_world_object_array(
        message, record_stamp_ns=SOURCE_NS + 1_000
    )

    assert recorded[0].detection_id == "__ego__"
    assert recorded[0].record_stamp_ns == SOURCE_NS + 1_000
    assert recorded[0].modes[0].source_stamp_ns == SOURCE_NS
    np.testing.assert_array_equal(recorded[0].modes[0].xy, [[1.0, 2.0]])


def test_reads_scoring_source_stamp_from_committed_mcap():
    pytest.importorskip("rosbag2_py")

    assert rung4.read_replay_source_stamp(FIXTURE) == SOURCE_NS
