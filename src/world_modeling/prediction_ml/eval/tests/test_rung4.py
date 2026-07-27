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
SOURCE_NS = 2_000_000_000


def _mode(xy: np.ndarray, confidence: float = 0.5) -> rung4.RecordedMode:
    step_ns = int(round(rep.DEPLOYED_STEP_S * 1_000_000_000))
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


def test_score_prefers_later_multimode_cache_output_over_immediate_fallback():
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


def test_reads_scoring_source_stamp_from_committed_mcap():
    pytest.importorskip("rosbag2_py")

    assert rung4.read_replay_source_stamp(FIXTURE) == SOURCE_NS
