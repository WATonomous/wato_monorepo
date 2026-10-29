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

import importlib
import json
import pathlib
import sys

import numpy as np
import pytest

from wato_mtr_eval import representation as rep
from wato_mtr_eval import rung3, tensor_io


def _bundle() -> dict[str, np.ndarray]:
    tensors = {
        "obj_trajs": np.zeros(
            (
                rep.TARGET_CAPACITY,
                rep.CONTEXT_CAPACITY,
                rep.HISTORY_STEPS,
                rep.OBJ_FEATURE_DIM,
            ),
            dtype=np.float32,
        ),
        "obj_trajs_mask": np.zeros(
            (
                rep.TARGET_CAPACITY,
                rep.CONTEXT_CAPACITY,
                rep.HISTORY_STEPS,
            ),
            dtype=np.uint8,
        ),
        "obj_trajs_last_pos": np.zeros(
            (rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, 3),
            dtype=np.float32,
        ),
        "map_polylines": np.zeros(
            (
                rep.TARGET_CAPACITY,
                rep.NUM_SRC_POLYLINES,
                rep.POINTS_PER_POLYLINE,
                rep.MAP_FEATURE_DIM,
            ),
            dtype=np.float32,
        ),
        "map_polylines_mask": np.zeros(
            (
                rep.TARGET_CAPACITY,
                rep.NUM_SRC_POLYLINES,
                rep.POINTS_PER_POLYLINE,
            ),
            dtype=np.uint8,
        ),
        "map_polylines_center": np.zeros(
            (rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, 3),
            dtype=np.float32,
        ),
        "track_index_to_predict": np.zeros(
            rep.TARGET_CAPACITY, dtype=np.int64
        ),
        "center_type_ids": np.zeros(rep.TARGET_CAPACITY, dtype=np.int64),
    }
    tensors["obj_trajs_mask"][0, 0] = 1
    tensors["obj_trajs_mask"][1, 1] = 1
    tensors["map_polylines_mask"][:2, 0] = 1
    tensors["track_index_to_predict"][1] = 1
    tensors["center_type_ids"][1] = 1
    return tensors


def _fixture(root: pathlib.Path) -> pathlib.Path:
    fixture = root / "scenario-001"
    fixture.mkdir()
    tensor_io.write_bundle(fixture / "expected_tensors.bin.gz", _bundle())
    (fixture / "target_ids.txt").write_text("scene-a\nscene-b\n", encoding="utf-8")
    (fixture / "target_id_map.tsv").write_text(
        "slot\tgt_row\twaymo_id\tscene_builder_id\n"
        "0\t0\twaymo-a\tscene-a\n"
        "1\t1\twaymo-b\tscene-b\n",
        encoding="utf-8",
    )
    (fixture / "target_frames.tsv").write_text(
        "slot\tgt_row\tcenter_x\tcenter_y\theading\n"
        "0\t0\t0\t0\t0\n"
        "1\t1\t0\t0\t0\n",
        encoding="utf-8",
    )
    np.savez(
        fixture / "ground_truth.npz",
        gt=np.zeros((2, 30, 2), dtype=np.float32),
        valid=np.ones((2, 30), dtype=np.uint8),
        agent_ids=np.asarray(["waymo-a", "waymo-b"]),
        types=np.asarray(["vehicle", "pedestrian"]),
    )
    return fixture


def _official_outputs() -> tuple[np.ndarray, np.ndarray]:
    scores = np.asarray(
        [
            [0.50, 0.20, 0.10, 0.08, 0.07, 0.05],
            [0.10, 0.60, 0.10, 0.08, 0.07, 0.05],
        ],
        dtype=np.float32,
    )
    trajectories = np.zeros(
        (2, rep.NUM_MODES, rep.NUM_FUTURE_FRAMES, rep.TRAJ_CHANNELS),
        dtype=np.float32,
    )
    return scores, trajectories


class FakeRunner:
    def __init__(self):
        self.calls = 0
        self.peak_memory_mb = 12.5

    def infer(self, inputs):
        self.calls += 1
        assert set(inputs) == set(rung3._INPUT_NAMES)
        assert inputs["obj_trajs"].shape[0] == rep.TARGET_CAPACITY
        assert inputs["obj_trajs_mask"].dtype == np.bool_
        assert inputs["map_polylines_mask"].dtype == np.bool_
        reference_scores, _ = _official_outputs()
        scores = np.zeros(
            (rep.TARGET_CAPACITY, rep.NUM_MODES), dtype=np.float32
        )
        scores[:2] = reference_scores
        scores[0, :2] += np.asarray([-0.01, 0.01], dtype=np.float32)
        scores[1] = np.asarray(
            [0.70, 0.20, 0.04, 0.03, 0.02, 0.01], dtype=np.float32
        )
        trajectories = np.zeros(
            (
                rep.TARGET_CAPACITY,
                rep.NUM_MODES,
                rep.NUM_FUTURE_FRAMES,
                rep.TRAJ_CHANNELS,
            ),
            dtype=np.float32,
        )
        trajectories[:2, :, :, 0] = 5e-4
        return {"pred_scores": scores, "pred_trajs": trajectories}


def _run(
    tmp_path: pathlib.Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    correctness_only: bool,
    benchmark_iterations: int = 20,
) -> tuple[rung3.Rung3Report, FakeRunner]:
    fixture_root = tmp_path / "fixtures"
    fixture_root.mkdir()
    _fixture(fixture_root)
    checkpoint = tmp_path / "checkpoint.pth"
    checkpoint.touch()
    mtr_root = tmp_path / "mtr"
    config = mtr_root / "tools/cfgs/waymo/mtr+100_percent_data.yaml"
    config.parent.mkdir(parents=True)
    config.touch()

    sentinel_net = object()
    monkeypatch.setattr(
        rung3.model,
        "load_checkpoint",
        lambda *args, **kwargs: sentinel_net,
    )

    def forward_batch(net, batch):
        assert net is sentinel_net
        assert batch["input_dict"]["obj_trajs"].shape[0] == 2
        return _official_outputs()

    monkeypatch.setattr(rung3.model, "forward_batch", forward_batch)
    runner = FakeRunner()
    report = rung3.run(
        fixture_root,
        tmp_path / "unused.engine",
        checkpoint,
        mtr_root,
        correctness_only=correctness_only,
        benchmark_iterations=benchmark_iterations,
        runner=runner,
    )
    return report, runner


def test_import_does_not_load_tensorrt():
    sys.modules.pop("tensorrt", None)
    importlib.reload(rung3)
    assert "tensorrt" not in sys.modules


def test_correctness_only_reports_every_agent_and_omits_performance(
    tmp_path, monkeypatch
):
    report, runner = _run(
        tmp_path, monkeypatch, correctness_only=True
    )

    assert runner.calls == 1
    assert report.num_scenarios == 1
    assert report.num_targets == 2
    assert report.num_dropped_no_future == 0
    assert report.gate_ok()
    assert report.paired_minfde.count == 2
    assert report.paired_minfde.near_zero_fraction == 1.0
    assert report.paired_minfde.mean == pytest.approx(5e-4)
    assert len(report.paired_mode_probability) == rep.NUM_MODES
    assert report.paired_mode_probability[0].count == 2
    assert report.top_mode_agreement == 0.5
    assert report.latency_ms_p50 is None
    assert report.latency_ms_p99 is None
    assert report.peak_memory_mb is None
    assert "Performance: OMITTED" in report.format()

    first, second = report.per_agent
    assert first.ledger_key == "scenario-001::waymo-a"
    assert first.paired_minfde_delta_m == pytest.approx(5e-4)
    assert first.paired_mode_probability_deltas[:2] == pytest.approx(
        (-0.01, 0.01)
    )
    assert first.top_mode_agrees
    assert not second.top_mode_agrees


def test_json_report_is_a_strict_rung4_per_agent_ledger(
    tmp_path, monkeypatch
):
    report, _ = _run(tmp_path, monkeypatch, correctness_only=True)
    output = tmp_path / "reports" / "rung3.json"
    report.write_json(output)
    raw = json.loads(output.read_text(encoding="utf-8"))

    assert raw["schema_version"] == 1
    assert raw["rung"] == "rung3"
    assert raw["paired_mode_probability"][0]["count"] == 2
    assert raw["latency_ms_p50"] is None
    row = raw["per_agent"]["scenario-001::waymo-b"]
    assert row["scenario_id"] == "scenario-001"
    assert row["waymo_id"] == "waymo-b"
    assert len(row["paired_mode_probability_deltas"]) == rep.NUM_MODES
    assert row["top_mode_agrees"] is False


def test_performance_mode_warms_up_and_records_only_measured_iterations(
    tmp_path, monkeypatch
):
    report, runner = _run(
        tmp_path,
        monkeypatch,
        correctness_only=False,
        benchmark_iterations=2,
    )

    assert runner.calls == 5
    assert report.latency_ms_p50 is not None
    assert report.latency_ms_p99 is not None
    assert report.peak_memory_mb == 12.5


def test_engine_output_shape_is_validated():
    with pytest.raises(ValueError, match="pred_scores"):
        rung3._engine_outputs(
            {
                "pred_scores": np.zeros((1, rep.NUM_MODES), dtype=np.float32),
                "pred_trajs": np.zeros(
                    (
                        rep.TARGET_CAPACITY,
                        rep.NUM_MODES,
                        rep.NUM_FUTURE_FRAMES,
                        rep.TRAJ_CHANNELS,
                    ),
                    dtype=np.float32,
                ),
            }
        )
