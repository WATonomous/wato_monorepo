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

import dataclasses
import pathlib
import subprocess

import numpy as np
import pytest

from wato_mtr_eval import metrics, rung2, tensor_io


def _paired(values: np.ndarray) -> metrics.PairedDelta:
    count = len(values)
    return metrics.paired_delta(
        np.asarray(values, dtype=np.float64),
        np.zeros(count),
        [f"a{index}" for index in range(count)],
        ["s"] * count,
    )


def _bundle() -> dict[str, np.ndarray]:
    return {
        "obj_trajs": np.zeros(
            (8, 128, 11, 29), dtype=np.float32
        ),
        "obj_trajs_mask": np.zeros((8, 128, 11), dtype=np.uint8),
        "obj_trajs_last_pos": np.zeros(
            (8, 128, 3), dtype=np.float32
        ),
        "map_polylines": np.zeros(
            (8, 768, 20, 9), dtype=np.float32
        ),
        "map_polylines_mask": np.zeros(
            (8, 768, 20), dtype=np.uint8
        ),
        "map_polylines_center": np.zeros(
            (8, 768, 3), dtype=np.float32
        ),
        "track_index_to_predict": np.zeros(8, dtype=np.int64),
        "center_type_ids": np.zeros(8, dtype=np.int64),
    }


def _target(
    slot: int = 0,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    heading: float = 0.0,
) -> rung2.TargetInfo:
    return rung2.TargetInfo(
        slot=slot,
        gt_row=slot,
        waymo_id=str(100 + slot),
        scene_builder_id=str(100 + slot),
        center_x=center_x,
        center_y=center_y,
        heading=heading,
    )


def test_gate_requires_nonempty_clean_cohort_and_99_percent():
    empty = _paired(np.asarray([]))
    passing = _paired(np.zeros(100))
    failing_values = np.zeros(100)
    failing_values[:2] = metrics.NEAR_ZERO_M
    failing = _paired(failing_values)

    assert rung2.Rung2Report(passing, empty, passing, 3).gate_ok()
    assert not rung2.Rung2Report(passing, passing, empty, 3).gate_ok()
    assert not rung2.Rung2Report(passing, empty, passing, 0).gate_ok()
    assert not rung2.Rung2Report(passing, empty, failing, 3).gate_ok()


def test_report_formats_cohorts_gate_and_acceleration_decision():
    clean = _paired(np.zeros(4))
    masked = _paired(np.asarray([0.0, 0.02]))
    report = rung2.Rung2Report(
        paired_minfde=_paired(np.asarray([0.0, 0.0, 0.0, 0.02])),
        paired_minfde_masked_history=masked,
        paired_minfde_clean_history=clean,
        num_scenarios=3,
        num_targets=4,
        num_dropped_no_future=1,
    )

    text = report.format()
    assert "gap-free target history" in text
    assert "masked target history" in text
    assert "4 accepted targets" in text
    assert "Dropped without paired finite future: 1" in text
    assert "ACCELERATION-RULE COUNTERFACTUAL: DEFERRED" in text
    assert "GATE B: PASS" in text


def test_clean_cohort_uses_only_each_targets_own_context_row():
    tensors = _bundle()
    tensors["track_index_to_predict"][:2] = [3, 7]
    tensors["obj_trajs_mask"][0, 3] = 1
    tensors["obj_trajs_mask"][1, 7] = 1
    # Gaps elsewhere in the 128 context rows do not make the target gappy.
    assert rung2._target_history_clean(tensors, 2).tolist() == [True, True]

    tensors["obj_trajs_mask"][1, 7, 4] = 0
    assert rung2._target_history_clean(tensors, 2).tolist() == [True, False]


def test_world_ground_truth_is_rotated_into_each_target_frame():
    targets = [
        _target(center_x=10.0, center_y=20.0, heading=np.pi / 2.0)
    ]
    world = np.asarray([[[10.0, 21.0], [9.0, 20.0]]])

    local = rung2._world_ground_truth_to_target(world, targets)

    np.testing.assert_allclose(
        local, np.asarray([[[1.0, 0.0], [0.0, 1.0]]]), atol=1e-12
    )


def test_deployed_ground_truth_uses_native_indices_1_through_29():
    targets = [_target()]
    world = np.zeros((1, 80, 2), dtype=np.float64)
    world[0, :, 0] = np.arange(80)
    valid = np.ones((1, 80), dtype=bool)

    deployed, deployed_valid = rung2._deployed_ground_truth(
        world, valid, targets, 15
    )

    np.testing.assert_array_equal(
        deployed[0, :, 0], np.arange(1, 30, 2)
    )
    assert deployed_valid.all()


def test_model_batch_slices_real_targets_and_converts_masks_to_bool():
    tensors = _bundle()
    tensors["center_type_ids"][:2] = [0, 2]
    batch = rung2._model_batch(
        tensors, [_target(0), _target(1)], "scenario"
    )
    inputs = batch["input_dict"]

    assert inputs["obj_trajs"].shape == (2, 128, 11, 29)
    assert inputs["obj_trajs_mask"].dtype.is_floating_point is False
    assert str(inputs["obj_trajs_mask"].dtype) == "torch.bool"
    assert inputs["center_objects_type"] == [
        "TYPE_VEHICLE",
        "TYPE_CYCLIST",
    ]
    assert batch["batch_sample_count"] == [2]


def test_target_sidecars_join_ground_truth_and_frames(tmp_path):
    (tmp_path / "target_ids.txt").write_text("2\n__ego__\n")
    (tmp_path / "target_id_map.tsv").write_text(
        "slot\tgt_row\twaymo_id\tscene_builder_id\n"
        "0\t1\t2\t2\n"
        "1\t2\t9\t__ego__\n"
    )
    (tmp_path / "target_frames.tsv").write_text(
        "slot\tgt_row\tcenter_x\tcenter_y\theading\n"
        "0\t1\t1.25\t-2\t0.5\n"
        "1\t2\t3\t4\t-0.25\n"
    )

    targets = rung2._read_targets(
        tmp_path, np.asarray(["unused", "2", "9"])
    )

    assert [target.scene_builder_id for target in targets] == ["2", "__ego__"]
    assert [target.gt_row for target in targets] == [1, 2]
    assert targets[0].center_x == pytest.approx(1.25)


def test_target_type_codes_are_joined_through_gt_rows():
    tensors = _bundle()
    tensors["center_type_ids"][:2] = [0, 2]
    targets = [
        dataclasses.replace(_target(0), gt_row=1),
        dataclasses.replace(_target(1), gt_row=0),
    ]
    types = np.asarray(["cyclist", "vehicle"])

    rung2._validate_target_type_codes(tensors, targets, types, "fixture")
    tensors["center_type_ids"][1] = 1
    with pytest.raises(ValueError, match="do not match"):
        rung2._validate_target_type_codes(
            tensors, targets, types, "fixture"
        )


def test_paired_direction_is_rung2_minus_rung1():
    paired = rung2._paired_subset(
        np.asarray([2.5]),
        np.asarray([1.0]),
        ["agent"],
        ["scenario"],
        np.asarray([True]),
    )
    assert paired.mean == pytest.approx(1.5)


def test_committed_scorable_fixture_cohorts_have_eight_clean_one_masked():
    fixture_root = (
        pathlib.Path(__file__).resolve().parents[2] / "test" / "fixtures"
    )
    cohort: list[bool] = []
    for fixture in sorted(path for path in fixture_root.iterdir() if path.is_dir()):
        bundle = tensor_io.read_bundle(
            fixture / "expected_tensors.bin.gz"
        )
        count = len(
            (fixture / "target_ids.txt").read_text().splitlines()
        )
        clean = rung2._target_history_clean(bundle, count)
        mapping_rows = (
            (fixture / "target_id_map.tsv").read_text().splitlines()[1:]
        )
        gt_rows = [int(row.split("\t")[1]) for row in mapping_rows]
        with np.load(
            fixture / "ground_truth.npz", allow_pickle=False
        ) as ground_truth:
            valid = ground_truth["valid"][gt_rows, 1:30:2]
        cohort.extend(clean[valid.any(axis=1)].tolist())

    assert cohort.count(True) == 8
    assert cohort.count(False) == 1


def test_scene_dump_subprocess_contract_is_direct_and_pinned(
    tmp_path,
    monkeypatch,
):
    executable = tmp_path / "scene_dump"
    executable.write_text("")
    fixture = tmp_path / "scenario"
    fixture.mkdir()
    output = tmp_path / "dump.bin.gz"
    captured = {}

    def fake_run(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        output.write_bytes(b"bundle")
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(rung2.subprocess, "run", fake_run)
    rung2._run_scene_dump(
        executable, fixture, output, ["2", "__ego__"], 17.0
    )

    assert captured["command"] == [
        str(executable),
        str(fixture),
        str(output),
        "--pinned",
        "2,__ego__",
    ]
    assert captured["kwargs"] == {
        "check": True,
        "capture_output": True,
        "text": True,
        "timeout": 17.0,
    }


def test_scene_dump_failure_includes_fixture_and_stderr(
    tmp_path,
    monkeypatch,
):
    executable = tmp_path / "scene_dump"
    fixture = tmp_path / "scenario"
    output = tmp_path / "dump.bin.gz"

    def fake_run(command, **kwargs):
        del kwargs
        raise subprocess.CalledProcessError(
            4, command, stderr="bad pinned ID"
        )

    monkeypatch.setattr(rung2.subprocess, "run", fake_run)
    with pytest.raises(
        RuntimeError, match="scenario.*exit 4.*bad pinned ID"
    ):
        rung2._run_scene_dump(
            executable, fixture, output, ["missing"], 10.0
        )


def test_run_rejects_cpu_before_starting_external_work(tmp_path):
    with pytest.raises(ValueError, match="requires.*cuda"):
        rung2.run(tmp_path, tmp_path / "checkpoint", tmp_path, device="cpu")


def test_run_fails_clearly_when_cuda_is_unavailable(tmp_path, monkeypatch):
    monkeypatch.setattr(rung2.torch.cuda, "is_available", lambda: False)
    with pytest.raises(RuntimeError, match="CUDA-capable"):
        rung2.run(tmp_path, tmp_path / "checkpoint", tmp_path, device="cuda")
