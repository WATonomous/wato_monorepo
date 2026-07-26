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

import dataclasses
import pathlib
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("torch")
rung0 = pytest.importorskip("wato_mtr_eval.rung0")


def _make_shards(root: pathlib.Path, count: int) -> list[pathlib.Path]:
    paths = [
        root / f"validation.tfrecord-{index:05d}-of-{count:05d}"
        for index in reversed(range(count))
    ]
    for path in paths:
        path.write_bytes(b"")
    return paths


def test_directory_expands_all_150_shards_in_deterministic_order(tmp_path):
    _make_shards(tmp_path, 150)

    expanded = rung0._expand_shards(tmp_path)

    assert len(expanded) == 150
    assert [path.name for path in expanded] == sorted(path.name for path in expanded)
    assert expanded[0].name.endswith("-00000-of-00150")
    assert expanded[-1].name.endswith("-00149-of-00150")


def test_directory_rejects_incomplete_shard_family(tmp_path):
    paths = _make_shards(tmp_path, 3)
    paths[0].unlink()

    with pytest.raises(ValueError, match="incomplete"):
        rung0._expand_shards(tmp_path)


def test_directory_rejects_mixed_tfrecord_family(tmp_path):
    _make_shards(tmp_path, 2)
    (
        tmp_path
        / "uncompressed_scenario_training_training.tfrecord-00000-of-00001"
    ).write_bytes(b"")

    with pytest.raises(ValueError, match="mixes TFRecord families"):
        rung0._expand_shards(tmp_path)


def test_directory_ignores_download_sidecars(tmp_path):
    expected = sorted(_make_shards(tmp_path, 2))
    (tmp_path / f"{expected[0].name}.aria2").write_bytes(b"partial")
    (tmp_path / f"{expected[1].name}.sha256").write_text("unused")

    assert rung0._expand_shards(tmp_path) == expected


def test_symlink_shards_preserve_canonical_names_and_detect_duplicate_targets(
    tmp_path,
):
    targets = tmp_path / "targets"
    shards_dir = tmp_path / "shards"
    targets.mkdir()
    shards_dir.mkdir()
    target_a = targets / "download-a"
    target_b = targets / "download-b"
    target_a.write_bytes(b"a")
    target_b.write_bytes(b"b")
    shard_a = shards_dir / "validation.tfrecord-00000-of-00002"
    shard_b = shards_dir / "validation.tfrecord-00001-of-00002"
    try:
        shard_a.symlink_to(target_a)
        shard_b.symlink_to(target_b)
    except OSError as exc:
        pytest.skip(f"symlinks unavailable: {exc}")

    expanded = rung0._expand_shards(shards_dir)

    assert [path.name for path in expanded] == [shard_a.name, shard_b.name]
    shard_b.unlink()
    shard_b.symlink_to(target_a)
    with pytest.raises(ValueError, match="same resolved validation shard target"):
        rung0._expand_shards(shards_dir)


def test_explicit_sidecar_is_not_accepted_as_a_shard(tmp_path):
    path = tmp_path / "validation.tfrecord-00000-of-00150.aria2"
    path.write_bytes(b"partial")

    with pytest.raises(ValueError, match="exact Waymo validation shard"):
        rung0._expand_shards(path)


def test_legacy_single_shard_name_is_smoke_only(tmp_path):
    path = tmp_path / (
        "uncompressed_scenario_validation_validation."
        "tfrecord-00000-of-00150"
    )
    path.write_bytes(b"")

    shards = rung0._expand_shards(path)

    assert shards == [path.resolve()]
    assert not rung0._gate_eligible_selection(path, shards, limit=None)


def test_directory_input_collects_scenarios_from_every_shard(tmp_path):
    _make_shards(tmp_path, 150)
    expanded = rung0._expand_shards(tmp_path)

    def fake_load(path, limit=None):
        assert limit is None
        yield SimpleNamespace(scenario_id=path.name)

    scenario_ids = rung0._collect_scenario_ids(expanded, fake_load, limit=None)

    assert scenario_ids == {path.name for path in expanded}


def test_duplicate_raw_scenario_ids_are_rejected(tmp_path):
    paths = sorted(_make_shards(tmp_path, 2))

    def fake_load(path, limit=None):
        del path, limit
        yield SimpleNamespace(scenario_id="duplicate")

    with pytest.raises(RuntimeError, match="duplicate raw scenario_id"):
        rung0._collect_scenario_ids(paths, fake_load, limit=None)


def test_global_limit_is_shared_across_shards(tmp_path):
    paths = sorted(_make_shards(tmp_path, 3))
    calls = []

    def fake_load(path, limit=None):
        calls.append((path, limit))
        for index in range(min(2, limit if limit is not None else 2)):
            yield SimpleNamespace(scenario_id=f"{path.name}:{index}")

    scenario_ids = rung0._collect_scenario_ids(paths, fake_load, limit=3)

    assert len(scenario_ids) == 3
    assert calls == [(paths[0], 3), (paths[1], 1)]


def test_limit_and_explicit_files_are_never_gate_eligible(tmp_path):
    _make_shards(tmp_path, 150)
    shards = rung0._expand_shards(tmp_path)

    assert rung0._gate_eligible_selection(tmp_path, shards, limit=None)
    assert not rung0._gate_eligible_selection(tmp_path, shards, limit=1)
    assert not rung0._gate_eligible_selection(shards, shards, limit=None)


def test_repeatable_cli_shards_are_preserved_for_expansion(tmp_path):
    paths = sorted(_make_shards(tmp_path, 2))

    args = rung0._parser().parse_args(
        [
            "--shard",
            str(paths[1]),
            "--shard",
            str(paths[0]),
            "--checkpoint",
            str(tmp_path / "checkpoint.pth"),
            "--mtr-root",
            str(tmp_path),
        ]
    )

    assert args.shard == [paths[1], paths[0]]
    assert rung0._expand_shards(args.shard) == paths


def test_mixed_scenario_batch_is_rejected():
    with pytest.raises(RuntimeError, match="mixes scenario IDs"):
        rung0._single_scenario_id(["selected", "unmatched"])


def test_duplicate_and_missing_official_scenarios_are_rejected():
    duplicate_dataset = SimpleNamespace(
        infos=[{"scenario_id": "a"}, {"scenario_id": "a"}]
    )
    missing_dataset = SimpleNamespace(infos=[{"scenario_id": "a"}])

    with pytest.raises(RuntimeError, match="duplicate selected scenario IDs"):
        rung0._validate_official_scenarios(duplicate_dataset, {"a"})
    with pytest.raises(RuntimeError, match="missing selected scenario IDs"):
        rung0._validate_official_scenarios(missing_dataset, {"a", "b"})


def test_gate_index_equality_prevents_false_pass_from_extra_official_scenarios():
    dataset = SimpleNamespace(
        infos=[{"scenario_id": "selected"}, {"scenario_id": "unmatched"}]
    )

    # A smoke run intentionally validates only its selected subset.
    rung0._validate_official_scenarios(
        dataset, {"selected"}, gate_eligible=False
    )
    with pytest.raises(RuntimeError, match="does not equal the raw validation"):
        rung0._validate_official_scenarios(
            dataset, {"selected"}, gate_eligible=True
        )


def test_gate_rejects_duplicate_anywhere_in_full_official_index():
    dataset = SimpleNamespace(
        infos=[
            {"scenario_id": "selected"},
            {"scenario_id": "unmatched"},
            {"scenario_id": "unmatched"},
        ]
    )

    # The duplicate is outside a smoke subset, so it is intentionally irrelevant there.
    rung0._validate_official_scenarios(
        dataset, {"selected"}, gate_eligible=False
    )
    with pytest.raises(RuntimeError, match="duplicate scenario IDs"):
        rung0._validate_official_scenarios(
            dataset, {"selected", "unmatched"}, gate_eligible=True
        )


def test_mean_accumulator_streams_only_finite_values():
    accumulator = rung0._MeanAccumulator()

    accumulator.add(np.asarray([1.0, np.nan, 3.0]))
    accumulator.add(np.asarray([5.0]))

    assert accumulator.count == 3
    assert accumulator.mean() == 3.0


def test_official_anchor_samples_two_hz_and_all_three_horizons():
    gt = np.zeros((1, 80, 2), dtype=np.float32)
    pred = np.zeros((1, 2, 80, 7), dtype=np.float32)
    valid = np.ones((1, 80), dtype=bool)

    ades, fdes = rung0._official_anchor_batch(pred, gt, valid)

    assert len(ades) == len(fdes) == 3
    for values in [*ades, *fdes]:
        np.testing.assert_array_equal(values, [0.0])


def test_report_only_opens_anchor_for_finite_full_results():
    passing = rung0.Rung0Report(
        num_scenarios=286,
        num_agents=1268,
        min_ade_native=rung0.PUBLISHED_MIN_ADE,
        min_fde_native=rung0.PUBLISHED_MIN_FDE,
        min_ade_deployed=0.5,
        min_fde_deployed=1.0,
        per_type={},
        gate_eligible=True,
    )
    incomplete = dataclasses.replace(passing, min_ade_native=float("nan"))
    smoke = dataclasses.replace(passing, gate_eligible=False)

    assert passing.anchor_ok()
    assert not incomplete.anchor_ok()
    assert not smoke.anchor_ok()
    assert "GATE A: INELIGIBLE" in smoke.format()
