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
"""Rung 2: score the consequence of C++ SceneBuilder packing.

Each committed fixture is dumped through the production C++ implementation.
Both that dump and the independent Python reference are forwarded through the
same official MTR checkpoint. The report is paired by scenario and raw Waymo
target identity, with a separate cohort for targets whose own history contains
masked slots.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import os
import pathlib
import shutil
import subprocess
import tempfile
from collections.abc import Sequence

import numpy as np
import torch

from . import metrics, model, representation as rep, tensor_io

_TYPE_NAMES = {
    0: "TYPE_VEHICLE",
    1: "TYPE_PEDESTRIAN",
    2: "TYPE_CYCLIST",
}
_GT_TYPE_CODES = {
    "vehicle": 0,
    "pedestrian": 1,
    "cyclist": 2,
}
_FLOAT_TENSORS = {
    "obj_trajs": (
        rep.TARGET_CAPACITY,
        rep.CONTEXT_CAPACITY,
        rep.HISTORY_STEPS,
        rep.OBJ_FEATURE_DIM,
    ),
    "obj_trajs_last_pos": (
        rep.TARGET_CAPACITY,
        rep.CONTEXT_CAPACITY,
        3,
    ),
    "map_polylines": (
        rep.TARGET_CAPACITY,
        rep.NUM_SRC_POLYLINES,
        rep.POINTS_PER_POLYLINE,
        rep.MAP_FEATURE_DIM,
    ),
    "map_polylines_center": (
        rep.TARGET_CAPACITY,
        rep.NUM_SRC_POLYLINES,
        3,
    ),
}
_MASK_TENSORS = {
    "obj_trajs_mask": (
        rep.TARGET_CAPACITY,
        rep.CONTEXT_CAPACITY,
        rep.HISTORY_STEPS,
    ),
    "map_polylines_mask": (
        rep.TARGET_CAPACITY,
        rep.NUM_SRC_POLYLINES,
        rep.POINTS_PER_POLYLINE,
    ),
}
_INDEX_TENSORS = {
    "track_index_to_predict": (rep.TARGET_CAPACITY,),
    "center_type_ids": (rep.TARGET_CAPACITY,),
}
_REQUIRED_TENSORS = set(_FLOAT_TENSORS) | set(_MASK_TENSORS) | set(
    _INDEX_TENSORS
)


@dataclasses.dataclass(frozen=True)
class TargetInfo:
    """Identity, ground-truth join, and target-local coordinate frame."""

    slot: int
    gt_row: int
    waymo_id: str
    scene_builder_id: str
    center_x: float
    center_y: float
    heading: float


@dataclasses.dataclass(frozen=True)
class Rung2Report:
    paired_minfde: metrics.PairedDelta
    paired_minfde_masked_history: metrics.PairedDelta
    paired_minfde_clean_history: metrics.PairedDelta
    num_scenarios: int = 0
    num_targets: int = 0
    num_dropped_no_future: int = 0

    def gate_ok(self) -> bool:
        """Gate B requires a nonempty clean cohort with 99% below 1e-3 m."""
        clean = self.paired_minfde_clean_history
        return bool(
            self.num_scenarios > 0
            and clean.count > 0
            and math.isfinite(clean.near_zero_fraction)
            and clean.near_zero_fraction >= 0.99
        )

    def format(self) -> str:
        clean = metrics.format_paired_delta(
            self.paired_minfde_clean_history,
            "R1 -> R2  minFDE  (gap-free target history)",
        )
        masked = metrics.format_paired_delta(
            self.paired_minfde_masked_history,
            "R1 -> R2  minFDE  (masked target history)",
        )
        all_agents = metrics.format_paired_delta(
            self.paired_minfde,
            "R1 -> R2  minFDE  (all agents)",
        )
        gate = "PASS" if self.gate_ok() else "FAIL"
        return "\n\n".join(
            [
                f"Rung 2 — {self.num_scenarios} fixture scenarios, "
                f"{self.num_targets} accepted targets",
                f"Dropped without paired finite future: "
                f"{self.num_dropped_no_future}",
                all_agents,
                clean,
                masked,
                "ACCELERATION-RULE COUNTERFACTUAL: DEFERRED "
                "(R1 and R2 intentionally share one rule)",
                f"GATE B: {gate}",
            ]
        )


def _require_file(path: pathlib.Path, label: str) -> pathlib.Path:
    resolved = pathlib.Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"{label} does not exist: {resolved}")
    return resolved


def _require_dir(path: pathlib.Path, label: str) -> pathlib.Path:
    resolved = pathlib.Path(path).expanduser().resolve()
    if not resolved.is_dir():
        raise FileNotFoundError(f"{label} does not exist: {resolved}")
    return resolved


def _read_tsv(
    path: pathlib.Path,
    expected_header: list[str],
) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        if reader.fieldnames != expected_header:
            raise ValueError(
                f"{path} header must be {expected_header}, got {reader.fieldnames}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} contains no target rows")
    return rows


def _parse_nonnegative_int(value: str, label: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise ValueError(f"{label} must be an integer, got {value!r}") from error
    if parsed < 0:
        raise ValueError(f"{label} must be non-negative, got {parsed}")
    return parsed


def _read_targets(
    fixture: pathlib.Path,
    ground_truth_agent_ids: np.ndarray,
) -> list[TargetInfo]:
    identity_path = fixture / "target_id_map.tsv"
    frame_path = fixture / "target_frames.tsv"
    identity = _read_tsv(
        identity_path,
        ["slot", "gt_row", "waymo_id", "scene_builder_id"],
    )
    frames = _read_tsv(
        frame_path,
        ["slot", "gt_row", "center_x", "center_y", "heading"],
    )
    if len(identity) != len(frames):
        raise ValueError(
            f"{identity_path} and {frame_path} have different row counts"
        )
    if len(identity) > rep.TARGET_CAPACITY:
        raise ValueError(
            f"{identity_path} exceeds target capacity {rep.TARGET_CAPACITY}"
        )

    packed_ids = (fixture / "target_ids.txt").read_text(
        encoding="utf-8"
    ).splitlines()
    if len(packed_ids) != len(identity):
        raise ValueError(
            f"{fixture / 'target_ids.txt'} does not match mapping row count"
        )

    gt_ids = [str(item) for item in np.asarray(ground_truth_agent_ids).tolist()]
    output: list[TargetInfo] = []
    seen_gt_rows: set[int] = set()
    for expected_slot, (identity_row, frame_row) in enumerate(
        zip(identity, frames, strict=True)
    ):
        slot = _parse_nonnegative_int(identity_row["slot"], "slot")
        gt_row = _parse_nonnegative_int(identity_row["gt_row"], "gt_row")
        frame_slot = _parse_nonnegative_int(frame_row["slot"], "frame slot")
        frame_gt_row = _parse_nonnegative_int(
            frame_row["gt_row"], "frame gt_row"
        )
        if slot != expected_slot:
            raise ValueError(
                f"{identity_path} slots must be contiguous from zero"
            )
        if gt_row in seen_gt_rows:
            raise ValueError(f"{identity_path} contains duplicate gt_row {gt_row}")
        seen_gt_rows.add(gt_row)
        if (frame_slot, frame_gt_row) != (slot, gt_row):
            raise ValueError(
                f"{frame_path} row {expected_slot} is not aligned to identity map"
            )
        if gt_row >= len(gt_ids):
            raise ValueError(
                f"{identity_path} gt_row {gt_row} exceeds ground-truth rows"
            )
        waymo_id = identity_row["waymo_id"]
        packed_id = identity_row["scene_builder_id"]
        if gt_ids[gt_row] != waymo_id:
            raise ValueError(
                f"{identity_path} Waymo ID {waymo_id!r} does not match "
                f"ground_truth.npz row {gt_row} ID {gt_ids[gt_row]!r}"
            )
        if packed_ids[slot] != packed_id:
            raise ValueError(
                f"target_ids.txt slot {slot} does not match {identity_path}"
            )
        try:
            center_x = float(frame_row["center_x"])
            center_y = float(frame_row["center_y"])
            heading = float(frame_row["heading"])
        except ValueError as error:
            raise ValueError(
                f"{frame_path} row {slot} contains a non-numeric frame"
            ) from error
        if not np.isfinite([center_x, center_y, heading]).all():
            raise ValueError(f"{frame_path} row {slot} contains a non-finite frame")
        output.append(
            TargetInfo(
                slot=slot,
                gt_row=gt_row,
                waymo_id=waymo_id,
                scene_builder_id=packed_id,
                center_x=center_x,
                center_y=center_y,
                heading=heading,
            )
        )
    return output


def _validate_bundle(tensors: dict[str, np.ndarray]) -> None:
    missing = _REQUIRED_TENSORS - tensors.keys()
    extra = tensors.keys() - _REQUIRED_TENSORS
    if missing or extra:
        raise ValueError(
            f"tensor bundle keys differ: missing={sorted(missing)}, "
            f"extra={sorted(extra)}"
        )
    for name, shape in _FLOAT_TENSORS.items():
        array = tensors[name]
        if array.dtype != np.float32 or array.shape != shape:
            raise ValueError(
                f"{name} must be float32 {shape}, got {array.dtype} {array.shape}"
            )
    for name, shape in _MASK_TENSORS.items():
        array = tensors[name]
        if array.dtype != np.uint8 or array.shape != shape:
            raise ValueError(
                f"{name} must be uint8 {shape}, got {array.dtype} {array.shape}"
            )
        if not np.isin(array, (0, 1)).all():
            raise ValueError(f"{name} contains values other than zero and one")
    for name, shape in _INDEX_TENSORS.items():
        array = tensors[name]
        if array.dtype != np.int64 or array.shape != shape:
            raise ValueError(
                f"{name} must be int64 {shape}, got {array.dtype} {array.shape}"
            )


def _model_batch(
    tensors: dict[str, np.ndarray],
    targets: list[TargetInfo],
    scenario_id: str,
) -> dict:
    _validate_bundle(tensors)
    count = len(targets)
    type_codes = tensors["center_type_ids"][:count]
    unknown = sorted(set(type_codes.tolist()) - _TYPE_NAMES.keys())
    if unknown:
        raise ValueError(f"unknown center_type_ids: {unknown}")
    input_dict = {
        "obj_trajs": torch.from_numpy(tensors["obj_trajs"][:count]),
        "obj_trajs_mask": torch.from_numpy(
            tensors["obj_trajs_mask"][:count].astype(bool)
        ),
        "obj_trajs_last_pos": torch.from_numpy(
            tensors["obj_trajs_last_pos"][:count]
        ),
        "map_polylines": torch.from_numpy(
            tensors["map_polylines"][:count]
        ),
        "map_polylines_mask": torch.from_numpy(
            tensors["map_polylines_mask"][:count].astype(bool)
        ),
        "map_polylines_center": torch.from_numpy(
            tensors["map_polylines_center"][:count]
        ),
        "track_index_to_predict": torch.from_numpy(
            tensors["track_index_to_predict"][:count]
        ),
        "center_objects_type": [
            _TYPE_NAMES[int(type_code)] for type_code in type_codes
        ],
        "center_objects_id": np.asarray(
            [target.scene_builder_id for target in targets]
        ),
        "scenario_id": np.asarray([scenario_id] * count),
    }
    return {
        "batch_size": 1,
        "batch_sample_count": [count],
        "input_dict": input_dict,
    }


def _target_history_clean(
    tensors: dict[str, np.ndarray],
    num_targets: int,
) -> np.ndarray:
    _validate_bundle(tensors)
    clean = np.zeros(num_targets, dtype=bool)
    for slot in range(num_targets):
        context_row = int(tensors["track_index_to_predict"][slot])
        if not 0 <= context_row < rep.CONTEXT_CAPACITY:
            raise ValueError(
                f"target slot {slot} has invalid context row {context_row}"
            )
        clean[slot] = bool(
            tensors["obj_trajs_mask"][slot, context_row].astype(bool).all()
        )
    return clean


def _validate_target_type_codes(
    tensors: dict[str, np.ndarray],
    targets: list[TargetInfo],
    ground_truth_types: np.ndarray,
    label: str,
) -> None:
    types = [str(item) for item in np.asarray(ground_truth_types).tolist()]
    expected: list[int] = []
    for target in targets:
        if target.gt_row >= len(types):
            raise ValueError(
                f"{label} target gt_row {target.gt_row} exceeds type rows"
            )
        try:
            expected.append(_GT_TYPE_CODES[types[target.gt_row]])
        except KeyError as error:
            raise ValueError(
                f"{label} has unsupported ground-truth type "
                f"{types[target.gt_row]!r}"
            ) from error
    actual = tensors["center_type_ids"][: len(targets)].tolist()
    if actual != expected:
        raise ValueError(
            f"{label} center_type_ids {actual} do not match ground-truth "
            f"types {expected}"
        )


def _world_ground_truth_to_target(
    world: np.ndarray,
    targets: list[TargetInfo],
) -> np.ndarray:
    world = np.asarray(world, dtype=np.float64)
    if world.shape[0] != len(targets) or world.ndim != 3 or world.shape[2] < 2:
        raise ValueError(
            f"world ground truth must have shape [{len(targets)},T,>=2], "
            f"got {world.shape}"
        )
    local = np.empty(world.shape[:2] + (2,), dtype=np.float64)
    for target in targets:
        delta_x = world[target.slot, :, 0] - target.center_x
        delta_y = world[target.slot, :, 1] - target.center_y
        cos_h = math.cos(target.heading)
        sin_h = math.sin(target.heading)
        local[target.slot, :, 0] = delta_x * cos_h + delta_y * sin_h
        local[target.slot, :, 1] = -delta_x * sin_h + delta_y * cos_h
    return local


def _deployed_ground_truth(
    world: np.ndarray,
    valid: np.ndarray,
    targets: list[TargetInfo],
    deployed_steps: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Transform native GT to target frames, then sample steps 1,3,...,29."""
    local_native = _world_ground_truth_to_target(world, targets)
    valid = np.asarray(valid).astype(bool)
    if valid.shape != local_native.shape[:2]:
        raise ValueError(
            f"ground-truth validity shape {valid.shape} does not match "
            f"positions {local_native.shape}"
        )
    required_native_steps = deployed_steps * 2
    if local_native.shape[1] < required_native_steps:
        raise ValueError(
            f"ground truth has only {local_native.shape[1]} steps; "
            f"need {required_native_steps}"
        )
    return (
        local_native[:, 1 : 1 + required_native_steps : 2],
        valid[:, 1 : 1 + required_native_steps : 2],
    )


def _resolve_scene_dump(scene_dump: pathlib.Path | str | None) -> pathlib.Path:
    configured = scene_dump or os.environ.get("PREDICTION_ML_SCENE_DUMP")
    if configured is not None:
        return _require_file(pathlib.Path(configured), "scene_dump executable")
    discovered = shutil.which("scene_dump")
    if discovered is None:
        raise FileNotFoundError(
            "scene_dump executable was not supplied and is not on PATH; pass "
            "--scene-dump or set PREDICTION_ML_SCENE_DUMP to the installed "
            "lib/prediction_ml/scene_dump executable"
        )
    return pathlib.Path(discovered).resolve()


def _run_scene_dump(
    executable: pathlib.Path,
    fixture: pathlib.Path,
    output: pathlib.Path,
    target_ids: list[str],
    timeout_s: float,
) -> None:
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("scene_dump timeout must be finite and positive")
    command = [
        str(executable),
        str(fixture),
        str(output),
        "--pinned",
        ",".join(target_ids),
    ]
    try:
        subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"scene_dump timed out after {timeout_s:g}s for {fixture.name}"
        ) from error
    except subprocess.CalledProcessError as error:
        details = (error.stderr or error.stdout or "").strip()
        suffix = f": {details}" if details else ""
        raise RuntimeError(
            f"scene_dump failed for {fixture.name} with exit "
            f"{error.returncode}{suffix}"
        ) from error
    if not output.is_file():
        raise RuntimeError(
            f"scene_dump succeeded for {fixture.name} but did not write {output}"
        )


def _fixture_directories(root: pathlib.Path) -> list[pathlib.Path]:
    fixtures = sorted(
        path
        for path in root.iterdir()
        if path.is_dir() and (path / "expected_tensors.bin.gz").is_file()
    )
    if not fixtures:
        raise RuntimeError(f"no rung-2 fixture directories found under {root}")
    return fixtures


def _paired_subset(
    rung2_values: np.ndarray,
    rung1_values: np.ndarray,
    agent_ids: list[str],
    scenario_ids: list[str],
    keep: np.ndarray,
) -> metrics.PairedDelta:
    indices = np.flatnonzero(keep)
    return metrics.paired_delta(
        rung2_values[indices],
        rung1_values[indices],
        [agent_ids[index] for index in indices],
        [scenario_ids[index] for index in indices],
    )


def run(
    fixture_dir: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    *,
    scene_dump: pathlib.Path | str | None = None,
    cfg_name: str = "mtr+100_percent_data.yaml",
    device: str = "cuda",
    dump_timeout_s: float = 120.0,
) -> Rung2Report:
    """Dump, forward, and pair every committed rung-2 fixture."""
    if device not in {"cuda", "cuda:0"}:
        raise ValueError(
            "official MTR calls .cuda() internally; rung 2 requires "
            "device='cuda' or 'cuda:0'"
        )
    if not torch.cuda.is_available():
        raise RuntimeError("rung 2 requires a CUDA-capable PyTorch environment")
    fixture_root = _require_dir(fixture_dir, "fixture directory")
    checkpoint_path = _require_file(checkpoint, "MTR checkpoint")
    mtr_root_path = _require_dir(mtr_root, "MTR root")
    cfg_path = _require_file(
        mtr_root_path / "tools/cfgs/waymo" / cfg_name,
        "official MTR config",
    )
    executable = _resolve_scene_dump(scene_dump)
    fixtures = _fixture_directories(fixture_root)
    net = model.load_checkpoint(
        checkpoint_path,
        cfg_path,
        device=device,
        mtr_root=mtr_root_path,
    )

    rung1_fde: list[np.ndarray] = []
    rung2_fde: list[np.ndarray] = []
    clean_cohort: list[np.ndarray] = []
    agent_ids: list[str] = []
    scenario_ids: list[str] = []

    with tempfile.TemporaryDirectory(prefix="wato-rung2-") as temp:
        output_root = pathlib.Path(temp)
        for fixture in fixtures:
            with np.load(fixture / "ground_truth.npz", allow_pickle=False) as data:
                required = {"gt", "valid", "agent_ids", "types"}
                available = set(data.files)
                if not required <= available:
                    raise ValueError(
                        f"{fixture / 'ground_truth.npz'} is missing "
                        f"{sorted(required - available)}"
                    )
                all_gt = np.asarray(data["gt"])
                all_valid = np.asarray(data["valid"]).astype(bool)
                all_types = np.asarray(data["types"])
                targets = _read_targets(fixture, data["agent_ids"])

            target_rows = [target.gt_row for target in targets]
            gt_world = all_gt[target_rows]
            valid = all_valid[target_rows]
            if gt_world.ndim != 3 or gt_world.shape[2] < 2:
                raise ValueError(
                    f"{fixture / 'ground_truth.npz'} gt has shape {gt_world.shape}"
                )
            if valid.shape != gt_world.shape[:2]:
                raise ValueError(
                    f"{fixture / 'ground_truth.npz'} valid shape {valid.shape} "
                    f"does not match gt {gt_world.shape}"
                )

            dumped_path = output_root / f"{fixture.name}.bin.gz"
            _run_scene_dump(
                executable,
                fixture,
                dumped_path,
                [target.scene_builder_id for target in targets],
                dump_timeout_s,
            )
            reference = tensor_io.read_bundle(
                fixture / "expected_tensors.bin.gz"
            )
            dumped = tensor_io.read_bundle(dumped_path)
            count = len(targets)
            _validate_target_type_codes(
                reference, targets, all_types, f"{fixture.name} reference"
            )
            _validate_target_type_codes(
                dumped, targets, all_types, f"{fixture.name} C++ dump"
            )
            reference_clean = _target_history_clean(reference, count)
            dumped_clean = _target_history_clean(dumped, count)
            if not np.array_equal(reference_clean, dumped_clean):
                raise RuntimeError(
                    f"{fixture.name} reference and C++ dump disagree on "
                    "target-history cohort membership"
                )

            _, reference_native = model.forward_batch(
                net, _model_batch(reference, targets, fixture.name)
            )
            _, dumped_native = model.forward_batch(
                net, _model_batch(dumped, targets, fixture.name)
            )
            reference_deployed = metrics.to_deployed(reference_native)
            dumped_deployed = metrics.to_deployed(dumped_native)
            deployed_steps = reference_deployed.shape[2]
            if dumped_deployed.shape != reference_deployed.shape:
                raise RuntimeError(
                    f"{fixture.name} C++ and reference predictions have "
                    f"different shapes: {dumped_deployed.shape} and "
                    f"{reference_deployed.shape}"
                )
            deployed_local, deployed_valid = _deployed_ground_truth(
                gt_world, valid, targets, deployed_steps
            )
            rung1_fde.append(
                metrics.min_fde(
                    reference_deployed, deployed_local, deployed_valid
                )
            )
            rung2_fde.append(
                metrics.min_fde(
                    dumped_deployed, deployed_local, deployed_valid
                )
            )
            clean_cohort.append(reference_clean)
            agent_ids.extend(target.waymo_id for target in targets)
            scenario_ids.extend([fixture.name] * count)

    rung1_values = np.concatenate(rung1_fde)
    rung2_values = np.concatenate(rung2_fde)
    clean = np.concatenate(clean_cohort)
    paired = metrics.paired_delta(
        rung2_values, rung1_values, agent_ids, scenario_ids
    )
    dropped = int(
        (
            ~np.isfinite(rung1_values)
            | ~np.isfinite(rung2_values)
        ).sum()
    )
    return Rung2Report(
        paired_minfde=paired,
        paired_minfde_masked_history=_paired_subset(
            rung2_values,
            rung1_values,
            agent_ids,
            scenario_ids,
            ~clean,
        ),
        paired_minfde_clean_history=_paired_subset(
            rung2_values,
            rung1_values,
            agent_ids,
            scenario_ids,
            clean,
        ),
        num_scenarios=len(fixtures),
        num_targets=len(agent_ids),
        num_dropped_no_future=dropped,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", type=pathlib.Path, required=True)
    parser.add_argument("--checkpoint", type=pathlib.Path, required=True)
    parser.add_argument("--mtr-root", type=pathlib.Path, required=True)
    parser.add_argument("--scene-dump", type=pathlib.Path)
    parser.add_argument("--cfg-name", default="mtr+100_percent_data.yaml")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--dump-timeout-s", type=float, default=120.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run(
        fixture_dir=args.fixtures,
        checkpoint=args.checkpoint,
        mtr_root=args.mtr_root,
        scene_dump=args.scene_dump,
        cfg_name=args.cfg_name,
        device=args.device,
        dump_timeout_s=args.dump_timeout_s,
    )
    print(report.format())
    return 0 if report.gate_ok() else 1


if __name__ == "__main__":
    raise SystemExit(main())
