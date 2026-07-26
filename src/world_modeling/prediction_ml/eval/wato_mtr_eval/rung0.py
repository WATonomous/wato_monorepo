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
"""Rung 0 — the golden anchor for validating the official MTR checkpoint.

This module deliberately refuses to score raw TFRecords without the official
MTR preprocessing output. Reimplementing preprocessing here would make rung 0
another harness representation rather than the independent golden reference.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import os
import pathlib
import re
from collections.abc import Callable, Iterable, Sequence
from typing import Any

import numpy as np
import torch

from . import metrics, model

PUBLISHED_MIN_ADE = 0.6046
PUBLISHED_MIN_FDE = 1.2251
TOLERANCE = 0.03
_OFFICIAL_SAMPLE_STRIDE = 5
_OFFICIAL_HORIZON_STEPS = (6, 10, 16)
_OFFICIAL_TYPES = ("TYPE_VEHICLE", "TYPE_PEDESTRIAN", "TYPE_CYCLIST")
_EXPECTED_VALIDATION_SHARDS = 150
_VALIDATION_SHARD_RE = re.compile(
    r"^(?P<family>validation|uncompressed_scenario_validation_validation)"
    r"\.tfrecord-"
    r"(?P<index>\d{5})-of-(?P<total>\d{5})$"
)
_GENERIC_SHARD_RE = re.compile(
    r"^.+\.tfrecord-(?P<index>\d{5})-of-(?P<total>\d{5})$"
)


@dataclasses.dataclass(frozen=True)
class Rung0Report:
    """Native and deployed-horizon quality from the official MTR path."""

    num_scenarios: int
    num_agents: int
    min_ade_native: float
    min_fde_native: float
    min_ade_deployed: float
    min_fde_deployed: float
    per_type: dict[str, tuple[float, float]]
    gate_eligible: bool

    def anchor_ok(self) -> bool:
        values = np.asarray([self.min_ade_native, self.min_fde_native])
        return bool(
            self.gate_eligible
            and self.num_scenarios > 0
            and self.num_agents > 0
            and np.isfinite(values).all()
            and abs(self.min_ade_native - PUBLISHED_MIN_ADE) / PUBLISHED_MIN_ADE
            < TOLERANCE
            and abs(self.min_fde_native - PUBLISHED_MIN_FDE) / PUBLISHED_MIN_FDE
            < TOLERANCE
        )

    def format(self) -> str:
        if not self.gate_eligible:
            status = "INELIGIBLE (requires one complete 150-shard directory, no limit)"
        else:
            status = "PASS" if self.anchor_ok() else "FAIL"
        lines = [
            f"Rung 0 (golden) — {self.num_scenarios} scenarios, "
            f"{self.num_agents} target agents",
            "",
            f"  official 8 s config  minADE {self.min_ade_native:.4f}  "
            f"(published {PUBLISHED_MIN_ADE:.4f})",
            f"                       minFDE {self.min_fde_native:.4f}  "
            f"(published {PUBLISHED_MIN_FDE:.4f})",
            f"  deployed 3 s  minADE {self.min_ade_deployed:.4f}",
            f"                minFDE {self.min_fde_deployed:.4f}",
            "",
            "  per type (deployed):",
        ]
        for name, (ade, fde) in sorted(self.per_type.items()):
            lines.append(f"    {name:<12} minADE {ade:.4f}  minFDE {fde:.4f}")
        lines += ["", f"  GATE A: {status}"]
        return "\n".join(lines)


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


def _shard_coordinates(path: pathlib.Path) -> tuple[int, int]:
    match = _VALIDATION_SHARD_RE.fullmatch(path.name)
    if match is None:
        raise ValueError(f"not an exact Waymo validation shard filename: {path}")
    index = int(match.group("index"))
    total = int(match.group("total"))
    if total <= 0 or index >= total:
        raise ValueError(
            f"invalid validation shard coordinates {index}-of-{total}: {path}"
        )
    return index, total


def _shard_family(path: pathlib.Path) -> str:
    match = _VALIDATION_SHARD_RE.fullmatch(path.name)
    if match is None:
        raise ValueError(f"not an exact Waymo validation shard filename: {path}")
    return match.group("family")


def _directory_shards(directory: pathlib.Path) -> list[pathlib.Path]:
    matches: list[pathlib.Path] = []
    wrong_family: list[str] = []
    for path in directory.iterdir():
        if not path.is_file():
            continue
        if _VALIDATION_SHARD_RE.fullmatch(path.name):
            matches.append(path.absolute())
        elif _GENERIC_SHARD_RE.fullmatch(path.name):
            wrong_family.append(path.name)
    if wrong_family:
        raise ValueError(
            "validation directory mixes TFRecord families: "
            + ", ".join(sorted(wrong_family))
        )
    if not matches:
        raise FileNotFoundError(
            f"Waymo validation shard directory contains no exact TFRecords: {directory}"
        )

    coordinates = [_shard_coordinates(path) for path in matches]
    families = {_shard_family(path) for path in matches}
    if len(families) != 1:
        raise ValueError(
            f"validation directory mixes shard filename families: {sorted(families)}"
        )
    totals = {total for _, total in coordinates}
    if len(totals) != 1:
        raise ValueError(
            f"validation directory mixes shard totals: {sorted(totals)}"
        )
    total = totals.pop()
    indices = [index for index, _ in coordinates]
    expected = set(range(total))
    actual = set(indices)
    if len(indices) != len(actual):
        raise ValueError("validation directory contains duplicate shard indices")
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(
            "validation directory is incomplete: "
            f"expected {total} shards, found {len(actual)}, "
            f"missing={missing[:5]}, extra={extra[:5]}"
        )
    return sorted(matches, key=lambda path: _shard_coordinates(path)[0])


def _expand_shards(
    shard_inputs: pathlib.Path | Sequence[pathlib.Path],
) -> list[pathlib.Path]:
    """Resolve file/directory inputs to one deterministic, de-duplicated shard list."""
    if isinstance(shard_inputs, (str, os.PathLike)):
        inputs = [pathlib.Path(shard_inputs)]
    else:
        inputs = [pathlib.Path(path) for path in shard_inputs]
    if not inputs:
        raise ValueError("at least one Waymo validation shard input is required")

    expanded: list[pathlib.Path] = []
    for candidate in inputs:
        lexical = candidate.expanduser().absolute()
        if lexical.is_file():
            _shard_coordinates(lexical)
            expanded.append(lexical)
            continue
        if lexical.is_dir():
            expanded.extend(_directory_shards(lexical))
            continue
        raise FileNotFoundError(
            f"Waymo validation shard input does not exist: {lexical}"
        )

    resolved_targets = [path.resolve() for path in expanded]
    if len(resolved_targets) != len(set(resolved_targets)):
        raise ValueError(
            "the same resolved validation shard target was selected more than once"
        )
    totals = {_shard_coordinates(path)[1] for path in expanded}
    if len(totals) != 1:
        raise ValueError(f"selected shards mix shard totals: {sorted(totals)}")
    indices = [_shard_coordinates(path)[0] for path in expanded]
    if len(indices) != len(set(indices)):
        raise ValueError("selected inputs contain duplicate shard indices")
    return sorted(expanded, key=lambda path: _shard_coordinates(path)[0])


def _gate_eligible_selection(
    shard_inputs: pathlib.Path | Sequence[pathlib.Path],
    shards: Sequence[pathlib.Path],
    limit: int | None,
) -> bool:
    """Only one complete 150-shard directory can open Gate A."""
    if limit is not None:
        return False
    if isinstance(shard_inputs, (str, os.PathLike)):
        inputs = [pathlib.Path(shard_inputs)]
    else:
        inputs = [pathlib.Path(path) for path in shard_inputs]
    if len(inputs) != 1 or not inputs[0].expanduser().resolve().is_dir():
        return False
    coordinates = [_shard_coordinates(path) for path in shards]
    return (
        len(coordinates) == _EXPECTED_VALIDATION_SHARDS
        and {_shard_family(path) for path in shards} == {"validation"}
        and {total for _, total in coordinates} == {_EXPECTED_VALIDATION_SHARDS}
        and {index for index, _ in coordinates}
        == set(range(_EXPECTED_VALIDATION_SHARDS))
    )


def _collect_scenario_ids(
    shards: Sequence[pathlib.Path],
    load_scenarios: Callable[..., Iterable[Any]],
    limit: int | None,
) -> set[str]:
    """Read only scenario IDs, applying ``limit`` once across all shards."""
    if limit is not None and limit < 0:
        raise ValueError("limit must be non-negative")

    scenario_ids: set[str] = set()
    remaining = limit
    for shard in shards:
        if remaining == 0:
            break
        records_read = 0
        for item in load_scenarios(shard, limit=remaining):
            if item.scenario_id in scenario_ids:
                raise RuntimeError(
                    f"duplicate raw scenario_id {item.scenario_id!r} in selected shards"
                )
            scenario_ids.add(item.scenario_id)
            records_read += 1
        if remaining is not None:
            remaining -= records_read
    return scenario_ids


def _single_scenario_id(scenario_ids: Sequence[str]) -> str:
    """Require batch-size-one collation to contain agents from one scenario."""
    unique = set(scenario_ids)
    if len(unique) != 1:
        raise RuntimeError(
            "official MTR batch mixes scenario IDs: " + ", ".join(sorted(unique))
        )
    return next(iter(unique))


def _validate_official_scenarios(
    dataset,
    selected_ids: set[str],
    *,
    gate_eligible: bool = False,
) -> None:
    """Validate the official index before safely terminating the loader early."""
    occurrences: dict[str, int] = {}
    for info in dataset.infos:
        scenario_id = str(info["scenario_id"])
        if gate_eligible or scenario_id in selected_ids:
            occurrences[scenario_id] = occurrences.get(scenario_id, 0) + 1
    duplicates = sorted(
        scenario_id for scenario_id, count in occurrences.items() if count != 1
    )
    missing = sorted(selected_ids - occurrences.keys())
    if duplicates:
        scope = "scenario IDs" if gate_eligible else "selected scenario IDs"
        raise RuntimeError(
            f"official MTR dataset contains duplicate {scope}: "
            + ", ".join(duplicates[:5])
        )
    if missing:
        raise RuntimeError(
            "official MTR dataset is missing selected scenario IDs: "
            + ", ".join(missing[:5])
        )
    if gate_eligible:
        extra = sorted(occurrences.keys() - selected_ids)
        if extra:
            raise RuntimeError(
                "full official MTR scenario index does not equal the raw validation "
                "scenario IDs; unexpected official IDs: "
                + ", ".join(extra[:5])
            )


@dataclasses.dataclass
class _MeanAccumulator:
    """Constant-memory finite-value mean."""

    total: float = 0.0
    count: int = 0

    def add(self, values: np.ndarray) -> None:
        finite = np.asarray(values, dtype=np.float64)
        finite = finite[np.isfinite(finite)]
        self.total += float(finite.sum())
        self.count += int(finite.size)

    def mean(self) -> float:
        return self.total / self.count if self.count else float("nan")


def _to_device(value, device: str):
    if torch.is_tensor(value):
        return value.to(device)
    if isinstance(value, np.ndarray) and value.dtype.kind not in {"O", "U", "S"}:
        return torch.from_numpy(value).to(device)
    if isinstance(value, dict):
        return {key: _to_device(item, device) for key, item in value.items()}
    if isinstance(value, list):
        return [_to_device(item, device) for item in value]
    if isinstance(value, tuple):
        return tuple(_to_device(item, device) for item in value)
    return value


def _input_field(batch: dict, name: str):
    source = batch.get("input_dict", batch)
    if name not in source:
        raise RuntimeError(f"official MTR batch is missing required field {name!r}")
    return source[name]


def _strings(value) -> list[str]:
    if torch.is_tensor(value):
        value = value.detach().cpu().numpy()
    return [str(item) for item in np.asarray(value).reshape(-1).tolist()]


def _as_numpy(value) -> np.ndarray:
    if torch.is_tensor(value):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _build_dataset(cfg, preprocessed_data: pathlib.Path):
    try:
        from mtr.datasets.waymo.waymo_dataset import WaymoDataset
    except ImportError as exc:
        raise RuntimeError("official MTR WaymoDataset could not be imported") from exc

    cfg.DATA_CONFIG.DATA_ROOT = str(preprocessed_data)
    logger = logging.getLogger("wato_mtr_eval.rung0")
    return WaymoDataset(
        dataset_cfg=cfg.DATA_CONFIG,
        training=False,
        logger=logger,
    )


def _official_anchor_batch(
    pred: np.ndarray,
    gt: np.ndarray,
    valid: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Reproduce MTR's official minADE/minFDE temporal breakdowns.

    ``waymo_eval.py`` samples 10 Hz futures at ``4::5`` to 2 Hz and evaluates
    the 3 s, 5 s, and 8 s measurement steps. The official final result averages
    those three breakdowns within each object type, then averages the types.
    """
    sampled_pred = pred[:, :, 4::_OFFICIAL_SAMPLE_STRIDE, :2]
    sampled_gt = gt[:, 4::_OFFICIAL_SAMPLE_STRIDE, :2]
    sampled_valid = valid[:, 4::_OFFICIAL_SAMPLE_STRIDE]
    ades: list[np.ndarray] = []
    fdes: list[np.ndarray] = []
    for steps in _OFFICIAL_HORIZON_STEPS:
        ades.append(
            metrics.min_ade(
                sampled_pred[:, :, :steps],
                sampled_gt[:, :steps],
                sampled_valid[:, :steps],
            )
        )
        fdes.append(
            metrics.min_fde(
                sampled_pred[:, :, :steps],
                sampled_gt[:, :steps],
                sampled_valid[:, :steps],
            )
        )
    return ades, fdes


def run(
    shard: pathlib.Path | Sequence[pathlib.Path],
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    cfg_name: str = "mtr+100_percent_data.yaml",
    limit: int | None = None,
    device: str = "cuda",
    preprocessed_data: pathlib.Path | None = None,
) -> Rung0Report:
    """Score one or more shards through the official preprocessed MTR dataset.

    ``preprocessed_data`` must be output from MTR's
    ``mtr/datasets/waymo/data_preprocess.py``. It may also be supplied through
    ``MTR_PREPROCESSED_WAYMO``. Each ``shard`` input may be a TFRecord or a
    directory of TFRecords. Raw shards are used only to select matching scenario
    IDs; they are never converted into model inputs by this harness.
    """
    shards = _expand_shards(shard)
    gate_eligible = _gate_eligible_selection(shard, shards, limit)
    checkpoint = _require_file(checkpoint, "MTR checkpoint")
    mtr_root = _require_dir(mtr_root, "MTR root")
    configured_data = preprocessed_data or os.environ.get("MTR_PREPROCESSED_WAYMO")
    if configured_data is None:
        raise RuntimeError(
            "Gate A requires official preprocessed Waymo data; pass "
            "--preprocessed-data or set MTR_PREPROCESSED_WAYMO after running "
            "mtr/datasets/waymo/data_preprocess.py"
        )
    preprocessed_root = _require_dir(
        pathlib.Path(configured_data), "official preprocessed Waymo dataset"
    )

    # Lazy import keeps `python -m ...rung0 --help` usable where the Waymo wheel
    # cannot be installed, while an actual run still requires the official proto.
    try:
        from . import scenario
    except ImportError as exc:
        raise RuntimeError(
            "the Waymo Open Dataset Python package is required to run Gate A"
        ) from exc

    raw_scenario_ids = _collect_scenario_ids(
        shards, scenario.load_scenarios, limit=limit
    )
    if not raw_scenario_ids:
        inputs = ", ".join(str(path) for path in shards)
        raise RuntimeError(f"no scenarios were read from: {inputs}")

    cfg_path = _require_file(
        mtr_root / "tools/cfgs/waymo" / cfg_name, "official MTR config"
    )
    net = model.load_checkpoint(checkpoint, cfg_path, device=device, mtr_root=mtr_root)
    cfg, _, _ = model._import_mtr(mtr_root)
    dataset = _build_dataset(cfg, preprocessed_root)
    _validate_official_scenarios(
        dataset, raw_scenario_ids, gate_eligible=gate_eligible
    )
    loader = torch.utils.data.DataLoader(
        dataset, batch_size=1, shuffle=False, collate_fn=dataset.collate_batch
    )

    official_ade: dict[str, list[_MeanAccumulator]] = {
        name: [_MeanAccumulator() for _ in _OFFICIAL_HORIZON_STEPS]
        for name in _OFFICIAL_TYPES
    }
    official_fde: dict[str, list[_MeanAccumulator]] = {
        name: [_MeanAccumulator() for _ in _OFFICIAL_HORIZON_STEPS]
        for name in _OFFICIAL_TYPES
    }
    deployed_ade = _MeanAccumulator()
    deployed_fde = _MeanAccumulator()
    per_type: dict[str, tuple[_MeanAccumulator, _MeanAccumulator]] = {}
    seen_scenarios: set[str] = set()
    remaining_scenario_ids = set(raw_scenario_ids)

    for batch in loader:
        scenario_ids = _strings(_input_field(batch, "scenario_id"))
        batch_scenario_id = _single_scenario_id(scenario_ids)
        if batch_scenario_id not in raw_scenario_ids:
            continue
        if batch_scenario_id in seen_scenarios:
            raise RuntimeError(
                "official MTR dataset emitted selected scenario more than once: "
                f"{batch_scenario_id}"
            )
        seen_scenarios.add(batch_scenario_id)
        remaining_scenario_ids.remove(batch_scenario_id)

        gt = _as_numpy(_input_field(batch, "center_gt_trajs"))[..., :2]
        valid = _as_numpy(_input_field(batch, "center_gt_trajs_mask")).astype(bool)
        types = _strings(_input_field(batch, "center_objects_type"))
        device_batch = _to_device(batch, device)
        _, trajs = model.forward_batch(net, device_batch)

        batch_ades, batch_fdes = _official_anchor_batch(trajs, gt, valid)
        type_array = np.asarray(types)
        for type_name in _OFFICIAL_TYPES:
            type_mask = type_array == type_name
            for horizon_index in range(len(_OFFICIAL_HORIZON_STEPS)):
                official_ade[type_name][horizon_index].add(
                    batch_ades[horizon_index][type_mask]
                )
                official_fde[type_name][horizon_index].add(
                    batch_fdes[horizon_index][type_mask]
                )

        deployed_pred = metrics.to_deployed(trajs)
        deployed_steps = deployed_pred.shape[2]
        deployed_gt = gt[:, 1 : 1 + 2 * deployed_steps : 2]
        deployed_valid = valid[:, 1 : 1 + 2 * deployed_steps : 2]
        deployed_a = metrics.min_ade(deployed_pred, deployed_gt, deployed_valid)
        deployed_f = metrics.min_fde(deployed_pred, deployed_gt, deployed_valid)
        deployed_ade.add(deployed_a)
        deployed_fde.add(deployed_f)
        for index, name in enumerate(types):
            ade_acc, fde_acc = per_type.setdefault(
                name, (_MeanAccumulator(), _MeanAccumulator())
            )
            ade_acc.add(deployed_a[index : index + 1])
            fde_acc.add(deployed_f[index : index + 1])

        if not remaining_scenario_ids:
            break

    if remaining_scenario_ids:
        raise RuntimeError(
            "official MTR loader ended before all selected scenarios were processed: "
            + ", ".join(sorted(remaining_scenario_ids)[:5])
        )
    if deployed_ade.count == 0:
        raise RuntimeError(
            "official MTR dataset contained no scorable agents for the requested shards; "
            "check --preprocessed-data and the config's validation split"
        )

    def official_type_mean(
        values: dict[str, list[_MeanAccumulator]], type_name: str
    ) -> float:
        horizon_means: list[float] = []
        for accumulator in values[type_name]:
            value = accumulator.mean()
            if not np.isfinite(value):
                return float("nan")
            horizon_means.append(value)
        return float(np.mean(horizon_means))

    official_type_ade = {
        name: official_type_mean(official_ade, name) for name in _OFFICIAL_TYPES
    }
    official_type_fde = {
        name: official_type_mean(official_fde, name) for name in _OFFICIAL_TYPES
    }
    return Rung0Report(
        num_scenarios=len(seen_scenarios),
        num_agents=deployed_ade.count,
        min_ade_native=float(np.mean(list(official_type_ade.values()))),
        min_fde_native=float(np.mean(list(official_type_fde.values()))),
        min_ade_deployed=deployed_ade.mean(),
        min_fde_deployed=deployed_fde.mean(),
        per_type={
            key: (ade_acc.mean(), fde_acc.mean())
            for key, (ade_acc, fde_acc) in per_type.items()
        },
        gate_eligible=gate_eligible,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--shard",
        type=pathlib.Path,
        action="append",
        required=True,
        help="validation TFRecord or directory; repeat for multiple inputs",
    )
    parser.add_argument("--checkpoint", type=pathlib.Path, required=True)
    parser.add_argument("--mtr-root", type=pathlib.Path, required=True)
    parser.add_argument("--preprocessed-data", type=pathlib.Path)
    parser.add_argument("--cfg-name", default="mtr+100_percent_data.yaml")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--device", default="cuda")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run(
        shard=args.shard,
        checkpoint=args.checkpoint,
        mtr_root=args.mtr_root,
        cfg_name=args.cfg_name,
        limit=args.limit,
        device=args.device,
        preprocessed_data=args.preprocessed_data,
    )
    print(report.format())
    return 0 if report.anchor_ok() else 1


if __name__ == "__main__":
    raise SystemExit(main())
