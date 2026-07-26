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
from collections.abc import Sequence

import numpy as np
import torch

from . import metrics, model

PUBLISHED_MIN_ADE = 0.6046
PUBLISHED_MIN_FDE = 1.2251
TOLERANCE = 0.03
_OFFICIAL_SAMPLE_STRIDE = 5
_OFFICIAL_HORIZON_STEPS = (6, 10, 16)
_OFFICIAL_TYPES = ("TYPE_VEHICLE", "TYPE_PEDESTRIAN", "TYPE_CYCLIST")


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

    def anchor_ok(self) -> bool:
        values = np.asarray([self.min_ade_native, self.min_fde_native])
        return bool(
            self.num_scenarios > 0
            and self.num_agents > 0
            and np.isfinite(values).all()
            and abs(self.min_ade_native - PUBLISHED_MIN_ADE) / PUBLISHED_MIN_ADE
            < TOLERANCE
            and abs(self.min_fde_native - PUBLISHED_MIN_FDE) / PUBLISHED_MIN_FDE
            < TOLERANCE
        )

    def format(self) -> str:
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
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    cfg_name: str = "mtr+100_percent_data.yaml",
    limit: int | None = None,
    device: str = "cuda",
    preprocessed_data: pathlib.Path | None = None,
) -> Rung0Report:
    """Score a shard through the official preprocessed MTR validation dataset.

    ``preprocessed_data`` must be output from MTR's
    ``mtr/datasets/waymo/data_preprocess.py``. It may also be supplied through
    ``MTR_PREPROCESSED_WAYMO``. The raw shard is used only to select matching
    scenario IDs; it is never converted into model inputs by this harness.
    """
    shard = _require_file(shard, "Waymo validation shard")
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

    raw_scenarios = {
        item.scenario_id: item for item in scenario.load_scenarios(shard, limit=limit)
    }
    if not raw_scenarios:
        raise RuntimeError(f"no scenarios were read from {shard}")

    cfg_path = _require_file(
        mtr_root / "tools/cfgs/waymo" / cfg_name, "official MTR config"
    )
    net = model.load_checkpoint(checkpoint, cfg_path, device=device, mtr_root=mtr_root)
    cfg, _, _ = model._import_mtr(mtr_root)
    dataset = _build_dataset(cfg, preprocessed_root)
    loader = torch.utils.data.DataLoader(
        dataset, batch_size=1, shuffle=False, collate_fn=dataset.collate_batch
    )

    official_ade: dict[str, list[list[np.ndarray]]] = {
        name: [[] for _ in _OFFICIAL_HORIZON_STEPS] for name in _OFFICIAL_TYPES
    }
    official_fde: dict[str, list[list[np.ndarray]]] = {
        name: [[] for _ in _OFFICIAL_HORIZON_STEPS] for name in _OFFICIAL_TYPES
    }
    deployed_ade: list[np.ndarray] = []
    deployed_fde: list[np.ndarray] = []
    per_type: dict[str, list[tuple[float, float]]] = {}
    seen_scenarios: set[str] = set()

    for batch in loader:
        scenario_ids = _strings(_input_field(batch, "scenario_id"))
        if not any(item in raw_scenarios for item in scenario_ids):
            continue
        seen_scenarios.update(item for item in scenario_ids if item in raw_scenarios)

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
                official_ade[type_name][horizon_index].append(
                    batch_ades[horizon_index][type_mask]
                )
                official_fde[type_name][horizon_index].append(
                    batch_fdes[horizon_index][type_mask]
                )

        deployed_pred = metrics.to_deployed(trajs)
        deployed_steps = deployed_pred.shape[2]
        deployed_gt = gt[:, 1 : 1 + 2 * deployed_steps : 2]
        deployed_valid = valid[:, 1 : 1 + 2 * deployed_steps : 2]
        deployed_a = metrics.min_ade(deployed_pred, deployed_gt, deployed_valid)
        deployed_f = metrics.min_fde(deployed_pred, deployed_gt, deployed_valid)
        deployed_ade.append(deployed_a)
        deployed_fde.append(deployed_f)
        for index, name in enumerate(types):
            per_type.setdefault(name, []).append(
                (float(deployed_a[index]), float(deployed_f[index]))
            )

    if not deployed_ade:
        raise RuntimeError(
            "official MTR dataset contained no scenarios matching the requested shard; "
            "check --preprocessed-data and the config's validation split"
        )

    def concatenate(values: Sequence[np.ndarray]) -> np.ndarray:
        return np.concatenate(values) if values else np.array([], dtype=np.float64)

    all_deployed_ade = concatenate(deployed_ade)
    all_deployed_fde = concatenate(deployed_fde)

    def official_type_mean(
        values: dict[str, list[list[np.ndarray]]], type_name: str
    ) -> float:
        horizon_means = []
        for horizon_values in values[type_name]:
            combined = concatenate(horizon_values)
            finite = combined[np.isfinite(combined)]
            if finite.size == 0:
                return float("nan")
            horizon_means.append(float(finite.mean()))
        return float(np.mean(horizon_means))

    official_type_ade = {
        name: official_type_mean(official_ade, name) for name in _OFFICIAL_TYPES
    }
    official_type_fde = {
        name: official_type_mean(official_fde, name) for name in _OFFICIAL_TYPES
    }
    return Rung0Report(
        num_scenarios=len(seen_scenarios),
        num_agents=int(np.isfinite(all_deployed_ade).sum()),
        min_ade_native=float(np.mean(list(official_type_ade.values()))),
        min_fde_native=float(np.mean(list(official_type_fde.values()))),
        min_ade_deployed=float(np.nanmean(all_deployed_ade)),
        min_fde_deployed=float(np.nanmean(all_deployed_fde)),
        per_type={
            key: (
                float(np.nanmean([row[0] for row in values])),
                float(np.nanmean([row[1] for row in values])),
            )
            for key, values in per_type.items()
        },
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shard", type=pathlib.Path, required=True)
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
