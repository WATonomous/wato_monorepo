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
"""Rung 1: price each map layer in the WATO input representation.

All policies keep object history, target selection, chunking, target-relative
geometry, and the official MTR checkpoint fixed. The ordered sweep removes one
map information axis at a time, so each adjacent paired delta has a specific
interpretation.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
from collections.abc import Sequence

import numpy as np
import torch

from . import (
    lanelet_query as lq,
    metrics,
    model,
    reference_pack,
    representation as rep,
    scenario,
)

_ALL_SUPPORTED = frozenset(
    {
        "lane",
        "road_line",
        "road_edge",
        "crosswalk",
        "speed_bump",
        "driveway",
    }
)
_TYPE_NAMES = {
    0: "TYPE_VEHICLE",
    1: "TYPE_PEDESTRIAN",
    2: "TYPE_CYCLIST",
}


@dataclasses.dataclass(frozen=True)
class Ablation:
    name: str
    keep_types: frozenset[str]
    collapse_global_type: bool
    query_strategy: str

    def __post_init__(self) -> None:
        if not self.name or self.name.strip() != self.name:
            raise ValueError("ablation name must be nonempty and trimmed")
        object.__setattr__(self, "keep_types", frozenset(self.keep_types))
        # Reuse the reference policy's strict feature validation.
        reference_pack.MapPackingOptions(
            self.keep_types, self.collapse_global_type
        )
        if self.query_strategy not in {"radius", "reachable"}:
            raise ValueError(
                f"unknown query strategy {self.query_strategy!r}"
            )

    def map_options(self) -> reference_pack.MapPackingOptions:
        return reference_pack.MapPackingOptions(
            keep_types=self.keep_types,
            collapse_global_type=self.collapse_global_type,
        )


# One-axis degradation chain. Stop signs and dynamic signal state are absent
# from Scenario's supported polyline geometry and are therefore not claimed by
# the typed baseline.
SWEEP = [
    Ablation(
        "all_supported_typed_radius",
        _ALL_SUPPORTED,
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "no_driveways",
        _ALL_SUPPORTED - {"driveway"},
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "no_speed_bumps",
        _ALL_SUPPORTED - {"driveway", "speed_bump"},
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "no_road_edges",
        _ALL_SUPPORTED - {"driveway", "speed_bump", "road_edge"},
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "no_crosswalks",
        {"lane", "road_line"},
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "no_road_lines",
        {"lane"},
        collapse_global_type=False,
        query_strategy="radius",
    ),
    Ablation(
        "lane_only_collapsed_radius",
        {"lane"},
        collapse_global_type=True,
        query_strategy="radius",
    ),
    Ablation(
        "lane_only_collapsed_reachable",
        {"lane"},
        collapse_global_type=True,
        query_strategy="reachable",
    ),
]


@dataclasses.dataclass(frozen=True)
class Rung1Report:
    ablation: Ablation
    num_scenarios: int
    num_targets: int
    num_dropped_no_future: int
    min_ade_deployed: float
    min_fde_deployed: float
    paired_minade_from_previous: metrics.PairedDelta | None
    paired_minfde_from_previous: metrics.PairedDelta | None
    map_candidate_chunks: int
    map_retained_target_chunks: int
    map_evicted_target_chunks_by_type: dict[str, int]
    agent_ids: list[str] = dataclasses.field(repr=False, compare=False)
    scenario_ids: list[str] = dataclasses.field(repr=False, compare=False)
    min_ade_values: np.ndarray = dataclasses.field(repr=False, compare=False)
    min_fde_values: np.ndarray = dataclasses.field(repr=False, compare=False)

    def format(self) -> str:
        lines = [
            f"Rung 1 — {self.ablation.name}",
            f"  scenarios {self.num_scenarios}  accepted targets "
            f"{self.num_targets}  dropped-no-future "
            f"{self.num_dropped_no_future}",
            f"  deployed minADE {self.min_ade_deployed:.4f} m",
            f"  deployed minFDE {self.min_fde_deployed:.4f} m",
            f"  map chunks candidate {self.map_candidate_chunks}, "
            f"retained target-slots {self.map_retained_target_chunks}",
            "  evicted target-chunks by type: "
            + (
                ", ".join(
                    f"{name}={count}"
                    for name, count in sorted(
                        self.map_evicted_target_chunks_by_type.items()
                    )
                )
                or "none"
            ),
        ]
        if self.paired_minade_from_previous is not None:
            lines += [
                "",
                metrics.format_paired_delta(
                    self.paired_minade_from_previous,
                    "previous -> current  deployed minADE",
                ),
                "",
                metrics.format_paired_delta(
                    self.paired_minfde_from_previous,
                    "previous -> current  deployed minFDE",
                ),
            ]
        return "\n".join(lines)


@dataclasses.dataclass
class _Accumulator:
    ade: list[np.ndarray] = dataclasses.field(default_factory=list)
    fde: list[np.ndarray] = dataclasses.field(default_factory=list)
    agent_ids: list[str] = dataclasses.field(default_factory=list)
    scenario_ids: list[str] = dataclasses.field(default_factory=list)
    candidate_chunks: int = 0
    retained_chunks: int = 0
    evicted: dict[str, int] = dataclasses.field(default_factory=dict)


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


def _model_batch(
    tensors: dict[str, np.ndarray],
    selections: list[reference_pack.TargetSelection],
    scenario_id: str,
) -> dict:
    count = len(selections)
    expected_shapes = {
        "obj_trajs": (8, 128, 11, 29),
        "obj_trajs_mask": (8, 128, 11),
        "obj_trajs_last_pos": (8, 128, 3),
        "map_polylines": (8, 768, 20, 9),
        "map_polylines_mask": (8, 768, 20),
        "map_polylines_center": (8, 768, 3),
        "track_index_to_predict": (8,),
        "center_type_ids": (8,),
    }
    for name, shape in expected_shapes.items():
        if name not in tensors or tensors[name].shape != shape:
            got = None if name not in tensors else tensors[name].shape
            raise ValueError(f"{name} must have shape {shape}, got {got}")
    for name in (
        "obj_trajs",
        "obj_trajs_last_pos",
        "map_polylines",
        "map_polylines_center",
    ):
        if tensors[name].dtype != np.float32:
            raise ValueError(f"{name} must be float32, got {tensors[name].dtype}")
    for name in ("obj_trajs_mask", "map_polylines_mask"):
        if tensors[name].dtype != np.uint8:
            raise ValueError(f"{name} must be uint8, got {tensors[name].dtype}")
        if not np.isin(tensors[name], (0, 1)).all():
            raise ValueError(f"{name} contains values other than zero and one")
    for name in ("track_index_to_predict", "center_type_ids"):
        if tensors[name].dtype != np.int64:
            raise ValueError(f"{name} must be int64, got {tensors[name].dtype}")
    type_codes = tensors["center_type_ids"][:count]
    unknown = sorted(set(type_codes.tolist()) - _TYPE_NAMES.keys())
    if unknown:
        raise ValueError(f"unknown center_type_ids: {unknown}")
    inputs = {
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
            [selection.scene_builder_id for selection in selections]
        ),
        "scenario_id": np.asarray([scenario_id] * count),
    }
    return {
        "batch_size": 1,
        "batch_sample_count": [count],
        "input_dict": inputs,
    }


def _deployed_ground_truth(
    scen: scenario.Scenario,
    selections: list[reference_pack.TargetSelection],
) -> tuple[np.ndarray, np.ndarray]:
    count = len(selections)
    native = np.zeros(
        (count, rep.NUM_FUTURE_FRAMES, 2), dtype=np.float64
    )
    valid = np.zeros(
        (count, rep.NUM_FUTURE_FRAMES), dtype=bool
    )
    start = scen.current_time_index + 1
    for selection in selections:
        track = scen.tracks[selection.track_index]
        states = track.states[start : start + rep.NUM_FUTURE_FRAMES]
        available = min(len(states), rep.NUM_FUTURE_FRAMES)
        if not available:
            continue
        center = track.states[scen.current_time_index]
        heading = float(center[scenario.STATE_COL["heading"]])
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        delta_x = states[:available, scenario.STATE_COL["x"]] - float(
            center[scenario.STATE_COL["x"]]
        )
        delta_y = states[:available, scenario.STATE_COL["y"]] - float(
            center[scenario.STATE_COL["y"]]
        )
        native[selection.slot, :available, 0] = (
            delta_x * cos_h + delta_y * sin_h
        )
        native[selection.slot, :available, 1] = (
            -delta_x * sin_h + delta_y * cos_h
        )
        valid[selection.slot, :available] = (
            states[:available, scenario.STATE_COL["valid"]] > 0.0
        )
    return native[:, 1:30:2], valid[:, 1:30:2]


def _finite_mean(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    return float(finite.mean()) if finite.size else float("nan")


def _reports_from_accumulators(
    ablations: Sequence[Ablation],
    accumulators: list[_Accumulator],
    num_scenarios: int,
) -> list[Rung1Report]:
    reports: list[Rung1Report] = []
    for index, (ablation, accumulator) in enumerate(
        zip(ablations, accumulators, strict=True)
    ):
        ade = np.concatenate(accumulator.ade)
        fde = np.concatenate(accumulator.fde)
        dropped = int((~np.isfinite(ade) | ~np.isfinite(fde)).sum())
        paired_ade = None
        paired_fde = None
        if reports:
            previous = reports[-1]
            if (
                accumulator.agent_ids != previous.agent_ids
                or accumulator.scenario_ids != previous.scenario_ids
            ):
                raise RuntimeError(
                    f"ablation {ablation.name} changed target pairing"
                )
            paired_ade = metrics.paired_delta(
                ade,
                previous.min_ade_values,
                accumulator.agent_ids,
                accumulator.scenario_ids,
            )
            paired_fde = metrics.paired_delta(
                fde,
                previous.min_fde_values,
                accumulator.agent_ids,
                accumulator.scenario_ids,
            )
        reports.append(
            Rung1Report(
                ablation=ablation,
                num_scenarios=num_scenarios,
                num_targets=len(accumulator.agent_ids),
                num_dropped_no_future=dropped,
                min_ade_deployed=_finite_mean(ade),
                min_fde_deployed=_finite_mean(fde),
                paired_minade_from_previous=paired_ade,
                paired_minfde_from_previous=paired_fde,
                map_candidate_chunks=accumulator.candidate_chunks,
                map_retained_target_chunks=accumulator.retained_chunks,
                map_evicted_target_chunks_by_type=dict(accumulator.evicted),
                agent_ids=list(accumulator.agent_ids),
                scenario_ids=list(accumulator.scenario_ids),
                min_ade_values=ade,
                min_fde_values=fde,
            )
        )
    return reports


def _evaluate(
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    ablations: Sequence[Ablation],
    *,
    cfg_name: str,
    limit: int | None,
    device: str,
) -> list[Rung1Report]:
    if limit is not None and limit < 0:
        raise ValueError("limit must be non-negative")
    if device not in {"cuda", "cuda:0"}:
        raise ValueError(
            "official MTR calls .cuda() internally; rung 1 requires "
            "device='cuda' or 'cuda:0'"
        )
    if not torch.cuda.is_available():
        raise RuntimeError("rung 1 requires a CUDA-capable PyTorch environment")
    shard_path = _require_file(shard, "Waymo scenario shard")
    checkpoint_path = _require_file(checkpoint, "MTR checkpoint")
    mtr_root_path = _require_dir(mtr_root, "MTR root")
    cfg_path = _require_file(
        mtr_root_path / "tools/cfgs/waymo" / cfg_name,
        "official MTR config",
    )
    if not ablations:
        raise ValueError("at least one ablation is required")
    names = [ablation.name for ablation in ablations]
    if len(names) != len(set(names)):
        raise ValueError("ablation names must be unique")

    net = model.load_checkpoint(
        checkpoint_path,
        cfg_path,
        device=device,
        mtr_root=mtr_root_path,
    )
    accumulators = [_Accumulator() for _ in ablations]
    num_scenarios = 0
    for scen in scenario.load_scenarios(shard_path, limit=limit):
        selections = reference_pack.target_selection(scen)
        if not selections:
            continue
        gt, valid = _deployed_ground_truth(scen, selections)
        for ablation, accumulator in zip(
            ablations, accumulators, strict=True
        ):
            query = lq.QueryConfig(strategy=ablation.query_strategy)
            options = ablation.map_options()
            tensors = reference_pack.pack(
                scen, query, map_options=options
            )
            _, native = model.forward_batch(
                net,
                _model_batch(tensors, selections, scen.scenario_id),
            )
            deployed = metrics.to_deployed(native)
            if deployed.shape[0] != len(selections):
                raise RuntimeError(
                    f"{scen.scenario_id} {ablation.name} emitted "
                    f"{deployed.shape[0]} predictions for "
                    f"{len(selections)} targets"
                )
            accumulator.ade.append(metrics.min_ade(deployed, gt, valid))
            accumulator.fde.append(metrics.min_fde(deployed, gt, valid))
            accumulator.agent_ids.extend(
                selection.waymo_id for selection in selections
            )
            accumulator.scenario_ids.extend(
                [scen.scenario_id] * len(selections)
            )
            budget = reference_pack.map_packing_budget(
                scen, query, options
            )
            accumulator.candidate_chunks += budget.candidate_chunks
            accumulator.retained_chunks += budget.retained_target_chunks
            for feature_type, count in (
                budget.evicted_target_chunks_by_type.items()
            ):
                accumulator.evicted[feature_type] = (
                    accumulator.evicted.get(feature_type, 0) + count
                )
        num_scenarios += 1
    if num_scenarios == 0:
        raise RuntimeError(f"no scorable scenarios were read from {shard_path}")
    return _reports_from_accumulators(
        ablations, accumulators, num_scenarios
    )


def run(
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    ablation: Ablation,
    limit: int | None = None,
    *,
    cfg_name: str = "mtr+100_percent_data.yaml",
    device: str = "cuda",
) -> Rung1Report:
    """Score one ablation over a raw Waymo scenario shard."""
    return _evaluate(
        shard,
        checkpoint,
        mtr_root,
        [ablation],
        cfg_name=cfg_name,
        limit=limit,
        device=device,
    )[0]


def sweep(
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    limit: int | None = None,
    *,
    cfg_name: str = "mtr+100_percent_data.yaml",
    device: str = "cuda",
) -> list[Rung1Report]:
    """Score the complete one-axis degradation chain with adjacent pairing."""
    return _evaluate(
        shard,
        checkpoint,
        mtr_root,
        SWEEP,
        cfg_name=cfg_name,
        limit=limit,
        device=device,
    )


def format_sweep(reports: Sequence[Rung1Report]) -> str:
    if not reports:
        raise ValueError("cannot format an empty sweep")
    lines = [
        "Map budget sweep",
        "",
        f"{'policy':<36} {'targets':>7} {'minADE':>9} {'minFDE':>9} "
        f"{'ΔFDE mean':>11} {'chunks':>9} {'evicted':>9}",
    ]
    for report in reports:
        delta = report.paired_minfde_from_previous
        delta_text = "—" if delta is None else f"{delta.mean:+.4f}"
        evicted = sum(report.map_evicted_target_chunks_by_type.values())
        lines.append(
            f"{report.ablation.name:<36} {report.num_targets:>7} "
            f"{report.min_ade_deployed:>9.4f} "
            f"{report.min_fde_deployed:>9.4f} "
            f"{delta_text:>11} {report.map_candidate_chunks:>9} "
            f"{evicted:>9}"
        )
    details = [
        report.format()
        for report in reports[1:]
        if report.paired_minfde_from_previous is not None
    ]
    return "\n".join(lines) + ("\n\n" + "\n\n".join(details) if details else "")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shard", type=pathlib.Path, required=True)
    parser.add_argument("--checkpoint", type=pathlib.Path, required=True)
    parser.add_argument("--mtr-root", type=pathlib.Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sweep", action="store_true")
    mode.add_argument(
        "--ablation",
        choices=[ablation.name for ablation in SWEEP],
        default=SWEEP[-1].name,
    )
    parser.add_argument("--limit", type=int)
    parser.add_argument("--cfg-name", default="mtr+100_percent_data.yaml")
    parser.add_argument("--device", default="cuda")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.sweep:
        reports = sweep(
            args.shard,
            args.checkpoint,
            args.mtr_root,
            limit=args.limit,
            cfg_name=args.cfg_name,
            device=args.device,
        )
        print(format_sweep(reports))
    else:
        selected = next(
            ablation
            for ablation in SWEEP
            if ablation.name == args.ablation
        )
        report = run(
            args.shard,
            args.checkpoint,
            args.mtr_root,
            selected,
            limit=args.limit,
            cfg_name=args.cfg_name,
            device=args.device,
        )
        print(report.format())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
