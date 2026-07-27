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
"""Rung 3: compare a TensorRT engine with the official MTR checkpoint.

The engine always consumes the frozen eight-target contract. Official MTR is
run only for the populated target slots, and both outputs are scored against
the same target-local ground truth. TensorRT is imported only when the default
runner is constructed, so the correctness and reporting logic can be tested
without a TensorRT installation.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
import time
from collections.abc import Mapping, Sequence
from typing import Any, Protocol

import numpy as np

from . import metrics, model, representation as rep, rung2, tensor_io

_INPUT_NAMES = (
    "obj_trajs",
    "obj_trajs_mask",
    "obj_trajs_last_pos",
    "map_polylines",
    "map_polylines_mask",
    "map_polylines_center",
    "track_index_to_predict",
    "center_type_ids",
)
_OUTPUT_NAMES = ("pred_scores", "pred_trajs")


class EngineRunner(Protocol):
    """Minimal injectable inference surface used by the scoring harness."""

    def infer(
        self, inputs: Mapping[str, np.ndarray]
    ) -> Mapping[str, np.ndarray]: ...

    @property
    def peak_memory_mb(self) -> float | None: ...


@dataclasses.dataclass(frozen=True)
class AgentResult:
    scenario_id: str
    waymo_id: str
    scene_builder_id: str
    slot: int
    official_minfde_m: float
    engine_minfde_m: float
    paired_minfde_delta_m: float
    official_mode_probabilities: tuple[float, ...]
    engine_mode_probabilities: tuple[float, ...]
    paired_mode_probability_deltas: tuple[float, ...]
    official_top_mode: int
    engine_top_mode: int
    top_mode_agrees: bool

    @property
    def ledger_key(self) -> str:
        return f"{self.scenario_id}::{self.waymo_id}"


@dataclasses.dataclass(frozen=True)
class Rung3Report:
    paired_minfde: metrics.PairedDelta
    paired_mode_probability: tuple[metrics.PairedDelta, ...]
    top_mode_agreement: float
    latency_ms_p50: float | None
    latency_ms_p99: float | None
    peak_memory_mb: float | None
    per_agent: tuple[AgentResult, ...]
    correctness_only: bool
    num_scenarios: int = 0
    num_targets: int = 0
    num_dropped_no_future: int = 0

    def gate_ok(self) -> bool:
        paired = self.paired_minfde
        return bool(
            self.num_scenarios > 0
            and paired.count > 0
            and math.isfinite(paired.near_zero_fraction)
            and paired.near_zero_fraction >= 0.99
        )

    def format(self) -> str:
        lines = [
            f"Rung 3 - {self.num_scenarios} fixture scenarios, "
            f"{self.num_targets} accepted targets",
            f"Dropped without paired finite future: "
            f"{self.num_dropped_no_future}",
            metrics.format_paired_delta(
                self.paired_minfde, "Official MTR -> TensorRT  minFDE"
            ),
        ]
        for mode, delta in enumerate(self.paired_mode_probability):
            lines.append(
                metrics.format_paired_delta(
                    delta,
                    f"Official MTR -> TensorRT  mode {mode} probability",
                    units="probability",
                )
            )
        lines.append(f"Top-mode agreement: {self.top_mode_agreement:.1%}")
        if self.correctness_only:
            lines.append(
                "Performance: OMITTED (--correctness-only; no development-machine "
                "latency or memory is reportable)"
            )
        else:
            lines.append(
                f"Latency: p50={self.latency_ms_p50:.3f} ms, "
                f"p99={self.latency_ms_p99:.3f} ms"
            )
            lines.append(f"Peak memory: {self.peak_memory_mb:.1f} MB")
        ledger_lines = ["Per-agent ledger:"]
        ledger_lines.append(
            "  scenario_id::waymo_id  delta_minFDE_m  top(ref/engine)  "
            "delta_mode_probabilities"
        )
        for row in self.per_agent:
            probability_delta = ",".join(
                f"{value:+.6f}"
                for value in row.paired_mode_probability_deltas
            )
            ledger_lines.append(
                f"  {row.ledger_key}  {row.paired_minfde_delta_m:+.6f}  "
                f"{row.official_top_mode}/{row.engine_top_mode}  "
                f"[{probability_delta}]"
            )
        lines.append("\n".join(ledger_lines))
        lines.append(f"RUNG 3 GATE: {'PASS' if self.gate_ok() else 'FAIL'}")
        return "\n\n".join(lines)

    def as_dict(self) -> dict[str, Any]:
        """Return the stable Rung-4 handoff schema."""
        ledger: dict[str, dict[str, Any]] = {}
        for row in self.per_agent:
            if row.ledger_key in ledger:
                raise ValueError(f"duplicate per-agent ledger key {row.ledger_key!r}")
            ledger[row.ledger_key] = dataclasses.asdict(row)
        raw = {
            "schema_version": 1,
            "rung": "rung3",
            "correctness_only": self.correctness_only,
            "num_scenarios": self.num_scenarios,
            "num_targets": self.num_targets,
            "num_dropped_no_future": self.num_dropped_no_future,
            "gate_ok": self.gate_ok(),
            "paired_minfde": dataclasses.asdict(self.paired_minfde),
            "paired_mode_probability": [
                dataclasses.asdict(value)
                for value in self.paired_mode_probability
            ],
            "top_mode_agreement": self.top_mode_agreement,
            "latency_ms_p50": self.latency_ms_p50,
            "latency_ms_p99": self.latency_ms_p99,
            "peak_memory_mb": self.peak_memory_mb,
            "per_agent": ledger,
        }
        return _finite_json_values(raw)

    def write_json(self, path: pathlib.Path) -> None:
        output = pathlib.Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                self.as_dict(),
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )


@dataclasses.dataclass(frozen=True)
class _FixtureCase:
    path: pathlib.Path
    targets: tuple[rung2.TargetInfo, ...]
    tensors: dict[str, np.ndarray]
    gt_world: np.ndarray
    valid: np.ndarray
    official_scores: np.ndarray
    official_native: np.ndarray


def _finite_json_values(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {
            key: _finite_json_values(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_finite_json_values(item) for item in value]
    return value


def _engine_inputs(tensors: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    rung2._validate_bundle(tensors)
    inputs = {
        name: np.ascontiguousarray(tensors[name])
        for name in _INPUT_NAMES
    }
    for name in ("obj_trajs_mask", "map_polylines_mask"):
        inputs[name] = np.ascontiguousarray(inputs[name].astype(np.bool_))
    return inputs


def _engine_outputs(
    raw: Mapping[str, np.ndarray] | Sequence[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    if isinstance(raw, Mapping):
        missing = set(_OUTPUT_NAMES) - raw.keys()
        if missing:
            raise ValueError(
                f"engine output is missing bindings {sorted(missing)}"
            )
        scores = np.asarray(raw["pred_scores"])
        trajectories = np.asarray(raw["pred_trajs"])
    elif isinstance(raw, Sequence) and len(raw) == 2:
        scores = np.asarray(raw[0])
        trajectories = np.asarray(raw[1])
    else:
        raise TypeError(
            "engine inference must return a binding mapping or "
            "(pred_scores, pred_trajs)"
        )
    expected_scores = (rep.TARGET_CAPACITY, rep.NUM_MODES)
    expected_trajectories = (
        rep.TARGET_CAPACITY,
        rep.NUM_MODES,
        rep.NUM_FUTURE_FRAMES,
        rep.TRAJ_CHANNELS,
    )
    if scores.shape != expected_scores:
        raise ValueError(
            f"engine pred_scores must be {expected_scores}, got {scores.shape}"
        )
    if trajectories.shape != expected_trajectories:
        raise ValueError(
            "engine pred_trajs must be "
            f"{expected_trajectories}, got {trajectories.shape}"
        )
    if not np.issubdtype(scores.dtype, np.floating):
        raise ValueError(f"engine pred_scores must be floating, got {scores.dtype}")
    if not np.issubdtype(trajectories.dtype, np.floating):
        raise ValueError(
            f"engine pred_trajs must be floating, got {trajectories.dtype}"
        )
    return scores.astype(np.float32), trajectories.astype(np.float32)


class _TensorRtRunner:
    """Static TensorRT runner backed by torch CUDA allocations."""

    def __init__(self, engine_path: pathlib.Path):
        try:
            import tensorrt as trt
            import torch
        except ImportError as error:
            raise RuntimeError(
                "TensorRT inference requires both tensorrt and CUDA PyTorch"
            ) from error
        if not torch.cuda.is_available():
            raise RuntimeError("TensorRT inference requires a CUDA device")

        self._trt = trt
        self._torch = torch
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._free_before, _ = torch.cuda.mem_get_info()
        self._minimum_free = self._free_before
        self._stream = torch.cuda.Stream()
        self._runtime = trt.Runtime(self._logger)
        serialized = pathlib.Path(engine_path).read_bytes()
        self._engine = self._runtime.deserialize_cuda_engine(serialized)
        if self._engine is None:
            raise RuntimeError(f"could not deserialize TensorRT engine {engine_path}")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("could not create TensorRT execution context")

        if not hasattr(self._engine, "num_io_tensors"):
            raise RuntimeError(
                "rung3 requires TensorRT's name-based tensor API "
                "(num_io_tensors/execute_async_v3)"
            )
        self._input_names: list[str] = []
        self._output_names: list[str] = []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            mode = self._engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self._input_names.append(name)
            else:
                self._output_names.append(name)
        if set(self._input_names) != set(_INPUT_NAMES):
            raise ValueError(
                f"engine inputs {self._input_names} do not match {_INPUT_NAMES}"
            )
        if set(self._output_names) != set(_OUTPUT_NAMES):
            raise ValueError(
                f"engine outputs {self._output_names} do not match {_OUTPUT_NAMES}"
            )

    @property
    def peak_memory_mb(self) -> float:
        return (self._free_before - self._minimum_free) / (1024.0 * 1024.0)

    def _torch_dtype(self, name: str):
        np_dtype = np.dtype(self._trt.nptype(self._engine.get_tensor_dtype(name)))
        return self._torch.from_numpy(np.empty((), dtype=np_dtype)).dtype

    def infer(
        self, inputs: Mapping[str, np.ndarray]
    ) -> Mapping[str, np.ndarray]:
        missing = set(self._input_names) - inputs.keys()
        extra = inputs.keys() - set(self._input_names)
        if missing or extra:
            raise ValueError(
                f"engine inputs differ: missing={sorted(missing)}, "
                f"extra={sorted(extra)}"
            )
        device_tensors: dict[str, Any] = {}
        with self._torch.cuda.stream(self._stream):
            for name in self._input_names:
                array = np.ascontiguousarray(inputs[name])
                tensor = self._torch.from_numpy(array).to(
                    device="cuda", dtype=self._torch_dtype(name)
                )
                expected_shape = tuple(self._engine.get_tensor_shape(name))
                if tuple(tensor.shape) != expected_shape:
                    raise ValueError(
                        f"engine input {name} must be {expected_shape}, "
                        f"got {tuple(tensor.shape)}"
                    )
                device_tensors[name] = tensor

            for name in self._output_names:
                shape = tuple(self._context.get_tensor_shape(name))
                if any(dimension <= 0 for dimension in shape):
                    raise ValueError(
                        f"engine output {name} has unresolved shape {shape}"
                    )
                device_tensors[name] = self._torch.empty(
                    shape, device="cuda", dtype=self._torch_dtype(name)
                )

            for name, tensor in device_tensors.items():
                if not self._context.set_tensor_address(name, int(tensor.data_ptr())):
                    raise RuntimeError(f"could not bind TensorRT tensor {name}")
            if not self._context.execute_async_v3(self._stream.cuda_stream):
                raise RuntimeError("TensorRT execute_async_v3 failed")
        self._stream.synchronize()
        free_now, _ = self._torch.cuda.mem_get_info()
        self._minimum_free = min(self._minimum_free, free_now)
        return {
            name: device_tensors[name].detach().cpu().numpy().copy()
            for name in self._output_names
        }


def _infer(
    runner: EngineRunner,
    inputs: dict[str, np.ndarray],
    *,
    correctness_only: bool,
    benchmark_iterations: int,
) -> tuple[tuple[np.ndarray, np.ndarray], list[float]]:
    if correctness_only:
        return _engine_outputs(runner.infer(inputs)), []
    if benchmark_iterations <= 0:
        raise ValueError("benchmark_iterations must be positive")
    for _ in range(3):
        runner.infer(inputs)
    timings = []
    raw: Mapping[str, np.ndarray] | Sequence[np.ndarray] | None = None
    for _ in range(benchmark_iterations):
        start = time.perf_counter()
        raw = runner.infer(inputs)
        timings.append((time.perf_counter() - start) * 1000.0)
    assert raw is not None
    return _engine_outputs(raw), timings


def _paired_probability(
    engine: np.ndarray,
    official: np.ndarray,
    rows: list[AgentResult],
) -> tuple[metrics.PairedDelta, ...]:
    agent_ids = [row.waymo_id for row in rows]
    scenario_ids = [row.scenario_id for row in rows]
    return tuple(
        metrics.paired_delta(
            engine[:, mode],
            official[:, mode],
            agent_ids,
            scenario_ids,
        )
        for mode in range(rep.NUM_MODES)
    )


def run(
    fixtures: pathlib.Path,
    engine: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    *,
    cfg_name: str = "mtr+100_percent_data.yaml",
    correctness_only: bool = False,
    benchmark_iterations: int = 20,
    runner: EngineRunner | None = None,
) -> Rung3Report:
    """Compare TensorRT and official MTR on every committed fixture."""
    fixture_root = rung2._require_dir(fixtures, "fixture directory")
    checkpoint_path = rung2._require_file(checkpoint, "MTR checkpoint")
    mtr_root_path = rung2._require_dir(mtr_root, "MTR root")
    cfg_path = rung2._require_file(
        mtr_root_path / "tools/cfgs/waymo" / cfg_name,
        "official MTR config",
    )
    engine_path = (
        rung2._require_file(engine, "TensorRT engine")
        if runner is None
        else None
    )
    fixture_paths = rung2._fixture_directories(fixture_root)
    net = model.load_checkpoint(
        checkpoint_path,
        cfg_path,
        device="cuda",
        mtr_root=mtr_root_path,
    )
    rows: list[AgentResult] = []
    official_fde_all: list[np.ndarray] = []
    engine_fde_all: list[np.ndarray] = []
    official_probability_all: list[np.ndarray] = []
    engine_probability_all: list[np.ndarray] = []
    latency_ms: list[float] = []
    dropped = 0
    cases: list[_FixtureCase] = []

    for fixture in fixture_paths:
        with np.load(fixture / "ground_truth.npz", allow_pickle=False) as data:
            required = {"gt", "valid", "agent_ids", "types"}
            available = set(data.files)
            if not required <= available:
                raise ValueError(
                    f"{fixture / 'ground_truth.npz'} is missing "
                    f"{sorted(required - available)}"
                )
            targets = rung2._read_targets(fixture, data["agent_ids"])
            all_gt = np.asarray(data["gt"])
            all_valid = np.asarray(data["valid"]).astype(bool)
            all_types = np.asarray(data["types"])

        tensors = tensor_io.read_bundle(
            fixture / "expected_tensors.bin.gz"
        )
        rung2._validate_target_type_codes(
            tensors, targets, all_types, fixture.name
        )
        target_rows = [target.gt_row for target in targets]
        gt_world = all_gt[target_rows]
        valid = all_valid[target_rows]
        official_scores, official_native = model.forward_batch(
            net, rung2._model_batch(tensors, targets, fixture.name)
        )
        cases.append(
            _FixtureCase(
                path=fixture,
                targets=tuple(targets),
                tensors=tensors,
                gt_world=gt_world,
                valid=valid,
                official_scores=official_scores,
                official_native=official_native,
            )
        )

    # Run every official-MTR reference first. The default runner can now take
    # its memory baseline after PyTorch has populated its CUDA allocator cache,
    # but before TensorRT deserializes the engine.
    if runner is None:
        assert engine_path is not None
        runner = _TensorRtRunner(engine_path)

    for case in cases:
        fixture = case.path
        targets = list(case.targets)
        tensors = case.tensors
        gt_world = case.gt_world
        valid = case.valid
        official_scores = case.official_scores
        official_native = case.official_native
        count = len(targets)
        (engine_scores_all, engine_native_all), measured = _infer(
            runner,
            _engine_inputs(tensors),
            correctness_only=correctness_only,
            benchmark_iterations=benchmark_iterations,
        )
        latency_ms.extend(measured)
        engine_scores = engine_scores_all[:count]
        engine_native = engine_native_all[:count]
        official_deployed = metrics.to_deployed(official_native)
        engine_deployed = metrics.to_deployed(engine_native)
        local_gt, deployed_valid = rung2._deployed_ground_truth(
            gt_world,
            valid,
            targets,
            official_deployed.shape[2],
        )
        official_fde = metrics.min_fde(
            official_deployed, local_gt, deployed_valid
        )
        engine_fde = metrics.min_fde(
            engine_deployed, local_gt, deployed_valid
        )
        finite = np.isfinite(official_fde) & np.isfinite(engine_fde)
        dropped += int((~finite).sum())
        official_fde_all.append(official_fde[finite])
        engine_fde_all.append(engine_fde[finite])
        official_probability_all.append(official_scores[finite])
        engine_probability_all.append(engine_scores[finite])

        for index in np.flatnonzero(finite):
            target = targets[int(index)]
            reference_probabilities = official_scores[index]
            trt_probabilities = engine_scores[index]
            rows.append(
                AgentResult(
                    scenario_id=fixture.name,
                    waymo_id=target.waymo_id,
                    scene_builder_id=target.scene_builder_id,
                    slot=target.slot,
                    official_minfde_m=float(official_fde[index]),
                    engine_minfde_m=float(engine_fde[index]),
                    paired_minfde_delta_m=float(
                        engine_fde[index] - official_fde[index]
                    ),
                    official_mode_probabilities=tuple(
                        float(value) for value in reference_probabilities
                    ),
                    engine_mode_probabilities=tuple(
                        float(value) for value in trt_probabilities
                    ),
                    paired_mode_probability_deltas=tuple(
                        float(value)
                        for value in (
                            trt_probabilities - reference_probabilities
                        )
                    ),
                    official_top_mode=int(reference_probabilities.argmax()),
                    engine_top_mode=int(trt_probabilities.argmax()),
                    top_mode_agrees=bool(
                        reference_probabilities.argmax()
                        == trt_probabilities.argmax()
                    ),
                )
            )

    official_fde_values = np.concatenate(official_fde_all)
    engine_fde_values = np.concatenate(engine_fde_all)
    official_probabilities = np.concatenate(official_probability_all)
    engine_probabilities = np.concatenate(engine_probability_all)
    agent_ids = [row.waymo_id for row in rows]
    scenario_ids = [row.scenario_id for row in rows]
    agreement = (
        float(np.mean([row.top_mode_agrees for row in rows]))
        if rows
        else math.nan
    )
    if correctness_only:
        latency_p50 = None
        latency_p99 = None
        peak_memory = None
    else:
        latency_p50 = float(np.percentile(latency_ms, 50))
        latency_p99 = float(np.percentile(latency_ms, 99))
        peak_memory = runner.peak_memory_mb
        if peak_memory is None:
            raise RuntimeError("engine runner did not report peak memory")
    return Rung3Report(
        paired_minfde=metrics.paired_delta(
            engine_fde_values,
            official_fde_values,
            agent_ids,
            scenario_ids,
        ),
        paired_mode_probability=_paired_probability(
            engine_probabilities, official_probabilities, rows
        ),
        top_mode_agreement=agreement,
        latency_ms_p50=latency_p50,
        latency_ms_p99=latency_p99,
        peak_memory_mb=peak_memory,
        per_agent=tuple(rows),
        correctness_only=correctness_only,
        num_scenarios=len(fixture_paths),
        num_targets=len(rows),
        num_dropped_no_future=dropped,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", type=pathlib.Path, required=True)
    parser.add_argument("--engine", type=pathlib.Path, required=True)
    parser.add_argument("--checkpoint", type=pathlib.Path, required=True)
    parser.add_argument("--mtr-root", type=pathlib.Path, required=True)
    parser.add_argument("--cfg-name", default="mtr+100_percent_data.yaml")
    parser.add_argument("--correctness-only", action="store_true")
    parser.add_argument("--benchmark-iterations", type=int, default=20)
    parser.add_argument("--report-json", type=pathlib.Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run(
        fixtures=args.fixtures,
        engine=args.engine,
        checkpoint=args.checkpoint,
        mtr_root=args.mtr_root,
        cfg_name=args.cfg_name,
        correctness_only=args.correctness_only,
        benchmark_iterations=args.benchmark_iterations,
    )
    if args.report_json is not None:
        report.write_json(args.report_json)
    print(report.format())
    return 0 if report.gate_ok() else 1


if __name__ == "__main__":
    raise SystemExit(main())
