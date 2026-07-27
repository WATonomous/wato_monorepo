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
"""Build a machine-local TensorRT engine from the portable MTR artifacts."""

from __future__ import annotations

import argparse
import dataclasses
import importlib
import json
import os
import pathlib
import tempfile
import time
from collections.abc import Mapping, Sequence
from typing import Any

import yaml

_PRECISIONS = frozenset({"fp16", "fp32"})
_TACTIC_SOURCES = frozenset(
    {
        "EDGE_MASK_CONVOLUTIONS",
        "JIT_CONVOLUTIONS",
    }
)
_CONFIG_KEYS = frozenset(
    {
        "precision",
        "workspace_size_mb",
        "optimization_profiles",
        "tactic_sources",
        "builder_optimization_level",
    }
)


@dataclasses.dataclass(frozen=True)
class EngineInfo:
    """Metadata that identifies a non-portable TensorRT build."""

    sm_arch: str
    trt_version: str
    build_seconds: float


@dataclasses.dataclass(frozen=True)
class EngineBuildConfig:
    """Validated subset of TensorRT builder settings used by this harness."""

    precision: str
    workspace_size_mb: int
    optimization_profiles: tuple[object, ...]
    tactic_sources: tuple[str, ...]
    builder_optimization_level: int


def _require_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return value


def load_config(path: pathlib.Path) -> EngineBuildConfig:
    """Load and strictly validate an engine-build configuration."""
    path = pathlib.Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"engine build config does not exist: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("engine build config root must be a mapping")

    missing = _CONFIG_KEYS - raw.keys()
    extra = raw.keys() - _CONFIG_KEYS
    if missing:
        raise ValueError(f"engine build config is missing: {sorted(missing)}")
    if extra:
        raise ValueError(f"unknown engine build settings: {sorted(extra)}")

    precision = raw["precision"]
    if precision not in _PRECISIONS:
        raise ValueError(
            f"precision must be one of {sorted(_PRECISIONS)}, got {precision!r}"
        )

    workspace_size_mb = _require_int(raw["workspace_size_mb"], "workspace_size_mb")
    if workspace_size_mb <= 0:
        raise ValueError("workspace_size_mb must be positive")

    profiles = raw["optimization_profiles"]
    if not isinstance(profiles, list):
        raise ValueError("optimization_profiles must be a list")
    if profiles:
        raise ValueError(
            "optimization_profiles must be empty: the frozen MTR contract is static"
        )

    tactic_sources = raw["tactic_sources"]
    if (
        not isinstance(tactic_sources, list)
        or not tactic_sources
        or any(not isinstance(source, str) for source in tactic_sources)
    ):
        raise ValueError("tactic_sources must be a non-empty list of names")
    if len(tactic_sources) != len(set(tactic_sources)):
        raise ValueError("tactic_sources must not contain duplicates")
    unknown_sources = set(tactic_sources) - _TACTIC_SOURCES
    if unknown_sources:
        raise ValueError(f"unknown tactic sources: {sorted(unknown_sources)}")

    optimization_level = _require_int(
        raw["builder_optimization_level"], "builder_optimization_level"
    )
    if not 0 <= optimization_level <= 5:
        raise ValueError("builder_optimization_level must be in [0, 5]")

    return EngineBuildConfig(
        precision=str(precision),
        workspace_size_mb=workspace_size_mb,
        optimization_profiles=tuple(profiles),
        tactic_sources=tuple(tactic_sources),
        builder_optimization_level=optimization_level,
    )


def _validate_onnx(path: pathlib.Path) -> None:
    """Reject malformed or dynamic ONNX before starting an expensive TRT build."""
    try:
        onnx = importlib.import_module("onnx")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "ONNX validation requires the 'onnx' package in the model environment"
        ) from exc

    try:
        model = onnx.load(str(path))
        onnx.checker.check_model(model)
    except Exception as exc:
        raise ValueError(f"invalid ONNX model {path}: {exc}") from exc

    for value in (*model.graph.input, *model.graph.output):
        tensor_type = value.type.tensor_type
        if not tensor_type.HasField("shape"):
            raise ValueError(f"ONNX binding {value.name!r} has no tensor shape")
        dimensions = tensor_type.shape.dim
        if not dimensions:
            raise ValueError(f"ONNX binding {value.name!r} has a scalar shape")
        if any(dim.dim_value <= 0 or dim.dim_param for dim in dimensions):
            raise ValueError(
                f"ONNX binding {value.name!r} is dynamic; "
                "the frozen MTR contract requires positive static dimensions"
            )


def _import_tensorrt() -> Any:
    try:
        return importlib.import_module("tensorrt")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "TensorRT Python bindings are not installed in this environment; "
            "install the TensorRT version paired with the target vehicle runtime"
        ) from exc


def _check_cuda(error: object, operation: str, success: object) -> None:
    if error != success:
        raise RuntimeError(f"{operation} failed with CUDA error {error}")


def _detect_sm_arch() -> str:
    """Return the active CUDA device architecture as ``sm_<major><minor>``."""
    try:
        runtime = importlib.import_module("cuda.bindings.runtime")
        success = runtime.cudaError_t.cudaSuccess
        error, device = runtime.cudaGetDevice()
        _check_cuda(error, "cudaGetDevice", success)
        attr = runtime.cudaDeviceAttr
        error, major = runtime.cudaDeviceGetAttribute(
            attr.cudaDevAttrComputeCapabilityMajor, device
        )
        _check_cuda(error, "CUDA major capability query", success)
        error, minor = runtime.cudaDeviceGetAttribute(
            attr.cudaDevAttrComputeCapabilityMinor, device
        )
        _check_cuda(error, "CUDA minor capability query", success)
        return f"sm_{major}{minor}"
    except (ImportError, AttributeError):
        pass

    try:
        torch = importlib.import_module("torch")
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        major, minor = torch.cuda.get_device_capability()
        return f"sm_{major}{minor}"
    except (ImportError, AttributeError) as exc:
        raise RuntimeError(
            "cannot determine the active GPU architecture; install cuda-python "
            "or PyTorch with CUDA support"
        ) from exc


def _parser_errors(parser: Any) -> str:
    errors = [str(parser.get_error(index)) for index in range(parser.num_errors)]
    return "\n".join(errors) if errors else "TensorRT returned no parser diagnostic"


def _network_flags(trt: Any) -> int:
    explicit_batch = getattr(trt.NetworkDefinitionCreationFlag, "EXPLICIT_BATCH", None)
    if explicit_batch is None:
        return 0
    return 1 << int(explicit_batch)


def _configure_builder(
    trt: Any, builder: Any, builder_config: Any, cfg: EngineBuildConfig
) -> None:
    builder_config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, cfg.workspace_size_mb * 1024 * 1024
    )
    builder_config.builder_optimization_level = cfg.builder_optimization_level

    if cfg.precision == "fp16":
        fp16_flag = getattr(trt.BuilderFlag, "FP16", None)
        if fp16_flag is None:
            raise RuntimeError(
                f"TensorRT {trt.__version__} uses strongly typed networks and "
                "cannot convert an FP32 ONNX model via BuilderFlag.FP16; convert "
                "the portable ONNX to mixed FP16 with NVIDIA ModelOpt AutoCast "
                "before building"
            )
        if not getattr(builder, "platform_has_fast_fp16", False):
            raise RuntimeError(
                "requested fp16 build but the active GPU has no fast fp16"
            )
        builder_config.set_flag(fp16_flag)

    tactic_mask = 0
    unavailable = []
    for source_name in cfg.tactic_sources:
        source = getattr(trt.TacticSource, source_name, None)
        if source is None:
            unavailable.append(source_name)
        else:
            tactic_mask |= 1 << int(source)
    if unavailable:
        raise RuntimeError(
            f"TensorRT {trt.__version__} does not expose configured tactic sources: "
            f"{unavailable}"
        )
    if builder_config.set_tactic_sources(tactic_mask) is False:
        raise RuntimeError("TensorRT rejected the configured tactic-source mask")


def _write_atomic(path: pathlib.Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=path.parent, prefix=f".{path.name}.", delete=False
        ) as output:
            temp_name = output.name
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temp_name, path)
        temp_name = None
    finally:
        if temp_name is not None:
            pathlib.Path(temp_name).unlink(missing_ok=True)


def build(
    onnx_path: pathlib.Path,
    config_path: pathlib.Path,
    out_path: pathlib.Path,
) -> EngineInfo:
    """Build and atomically write a TensorRT plan for the active GPU."""
    onnx_path = pathlib.Path(onnx_path).expanduser().resolve()
    config_path = pathlib.Path(config_path).expanduser().resolve()
    out_path = pathlib.Path(out_path).expanduser().resolve()
    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model does not exist: {onnx_path}")
    if out_path == onnx_path or out_path == config_path:
        raise ValueError("engine output must not overwrite a portable input artifact")

    cfg = load_config(config_path)
    _validate_onnx(onnx_path)
    trt = _import_tensorrt()
    sm_arch = _detect_sm_arch()

    logger = trt.Logger(trt.Logger.WARNING)
    started = time.perf_counter()
    builder = trt.Builder(logger)
    network = builder.create_network(_network_flags(trt))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse_from_file(str(onnx_path)):
        raise RuntimeError(
            f"TensorRT could not parse {onnx_path}:\n{_parser_errors(parser)}"
        )

    builder_config = builder.create_builder_config()
    _configure_builder(trt, builder, builder_config, cfg)
    serialized = builder.build_serialized_network(network, builder_config)
    if serialized is None:
        raise RuntimeError("TensorRT engine build failed without a serialized plan")
    payload = bytes(serialized)
    if not payload:
        raise RuntimeError("TensorRT engine build produced an empty serialized plan")
    _write_atomic(out_path, payload)
    elapsed = time.perf_counter() - started
    return EngineInfo(
        sm_arch=sm_arch,
        trt_version=str(trt.__version__),
        build_seconds=elapsed,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    info = build(args.onnx, args.config, args.out)
    print(json.dumps(dataclasses.asdict(info), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
