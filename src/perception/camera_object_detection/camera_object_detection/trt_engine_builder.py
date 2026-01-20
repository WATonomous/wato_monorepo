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
"""
TensorRT engine builder for YOLOv8 models with batched optimization.

Decouples engine build from runtime inference used by yolov8_detection.py.

Two input modes:
1) Build from ONNX directly (fastest)
2) Build from a PyTorch .pt by exporting to ONNX first (via Ultralytics)

Examples:
  # Build from ONNX
  python -m camera_object_detection.trt_engine_builder \
    --onnx /perception_models/tensorRT.onnx \
    --engine /perception_models/tensorRT.engine \
    --min-shape 1 3 640 640 \
    --opt-shape 3 3 640 640 \
    --max-shape 6 3 640 640 \
    --fp16

  # Build from PyTorch .pt (exports to ONNX under the hood)
  python -m camera_object_detection.trt_engine_builder \
    --pt /perception_models/yolov8m.pt \
    --engine /perception_models/tensorRT.engine \
    --imgsz 640 \
    --dynamic \
    --opset 12 \
    --fp16
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import torch
import tensorrt as trt
from pathlib import Path

# Optional import: Ultralytics for .pt -> ONNX export
try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dep at build time
    YOLO = None  # type: ignore


def _log_shapes(network: trt.INetworkDefinition, logger: trt.ILogger) -> None:
    inputs = [network.get_input(i) for i in range(network.num_inputs)]
    outputs = [network.get_output(i) for i in range(network.num_outputs)]
    for t in inputs:
        logger.log(trt.Logger.INFO, f"Model {t.name} shape: {t.shape} {t.dtype}")
    for t in outputs:
        logger.log(trt.Logger.INFO, f"Model {t.name} shape: {t.shape} {t.dtype}")


def build_trt_engine(
    onnx_model_path: str,
    engine_output_path: str,
    min_shape: List[int] = None,
    opt_shape: List[int] = None,
    max_shape: List[int] = None,
    fp16: bool = True,
    int8: bool = False,
) -> None:
    if not os.path.isfile(onnx_model_path):
        raise FileNotFoundError(f"ONNX model not found: {onnx_model_path}")

    logger = trt.Logger(trt.Logger.VERBOSE)
    trt.init_libnvinfer_plugins(logger, namespace="")

    builder = trt.Builder(logger)
    config = builder.create_builder_config()

    # Use device memory as workspace limit when available
    if torch.cuda.is_available():
        total_mem = torch.cuda.get_device_properties(0).total_memory
    else:
        # Fallback to a conservative default (1 GiB)
        total_mem = 1 << 30
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, total_mem)

    # Maintain a timing cache to speed up rebuilds
    cache = config.create_timing_cache(b"")
    config.set_timing_cache(cache, ignore_mismatch=False)

    # Create network with explicit batch
    flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flag)
    parser = trt.OnnxParser(network, logger)

    with open(onnx_model_path, "rb") as f:
        parsed = parser.parse(f.read())
    if not parsed:
        logger.log(trt.Logger.ERROR, "Failed to parse ONNX file")
        for i in range(parser.num_errors):
            err = parser.get_error(i)
            logger.log(trt.Logger.ERROR, str(err))
        raise RuntimeError("ONNX parsing failed")

    _log_shapes(network, logger)

    # Optimization profile for dynamic batch sizes
    if max_shape is not None:
        profile = builder.create_optimization_profile()
        # Apply the same profile to all network inputs
        for i in range(network.num_inputs):
            inp = network.get_input(i)
            profile.set_shape(
                inp.name, tuple(min_shape), tuple(opt_shape), tuple(max_shape)
            )
        config.add_optimization_profile(profile)

    if fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    if int8:
        config.set_flag(trt.BuilderFlag.INT8)

    engine_bytes = builder.build_serialized_network(network, config)
    if engine_bytes is None:
        raise RuntimeError("Failed to build TensorRT engine")

    os.makedirs(os.path.dirname(engine_output_path) or ".", exist_ok=True)
    with open(engine_output_path, "wb") as f:
        f.write(engine_bytes)

    logger.log(trt.Logger.INFO, f"Wrote TensorRT engine -> {engine_output_path}")


def export_pt_to_onnx(
    pt_path: str,
    onnx_out: str,
    imgsz: int = 640,
    dynamic: bool = True,
    opset: int = 12,
) -> str:
    """Export a YOLOv8 PyTorch model to ONNX using Ultralytics.

    Returns the ONNX path (same as onnx_out) on success.
    """
    if YOLO is None:
        raise ImportError(
            "Ultralytics is required to export .pt to ONNX. Install 'ultralytics'."
        )

    model = YOLO(pt_path)
    # imgsz may be int or [H,W]; we use square for simplicity here
    export_kwargs = {
        "format": "onnx",
        "opset": int(opset),
        "dynamic": bool(dynamic),
        "imgsz": int(imgsz),
        "optimize": False,
    }
    # Ultralytics sometimes forces CPU export by setting CUDA_VISIBLE_DEVICES=-1.
    # Preserve the caller's GPU visibility so subsequent TensorRT build sees the GPU.
    prev_cuda_visible = os.environ.get("CUDA_VISIBLE_DEVICES")
    try:
        # Ultralytics will write the file near the model by default, but we want a controlled path
        # So let it export, then move/rename if needed.
        result = model.export(**export_kwargs)
    finally:
        if prev_cuda_visible is None:
            os.environ.pop("CUDA_VISIBLE_DEVICES", None)
        else:
            os.environ["CUDA_VISIBLE_DEVICES"] = prev_cuda_visible
    # result can be a list or a path depending on version; find the .onnx
    out_path: Optional[Path] = None
    if isinstance(result, (list, tuple)):
        for p in result:
            if str(p).endswith(".onnx"):
                out_path = Path(p)
                break
    elif isinstance(result, (str, Path)) and str(result).endswith(".onnx"):
        out_path = Path(result)

    if out_path is None or not out_path.exists():
        # As a fallback, probe common default name next to the .pt
        probe = Path(pt_path).with_suffix(".onnx")
        if probe.exists():
            out_path = probe
        else:
            raise RuntimeError("Failed to locate exported ONNX file from Ultralytics")

    onnx_out_path = Path(onnx_out)
    onnx_out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.resolve() != onnx_out_path.resolve():
        onnx_out_path.write_bytes(out_path.read_bytes())

    return str(onnx_out_path)


def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build TensorRT engine from ONNX with batching"
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--onnx", help="Path to ONNX model")
    src.add_argument("--pt", help="Path to PyTorch .pt model (Ultralytics)")
    p.add_argument("--engine", required=True, help="Output .engine path")
    p.add_argument(
        "--min-shape",
        nargs=4,
        type=int,
        default=[1, 3, 640, 640],
        metavar=("B", "C", "H", "W"),
        help="Min input shape (explicit batch)",
    )
    p.add_argument(
        "--opt-shape",
        nargs=4,
        type=int,
        default=[3, 3, 640, 640],
        metavar=("B", "C", "H", "W"),
        help="Opt input shape (explicit batch)",
    )
    p.add_argument(
        "--max-shape",
        nargs=4,
        type=int,
        default=[6, 3, 640, 640],
        metavar=("B", "C", "H", "W"),
        help="Max input shape (explicit batch)",
    )
    p.add_argument("--fp16", action="store_true", help="Enable FP16")
    p.add_argument("--int8", action="store_true", help="Enable INT8")
    # Options for .pt -> ONNX export
    p.add_argument("--onnx-out", help="Where to write exported ONNX (when using --pt)")
    p.add_argument("--imgsz", type=int, default=640, help="Export image size (square)")
    p.add_argument(
        "--dynamic", action="store_true", help="Export ONNX with dynamic axes"
    )
    p.add_argument("--opset", type=int, default=12, help="ONNX opset for export")
    return p.parse_args(argv)


def main(argv: List[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    onnx_path = args.onnx
    # If .pt is provided, export to ONNX first
    if args.pt:
        # Decide ONNX temp output
        if args.onnx_out:
            onnx_out = args.onnx_out
        else:
            # Default next to engine path, named after the .pt
            pt_stem = Path(args.pt).stem
            onnx_out = str(Path(args.engine).with_name(f"{pt_stem}.onnx"))
        onnx_path = export_pt_to_onnx(
            pt_path=args.pt,
            onnx_out=onnx_out,
            imgsz=int(args.imgsz),
            dynamic=bool(args.dynamic),
            opset=int(args.opset),
        )

    build_trt_engine(
        onnx_model_path=onnx_path,
        engine_output_path=args.engine,
        min_shape=args.min_shape,
        opt_shape=args.opt_shape,
        max_shape=args.max_shape,
        fp16=bool(args.fp16),
        int8=bool(args.int8),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
