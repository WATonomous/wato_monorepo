#!/usr/bin/env python3
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
"""Re-export a six-camera BEVFusion model for a different camera count.

The camera encoder shares one set of weights across views (cameras are its
batch axis), and BEV pooling runs in CUDA outside the graphs. So the six-view
count lives only in camera.backbone.onnx's input/output shapes and in the
constant shapes its Reshape / Resize / Expand / Tile nodes take. This rewrites
those; learned tensors and quantization scales are unchanged, and every other
file in the model directory is copied as is. No retraining, but the model
never saw more than six nuScenes views: accuracy with a different rig is
unmeasured.

The result is checked by running both graphs in onnxruntime: views are
encoded independently, so every view's outputs from the new graph must equal
the six-camera graph's outputs for the same view.

Offline tool, not needed at runtime: pip install onnx onnxruntime.

Usage:
    python3 export_cameras.py \\
        --model-dir /opt/watonomous/models/bevfusion/resnet50_trt11_int8 \\
        --output-dir /opt/watonomous/models/bevfusion/resnet50_trt11_int8_8cam \\
        --num-cameras 8
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

BACKBONE = "camera.backbone.onnx"
EXPORTED_CAMERAS = 6
# Node inputs that hold a target shape, by op type.
SHAPE_INPUTS = {"Reshape": 1, "Resize": 3, "Expand": 1, "Tile": 1}


def camera_axis(shape: list[int]) -> int | None:
    """Position of the camera count in a constant shape, or None.

    The graph keeps cameras either leading, (cameras, ...), or after the batch
    in its 5-D input layout, (batch, cameras, C, H, W). A 6 anywhere else is
    something else (e.g. Swin's attention heads) and is left alone.
    """
    if shape and shape[0] == EXPORTED_CAMERAS:
        return 0
    if len(shape) == 5 and shape[0] in (1, -1) and shape[1] == EXPORTED_CAMERAS:
        return 1
    return None


def export_backbone(source: Path, destination: Path, num_cameras: int) -> list[str]:
    import onnx
    from onnx import numpy_helper

    model = onnx.load(str(source))
    graph = model.graph
    tensors = {t.name: t for t in graph.initializer}
    for node in graph.node:
        if node.op_type == "Constant":
            for attr in node.attribute:
                if attr.name == "value":
                    tensors[node.output[0]] = attr.t

    edits = []
    seen = set()
    for node in graph.node:
        index = SHAPE_INPUTS.get(node.op_type)
        if index is None or len(node.input) <= index:
            continue
        name = node.input[index]
        if name in seen or name not in tensors:
            continue  # computed at runtime, or already rewritten
        seen.add(name)
        value = numpy_helper.to_array(tensors[name]).copy()
        axis = camera_axis(value.tolist())
        if axis is None:
            continue
        before = value.tolist()
        value[axis] = num_cameras
        tensors[name].CopyFrom(numpy_helper.from_array(value, name=tensors[name].name))
        edits.append(f"{node.op_type} {name}: {before} -> {value.tolist()}")
    if not edits:
        raise ValueError(f"{source}: no constant shape carries the camera count")

    for value, axis in [(v, 1) for v in graph.input] + [(v, 0) for v in graph.output]:
        dim = value.type.tensor_type.shape.dim[axis]
        if dim.dim_value != EXPORTED_CAMERAS:
            raise ValueError(f"{value.name}: axis {axis} is {dim}, expected the camera count")
        dim.dim_value = num_cameras
    del graph.value_info[:]
    model = onnx.shape_inference.infer_shapes(model, strict_mode=True)
    onnx.checker.check_model(model, full_check=True)
    onnx.save(model, str(destination))
    return edits


def verify(source: Path, exported: Path, num_cameras: int) -> float:
    """Max abs difference between the two graphs' per-view outputs.

    Runs the six-camera graph on windows of six views covering all
    num_cameras views (padded with extra views when there are fewer than
    six) and compares each view's outputs with the exported graph's.
    """
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.log_severity_level = 3
    six = ort.InferenceSession(str(source), options, providers=["CPUExecutionProvider"])
    many = ort.InferenceSession(str(exported), options, providers=["CPUExecutionProvider"])
    dtypes = {"tensor(float)": np.float32, "tensor(float16)": np.float16}
    rng = np.random.default_rng(0)
    total = max(num_cameras, EXPORTED_CAMERAS)
    views = {}
    for arg in six.get_inputs():
        shape = [total] + list(arg.shape[2:])
        views[arg.name] = rng.standard_normal(shape).astype(dtypes[arg.type])
    got = many.run(None, {k: v[None, :num_cameras] for k, v in views.items()})

    worst = 0.0
    last = max(num_cameras - EXPORTED_CAMERAS, 0)
    for start in sorted({min(s, last) for s in range(0, num_cameras, EXPORTED_CAMERAS)}):
        window = slice(start, start + EXPORTED_CAMERAS)
        want = six.run(None, {k: v[None, window] for k, v in views.items()})
        for expected, actual in zip(want, got):
            n = min(EXPORTED_CAMERAS, num_cameras - start)
            diff = np.abs(expected[:n].astype(np.float64) - actual[start : start + n])
            scale = np.abs(expected[:n]).max()
            if not np.allclose(actual[start : start + n], expected[:n], rtol=1e-3, atol=1e-3 * scale):
                raise RuntimeError(
                    f"views {start}..{start + n - 1} differ from the six-camera graph "
                    f"(max abs diff {diff.max():.3g}, output scale {scale:.3g})"
                )
            worst = max(worst, float(diff.max()))
    return worst


def export(model_dir: Path, output_dir: Path, num_cameras: int) -> dict:
    if num_cameras < 1:
        raise ValueError("--num-cameras must be positive")
    if model_dir.resolve() == output_dir.resolve():
        raise ValueError("export into a new directory to keep the six-camera model")
    files = sorted(p for p in model_dir.iterdir() if p.is_file())
    if BACKBONE not in {p.name for p in files}:
        raise FileNotFoundError(f"{model_dir / BACKBONE} not found")
    output_dir.mkdir(parents=True, exist_ok=True)
    # build/ holds TensorRT plans for the six-camera graph; the node rebuilds
    # them under <output_dir>/build on first activation.
    for path in files:
        if path.name == BACKBONE:
            edits = export_backbone(path, output_dir / path.name, num_cameras)
        else:
            shutil.copyfile(path, output_dir / path.name)
    info = {
        "num_cameras": num_cameras,
        "source": str(model_dir.resolve()),
        "edits": edits,
        "max_abs_diff_vs_six_camera": verify(model_dir / BACKBONE, output_dir / BACKBONE, num_cameras),
    }
    (output_dir / "camera_export.json").write_text(json.dumps(info, indent=2) + "\n")
    return info


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-dir", type=Path, required=True, help="six-camera model directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--num-cameras", type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.model_dir, args.output_dir, args.num_cameras), indent=2))


if __name__ == "__main__":
    main()
