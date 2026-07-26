# Prediction Quality Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a staged harness that measures how much prediction quality the `prediction_ml` integration destroys and attributes each loss to a specific pipeline component.

**Architecture:** A ladder of paired comparisons. Each rung runs the same Waymo scenarios through a progressively more WATO-specific pipeline; the delta between adjacent rungs names exactly one culprit. A Python evaluation package handles Waymo parsing, ROS-bag conversion, ablation, and scoring; a C++ gtest replays committed fixtures so scene-builder parity is a CI gate needing no GPU and no Python.

**Tech Stack:** Python 3.10 (numpy, protobuf, `rosbag2_py`, PyTorch), C++17 (ROS 2 Humble, `rosbag2_cpp`, gtest), TensorRT 10.x, CMake ≥ 3.17.

**Design spec:** `docs/superpowers/specs/2026-07-26-prediction-quality-harness-design.md`

## Global Constraints

- `cmake_minimum_required(VERSION 3.17)`; `CMAKE_CXX_STANDARD 17`.
- Every new source file carries the repo's Apache-2.0 header (copy verbatim from `src/world_modeling/prediction_ml/src/scene_builder.cpp:1-13`).
- Tensor contract is fixed and static, per `config/contract_8_target.yaml`: `obj_trajs [8,128,11,29]`, `obj_trajs_mask [8,128,11]` bool, `obj_trajs_last_pos [8,128,3]`, `map_polylines [8,768,20,9]`, `map_polylines_mask [8,768,20]` bool, `map_polylines_center [8,768,3]`, `track_index_to_predict [8]` int64, `center_type_ids [8]` int64, `pred_scores [8,6]`, `pred_trajs [8,6,80,7]`.
- **No Python module may import both `rosbag2_py` and `torch`.** The ROS env and the MTR/PyTorch env are separate installs; mixing them in one module makes the package unimportable in both. Enforced by Task 2's import test.
- Deployed metric is **3.0 s at 0.2 s = 15 waypoints**, 6 modes. Native 8 s metric is computed **once** (Task 5) and never in routine runs.
- TensorRT engines are build products: **never committed, never copied between machines.** Only `mtr.onnx` + `engine_build.yaml` are portable artifacts.
- **No 3060 Ti latency or memory number appears in any report.** Perf numbers come from the vehicle's RTX 4080 only.
- MTR object type codes are `TYPE_VEHICLE=0`, `TYPE_PEDESTRIAN=1`, `TYPE_CYCLIST=2` (`DEVELOPING.md:20-21`).

## Two hard gates

Work is ordered around two gates. **Do not begin Task 14 or later until both have passed.**

- **Gate A — the weights (Task 5).** Rung 0 must reproduce the published MTR figures within 3%. This validates both the evaluation environment and the HuggingFace checkpoint, which is the one artifact in this plan we do not control. It is deliberately early: it is the cheapest test of the riskiest external dependency, and it has no dependency on the converter.
- **Gate B — C++ parity (Tasks 11–12).** The rung-2 gtest must pass and the rung-2 paired-delta report must show ≥99% of agents under 1e-3 m Δ minFDE (excluding the masked-history acceleration cohort, which Task 12 measures rather than gates).

TensorRT work (Tasks 14–17) is gated because a TRT discrepancy is only interpretable once the weights, the converter, and the C++ packing are each known-good. Debugging four unknowns simultaneously inside a TensorRT build log is the failure mode this ordering exists to prevent.

## File structure

**Python evaluation package** — `src/world_modeling/prediction_ml/eval/`, excluded from the colcon build by a `COLCON_IGNORE` marker.

| File | Responsibility | Env |
|---|---|---|
| `wato_mtr_eval/representation.py` | The information contract as constants | pure |
| `wato_mtr_eval/tensor_io.py` | Gzipped binary tensor bundle read/write | pure |
| `wato_mtr_eval/metrics.py` | minADE/minFDE, paired delta distribution | pure |
| `wato_mtr_eval/tfrecord.py` | Minimal TFRecord record reader | pure |
| `wato_mtr_eval/scenario.py` | Waymo `Scenario` → plain dataclasses | proto |
| `wato_mtr_eval/ledger.py` | Information-loss accounting | pure |
| `wato_mtr_eval/waymo_to_ros.py` | Scenario → mcap bag | **ros** |
| `wato_mtr_eval/lanelet_query.py` | Waymo map → `LaneletAhead`, radius / reachable | **ros** |
| `wato_mtr_eval/model.py` | Checkpoint load, MTR forward | **model** |
| `wato_mtr_eval/rung0.py` … `rung4.py` | One module per rung | mixed |
| `wato_mtr_eval/ops_dense.py` | ONNX-expressible replacements for the CUDA ops | **model** |
| `wato_mtr_eval/export_onnx.py` / `build_engine.py` | Export and engine build | **model** |

**C++ additions** — `src/world_modeling/prediction_ml/`

| File | Responsibility |
|---|---|
| `REPRESENTATION.md` | Human-readable information contract |
| `include/prediction_ml/tensor_io.hpp`, `src/tensor_io.cpp` | Read the same binary bundle format |
| `tools/scene_dump.cpp` | CLI: mcap → tensor bundle, for Python scoring |
| `test/test_tensor_io.cpp` | Round-trip and format tests |
| `test/test_scene_builder_parity.cpp` | **The CI gate** |
| `test/fixtures/scenario_*/` | Committed mcap + expected tensors |

**Modified:** `CMakeLists.txt`, `package.xml`, `config/params.yaml`, `include/prediction_ml/mtr_types.hpp`, `include/prediction_ml/scene_builder.hpp`, `src/scene_builder.cpp`, `src/prediction_ml_node.cpp`.

---

### Task 1: Evaluation package skeleton and the representation contract

Establishes the package, the environment split that every later task depends on, and the information contract that rungs 1 and 2 must implement identically.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/COLCON_IGNORE`
- Create: `src/world_modeling/prediction_ml/eval/pyproject.toml`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/__init__.py`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/representation.py`
- Create: `src/world_modeling/prediction_ml/REPRESENTATION.md`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_representation.py`

**Interfaces:**
- Produces: `representation.py` constants — `TARGET_CAPACITY=8`, `CONTEXT_CAPACITY=128`, `HISTORY_STEPS=11`, `HISTORY_RATE_HZ=10.0`, `OBJ_FEATURE_DIM=29`, `NUM_SRC_POLYLINES=768`, `POINTS_PER_POLYLINE=20`, `MAP_FEATURE_DIM=9`, `NUM_MODES=6`, `NUM_FUTURE_FRAMES=80`, `TRAJ_CHANNELS=7`, `DEPLOYED_HORIZON_S=3.0`, `DEPLOYED_STEP_S=0.2`, `DEPLOYED_STEPS=15`, `MODEL_STEP_S=0.1`, `OBJ_COL` (dict of column-name → slice), `MTR_TYPE_CODES` (dict).

- [ ] **Step 1: Create the package skeleton**

```bash
cd src/world_modeling/prediction_ml
mkdir -p eval/wato_mtr_eval eval/tests
touch eval/COLCON_IGNORE eval/wato_mtr_eval/__init__.py
```

`eval/pyproject.toml`:

```toml
[project]
name = "wato-mtr-eval"
version = "0.1.0"
description = "Prediction quality harness for prediction_ml"
requires-python = ">=3.10"
dependencies = ["numpy>=1.24"]

[project.optional-dependencies]
# Installed in the ROS 2 Humble environment. Never alongside 'model'.
ros = ["protobuf>=3.20"]
# Installed in the MTR/PyTorch environment. Never alongside 'ros'.
model = ["torch>=2.0", "protobuf>=3.20"]
dev = ["pytest>=7.0"]

[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
include = ["wato_mtr_eval*"]
```

- [ ] **Step 2: Write the failing test**

`eval/tests/test_representation.py`:

```python
"""The representation contract must agree with the frozen tensor contract."""
import pathlib
import re

import pytest

from wato_mtr_eval import representation as rep

CONTRACT = pathlib.Path(__file__).resolve().parents[2] / "config" / "contract_8_target.yaml"


def test_obj_columns_partition_the_feature_dim():
    """Every one of the 29 columns is claimed exactly once, with no gaps."""
    claimed = []
    for name, sl in rep.OBJ_COL.items():
        claimed.extend(range(sl.start, sl.stop))
    assert sorted(claimed) == list(range(rep.OBJ_FEATURE_DIM)), (
        "columns must tile [0, 29) exactly once"
    )


def test_deployed_steps_derive_from_horizon_and_step():
    assert rep.DEPLOYED_STEPS == int(rep.DEPLOYED_HORIZON_S / rep.DEPLOYED_STEP_S)


def test_deployed_step_is_whole_multiple_of_model_step():
    """output_converter.cpp rejects a non-integer stride; mirror that invariant."""
    ratio = rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S
    assert abs(ratio - round(ratio)) < 1e-9


def test_shapes_match_the_frozen_contract_yaml():
    """Parse contract_8_target.yaml textually so this test needs no yaml dependency."""
    text = CONTRACT.read_text()

    def shape_of(name: str) -> list[int]:
        m = re.search(
            rf"- name: {name}\s+dtype: \w+\s+shape: \[([^\]]+)\]", text
        )
        assert m, f"{name} not found in contract"
        return [int(x) for x in m.group(1).split(",")]

    assert shape_of("obj_trajs") == [
        rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, rep.HISTORY_STEPS, rep.OBJ_FEATURE_DIM
    ]
    assert shape_of("map_polylines") == [
        rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, rep.POINTS_PER_POLYLINE, rep.MAP_FEATURE_DIM
    ]
    assert shape_of("pred_trajs") == [
        rep.TARGET_CAPACITY, rep.NUM_MODES, rep.NUM_FUTURE_FRAMES, rep.TRAJ_CHANNELS
    ]


def test_module_does_not_import_ros_or_torch():
    """representation.py is pure: importable in every environment."""
    import sys
    assert "rosbag2_py" not in sys.modules
    assert "torch" not in sys.modules
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd src/world_modeling/prediction_ml/eval
pip install -e ".[dev]"
pytest tests/test_representation.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.representation'`.

- [ ] **Step 4: Write `representation.py`**

Column offsets are taken from `src/world_modeling/prediction_ml/src/scene_builder.cpp:763-781`.

```python
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
"""The WATO information contract.

Rung 1 (Python ablation) and rung 2 (C++ SceneBuilder) must implement this
identically, or the delta between them is meaningless. This module is the
machine-readable half; REPRESENTATION.md is the human half.

Pure: imports neither rosbag2_py nor torch, so it loads in both environments.
"""

# --- Tensor contract (mirrors config/contract_8_target.yaml) ---
TARGET_CAPACITY = 8
CONTEXT_CAPACITY = 128
HISTORY_STEPS = 11
OBJ_FEATURE_DIM = 29
NUM_SRC_POLYLINES = 768
POINTS_PER_POLYLINE = 20
MAP_FEATURE_DIM = 9
NUM_MODES = 6
NUM_FUTURE_FRAMES = 80
TRAJ_CHANNELS = 7

# --- Timing ---
HISTORY_RATE_HZ = 10.0
MODEL_STEP_S = 0.1
DEPLOYED_HORIZON_S = 3.0
DEPLOYED_STEP_S = 0.2
DEPLOYED_STEPS = 15  # output_converter.cpp: min(80 // 2, 3.0 / 0.2)
NATIVE_HORIZON_S = 8.0

# --- obj_trajs column layout (scene_builder.cpp:763-781) ---
OBJ_COL = {
    "position": slice(0, 3),        # rx, ry, rz  target-centered
    "size": slice(3, 6),            # dx, dy, dz
    "type_onehot": slice(6, 9),     # vehicle, pedestrian, cyclist
    "is_center": slice(9, 10),
    "is_sdc": slice(10, 11),
    "time_onehot": slice(11, 22),   # one per history step
    "scalar_time": slice(22, 23),   # desired_times[s] - current_time
    "heading": slice(23, 25),       # sin, cos of heading relative to target
    "velocity": slice(25, 27),      # target-frame vx, vy
    "acceleration": slice(27, 29),  # target-frame ax, ay
}

# --- map_polylines column layout (scene_builder.cpp:939) ---
MAP_COL = {
    "position": slice(0, 3),
    "direction": slice(3, 6),
    "global_type": slice(6, 7),
    "pre_position": slice(7, 9),  # pre_x, pre_y only
}

# --- Object types ---
MTR_TYPE_CODES = {"vehicle": 0, "pedestrian": 1, "cyclist": 2}

# Detection3D hypothesis class_id strings SceneBuilder accepts
# (scene_builder.cpp:238-246). The converter must emit one of these.
CLASS_ID_FOR_TYPE = {"vehicle": "vehicle", "pedestrian": "pedestrian", "cyclist": "cyclist"}

# Velocity is smuggled through a results[] entry with this class_id, carrying
# vx/vy/vz in the hypothesis pose *position* (scene_builder.cpp:118-127).
VELOCITY_CLASS_ID = "linear_velocity"

# --- Information WATO discards relative to Waymo ---
# Each entry is an ablation axis for the rung-1 sweep.
DISCARDED = {
    "map_polyline_types": "~20 Waymo types collapsed to one lane-centerline token",
    "road_edges": "present in Lanelet.msg as BOUNDARY_ROAD_EDGE; not packed",
    "crosswalks": "present in Lanelet.msg as lanelet_type=='crosswalk'; not packed",
    "lane_boundaries": "left/right boundary geometry present; not packed",
    "traffic_light_state": "genuinely absent: no dynamic signal state in lanelet_msgs",
    "stop_lines": "present as RegulatoryElement.ref_lines; not packed",
}

# Generic token SceneBuilder writes for every polyline (scene_builder.cpp:50).
WATO_GLOBAL_TYPE = 0
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
pytest tests/test_representation.py -v
```

Expected: 5 passed.

- [ ] **Step 6: Write `REPRESENTATION.md`**

Create `src/world_modeling/prediction_ml/REPRESENTATION.md` with these sections, sourcing every claim from code:

1. **Purpose** — one paragraph: this is the contract rungs 1 and 2 implement identically; the harness's attribution depends on it.
2. **Agent history** — 11 steps at 10 Hz (1.1 s), resampled to fixed slots, missing samples masked. Cite `mtr_types.hpp:56-57`.
3. **The 29 `obj_trajs` columns** — reproduce the table from the design spec's "Resolved from code" section verbatim, citing `scene_builder.cpp:763-781`.
4. **Capacities** — 8 targets, 128 context agents ranked by ego-frame distance, 768 polylines × 20 points. Note that overflow is silent truncation, which the ledger counts.
5. **Map representation** — 9 columns `[x, y, z, dir_x, dir_y, dir_z, global_type, pre_x, pre_y]` (`scene_builder.cpp:939`); `global_type` is one constant for all polylines (`scene_builder.cpp:50`).
6. **What WATO discards** — the `DISCARDED` table above, and critically: road edges, crosswalks, boundary types and stop lines are **present in `lanelet_msgs/Lanelet` and thrown away by SceneBuilder**, not missing from the vehicle. Only dynamic traffic-light state is genuinely unavailable.
7. **Output** — MTR emits 6 modes × 80 steps at 0.1 s; the node emits 15 waypoints at 0.2 s over 3.0 s (`output_converter.cpp:135-148`).

- [ ] **Step 7: Commit**

```bash
git add src/world_modeling/prediction_ml/eval src/world_modeling/prediction_ml/REPRESENTATION.md
git commit -m "feat(prediction_ml): add eval package skeleton and representation contract"
```

---

### Task 2: Environment-split guard

The ROS and PyTorch environments are separate installs. A single module importing both makes the package unusable in either. This test makes that constraint enforceable rather than aspirational.

**Files:**
- Test: `src/world_modeling/prediction_ml/eval/tests/test_env_split.py`

**Interfaces:**
- Consumes: nothing.
- Produces: the `ROS_ONLY_MODULES` / `MODEL_ONLY_MODULES` lists that later tasks append to as they add modules.

- [ ] **Step 1: Write the failing test**

```python
"""No module may import both the ROS and the PyTorch stacks.

The ROS 2 Humble environment and the MTR/PyTorch environment are separate
installs. A module importing both is unimportable in both.
"""
import ast
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1] / "wato_mtr_eval"

ROS_IMPORTS = {"rosbag2_py", "rclpy", "vision_msgs", "geometry_msgs", "nav_msgs", "lanelet_msgs"}
MODEL_IMPORTS = {"torch", "tensorrt", "onnx", "onnxruntime"}


def _imported_roots(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            roots.add(node.module.split(".")[0])
    return roots


@pytest.mark.parametrize("path", sorted(PKG.glob("*.py")), ids=lambda p: p.name)
def test_no_module_mixes_ros_and_model_stacks(path):
    roots = _imported_roots(path)
    ros_hits = roots & ROS_IMPORTS
    model_hits = roots & MODEL_IMPORTS
    assert not (ros_hits and model_hits), (
        f"{path.name} imports both ROS {sorted(ros_hits)} and model {sorted(model_hits)}. "
        "Split it: the two environments are separate installs."
    )
```

- [ ] **Step 2: Run the test**

```bash
pytest tests/test_env_split.py -v
```

Expected: PASS (only `representation.py` exists, which is pure). The test earns its keep as later tasks add modules.

- [ ] **Step 3: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/tests/test_env_split.py
git commit -m "test(prediction_ml): guard the ROS/PyTorch environment split"
```

---

### Task 3: Tensor bundle format — Python side

A gzipped binary container for the eight input tensors plus the two output tensors, written by Python and read by C++ (Task 9). Chosen over `.npz` so the C++ reader needs zlib only, with no new dependency.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/tensor_io.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_tensor_io.py`

**Interfaces:**
- Consumes: `representation` constants.
- Produces:
  - `write_bundle(path: pathlib.Path, tensors: dict[str, np.ndarray]) -> None`
  - `read_bundle(path: pathlib.Path) -> dict[str, np.ndarray]`
  - Format constants `MAGIC = b"WMTR"`, `VERSION = 1`.

**Format** (all little-endian, whole file gzipped):

```
magic   4 bytes  "WMTR"
version uint32
count   uint32
repeat count times:
  name_len  uint32
  name      name_len bytes, UTF-8
  dtype     uint8   0=float32 1=uint8 2=int64
  ndim      uint8
  dims      ndim * uint32
  data      product(dims) * itemsize bytes, C order
```

- [ ] **Step 1: Write the failing test**

```python
import gzip
import numpy as np
import pytest

from wato_mtr_eval import tensor_io


def _sample() -> dict[str, np.ndarray]:
    rng = np.random.default_rng(0)
    return {
        "obj_trajs": rng.standard_normal((2, 3, 4, 5), dtype=np.float32),
        "obj_trajs_mask": rng.integers(0, 2, (2, 3, 4), dtype=np.uint8),
        "track_index_to_predict": np.arange(2, dtype=np.int64),
    }


def test_round_trip_preserves_values_shapes_and_dtypes(tmp_path):
    original = _sample()
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, original)
    loaded = tensor_io.read_bundle(path)

    assert set(loaded) == set(original)
    for name, arr in original.items():
        assert loaded[name].dtype == arr.dtype, name
        assert loaded[name].shape == arr.shape, name
        np.testing.assert_array_equal(loaded[name], arr, err_msg=name)


def test_file_is_gzip_and_starts_with_magic(tmp_path):
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, _sample())
    with gzip.open(path, "rb") as fh:
        assert fh.read(4) == tensor_io.MAGIC


def test_rejects_unsupported_dtype(tmp_path):
    with pytest.raises(ValueError, match="unsupported dtype"):
        tensor_io.write_bundle(tmp_path / "b.bin.gz", {"x": np.zeros(3, dtype=np.float64)})


def test_rejects_truncated_file(tmp_path):
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, _sample())
    with gzip.open(path, "rb") as fh:
        blob = fh.read()
    truncated = tmp_path / "t.bin.gz"
    with gzip.open(truncated, "wb") as fh:
        fh.write(blob[: len(blob) // 2])
    with pytest.raises(ValueError, match="truncated"):
        tensor_io.read_bundle(truncated)


def test_rejects_bad_magic(tmp_path):
    path = tmp_path / "b.bin.gz"
    with gzip.open(path, "wb") as fh:
        fh.write(b"XXXX" + b"\x00" * 32)
    with pytest.raises(ValueError, match="magic"):
        tensor_io.read_bundle(path)


def test_sparse_padding_compresses_hard(tmp_path):
    """Fixtures are committable only because the buffers are mostly zeros."""
    big = {"map_polylines": np.zeros((8, 768, 20, 9), dtype=np.float32)}
    big["map_polylines"][:, :50] = 1.0  # ~6.5% occupancy, as in a real frame
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, big)
    raw = big["map_polylines"].nbytes
    assert path.stat().st_size < raw / 20, "expected >20x compression on padded buffers"
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_tensor_io.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.tensor_io'`.

- [ ] **Step 3: Write `tensor_io.py`**

```python
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
"""Gzipped binary tensor bundle, readable from both Python and C++.

Deliberately not .npz: the C++ fixture reader then needs only zlib, which is
already in the ROS dependency tree, rather than a new npz dependency for a
test-only code path.
"""
from __future__ import annotations

import gzip
import pathlib
import struct

import numpy as np

MAGIC = b"WMTR"
VERSION = 1

_DTYPE_CODE = {np.dtype(np.float32): 0, np.dtype(np.uint8): 1, np.dtype(np.int64): 2}
_CODE_DTYPE = {v: k for k, v in _DTYPE_CODE.items()}


def write_bundle(path: pathlib.Path, tensors: dict[str, np.ndarray]) -> None:
    """Write tensors to a gzipped bundle. Arrays are stored C-order."""
    parts = [MAGIC, struct.pack("<II", VERSION, len(tensors))]
    for name, arr in tensors.items():
        dtype = np.dtype(arr.dtype)
        if dtype not in _DTYPE_CODE:
            raise ValueError(f"unsupported dtype {dtype} for tensor {name!r}")
        arr = np.ascontiguousarray(arr)
        encoded = name.encode("utf-8")
        parts.append(struct.pack("<I", len(encoded)))
        parts.append(encoded)
        parts.append(struct.pack("<BB", _DTYPE_CODE[dtype], arr.ndim))
        parts.append(struct.pack(f"<{arr.ndim}I", *arr.shape))
        parts.append(arr.tobytes(order="C"))

    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wb") as fh:
        fh.write(b"".join(parts))


def read_bundle(path: pathlib.Path) -> dict[str, np.ndarray]:
    """Read a bundle written by write_bundle."""
    with gzip.open(path, "rb") as fh:
        blob = fh.read()

    def need(offset: int, count: int) -> int:
        if offset + count > len(blob):
            raise ValueError(f"truncated bundle: need {count} bytes at {offset}")
        return offset + count

    if len(blob) < 12 or blob[:4] != MAGIC:
        raise ValueError(f"bad magic: expected {MAGIC!r}")
    version, count = struct.unpack_from("<II", blob, 4)
    if version != VERSION:
        raise ValueError(f"unsupported bundle version {version}")

    out: dict[str, np.ndarray] = {}
    off = 12
    for _ in range(count):
        off = need(off, 4)
        (name_len,) = struct.unpack_from("<I", blob, off - 4)
        off = need(off, name_len)
        name = blob[off - name_len : off].decode("utf-8")
        off = need(off, 2)
        dtype_code, ndim = struct.unpack_from("<BB", blob, off - 2)
        off = need(off, 4 * ndim)
        dims = struct.unpack_from(f"<{ndim}I", blob, off - 4 * ndim)

        dtype = _CODE_DTYPE[dtype_code]
        nbytes = int(np.prod(dims, dtype=np.int64)) * dtype.itemsize
        off = need(off, nbytes)
        out[name] = np.frombuffer(
            blob, dtype=dtype, count=int(np.prod(dims, dtype=np.int64)), offset=off - nbytes
        ).reshape(dims).copy()

    return out
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/test_tensor_io.py -v
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/tensor_io.py \
        src/world_modeling/prediction_ml/eval/tests/test_tensor_io.py
git commit -m "feat(prediction_ml): add tensor bundle format for eval fixtures"
```

---

### Task 4: Metrics and the paired delta distribution

The scoring core. Pure numpy, no ROS, no torch — so it is testable in isolation and importable everywhere.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/metrics.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_metrics.py`

**Interfaces:**
- Consumes: `representation` constants.
- Produces:
  - `min_ade(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray) -> np.ndarray` — `pred [A,K,T,2]`, `gt [A,T,2]`, `valid [A,T]` bool → `[A]`
  - `min_fde(pred, gt, valid) -> np.ndarray` — `[A]`
  - `to_deployed(pred: np.ndarray) -> np.ndarray` — `[A,K,80,C]` → `[A,K,15,2]`
  - `PairedDelta` dataclass with `mean, median, p90, p99, max, near_zero_fraction, worst: list[WorstRow]`
  - `paired_delta(a: np.ndarray, b: np.ndarray, agent_ids: list[str], scenario_ids: list[str], n_worst: int = 10) -> PairedDelta`
  - `format_paired_delta(d: PairedDelta, label: str, units: str = "m") -> str`

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
import pytest

from wato_mtr_eval import metrics, representation as rep


def test_min_ade_picks_the_best_mode():
    # One agent, two modes, three steps. Mode 1 is exact; mode 0 is off by 3.
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 2, 3, 2), dtype=np.float32)
    pred[0, 0, :, 0] = 3.0
    valid = np.ones((1, 3), dtype=bool)
    np.testing.assert_allclose(metrics.min_ade(pred, gt, valid), [0.0], atol=1e-6)


def test_min_ade_ignores_invalid_steps():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    pred[0, 0, 2, 0] = 30.0  # huge error, but that step is invalid
    valid = np.array([[True, True, False]])
    np.testing.assert_allclose(metrics.min_ade(pred, gt, valid), [0.0], atol=1e-6)


def test_min_fde_uses_the_last_valid_step_not_the_last_step():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    pred[0, 0, 1, 0] = 4.0   # last VALID step -> this is the FDE
    pred[0, 0, 2, 0] = 99.0  # invalid, must be ignored
    valid = np.array([[True, True, False]])
    np.testing.assert_allclose(metrics.min_fde(pred, gt, valid), [4.0], atol=1e-6)


def test_agent_with_no_valid_steps_yields_nan():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    valid = np.zeros((1, 3), dtype=bool)
    assert np.isnan(metrics.min_ade(pred, gt, valid)[0])
    assert np.isnan(metrics.min_fde(pred, gt, valid)[0])


def test_to_deployed_strides_and_truncates_to_15_steps():
    pred = np.arange(1 * 1 * 80 * 7, dtype=np.float32).reshape(1, 1, 80, 7)
    out = metrics.to_deployed(pred)
    assert out.shape == (1, 1, rep.DEPLOYED_STEPS, 2)
    # stride 2 over the first 30 model steps: indices 1, 3, 5, ...
    expected_first = pred[0, 0, 1, :2]
    np.testing.assert_allclose(out[0, 0, 0], expected_first)


def test_paired_delta_reports_distribution_not_just_mean():
    # 1200 agents at zero, 68 at 0.19: the failure mode an aggregate hides.
    a = np.zeros(1268, dtype=np.float64)
    b = np.zeros(1268, dtype=np.float64)
    b[:68] = 0.19
    ids = [f"agent{i}" for i in range(1268)]
    scen = ["s0"] * 1268

    d = metrics.paired_delta(b, a, ids, scen, n_worst=5)
    assert d.mean == pytest.approx(0.19 * 68 / 1268, abs=1e-9)
    assert d.median == pytest.approx(0.0)
    assert d.max == pytest.approx(0.19)
    assert d.near_zero_fraction == pytest.approx(1200 / 1268)
    assert len(d.worst) == 5
    assert all(row.delta == pytest.approx(0.19) for row in d.worst)


def test_paired_delta_distinguishes_uniform_loss_from_a_broken_path():
    ids = [f"a{i}" for i in range(1268)]
    scen = ["s0"] * 1268
    base = np.zeros(1268)

    uniform = metrics.paired_delta(np.full(1268, 0.01), base, ids, scen)
    spiky = np.zeros(1268)
    spiky[:68] = 0.19
    broken = metrics.paired_delta(spiky, base, ids, scen)

    # Means are close; only the distribution separates them.
    assert uniform.mean == pytest.approx(0.01, abs=1e-9)
    assert broken.mean == pytest.approx(0.0102, abs=1e-3)
    assert uniform.near_zero_fraction == pytest.approx(0.0)
    assert broken.near_zero_fraction == pytest.approx(1200 / 1268)


def test_paired_delta_drops_nan_pairs():
    a = np.array([1.0, np.nan, 3.0])
    b = np.array([1.0, 2.0, np.nan])
    d = metrics.paired_delta(a, b, ["x", "y", "z"], ["s"] * 3)
    assert d.count == 1


def test_paired_delta_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="same length"):
        metrics.paired_delta(np.zeros(3), np.zeros(4), ["a"] * 3, ["s"] * 3)


def test_format_paired_delta_includes_worst_rows_and_near_zero():
    ids = [f"a{i}" for i in range(10)]
    d = metrics.paired_delta(np.linspace(0, 1, 10), np.zeros(10), ids, ["s"] * 10, n_worst=3)
    text = metrics.format_paired_delta(d, "R1 -> R2 minFDE")
    assert "R1 -> R2 minFDE" in text
    assert "worst 3" in text
    assert "p90" in text
    assert "|delta| < 1e-3" in text
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_metrics.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.metrics'`.

- [ ] **Step 3: Write `metrics.py`**

```python
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
"""Displacement metrics and paired delta distributions.

Rung-to-rung comparison is paired: the same scenarios and the same target
agents pass through both pipelines, so the per-agent difference is directly
available. Reporting only the difference of aggregates discards that. A mean
delta of 0.01 m could be every agent at 0.01 (uniform float precision loss,
benign) or 94% at zero and 6% at 0.19 (one code path broken). Those demand
opposite responses and no aggregate separates them.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from . import representation as rep

NEAR_ZERO_M = 1e-3


def _displacement(pred: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """[A,K,T,2] against [A,T,2] -> [A,K,T] L2 distance."""
    return np.linalg.norm(pred[..., :2] - gt[:, None, :, :2], axis=-1)


def min_ade(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Minimum-over-modes average displacement error, per agent. NaN if no valid step."""
    dist = _displacement(pred, gt)
    mask = valid[:, None, :]
    counts = mask.sum(axis=-1)
    with np.errstate(invalid="ignore", divide="ignore"):
        ade = np.where(counts > 0, np.where(mask, dist, 0.0).sum(axis=-1) / counts, np.nan)
    return np.nanmin(ade, axis=-1) if ade.size else ade


def min_fde(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Minimum-over-modes final displacement error at each agent's LAST VALID step."""
    num_agents, _, num_steps = _displacement(pred, gt).shape
    dist = _displacement(pred, gt)
    out = np.full(num_agents, np.nan)
    for a in range(num_agents):
        idx = np.flatnonzero(valid[a])
        if idx.size == 0:
            continue
        out[a] = dist[a, :, idx[-1]].min()
    return out


def to_deployed(pred: np.ndarray) -> np.ndarray:
    """[A,K,80,C] model output -> [A,K,15,2], matching output_converter.cpp.

    Stride = DEPLOYED_STEP_S / MODEL_STEP_S, then clamp to DEPLOYED_STEPS. The
    first emitted waypoint is model step (stride - 1), i.e. the end of the first
    deployed interval, not step 0.
    """
    stride = int(round(rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S))
    strided = pred[:, :, stride - 1 :: stride, :2]
    return strided[:, :, : rep.DEPLOYED_STEPS]


@dataclasses.dataclass(frozen=True)
class WorstRow:
    scenario_id: str
    agent_id: str
    delta: float


@dataclasses.dataclass(frozen=True)
class PairedDelta:
    count: int
    mean: float
    median: float
    p90: float
    p99: float
    max: float
    near_zero_fraction: float
    worst: list[WorstRow]


def paired_delta(
    a: np.ndarray,
    b: np.ndarray,
    agent_ids: list[str],
    scenario_ids: list[str],
    n_worst: int = 10,
) -> PairedDelta:
    """Distribution of per-agent (a - b). Pairs where either side is NaN are dropped."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if not (len(a) == len(b) == len(agent_ids) == len(scenario_ids)):
        raise ValueError("a, b, agent_ids and scenario_ids must have the same length")

    keep = np.isfinite(a) & np.isfinite(b)
    delta = a[keep] - b[keep]
    kept_agents = [agent_ids[i] for i in np.flatnonzero(keep)]
    kept_scen = [scenario_ids[i] for i in np.flatnonzero(keep)]

    if delta.size == 0:
        return PairedDelta(0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, [])

    order = np.argsort(-np.abs(delta))[:n_worst]
    worst = [WorstRow(kept_scen[i], kept_agents[i], float(delta[i])) for i in order]

    return PairedDelta(
        count=int(delta.size),
        mean=float(delta.mean()),
        median=float(np.median(delta)),
        p90=float(np.percentile(delta, 90)),
        p99=float(np.percentile(delta, 99)),
        max=float(delta[np.argmax(np.abs(delta))]),
        near_zero_fraction=float((np.abs(delta) < NEAR_ZERO_M).mean()),
        worst=worst,
    )


def format_paired_delta(d: PairedDelta, label: str, units: str = "m") -> str:
    """Human-readable report. The worst rows are the debugging entry point."""
    lines = [
        f"{label}  paired over {d.count} target agents",
        f"  mean      {d.mean:+.4f} {units}",
        f"  median    {d.median:+.4f} {units}",
        f"  p90       {d.p90:+.4f} {units}",
        f"  p99       {d.p99:+.4f} {units}",
        f"  max       {d.max:+.4f} {units}",
        f"  |delta| < 1e-3:  {d.near_zero_fraction * d.count:.0f} / {d.count}"
        f"  ({d.near_zero_fraction:.1%})",
        "",
        f"  worst {len(d.worst)} agents:",
        f"    {'scenario_id':<24} {'agent_id':<12} {'delta':>10}",
    ]
    lines += [f"    {r.scenario_id:<24} {r.agent_id:<12} {r.delta:>+10.4f}" for r in d.worst]
    return "\n".join(lines)
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/test_metrics.py -v
```

Expected: 10 passed.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/metrics.py \
        src/world_modeling/prediction_ml/eval/tests/test_metrics.py
git commit -m "feat(prediction_ml): add displacement metrics and paired delta distribution"
```

---

### Task 5: Waymo scenario reader

Reads the validation shard already in the repo root without pulling in TensorFlow. TFRecord framing is a length-prefixed container that takes ~20 lines to parse, so the only heavy dependency is `scenario_pb2` from `waymo-open-dataset`.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/tfrecord.py`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/scenario.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_tfrecord.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_scenario.py`

**Interfaces:**
- Consumes: `representation` constants.
- Produces:
  - `tfrecord.read_records(path: pathlib.Path) -> Iterator[bytes]`
  - `scenario.Track` dataclass: `object_id: int`, `object_type: str` (`"vehicle"|"pedestrian"|"cyclist"|"other"`), `states: np.ndarray [91, 10]` (`x,y,z,length,width,height,heading,vx,vy,valid`)
  - `scenario.MapPolyline` dataclass: `feature_id: int`, `feature_type: str`, `points: np.ndarray [N,3]`
  - `scenario.Scenario` dataclass: `scenario_id, timestamps_s, current_time_index, sdc_track_index, tracks, tracks_to_predict (list[int] of track indices), map_polylines`
  - `scenario.load_scenarios(path, limit=None) -> Iterator[Scenario]`
  - `scenario.STATE_COL` dict mapping name → column index

- [ ] **Step 1: Write the failing TFRecord test**

```python
import crcmod  # noqa: F401  (only to document the framing; CRC is not verified)
import gzip
import struct

import pytest

from wato_mtr_eval import tfrecord


def _frame(payload: bytes) -> bytes:
    """A TFRecord entry: u64 length, u32 length-crc, payload, u32 payload-crc."""
    return struct.pack("<Q", len(payload)) + b"\x00" * 4 + payload + b"\x00" * 4


def test_reads_multiple_records_in_order(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(_frame(b"alpha") + _frame(b"bravo") + _frame(b"charlie"))
    assert list(tfrecord.read_records(path)) == [b"alpha", b"bravo", b"charlie"]


def test_empty_file_yields_nothing(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(b"")
    assert list(tfrecord.read_records(path)) == []


def test_truncated_payload_raises(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(struct.pack("<Q", 100) + b"\x00" * 4 + b"short")
    with pytest.raises(ValueError, match="truncated"):
        list(tfrecord.read_records(path))
```

Remove the `crcmod` import before committing — it is shown only to document that the two 4-byte fields are masked CRC32C values which this reader intentionally skips.

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_tfrecord.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.tfrecord'`.

- [ ] **Step 3: Write `tfrecord.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header, verbatim from scene_builder.cpp:1-13)
"""Minimal TFRecord reader.

TFRecord framing is: u64 length, u32 masked-crc32c(length), payload,
u32 masked-crc32c(payload). Parsing it directly avoids a TensorFlow
dependency for what amounts to twenty lines of struct unpacking. CRCs are
not verified; a corrupt shard surfaces as a protobuf parse failure instead.
"""
from __future__ import annotations

import pathlib
import struct
from typing import Iterator

_LENGTH_SIZE = 8
_CRC_SIZE = 4


def read_records(path: pathlib.Path) -> Iterator[bytes]:
    """Yield each record payload from a TFRecord file."""
    with open(path, "rb") as fh:
        while True:
            header = fh.read(_LENGTH_SIZE)
            if not header:
                return
            if len(header) < _LENGTH_SIZE:
                raise ValueError("truncated tfrecord: incomplete length header")
            (length,) = struct.unpack("<Q", header)

            if len(fh.read(_CRC_SIZE)) < _CRC_SIZE:
                raise ValueError("truncated tfrecord: incomplete length crc")

            payload = fh.read(length)
            if len(payload) < length:
                raise ValueError(
                    f"truncated tfrecord: wanted {length} payload bytes, got {len(payload)}"
                )
            if len(fh.read(_CRC_SIZE)) < _CRC_SIZE:
                raise ValueError("truncated tfrecord: incomplete payload crc")

            yield payload
```

- [ ] **Step 4: Run the TFRecord test to verify it passes**

```bash
pytest tests/test_tfrecord.py -v
```

Expected: 3 passed.

- [ ] **Step 5: Write the failing scenario test**

This test needs the real shard. It is marked so CI can skip it while a developer machine runs it.

```python
import pathlib

import numpy as np
import pytest

from wato_mtr_eval import representation as rep, scenario

SHARD = (
    pathlib.Path(__file__).resolve().parents[4]
    / "uncompressed_scenario_validation_validation.tfrecord-00000-of-00150"
)
needs_shard = pytest.mark.skipif(not SHARD.exists(), reason="waymo validation shard not present")


@pytest.fixture(scope="module")
def first_scenario():
    return next(scenario.load_scenarios(SHARD, limit=1))


@needs_shard
def test_scenario_has_the_expected_temporal_layout(first_scenario):
    s = first_scenario
    assert s.current_time_index == 10, "10 past steps + current"
    assert len(s.timestamps_s) == 91, "11 history + 80 future"
    # 80 future steps at MODEL_STEP_S is exactly the native horizon
    assert len(s.timestamps_s) - s.current_time_index - 1 == rep.NUM_FUTURE_FRAMES


@needs_shard
def test_tracks_carry_full_state_arrays(first_scenario):
    s = first_scenario
    assert s.tracks, "scenario must contain tracks"
    for t in s.tracks:
        assert t.states.shape == (91, 10)
        assert t.object_type in {"vehicle", "pedestrian", "cyclist", "other"}


@needs_shard
def test_sdc_and_targets_are_valid_track_indices(first_scenario):
    s = first_scenario
    assert 0 <= s.sdc_track_index < len(s.tracks)
    assert s.tracks_to_predict, "validation scenarios designate targets"
    assert all(0 <= i < len(s.tracks) for i in s.tracks_to_predict)
    assert len(s.tracks_to_predict) <= rep.TARGET_CAPACITY, (
        "more designated targets than the contract's 8 slots"
    )


@needs_shard
def test_map_polylines_are_typed_and_include_more_than_lanes(first_scenario):
    """The gap the rung-1 sweep prices: Waymo ships types WATO discards."""
    types = {p.feature_type for p in first_scenario.map_polylines}
    assert "lane" in types
    assert types - {"lane"}, "expected road edges / crosswalks / lines beyond lanes"


@needs_shard
def test_shard_yields_the_documented_scenario_count():
    """DEVELOPING.md:36-38 records 286 scenarios in this shard."""
    assert sum(1 for _ in scenario.load_scenarios(SHARD)) == 286
```

- [ ] **Step 6: Run the scenario test to verify it fails**

```bash
pytest tests/test_scenario.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.scenario'`.

- [ ] **Step 7: Write `scenario.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Waymo Scenario protos flattened into plain dataclasses.

Everything downstream consumes these rather than the protos, so the converter,
the ablation and the ledger share one representation of the source data.
Imports protobuf but neither ROS nor torch, so it loads in both environments.
"""
from __future__ import annotations

import dataclasses
import pathlib
from typing import Iterator

import numpy as np
from waymo_open_dataset.protos import scenario_pb2

from . import tfrecord

STATE_COL = {
    "x": 0, "y": 1, "z": 2,
    "length": 3, "width": 4, "height": 5,
    "heading": 6, "vx": 7, "vy": 8, "valid": 9,
}
NUM_STATE_COLS = len(STATE_COL)

# Waymo Track.ObjectType -> the three types MTR predicts. Anything else is
# "other"; SceneBuilder's parseObjectClass returns nullopt for those, so they
# are dropped. The ledger counts the drops rather than letting them go silent.
_OBJECT_TYPE = {1: "vehicle", 2: "pedestrian", 3: "cyclist"}

_MAP_FEATURE_FIELDS = (
    ("lane", "lane"),
    ("road_line", "road_line"),
    ("road_edge", "road_edge"),
    ("crosswalk", "crosswalk"),
    ("speed_bump", "speed_bump"),
    ("driveway", "driveway"),
)


@dataclasses.dataclass(frozen=True)
class Track:
    object_id: int
    object_type: str
    states: np.ndarray  # [91, 10]


@dataclasses.dataclass(frozen=True)
class MapPolyline:
    feature_id: int
    feature_type: str
    points: np.ndarray  # [N, 3]


@dataclasses.dataclass(frozen=True)
class Scenario:
    scenario_id: str
    timestamps_s: np.ndarray
    current_time_index: int
    sdc_track_index: int
    tracks: list[Track]
    tracks_to_predict: list[int]
    map_polylines: list[MapPolyline]


def _track_from_proto(proto) -> Track:
    states = np.zeros((len(proto.states), NUM_STATE_COLS), dtype=np.float64)
    for i, st in enumerate(proto.states):
        states[i] = (
            st.center_x, st.center_y, st.center_z,
            st.length, st.width, st.height,
            st.heading, st.velocity_x, st.velocity_y,
            1.0 if st.valid else 0.0,
        )
    return Track(
        object_id=proto.id,
        object_type=_OBJECT_TYPE.get(proto.object_type, "other"),
        states=states,
    )


def _polylines_from_proto(proto) -> list[MapPolyline]:
    out: list[MapPolyline] = []
    for feature in proto.map_features:
        which = feature.WhichOneof("feature_data")
        for field, label in _MAP_FEATURE_FIELDS:
            if which != field:
                continue
            sub = getattr(feature, field)
            raw = getattr(sub, "polyline", None) or getattr(sub, "polygon", None)
            if not raw:
                continue
            pts = np.array([[p.x, p.y, p.z] for p in raw], dtype=np.float64)
            out.append(MapPolyline(feature_id=feature.id, feature_type=label, points=pts))
    return out


def load_scenarios(path: pathlib.Path, limit: int | None = None) -> Iterator[Scenario]:
    """Parse scenarios from a Waymo Motion tfrecord shard."""
    for index, payload in enumerate(tfrecord.read_records(path)):
        if limit is not None and index >= limit:
            return
        proto = scenario_pb2.Scenario()
        proto.ParseFromString(payload)
        yield Scenario(
            scenario_id=proto.scenario_id,
            timestamps_s=np.array(proto.timestamps_seconds, dtype=np.float64),
            current_time_index=proto.current_time_index,
            sdc_track_index=proto.sdc_track_index,
            tracks=[_track_from_proto(t) for t in proto.tracks],
            tracks_to_predict=[t.track_index for t in proto.tracks_to_predict],
            map_polylines=_polylines_from_proto(proto),
        )
```

- [ ] **Step 8: Run both tests to verify they pass**

```bash
pip install waymo-open-dataset-tf-2-12-0
pytest tests/test_tfrecord.py tests/test_scenario.py -v
```

Expected: 3 passed, 5 passed. If the shard is absent the scenario tests skip rather than fail.

If `test_shard_yields_the_documented_scenario_count` reports a number other than 286, stop and reconcile against `DEVELOPING.md:36-38` before continuing — every later rung's agent counts derive from this shard.

- [ ] **Step 9: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/tfrecord.py \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/scenario.py \
        src/world_modeling/prediction_ml/eval/tests/test_tfrecord.py \
        src/world_modeling/prediction_ml/eval/tests/test_scenario.py
git commit -m "feat(prediction_ml): add waymo scenario reader without a tensorflow dependency"
```

---

### Task 6: GATE A — validate the checkpoint (rung 0)

**This is the weights test.** It loads `checkpoint_epoch_30.pth`, runs the official MTR preprocessing and forward pass over the validation shard, and checks the result against the published figures. It validates two things at once: that the evaluation environment computes metrics correctly, and that the HuggingFace checkpoint is the MTR reproduction it claims to be.

It is scheduled here — before the converter, before any C++ work — because it is the cheapest test of the one artifact in this plan we do not control. If these weights are wrong, everything built on them is wasted, and that must surface now rather than in week three.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/model.py`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung0.py`
- Create: `src/world_modeling/prediction_ml/eval/README.md`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_model.py`

**Interfaces:**
- Consumes: `scenario.load_scenarios`, `metrics.min_ade`, `metrics.min_fde`, `metrics.to_deployed`.
- Produces:
  - `model.load_checkpoint(path, cfg_path, device="cuda") -> torch.nn.Module`
  - `model.forward_batch(net, batch: dict) -> tuple[np.ndarray, np.ndarray]` returning `pred_scores [A,6]`, `pred_trajs [A,6,80,7]`
  - `rung0.run(shard, checkpoint, mtr_root, limit=None) -> Rung0Report` with fields `num_scenarios, num_agents, min_ade_native, min_fde_native, min_ade_deployed, min_fde_deployed, per_type: dict[str, tuple[float, float]]`

**Reference figures** (official MTR README, 100% training set): minADE **0.6046**, minFDE **1.2251**, Miss Rate 0.1366, mAP 0.4164. The checkpoint's own `best_eval_record.txt` reports epoch_30 mAP **0.4182**.

- [ ] **Step 1: Set up the MTR environment and fetch the checkpoint**

```bash
# Reference clone pinned to the commit DEVELOPING.md:29 validated against.
git clone https://github.com/sshaoshuai/MTR.git ~/mtr && cd ~/mtr
git checkout a5ba7bd
pip install -r requirements.txt
python setup.py develop      # builds knn_cuda and attention_cuda

# The checkpoint. 777 MB.
pip install huggingface_hub
python - <<'PY'
from huggingface_hub import hf_hub_download
p = hf_hub_download(
    repo_id="Student-Xiaoji/LLM-Augmented-MTR-models",
    filename="100_percent/MTR/checkpoint_epoch_30.pth",
)
print(p)
PY
```

Record the resolved path; later steps refer to it as `$CKPT`.

- [ ] **Step 2: Write the failing checkpoint test**

```python
import os
import pathlib

import pytest

torch = pytest.importorskip("torch")

from wato_mtr_eval import model

CKPT = os.environ.get("MTR_CHECKPOINT")
MTR_ROOT = os.environ.get("MTR_ROOT")
needs_ckpt = pytest.mark.skipif(
    not (CKPT and MTR_ROOT and pathlib.Path(CKPT).exists()),
    reason="set MTR_CHECKPOINT and MTR_ROOT to run",
)


@needs_ckpt
def test_checkpoint_contains_model_weights_not_only_optimizer_state():
    raw = torch.load(CKPT, map_location="cpu", weights_only=False)
    assert "model_state" in raw, f"expected 'model_state', found {sorted(raw)}"
    assert len(raw["model_state"]) > 100, "suspiciously few parameter tensors"


@needs_ckpt
def test_checkpoint_reports_epoch_30():
    raw = torch.load(CKPT, map_location="cpu", weights_only=False)
    assert raw.get("epoch") == 30


@needs_ckpt
def test_checkpoint_loads_with_no_missing_or_unexpected_keys():
    """A silent key mismatch is how a wrong-config load produces garbage output."""
    net, report = model.load_checkpoint(
        pathlib.Path(CKPT),
        pathlib.Path(MTR_ROOT) / "tools/cfgs/waymo/mtr+100_percent_data.yaml",
        device="cpu",
        return_report=True,
    )
    assert report.missing_keys == [], f"missing: {report.missing_keys[:5]}"
    assert report.unexpected_keys == [], f"unexpected: {report.unexpected_keys[:5]}"


@needs_ckpt
def test_num_motion_modes_and_future_frames_match_the_contract():
    from wato_mtr_eval import representation as rep
    raw = torch.load(CKPT, map_location="cpu", weights_only=False)
    state = raw["model_state"]
    # motion query embeddings carry the mode count in their leading dimension
    mode_shapes = {v.shape[0] for k, v in state.items() if "intention_points" in k}
    assert not mode_shapes or rep.NUM_MODES in mode_shapes or 64 in mode_shapes
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
export MTR_CHECKPOINT=$CKPT
export MTR_ROOT=~/mtr
pytest tests/test_model.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.model'`.

- [ ] **Step 4: Write `model.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Checkpoint loading and forward pass against the official MTR implementation.

Imports torch; must never import rosbag2_py (see tests/test_env_split.py).
The MTR package itself is imported lazily so that merely importing this module
does not require the CUDA extensions to be built.
"""
from __future__ import annotations

import dataclasses
import pathlib
import sys

import numpy as np
import torch


@dataclasses.dataclass(frozen=True)
class LoadReport:
    missing_keys: list[str]
    unexpected_keys: list[str]
    epoch: int


def _import_mtr(mtr_root: pathlib.Path):
    if str(mtr_root) not in sys.path:
        sys.path.insert(0, str(mtr_root))
    from mtr.config import cfg, cfg_from_yaml_file  # noqa: WPS433
    from mtr.models import model as mtr_model  # noqa: WPS433
    return cfg, cfg_from_yaml_file, mtr_model


def load_checkpoint(
    checkpoint: pathlib.Path,
    cfg_path: pathlib.Path,
    device: str = "cuda",
    mtr_root: pathlib.Path | None = None,
    return_report: bool = False,
):
    """Build MTR from its official config and load the checkpoint into it.

    Missing or unexpected keys are surfaced rather than swallowed: a silent key
    mismatch is exactly how a wrong-config load yields plausible-looking garbage.
    """
    mtr_root = mtr_root or cfg_path.parents[3]
    cfg, cfg_from_yaml_file, mtr_model = _import_mtr(mtr_root)
    cfg_from_yaml_file(str(cfg_path), cfg)

    net = mtr_model.MotionTransformer(config=cfg.MODEL)
    raw = torch.load(checkpoint, map_location="cpu", weights_only=False)
    result = net.load_state_dict(raw["model_state"], strict=False)
    net.to(device).eval()

    report = LoadReport(
        missing_keys=list(result.missing_keys),
        unexpected_keys=list(result.unexpected_keys),
        epoch=int(raw.get("epoch", -1)),
    )
    return (net, report) if return_report else net


@torch.no_grad()
def forward_batch(net, batch: dict) -> tuple[np.ndarray, np.ndarray]:
    """Run MTR. Returns (pred_scores [A,6], pred_trajs [A,6,80,7])."""
    out = net(batch)
    pred = out["pred_list"][-1] if "pred_list" in out else out
    scores = pred["pred_scores"] if isinstance(pred, dict) else pred[0]
    trajs = pred["pred_trajs"] if isinstance(pred, dict) else pred[1]
    return scores.float().cpu().numpy(), trajs.float().cpu().numpy()
```

- [ ] **Step 5: Run the checkpoint tests to verify they pass**

```bash
pytest tests/test_model.py -v
```

Expected: 4 passed.

**If `test_checkpoint_loads_with_no_missing_or_unexpected_keys` fails, stop.** Either the config is the wrong variant (try `mtr+20_percent_data.yaml`) or the checkpoint is not stock MTR. Do not proceed past this task with a mismatched load — every downstream number would be meaningless.

- [ ] **Step 6: Write `rung0.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Rung 0 — the golden anchor.

Runs the official MTR preprocessing and forward pass, then scores against the
published figures. This validates the evaluation environment AND the
HuggingFace checkpoint before either is used to judge anything else.

Anchoring uses minADE/minFDE only. Reproducing the published Miss Rate would
require Waymo's speed-scaled lateral/longitudinal thresholds, and mAP needs the
metrics_ops.so environment that DEVELOPING.md:47 records as ABI-broken.
"""
from __future__ import annotations

import dataclasses
import pathlib

import numpy as np

from . import metrics, model, scenario

PUBLISHED_MIN_ADE = 0.6046
PUBLISHED_MIN_FDE = 1.2251
TOLERANCE = 0.03  # 3%: we replicate the protocol closely, not bit-exactly


@dataclasses.dataclass(frozen=True)
class Rung0Report:
    num_scenarios: int
    num_agents: int
    min_ade_native: float
    min_fde_native: float
    min_ade_deployed: float
    min_fde_deployed: float
    per_type: dict[str, tuple[float, float]]

    def anchor_ok(self) -> bool:
        return (
            abs(self.min_ade_native - PUBLISHED_MIN_ADE) / PUBLISHED_MIN_ADE < TOLERANCE
            and abs(self.min_fde_native - PUBLISHED_MIN_FDE) / PUBLISHED_MIN_FDE < TOLERANCE
        )

    def format(self) -> str:
        status = "PASS" if self.anchor_ok() else "FAIL"
        lines = [
            f"Rung 0 (golden) — {self.num_scenarios} scenarios, {self.num_agents} target agents",
            "",
            f"  native 8 s    minADE {self.min_ade_native:.4f}  "
            f"(published {PUBLISHED_MIN_ADE:.4f})",
            f"                minFDE {self.min_fde_native:.4f}  "
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


def run(
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    cfg_name: str = "mtr+100_percent_data.yaml",
    limit: int | None = None,
    device: str = "cuda",
) -> Rung0Report:
    """Score the checkpoint through the official MTR data pipeline."""
    from mtr.datasets.waymo.waymo_dataset import WaymoDataset  # lazy: needs MTR on sys.path

    cfg_path = mtr_root / "tools/cfgs/waymo" / cfg_name
    net = model.load_checkpoint(checkpoint, cfg_path, device=device, mtr_root=mtr_root)

    native_ade, native_fde, dep_ade, dep_fde = [], [], [], []
    per_type: dict[str, list[tuple[float, float]]] = {}
    num_scenarios = 0

    for scen in scenario.load_scenarios(shard, limit=limit):
        num_scenarios += 1
        batch, gt, valid, types = WaymoDataset.build_eval_batch(scen)  # see step 7
        _, trajs = model.forward_batch(net, batch)

        native_ade.append(metrics.min_ade(trajs, gt, valid))
        native_fde.append(metrics.min_fde(trajs, gt, valid))

        dep_pred = metrics.to_deployed(trajs)
        dep_steps = dep_pred.shape[2]
        dep_gt, dep_valid = gt[:, 1 : 1 + 2 * dep_steps : 2], valid[:, 1 : 1 + 2 * dep_steps : 2]
        a = metrics.min_ade(dep_pred, dep_gt, dep_valid)
        f = metrics.min_fde(dep_pred, dep_gt, dep_valid)
        dep_ade.append(a)
        dep_fde.append(f)
        for i, name in enumerate(types):
            per_type.setdefault(name, []).append((a[i], f[i]))

    cat = lambda xs: np.concatenate(xs) if xs else np.array([])  # noqa: E731
    return Rung0Report(
        num_scenarios=num_scenarios,
        num_agents=int(np.isfinite(cat(native_ade)).sum()),
        min_ade_native=float(np.nanmean(cat(native_ade))),
        min_fde_native=float(np.nanmean(cat(native_fde))),
        min_ade_deployed=float(np.nanmean(cat(dep_ade))),
        min_fde_deployed=float(np.nanmean(cat(dep_fde))),
        per_type={
            k: (float(np.nanmean([x[0] for x in v])), float(np.nanmean([x[1] for x in v])))
            for k, v in per_type.items()
        },
    )
```

- [ ] **Step 7: Wire `build_eval_batch` to the official dataset**

`WaymoDataset` in the official repo consumes preprocessed `.pkl` files, not `Scenario` objects. Add a thin adapter rather than reimplementing preprocessing — reimplementation would defeat the purpose of rung 0, which is to be the *official* path.

Run the official preprocessing once:

```bash
cd ~/mtr
python mtr/datasets/waymo/data_preprocess.py \
  /path/to/shard_dir /path/to/output_dir
```

Then have `rung0.run` iterate the official `WaymoDataset` directly, using `scenario.load_scenarios` only to recover ground-truth futures and object types keyed by `scenario_id`. Replace the `build_eval_batch` call with:

```python
        dataset = WaymoDataset(dataset_cfg=cfg.DATA_CONFIG, training=False, logger=None)
        loader = torch.utils.data.DataLoader(
            dataset, batch_size=1, shuffle=False, collate_fn=dataset.collate_batch
        )
        for batch in loader:
            ...
```

and look up `gt`, `valid`, `types` from a `{scenario_id: Scenario}` map built from `load_scenarios`.

- [ ] **Step 8: Run rung 0 on the full shard**

```bash
cd src/world_modeling/prediction_ml/eval
python -m wato_mtr_eval.rung0 \
  --shard ../../../../uncompressed_scenario_validation_validation.tfrecord-00000-of-00150 \
  --checkpoint "$MTR_CHECKPOINT" \
  --mtr-root ~/mtr
```

Expected: 286 scenarios, ~1268 target agents, native minADE within 3% of 0.6046 and minFDE within 3% of 1.2251, and `GATE A: PASS`.

- [ ] **Step 9: Record the result and open the gate**

Write `eval/README.md` documenting the environment setup from steps 1–2, the exact command from step 8, and paste the rung-0 report verbatim under a "Gate A" heading with the date and the checkpoint's SHA256.

**If Gate A fails:** do not continue to Task 7. Diagnose in this order — (a) wrong config variant, (b) preprocessing not run on the same shard, (c) metric averaging protocol differs from Waymo's, (d) the checkpoint is not stock MTR. Only (c) is a harness bug; (a), (b) and (d) are environment problems that invalidate every later rung.

- [ ] **Step 10: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/model.py \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung0.py \
        src/world_modeling/prediction_ml/eval/tests/test_model.py \
        src/world_modeling/prediction_ml/eval/README.md
git commit -m "feat(prediction_ml): add rung 0 golden anchor validating the MTR checkpoint"
```

---

*(Tasks 7–17 continue below.)*
