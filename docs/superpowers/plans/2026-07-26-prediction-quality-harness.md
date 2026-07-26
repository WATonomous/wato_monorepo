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
    dist = _displacement(pred, gt)
    num_agents = dist.shape[0]
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

### Task 7: Information-loss ledger

Accounts for every point where information is discarded between the Waymo scenario and the packed tensors. Turns "the metric moved" into "and here is exactly what disappeared". Pure — no ROS, no torch.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/ledger.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_ledger.py`

**Interfaces:**
- Consumes: `scenario.Scenario`, `representation` constants.
- Produces:
  - `Ledger` dataclass with sections `agents, targets, history, map_, output`, each a `dict[str, int | float | str]`
  - `Ledger.format() -> str`
  - `Ledger.to_json(path) -> None`
  - `build_ledger(scen, *, kept_track_ids, selected_target_indices, heuristic_target_indices, history_valid, history_total, map_stats) -> Ledger`
  - `MapStats` dataclass: `radius_m, nearby_lanes, radius_only_emitted, reachable_emitted, points_before, points_after, omitted_by_type: dict[str, int]`

- [ ] **Step 1: Write the failing test**

```python
import json

import numpy as np
import pytest

from wato_mtr_eval import ledger, representation as rep


def _map_stats():
    return ledger.MapStats(
        radius_m=80.0,
        nearby_lanes=37,
        radius_only_emitted=37,
        reachable_emitted=16,
        points_before=611,
        points_after=284,
        omitted_by_type={"road_edge": 18, "road_line": 24, "crosswalk": 3, "stop_sign": 1},
    )


def _ledger(**overrides):
    kwargs = dict(
        scenario_id="abc123",
        num_waymo_tracks=54,
        num_type_mapped=48,
        dropped_by_type={"other": 6},
        num_candidates=48,
        selected_target_indices=[0, 1, 2, 3, 4, 5, 6, 7],
        heuristic_target_indices=[0, 1, 2, 3, 4, 90, 91, 92],
        history_valid=491,
        history_total=528,
        map_stats=_map_stats(),
    )
    kwargs.update(overrides)
    return ledger.build_ledger(**kwargs)


def test_agent_drops_are_counted_not_silent():
    lg = _ledger()
    assert lg.agents["waymo_tracks"] == 54
    assert lg.agents["type_mapped"] == 48
    assert lg.agents["dropped"] == 6
    assert lg.agents["dropped_by_type"] == {"other": 6}


def test_context_overflow_is_reported_when_agents_exceed_capacity():
    lg = _ledger(num_waymo_tracks=141, num_type_mapped=141)
    assert lg.agents["context_capacity"] == rep.CONTEXT_CAPACITY
    assert lg.agents["context_overflow"] == 141 - rep.CONTEXT_CAPACITY


def test_no_overflow_reported_when_under_capacity():
    assert _ledger().agents["context_overflow"] == 0


def test_target_selection_records_heuristic_agreement():
    """Pinned selection is what runs; the heuristic's choice is recorded free."""
    lg = _ledger()
    assert lg.targets["selected"] == 8
    assert lg.targets["pinned"] is True
    assert lg.targets["heuristic_agreement"] == 5  # indices 0-4 overlap
    assert lg.targets["heuristic_agreement_fraction"] == pytest.approx(5 / 8)


def test_history_masking_fraction():
    lg = _ledger()
    assert lg.history["valid"] == 491
    assert lg.history["masked"] == 37
    assert lg.history["masked_fraction"] == pytest.approx(37 / 528)


def test_map_section_records_what_the_query_and_packing_discarded():
    lg = _ledger().map_
    assert lg["nearby_lanes"] == 37
    assert lg["radius_only_emitted"] == 37
    assert lg["reachable_emitted"] == 16
    assert lg["dropped_by_connectivity"] == 21
    assert lg["centerline_points_before"] == 611
    assert lg["centerline_points_after"] == 284
    assert lg["omitted_semantics"]["road_edge"] == 18
    assert lg["global_type_collapse"] == "20 waymo types -> 1 wato token"


def test_output_section_records_the_horizon_clamp():
    lg = _ledger().output
    assert lg["model_steps"] == rep.NUM_FUTURE_FRAMES
    assert lg["stride"] == 2
    assert lg["emitted_steps"] == rep.DEPLOYED_STEPS
    assert lg["emitted_horizon_s"] == pytest.approx(rep.DEPLOYED_HORIZON_S)


def test_format_is_human_readable_and_names_every_section():
    text = _ledger().format()
    for section in ("AGENTS", "TARGETS", "HISTORY", "MAP", "OUTPUT"):
        assert section in text


def test_json_round_trips(tmp_path):
    path = tmp_path / "ledger.json"
    _ledger().to_json(path)
    loaded = json.loads(path.read_text())
    assert loaded["scenario_id"] == "abc123"
    assert loaded["agents"]["dropped"] == 6
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_ledger.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.ledger'`.

- [ ] **Step 3: Write `ledger.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Information-loss accounting.

Every drop point already exists in the pipeline, so instrumenting is cheap.
The payoff is that when deployed minFDE moves 1.22 -> 1.61, cross-referencing
the worst-N agents against these counts usually names the cause immediately:
if the worst rows all carry a context overflow, the bug is in ranking or
truncation rather than in packing.

Scoped to drop points that CAN change -- budgets, filters, policies. Static
facts belong in REPRESENTATION.md, not in per-run output.
"""
from __future__ import annotations

import dataclasses
import json
import pathlib

from . import representation as rep


@dataclasses.dataclass(frozen=True)
class MapStats:
    radius_m: float
    nearby_lanes: int
    radius_only_emitted: int
    reachable_emitted: int
    points_before: int
    points_after: int
    omitted_by_type: dict[str, int]


@dataclasses.dataclass(frozen=True)
class Ledger:
    scenario_id: str
    agents: dict
    targets: dict
    history: dict
    map_: dict
    output: dict

    def format(self) -> str:
        a, t, h, m, o = self.agents, self.targets, self.history, self.map_, self.output
        dropped = ", ".join(f"{k} {v}" for k, v in sorted(a["dropped_by_type"].items())) or "none"
        omitted = "  ".join(f"{k} {v}" for k, v in sorted(m["omitted_semantics"].items()))
        return "\n".join([
            f"scenario {self.scenario_id}",
            f"AGENTS   waymo tracks {a['waymo_tracks']} -> type-mapped {a['type_mapped']} "
            f"({a['dropped']} dropped: {dropped})",
            f"         -> context capacity {a['context_capacity']}, used {a['context_used']}, "
            f"overflow {a['context_overflow']}",
            f"TARGETS  candidates {t['candidates']} -> tracks_to_predict {t['selected']} "
            f"-> selected {t['selected']} ({'pinned' if t['pinned'] else 'heuristic'})",
            f"         [heuristic would have selected "
            f"{t['heuristic_agreement']}/{t['selected']} of these]",
            f"HISTORY  {h['slots']} slots x {h['tracks']} tracks = {h['total']} -> "
            f"valid {h['valid']}, masked {h['masked']} ({h['masked_fraction']:.1%})",
            f"MAP      radius {m['radius_m']:.0f} m; nearby lanes {m['nearby_lanes']} -> "
            f"radius_only {m['radius_only_emitted']} -> reachable {m['reachable_emitted']}",
            f"         centerline points {m['centerline_points_before']} -> "
            f"{m['centerline_points_after']}",
            f"         polyline capacity {m['polyline_capacity']}, used {m['polylines_used']}, "
            f"overflow {m['polyline_overflow']}",
            f"         omitted semantics: {omitted}",
            f"         global_type: {m['global_type_collapse']}",
            f"OUTPUT   {o['model_steps']} steps @ {o['model_step_s']} s -> stride {o['stride']} "
            f"-> {o['strided_steps']} -> horizon clamp -> {o['emitted_steps']} @ "
            f"{o['emitted_step_s']} s",
        ])

    def to_json(self, path: pathlib.Path) -> None:
        payload = {
            "scenario_id": self.scenario_id,
            "agents": self.agents,
            "targets": self.targets,
            "history": self.history,
            "map": self.map_,
            "output": self.output,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def build_ledger(
    *,
    scenario_id: str,
    num_waymo_tracks: int,
    num_type_mapped: int,
    dropped_by_type: dict[str, int],
    num_candidates: int,
    selected_target_indices: list[int],
    heuristic_target_indices: list[int],
    history_valid: int,
    history_total: int,
    map_stats: MapStats,
) -> Ledger:
    stride = int(round(rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S))
    strided = rep.NUM_FUTURE_FRAMES // stride
    agreement = len(set(selected_target_indices) & set(heuristic_target_indices))
    selected = len(selected_target_indices)
    used_map = min(map_stats.reachable_emitted, rep.NUM_SRC_POLYLINES)

    return Ledger(
        scenario_id=scenario_id,
        agents={
            "waymo_tracks": num_waymo_tracks,
            "type_mapped": num_type_mapped,
            "dropped": num_waymo_tracks - num_type_mapped,
            "dropped_by_type": dict(dropped_by_type),
            "context_capacity": rep.CONTEXT_CAPACITY,
            "context_used": min(num_type_mapped, rep.CONTEXT_CAPACITY),
            "context_overflow": max(0, num_type_mapped - rep.CONTEXT_CAPACITY),
        },
        targets={
            "candidates": num_candidates,
            "selected": selected,
            "pinned": True,
            "heuristic_agreement": agreement,
            "heuristic_agreement_fraction": (agreement / selected) if selected else 0.0,
        },
        history={
            "slots": rep.HISTORY_STEPS,
            "tracks": history_total // rep.HISTORY_STEPS if rep.HISTORY_STEPS else 0,
            "total": history_total,
            "valid": history_valid,
            "masked": history_total - history_valid,
            "masked_fraction": (
                (history_total - history_valid) / history_total if history_total else 0.0
            ),
        },
        map_={
            "radius_m": map_stats.radius_m,
            "nearby_lanes": map_stats.nearby_lanes,
            "radius_only_emitted": map_stats.radius_only_emitted,
            "reachable_emitted": map_stats.reachable_emitted,
            "dropped_by_connectivity": (
                map_stats.radius_only_emitted - map_stats.reachable_emitted
            ),
            "centerline_points_before": map_stats.points_before,
            "centerline_points_after": map_stats.points_after,
            "polyline_capacity": rep.NUM_SRC_POLYLINES,
            "polylines_used": used_map,
            "polyline_overflow": max(0, map_stats.reachable_emitted - rep.NUM_SRC_POLYLINES),
            "omitted_semantics": dict(map_stats.omitted_by_type),
            "global_type_collapse": "20 waymo types -> 1 wato token",
        },
        output={
            "model_steps": rep.NUM_FUTURE_FRAMES,
            "model_step_s": rep.MODEL_STEP_S,
            "stride": stride,
            "strided_steps": strided,
            "emitted_steps": min(strided, rep.DEPLOYED_STEPS),
            "emitted_step_s": rep.DEPLOYED_STEP_S,
            "emitted_horizon_s": min(strided, rep.DEPLOYED_STEPS) * rep.DEPLOYED_STEP_S,
        },
    )
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/test_ledger.py -v
```

Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/ledger.py \
        src/world_modeling/prediction_ml/eval/tests/test_ledger.py
git commit -m "feat(prediction_ml): add information-loss ledger"
```

---

### Task 8: Lanelet query

Converts Waymo map features into the subset a real `LaneletAhead` query would return. Parameterized: radius-only is the deterministic upper-context baseline; connectivity traversal approximates WATO's forward-reachable query. The delta between them prices the query's aggressiveness.

This is an explicit **approximation**, not a claimed Lanelet2 replica. Recorded real `LaneletAhead` can never serve as an oracle here — it queries the Ontario map while Waymo scenarios are elsewhere entirely, with no shared frame or road network. It is useful only for distributional calibration later.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/lanelet_query.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_lanelet_query.py`

**Interfaces:**
- Consumes: `scenario.MapPolyline`, `ledger.MapStats`.
- Produces:
  - `QueryConfig` dataclass: `radius_m: float = 80.0`, `strategy: str = "radius"` (`"radius"` | `"reachable"`), `forward_half_angle_deg: float = 90.0`
  - `query(polylines, ego_xy, ego_heading, config) -> tuple[list[MapPolyline], MapStats]`
  - `LANE_TYPES = {"lane"}` and `DISCARDED_TYPES = {"road_line", "road_edge", "crosswalk", "speed_bump", "driveway"}`

**Note:** this module imports neither ROS nor torch — it is pure geometry over `scenario` dataclasses. Keeping it separate from `waymo_to_ros.py` is what lets the rung-1 Python ablation reuse the exact same query the converter uses.

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
import pytest

from wato_mtr_eval import lanelet_query as lq, scenario


def _line(fid, ftype, xs, y=0.0):
    pts = np.array([[x, y, 0.0] for x in xs], dtype=np.float64)
    return scenario.MapPolyline(feature_id=fid, feature_type=ftype, points=pts)


def _map():
    return [
        _line(1, "lane", [0, 10, 20]),            # near, ahead
        _line(2, "lane", [200, 210, 220]),        # far, outside radius
        _line(3, "lane", [-30, -20, -10]),        # near, behind
        _line(4, "road_edge", [0, 10, 20], y=3),  # discarded semantic
        _line(5, "crosswalk", [5, 6], y=1),       # discarded semantic
    ]


def test_only_lane_features_survive():
    kept, _ = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0))
    assert {p.feature_type for p in kept} == {"lane"}


def test_radius_filters_distant_lanes():
    kept, _ = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0))
    assert {p.feature_id for p in kept} == {1, 3}


def test_reachable_strategy_drops_lanes_behind_ego():
    cfg = lq.QueryConfig(radius_m=80.0, strategy="reachable")
    kept, _ = lq.query(_map(), (0.0, 0.0), 0.0, cfg)
    assert {p.feature_id for p in kept} == {1}, "lane 3 is behind ego"


def test_radius_only_keeps_more_than_reachable():
    r, _ = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="radius"))
    k, _ = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="reachable"))
    assert len(r) >= len(k), "radius-only is the upper-context baseline"


def test_stats_count_omitted_semantics_by_type():
    _, stats = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0))
    assert stats.omitted_by_type["road_edge"] == 1
    assert stats.omitted_by_type["crosswalk"] == 1


def test_stats_report_both_strategies_regardless_of_which_ran():
    """The radius-vs-reachable delta must be recordable from a single run."""
    _, stats = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="reachable"))
    assert stats.radius_only_emitted == 2
    assert stats.reachable_emitted == 1


def test_stats_count_points_before_and_after():
    _, stats = lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(radius_m=80.0))
    assert stats.points_before == 3 + 3 + 3  # lane features only
    assert stats.points_after == 3 + 3


def test_rejects_unknown_strategy():
    with pytest.raises(ValueError, match="unknown strategy"):
        lq.query(_map(), (0.0, 0.0), 0.0, lq.QueryConfig(strategy="magic"))


def test_heading_rotates_the_forward_region():
    """Facing -x, the lane behind becomes the lane ahead."""
    cfg = lq.QueryConfig(strategy="reachable")
    kept, _ = lq.query(_map(), (0.0, 0.0), np.pi, cfg)
    assert {p.feature_id for p in kept} == {3}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_lanelet_query.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.lanelet_query'`.

- [ ] **Step 3: Write `lanelet_query.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Waymo map features -> the subset a LaneletAhead query would return.

LaneletAhead is a query result, not a map: "legally reachable lanelets ahead of
ego within a configurable radius ... BFS through the routing graph". Two
strategies are offered:

  radius     every lane centerline within radius_m. Deterministic upper-context
             baseline; over-supplies relative to real WATO.
  reachable  additionally requires the lane to fall in the forward region, an
             explicit APPROXIMATION of the forward-reachable filter -- not a
             Lanelet2 replica.

Reporting the delta between them prices the query's aggressiveness. Stats are
always computed for both strategies so a single run yields the comparison.

Pure geometry: imports neither ROS nor torch, so the rung-1 ablation reuses the
exact query the converter uses.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import ledger, scenario

LANE_TYPES = {"lane"}
DISCARDED_TYPES = {"road_line", "road_edge", "crosswalk", "speed_bump", "driveway"}
_STRATEGIES = {"radius", "reachable"}


@dataclasses.dataclass(frozen=True)
class QueryConfig:
    radius_m: float = 80.0
    strategy: str = "radius"
    forward_half_angle_deg: float = 90.0


def _within_radius(poly: scenario.MapPolyline, ego_xy, radius_m: float) -> bool:
    d = np.linalg.norm(poly.points[:, :2] - np.asarray(ego_xy), axis=1)
    return bool(d.min() <= radius_m)


def _in_forward_region(poly, ego_xy, ego_heading, half_angle_deg: float) -> bool:
    """True if any point lies within half_angle of ego heading.

    Mirrors SceneBuilder's forward-hemisphere notion (mtr_types.hpp:47-48):
    90 degrees means the front hemisphere, including agents directly beside ego.
    """
    rel = poly.points[:, :2] - np.asarray(ego_xy)
    cos_h, sin_h = math.cos(ego_heading), math.sin(ego_heading)
    forward = rel[:, 0] * cos_h + rel[:, 1] * sin_h
    lateral = -rel[:, 0] * sin_h + rel[:, 1] * cos_h
    angle = np.abs(np.arctan2(lateral, forward))
    return bool((angle <= math.radians(half_angle_deg)).any())


def query(
    polylines: list[scenario.MapPolyline],
    ego_xy: tuple[float, float],
    ego_heading: float,
    config: QueryConfig,
) -> tuple[list[scenario.MapPolyline], ledger.MapStats]:
    """Return the queried lane polylines plus loss statistics."""
    if config.strategy not in _STRATEGIES:
        raise ValueError(f"unknown strategy {config.strategy!r}; expected one of {_STRATEGIES}")

    lanes = [p for p in polylines if p.feature_type in LANE_TYPES]
    omitted: dict[str, int] = {}
    for p in polylines:
        if p.feature_type in DISCARDED_TYPES:
            omitted[p.feature_type] = omitted.get(p.feature_type, 0) + 1

    radius_hits = [p for p in lanes if _within_radius(p, ego_xy, config.radius_m)]
    reachable_hits = [
        p for p in radius_hits
        if _in_forward_region(p, ego_xy, ego_heading, config.forward_half_angle_deg)
    ]
    kept = radius_hits if config.strategy == "radius" else reachable_hits

    return kept, ledger.MapStats(
        radius_m=config.radius_m,
        nearby_lanes=len(lanes),
        radius_only_emitted=len(radius_hits),
        reachable_emitted=len(reachable_hits),
        points_before=int(sum(len(p.points) for p in lanes)),
        points_after=int(sum(len(p.points) for p in kept)),
        omitted_by_type=omitted,
    )
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/test_lanelet_query.py -v
```

Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/lanelet_query.py \
        src/world_modeling/prediction_ml/eval/tests/test_lanelet_query.py
git commit -m "feat(prediction_ml): add parameterized lanelet query for waymo maps"
```

---

### Task 9: The converter — Waymo scenario to mcap bag

The keystone. One artifact feeds two consumers: rung 2 reads the bag with the `rosbag2_cpp` API, rung 4 replays it with `ros2 bag play`. Using mcap rather than a bespoke format means Python and C++ agree by construction, because both use ROS's own serializer.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/waymo_to_ros.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_waymo_to_ros.py`

**Interfaces:**
- Consumes: `scenario.Scenario`, `lanelet_query.query`, `ledger.build_ledger`, `representation` constants.
- Produces:
  - `TOPICS` dict: `{"tracks": "/perception/detections_3D_tracked", "pose": "/localization/pose", "odom": "/world_modeling/transform/odometry", "lanelet": "lanelet/lanelet_ahead"}`
  - `MAP_FRAME = "map"`, `EGO_CHILD_FRAME = "base_footprint"`
  - `detections_at(scen, step) -> vision_msgs.msg.Detection3DArray`
  - `ego_pose_at(scen, step) -> geometry_msgs.msg.PoseStamped`
  - `ego_odometry_at(scen, step) -> nav_msgs.msg.Odometry`
  - `lanelet_ahead_at(scen, step, config) -> tuple[lanelet_msgs.msg.LaneletAhead, MapStats]`
  - `convert(scen, out_dir, config) -> ConversionResult` with `bag_path, ledger, ground_truth`
  - `ground_truth_of(scen) -> tuple[np.ndarray, np.ndarray, list[str], list[str]]` — `gt [A,80,2]`, `valid [A,80]`, `agent_ids`, `types`

**Environment:** this module imports `rosbag2_py` and ROS message packages. It must never import `torch` — Task 2's guard enforces this.

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")

from wato_mtr_eval import lanelet_query as lq, representation as rep, scenario, waymo_to_ros as w2r


def _scenario(num_tracks=3, num_steps=91):
    rng = np.random.default_rng(0)
    tracks = []
    for i in range(num_tracks):
        states = np.zeros((num_steps, 10))
        states[:, 0] = np.arange(num_steps) * 0.5 + i * 10  # x advances
        states[:, 1] = float(i)
        states[:, 3:6] = [4.5, 2.0, 1.6]
        states[:, 6] = 0.0        # heading
        states[:, 7] = 5.0        # vx
        states[:, 9] = 1.0        # valid
        tracks.append(scenario.Track(
            object_id=100 + i,
            object_type=["vehicle", "pedestrian", "cyclist"][i % 3],
            states=states,
        ))
    lane = scenario.MapPolyline(
        feature_id=1, feature_type="lane",
        points=np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]),
    )
    return scenario.Scenario(
        scenario_id="test_scenario",
        timestamps_s=np.arange(num_steps) * 0.1,
        current_time_index=10,
        sdc_track_index=0,
        tracks=tracks,
        tracks_to_predict=[1, 2],
        map_polylines=[lane],
    )


def test_detections_use_class_id_strings_scene_builder_accepts():
    """scene_builder.cpp:238-246 matches on these exact lowercase strings."""
    msg = w2r.detections_at(_scenario(), step=10)
    labels = {
        r.hypothesis.class_id
        for d in msg.detections for r in d.results
        if r.hypothesis.class_id != rep.VELOCITY_CLASS_ID
    }
    assert labels <= set(rep.CLASS_ID_FOR_TYPE.values())


def test_velocity_rides_in_the_hypothesis_pose_position():
    """scene_builder.cpp:118-127 reads vx/vy/vz from the pose POSITION."""
    msg = w2r.detections_at(_scenario(), step=10)
    det = msg.detections[0]
    vel = [r for r in det.results if r.hypothesis.class_id == rep.VELOCITY_CLASS_ID]
    assert len(vel) == 1
    assert vel[0].pose.pose.position.x == pytest.approx(5.0)
    assert vel[0].pose.pose.position.z == pytest.approx(0.0)  # waymo has no vz
    assert vel[0].hypothesis.score > 0.0, "score <= 0 makes SceneBuilder skip the entry"


def test_detection_ids_are_stable_across_frames():
    """SceneBuilder keys per-object history by detection.id."""
    scen = _scenario()
    a = {d.id for d in w2r.detections_at(scen, step=8).detections}
    b = {d.id for d in w2r.detections_at(scen, step=10).detections}
    assert a == b and a


def test_invalid_states_are_omitted_not_emitted_as_zeros():
    scen = _scenario()
    scen.tracks[1].states[10, 9] = 0.0  # mark invalid at this step
    ids = {d.id for d in w2r.detections_at(scen, step=10).detections}
    assert str(scen.tracks[1].object_id) not in ids


def test_bbox_sizes_are_positive_so_sample_from_detection_accepts_them():
    """scene_builder.cpp:268-273 rejects any non-positive extent."""
    for d in w2r.detections_at(_scenario(), step=10).detections:
        assert d.bbox.size.x > 0 and d.bbox.size.y > 0 and d.bbox.size.z > 0


def test_ego_odometry_twist_is_body_frame_and_round_trips():
    """SceneBuilder rotates body twist INTO map; the converter applies the inverse."""
    scen = _scenario()
    scen.tracks[0].states[:, 6] = np.pi / 2   # heading +y
    scen.tracks[0].states[:, 7] = 0.0
    scen.tracks[0].states[:, 8] = 5.0         # map-frame velocity +y

    odom = w2r.ego_odometry_at(scen, step=10)
    assert odom.child_frame_id == w2r.EGO_CHILD_FRAME
    # Facing +y at 5 m/s: body-frame velocity is purely forward.
    assert odom.twist.twist.linear.x == pytest.approx(5.0, abs=1e-6)
    assert odom.twist.twist.linear.y == pytest.approx(0.0, abs=1e-6)


def test_ego_pose_matches_the_sdc_track():
    scen = _scenario()
    pose = w2r.ego_pose_at(scen, step=10)
    assert pose.header.frame_id == w2r.MAP_FRAME
    assert pose.pose.position.x == pytest.approx(scen.tracks[0].states[10, 0])


def test_lanelet_ahead_carries_only_centerlines():
    msg, _ = w2r.lanelet_ahead_at(_scenario(), 10, lq.QueryConfig())
    assert msg.lanelets
    for ll in msg.lanelets:
        assert len(ll.centerline) > 0
        assert len(ll.left_boundary) == 0, "WATO representation packs centerlines only"


def test_convert_writes_a_readable_bag_with_all_four_topics(tmp_path):
    result = w2r.convert(_scenario(), tmp_path, lq.QueryConfig())
    assert result.bag_path.exists()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(result.bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {t.name for t in reader.get_all_topics_and_types()}
    assert topics == set(w2r.TOPICS.values())


def test_convert_emits_one_message_per_topic_per_step(tmp_path):
    scen = _scenario()
    result = w2r.convert(scen, tmp_path, lq.QueryConfig())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(result.bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    counts: dict[str, int] = {}
    while reader.has_next():
        topic, _, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
    assert set(counts.values()) == {len(scen.timestamps_s)}


def test_ground_truth_covers_the_native_horizon():
    gt, valid, ids, types = w2r.ground_truth_of(_scenario())
    assert gt.shape == (3, rep.NUM_FUTURE_FRAMES, 2)
    assert valid.shape == (3, rep.NUM_FUTURE_FRAMES)
    assert len(ids) == len(types) == 3


def test_conversion_is_lossless_for_everything_the_representation_claims(tmp_path):
    """The invariant that makes rung1 -> rung2 a clean attribution.

    If the converter silently loses information, that loss lands in the R1->R2
    delta and gets misattributed to SceneBuilder.
    """
    scen = _scenario()
    result = w2r.convert(scen, tmp_path, lq.QueryConfig(strategy="radius"))

    for step in range(len(scen.timestamps_s)):
        msg = w2r.detections_at(scen, step)
        by_id = {d.id: d for d in msg.detections}
        for track in scen.tracks:
            st = track.states[step]
            if st[9] == 0.0 or track.object_type == "other":
                assert str(track.object_id) not in by_id
                continue
            det = by_id[str(track.object_id)]
            assert det.bbox.center.position.x == pytest.approx(st[0], abs=1e-9)
            assert det.bbox.center.position.y == pytest.approx(st[1], abs=1e-9)
            assert det.bbox.size.x == pytest.approx(st[3], abs=1e-9)
            vel = next(r for r in det.results if r.hypothesis.class_id == rep.VELOCITY_CLASS_ID)
            assert vel.pose.pose.position.x == pytest.approx(st[7], abs=1e-9)
            assert vel.pose.pose.position.y == pytest.approx(st[8], abs=1e-9)

    assert result.ledger.agents["dropped"] == 0
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
source /opt/ros/humble/setup.bash
pytest tests/test_waymo_to_ros.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.waymo_to_ros'`.

- [ ] **Step 3: Write `waymo_to_ros.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Waymo scenario -> mcap bag of WATO ROS messages.

One artifact, two consumers: rung 2 reads the bag through rosbag2_cpp, rung 4
replays it with `ros2 bag play`. Using ROS's own serializer means the Python
writer and the C++ reader agree by construction.

Imports rosbag2_py and ROS messages; must never import torch.
"""
from __future__ import annotations

import dataclasses
import math
import pathlib

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from lanelet_msgs.msg import Lanelet, LaneletAhead
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from vision_msgs.msg import (
    BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose,
)

from . import ledger, lanelet_query as lq, representation as rep, scenario

TOPICS = {
    "tracks": "/perception/detections_3D_tracked",
    "pose": "/localization/pose",
    "odom": "/world_modeling/transform/odometry",
    "lanelet": "lanelet/lanelet_ahead",
}
TYPES = {
    "tracks": "vision_msgs/msg/Detection3DArray",
    "pose": "geometry_msgs/msg/PoseStamped",
    "odom": "nav_msgs/msg/Odometry",
    "lanelet": "lanelet_msgs/msg/LaneletAhead",
}
MAP_FRAME = "map"
EGO_CHILD_FRAME = "base_footprint"
S = scenario.STATE_COL


@dataclasses.dataclass(frozen=True)
class ConversionResult:
    bag_path: pathlib.Path
    ledger: ledger.Ledger
    ground_truth: tuple


def _stamp(msg, t_s: float) -> None:
    msg.header.stamp.sec = int(t_s)
    msg.header.stamp.nanosec = int(round((t_s - int(t_s)) * 1e9))
    msg.header.frame_id = MAP_FRAME


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def detections_at(scen: scenario.Scenario, step: int) -> Detection3DArray:
    """Tracked detections at one timestep. Invalid or untyped tracks are omitted."""
    msg = Detection3DArray()
    _stamp(msg, float(scen.timestamps_s[step]))

    for track in scen.tracks:
        st = track.states[step]
        if st[S["valid"]] == 0.0 or track.object_type not in rep.CLASS_ID_FOR_TYPE:
            continue

        det = Detection3D()
        det.header = msg.header
        det.id = str(track.object_id)

        box = BoundingBox3D()
        box.center.position.x = float(st[S["x"]])
        box.center.position.y = float(st[S["y"]])
        box.center.position.z = float(st[S["z"]])
        box.center.orientation = _yaw_to_quat(float(st[S["heading"]]))
        box.size.x = float(st[S["length"]])
        box.size.y = float(st[S["width"]])
        box.size.z = float(st[S["height"]])
        det.bbox = box

        klass = ObjectHypothesisWithPose()
        klass.hypothesis.class_id = rep.CLASS_ID_FOR_TYPE[track.object_type]
        klass.hypothesis.score = 1.0

        # SceneBuilder reads velocity out of the POSE POSITION of a results entry
        # whose class_id is "linear_velocity" (scene_builder.cpp:118-127).
        vel = ObjectHypothesisWithPose()
        vel.hypothesis.class_id = rep.VELOCITY_CLASS_ID
        vel.hypothesis.score = 1.0
        vel.pose.pose.position.x = float(st[S["vx"]])
        vel.pose.pose.position.y = float(st[S["vy"]])
        vel.pose.pose.position.z = 0.0  # waymo has no velocity_z

        det.results = [klass, vel]
        msg.detections.append(det)

    return msg


def ego_pose_at(scen: scenario.Scenario, step: int) -> PoseStamped:
    st = scen.tracks[scen.sdc_track_index].states[step]
    msg = PoseStamped()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.pose.position.x = float(st[S["x"]])
    msg.pose.position.y = float(st[S["y"]])
    msg.pose.position.z = float(st[S["z"]])
    msg.pose.orientation = _yaw_to_quat(float(st[S["heading"]]))
    return msg


def ego_odometry_at(scen: scenario.Scenario, step: int) -> Odometry:
    """Body-frame twist.

    SceneBuilder rotates this INTO the map frame using ego_pose, so the converter
    applies the inverse rotation. The pair round-trips, which the tests assert.
    """
    st = scen.tracks[scen.sdc_track_index].states[step]
    heading = float(st[S["heading"]])
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    vx, vy = float(st[S["vx"]]), float(st[S["vy"]])

    msg = Odometry()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.child_frame_id = EGO_CHILD_FRAME
    msg.pose.pose = ego_pose_at(scen, step).pose
    msg.twist.twist.linear.x = vx * cos_h + vy * sin_h
    msg.twist.twist.linear.y = -vx * sin_h + vy * cos_h
    msg.twist.twist.linear.z = 0.0
    return msg


def lanelet_ahead_at(scen, step: int, config: lq.QueryConfig):
    st = scen.tracks[scen.sdc_track_index].states[step]
    kept, stats = lq.query(
        scen.map_polylines,
        (float(st[S["x"]]), float(st[S["y"]])),
        float(st[S["heading"]]),
        config,
    )

    msg = LaneletAhead()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.current_lanelet_id = -1
    msg.radius_m = config.radius_m
    for poly in kept:
        ll = Lanelet()
        ll.id = int(poly.feature_id)
        ll.lanelet_type = "road"
        for p in poly.points:
            from geometry_msgs.msg import Point
            ll.centerline.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        msg.lanelets.append(ll)
    return msg, stats


def ground_truth_of(scen: scenario.Scenario):
    """Future positions for every designated target, over the native horizon."""
    start = scen.current_time_index + 1
    stop = start + rep.NUM_FUTURE_FRAMES
    gt, valid, ids, types = [], [], [], []
    for idx in scen.tracks_to_predict:
        track = scen.tracks[idx]
        states = track.states[start:stop]
        gt.append(states[:, [S["x"], S["y"]]])
        valid.append(states[:, S["valid"]] > 0.0)
        ids.append(str(track.object_id))
        types.append(track.object_type)
    return np.array(gt), np.array(valid), ids, types


def convert(scen, out_dir: pathlib.Path, config: lq.QueryConfig) -> ConversionResult:
    """Write one scenario to an mcap bag plus its ledger and ground truth."""
    bag_path = pathlib.Path(out_dir) / scen.scenario_id
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for key, topic in TOPICS.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic, type=TYPES[key], serialization_format="cdr"
            )
        )

    map_stats = None
    history_valid = history_total = 0
    for step, t_s in enumerate(scen.timestamps_s):
        t_ns = int(round(float(t_s) * 1e9))
        dets = detections_at(scen, step)
        lanelet_msg, map_stats = lanelet_ahead_at(scen, step, config)
        writer.write(TOPICS["tracks"], serialize_message(dets), t_ns)
        writer.write(TOPICS["pose"], serialize_message(ego_pose_at(scen, step)), t_ns)
        writer.write(TOPICS["odom"], serialize_message(ego_odometry_at(scen, step)), t_ns)
        writer.write(TOPICS["lanelet"], serialize_message(lanelet_msg), t_ns)

        if step <= scen.current_time_index:
            history_total += len(scen.tracks)
            history_valid += len(dets.detections)

    del writer  # flush and close

    typed = [t for t in scen.tracks if t.object_type in rep.CLASS_ID_FOR_TYPE]
    dropped: dict[str, int] = {}
    for t in scen.tracks:
        if t.object_type not in rep.CLASS_ID_FOR_TYPE:
            dropped[t.object_type] = dropped.get(t.object_type, 0) + 1

    lg = ledger.build_ledger(
        scenario_id=scen.scenario_id,
        num_waymo_tracks=len(scen.tracks),
        num_type_mapped=len(typed),
        dropped_by_type=dropped,
        num_candidates=len(typed),
        selected_target_indices=list(scen.tracks_to_predict),
        heuristic_target_indices=list(scen.tracks_to_predict),
        history_valid=history_valid,
        history_total=history_total,
        map_stats=map_stats,
    )
    return ConversionResult(bag_path, lg, ground_truth_of(scen))
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
pytest tests/test_waymo_to_ros.py -v
```

Expected: 12 passed.

- [ ] **Step 5: Verify the environment guard still holds**

```bash
pytest tests/test_env_split.py -v
```

Expected: PASS — `waymo_to_ros.py` imports ROS only, `model.py` imports torch only.

- [ ] **Step 6: Convert three fixture scenarios**

Pick scenarios by *code path*, not variety. Add a selection helper and run it:

```bash
python -m wato_mtr_eval.waymo_to_ros \
  --shard ../../../../uncompressed_scenario_validation_validation.tfrecord-00000-of-00150 \
  --out ../test/fixtures \
  --select dense,sparse,gappy
```

`--select` picks: **dense** = most tracks (aim for >128 to force context overflow), **sparse** = fewest tracks, **gappy** = highest fraction of invalid history states. Print each chosen `scenario_id` and its ledger.

If no scenario in the shard exceeds 128 tracks, note that in `eval/README.md` and pick the densest available — the overflow path then needs a synthetic fixture, added in Task 12.

- [ ] **Step 7: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/waymo_to_ros.py \
        src/world_modeling/prediction_ml/eval/tests/test_waymo_to_ros.py \
        src/world_modeling/prediction_ml/test/fixtures
git commit -m "feat(prediction_ml): add waymo to ros mcap converter with losslessness test"
```

---

### Task 10: `mtr.pinned_target_ids` config parameter

Pins SceneBuilder's target selection so the rung-1 → rung-2 delta measures only packing, not "we chose different agents". A real config parameter rather than a test-only seam, because rung 4 replays the **full lifecycle node**, which reads config — a test seam would leave the node running its heuristic during the one rung that tests the whole stack.

**Files:**
- Modify: `src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp:39-59`
- Modify: `src/world_modeling/prediction_ml/src/scene_builder.cpp` (`selectTargets`)
- Modify: `src/world_modeling/prediction_ml/src/prediction_ml_node.cpp` (parameter declaration)
- Modify: `src/world_modeling/prediction_ml/config/params.yaml`
- Modify: `src/world_modeling/prediction_ml/README.md` (parameter table)
- Test: `src/world_modeling/prediction_ml/test/test_scene_builder.cpp` (append)

**Interfaces:**
- Consumes: existing `MtrConfig`.
- Produces: `MtrConfig::pinned_target_ids` of type `std::vector<std::string>`. Empty (the production default) means use the heuristic.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_scene_builder.cpp`, following the existing fixture style in that file:

```cpp
TEST(SceneBuilderPinnedTargets, EmptyPinnedListUsesHeuristic)
{
  MtrConfig config = makeTestConfig();
  ASSERT_TRUE(config.pinned_target_ids.empty()) << "production default must be the heuristic";

  SceneBuilder builder(config);
  const auto detections = makeDetections({"a", "b", "c"});
  builder.addFrame(detections);
  const auto tensors = builder.build(makeFrame(detections));

  ASSERT_TRUE(tensors.valid);
  EXPECT_FALSE(tensors.sidecar.targets.empty());
}

TEST(SceneBuilderPinnedTargets, PinnedIdsSelectExactlyThoseDetections)
{
  MtrConfig config = makeTestConfig();
  config.pinned_target_ids = {"c", "a"};

  SceneBuilder builder(config);
  const auto detections = makeDetections({"a", "b", "c", "d"});
  builder.addFrame(detections);
  const auto tensors = builder.build(makeFrame(detections));

  ASSERT_TRUE(tensors.valid);
  std::vector<std::string> got;
  for (const auto & target : tensors.sidecar.targets) {
    got.push_back(target.detection_id);
  }
  EXPECT_EQ(got, (std::vector<std::string>{"c", "a"})) << "pinned order must be preserved";
}

TEST(SceneBuilderPinnedTargets, UnknownPinnedIdIsSkippedNotFatal)
{
  MtrConfig config = makeTestConfig();
  config.pinned_target_ids = {"a", "does_not_exist"};

  SceneBuilder builder(config);
  const auto detections = makeDetections({"a", "b"});
  builder.addFrame(detections);
  const auto tensors = builder.build(makeFrame(detections));

  ASSERT_TRUE(tensors.valid);
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "a");
}

TEST(SceneBuilderPinnedTargets, PinnedListIsTruncatedToTargetCapacity)
{
  MtrConfig config = makeTestConfig();
  config.selected_target_agent_limit = 8;
  std::vector<std::string> ids;
  for (int i = 0; i < 12; ++i) {
    ids.push_back("id" + std::to_string(i));
  }
  config.pinned_target_ids = ids;

  SceneBuilder builder(config);
  const auto detections = makeDetections(ids);
  builder.addFrame(detections);
  const auto tensors = builder.build(makeFrame(detections));

  ASSERT_TRUE(tensors.valid);
  EXPECT_EQ(tensors.sidecar.targets.size(), 8U);
}

TEST(SceneBuilderPinnedTargets, PinningIgnoresTheForwardHemisphereFilter)
{
  // A pinned agent behind ego must still be selected: Waymo's tracks_to_predict
  // has no forward-region notion, and applying one would break the pairing.
  MtrConfig config = makeTestConfig();
  config.pinned_target_ids = {"behind"};

  SceneBuilder builder(config);
  auto detections = makeDetections({"behind"});
  detections.detections[0].bbox.center.position.x = -25.0;
  builder.addFrame(detections);
  const auto tensors = builder.build(makeFrame(detections));

  ASSERT_TRUE(tensors.valid);
  ASSERT_EQ(tensors.sidecar.targets.size(), 1U);
  EXPECT_EQ(tensors.sidecar.targets[0].detection_id, "behind");
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
colcon build --packages-select prediction_ml
colcon test --packages-select prediction_ml --ctest-args -R test_scene_builder
colcon test-result --verbose
```

Expected: compilation FAILS — `'struct prediction_ml::MtrConfig' has no member named 'pinned_target_ids'`.

- [ ] **Step 3: Add the field to `MtrConfig`**

In `include/prediction_ml/mtr_types.hpp`, inside `struct MtrConfig` (after `target_forward_half_angle_deg`):

```cpp
  // Evaluation-only: when non-empty, these detection ids are selected as MTR targets
  // verbatim, bypassing the forward-hemisphere/distance heuristic. The quality harness
  // sets this to Waymo's tracks_to_predict so rung-to-rung deltas measure tensor packing
  // rather than differing agent choices. Empty (the production default) uses the heuristic.
  std::vector<std::string> pinned_target_ids;
```

- [ ] **Step 4: Honour the field in `selectTargets`**

In `src/scene_builder.cpp`, at the top of `SceneBuilder::selectTargets`, before the existing heuristic:

```cpp
  if (!config_.pinned_target_ids.empty()) {
    std::vector<TargetCandidate> pinned;
    const auto limit = static_cast<std::size_t>(std::max(0, config_.selected_target_agent_limit));

    for (const auto & wanted : config_.pinned_target_ids) {
      if (pinned.size() >= limit) {
        break;
      }
      for (const auto & detection : detections.detections) {
        if (detection.id != wanted) {
          continue;
        }
        auto sample = sampleFromDetection(detection, current_time, detections.header.frame_id);
        if (!sample.has_value()) {
          break;
        }
        const double dx = sample->x - ego_pose.pose.position.x;
        const double dy = sample->y - ego_pose.pose.position.y;
        // in_forward_region is deliberately true: Waymo's tracks_to_predict has no
        // forward-region notion, and filtering here would break the pairing.
        pinned.push_back(TargetCandidate{*sample, dx * dx + dy * dy, true});
        break;
      }
    }
    return pinned;
  }
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
colcon build --packages-select prediction_ml
colcon test --packages-select prediction_ml --ctest-args -R test_scene_builder
colcon test-result --verbose
```

Expected: all `test_scene_builder` tests pass, including the five new ones and every pre-existing one.

- [ ] **Step 6: Declare the parameter on the node**

In `src/prediction_ml_node.cpp`, alongside the other `mtr.*` declarations:

```cpp
  config.pinned_target_ids =
    this->declare_parameter<std::vector<std::string>>("mtr.pinned_target_ids", {});
```

In `config/params.yaml`, under the `mtr:` block:

```yaml
    # Evaluation only. When non-empty, these detection ids become the MTR targets
    # verbatim, bypassing the selection heuristic. Leave empty in deployment.
    pinned_target_ids: []
```

Add a row to the `README.md` parameter table:

```markdown
| `mtr.pinned_target_ids` | `[]` | Evaluation only: pin MTR targets to these detection IDs, bypassing the selection heuristic |
```

- [ ] **Step 7: Verify the node still builds and launches**

```bash
colcon build --packages-select prediction_ml
source install/setup.bash
ros2 launch prediction_ml prediction_ml.launch.yaml &
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
ros2 param get /world_modeling/prediction_ml_node mtr.pinned_target_ids
```

Expected: `String values are: []`. Then kill the node.

- [ ] **Step 8: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/mtr_types.hpp \
        src/world_modeling/prediction_ml/src/scene_builder.cpp \
        src/world_modeling/prediction_ml/src/prediction_ml_node.cpp \
        src/world_modeling/prediction_ml/config/params.yaml \
        src/world_modeling/prediction_ml/README.md \
        src/world_modeling/prediction_ml/test/test_scene_builder.cpp
git commit -m "feat(prediction_ml): add mtr.pinned_target_ids for evaluation target pinning"
```

---

### Task 11: C++ tensor bundle reader

Reads the Task 3 format so the parity gtest can compare in-process — no GPU, no Python, no subprocess. Uses zlib, already in the ROS dependency tree, rather than adding an npz dependency for a test-only path.

**Files:**
- Create: `src/world_modeling/prediction_ml/include/prediction_ml/tensor_io.hpp`
- Create: `src/world_modeling/prediction_ml/src/tensor_io.cpp`
- Create: `src/world_modeling/prediction_ml/test/test_tensor_io.cpp`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt:38-50, 150-177`
- Modify: `src/world_modeling/prediction_ml/package.xml`

**Interfaces:**
- Produces:
  ```cpp
  namespace prediction_ml {
  enum class TensorDType { Float32 = 0, UInt8 = 1, Int64 = 2 };
  struct TensorBlob {
    TensorDType dtype;
    std::vector<uint32_t> dims;
    std::vector<float> f32;
    std::vector<uint8_t> u8;
    std::vector<int64_t> i64;
    std::size_t elementCount() const;
  };
  using TensorBundle = std::map<std::string, TensorBlob>;
  // Throws std::runtime_error on bad magic, version or truncation.
  TensorBundle readTensorBundle(const std::string & path);
  }
  ```

- [ ] **Step 1: Write the failing test**

`test/test_tensor_io.cpp`:

```cpp
// Copyright (c) 2025-present WATonomous. All rights reserved.
// ... (Apache-2.0 header)
#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "prediction_ml/tensor_io.hpp"

namespace
{
// Fixture written by the Python side; see eval/tests/conftest.py.
std::string fixturePath(const std::string & name)
{
  const char * root = std::getenv("PREDICTION_ML_FIXTURE_DIR");
  return std::string(root ? root : "test/fixtures") + "/" + name;
}
}  // namespace

TEST(TensorIo, ReadsFloatTensorWithCorrectShapeAndValues)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_sample.bin.gz"));
  ASSERT_EQ(bundle.count("obj_trajs"), 1U);

  const auto & blob = bundle.at("obj_trajs");
  EXPECT_EQ(blob.dtype, prediction_ml::TensorDType::Float32);
  EXPECT_EQ(blob.dims, (std::vector<uint32_t>{2, 3, 4, 5}));
  EXPECT_EQ(blob.f32.size(), 2U * 3U * 4U * 5U);
  EXPECT_EQ(blob.elementCount(), 120U);
}

TEST(TensorIo, ReadsMaskAsUint8AndIndexAsInt64)
{
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_sample.bin.gz"));

  const auto & mask = bundle.at("obj_trajs_mask");
  EXPECT_EQ(mask.dtype, prediction_ml::TensorDType::UInt8);
  EXPECT_EQ(mask.u8.size(), 2U * 3U * 4U);

  const auto & index = bundle.at("track_index_to_predict");
  EXPECT_EQ(index.dtype, prediction_ml::TensorDType::Int64);
  EXPECT_EQ(index.i64, (std::vector<int64_t>{0, 1}));
}

TEST(TensorIo, RoundTripsValuesWrittenByPython)
{
  // Python wrote arange(120) reshaped to [2,3,4,5]; C order must be preserved.
  const auto bundle = prediction_ml::readTensorBundle(fixturePath("tensor_io_ordered.bin.gz"));
  const auto & blob = bundle.at("ordered");
  ASSERT_EQ(blob.f32.size(), 120U);
  for (std::size_t i = 0; i < blob.f32.size(); ++i) {
    EXPECT_FLOAT_EQ(blob.f32[i], static_cast<float>(i)) << "at flat index " << i;
  }
}

TEST(TensorIo, ThrowsOnMissingFile)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("no_such_file.bin.gz")), std::runtime_error);
}

TEST(TensorIo, ThrowsOnBadMagic)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_bad_magic.bin.gz")),
    std::runtime_error);
}

TEST(TensorIo, ThrowsOnTruncatedPayload)
{
  EXPECT_THROW(
    prediction_ml::readTensorBundle(fixturePath("tensor_io_truncated.bin.gz")),
    std::runtime_error);
}
```

- [ ] **Step 2: Generate the C++ test fixtures from Python**

Add `eval/tests/make_cpp_fixtures.py`:

```python
"""Write the small bundles test_tensor_io.cpp reads. Run once; commit the output."""
import gzip
import pathlib
import sys

import numpy as np

from wato_mtr_eval import tensor_io

out = pathlib.Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)

rng = np.random.default_rng(0)
tensor_io.write_bundle(out / "tensor_io_sample.bin.gz", {
    "obj_trajs": rng.standard_normal((2, 3, 4, 5), dtype=np.float32),
    "obj_trajs_mask": rng.integers(0, 2, (2, 3, 4), dtype=np.uint8),
    "track_index_to_predict": np.arange(2, dtype=np.int64),
})
tensor_io.write_bundle(out / "tensor_io_ordered.bin.gz", {
    "ordered": np.arange(120, dtype=np.float32).reshape(2, 3, 4, 5),
})

with gzip.open(out / "tensor_io_bad_magic.bin.gz", "wb") as fh:
    fh.write(b"XXXX" + b"\x00" * 32)

with gzip.open(out / "tensor_io_sample.bin.gz", "rb") as fh:
    blob = fh.read()
with gzip.open(out / "tensor_io_truncated.bin.gz", "wb") as fh:
    fh.write(blob[: len(blob) // 2])
```

```bash
cd src/world_modeling/prediction_ml/eval
python tests/make_cpp_fixtures.py ../test/fixtures
```

- [ ] **Step 3: Wire the test into CMake**

In `CMakeLists.txt`, add `zlib` discovery near the other `find_package` calls (line ~36):

```cmake
find_package(ZLIB REQUIRED)
```

Add the source to `prediction_ml_lib` (after line 39):

```cmake
  src/tensor_io.cpp
```

Link zlib (inside the existing `target_link_libraries(prediction_ml_lib ...)` at line 130):

```cmake
  ZLIB::ZLIB
```

Register the test inside `if(BUILD_TESTING)`:

```cmake
  ament_add_gtest(test_tensor_io test/test_tensor_io.cpp)
  target_link_libraries(test_tensor_io prediction_ml_lib)
  set_tests_properties(test_tensor_io PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
```

Add to `package.xml`:

```xml
  <depend>zlib</depend>
```

- [ ] **Step 4: Run the test to verify it fails**

```bash
colcon build --packages-select prediction_ml
```

Expected: FAILS — `prediction_ml/tensor_io.hpp: No such file or directory`.

- [ ] **Step 5: Write `tensor_io.hpp`**

```cpp
// Copyright (c) 2025-present WATonomous. All rights reserved.
// ... (Apache-2.0 header)

#ifndef PREDICTION_ML__TENSOR_IO_HPP_
#define PREDICTION_ML__TENSOR_IO_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace prediction_ml
{

enum class TensorDType : uint8_t
{
  Float32 = 0,
  UInt8 = 1,
  Int64 = 2
};

// One tensor from a bundle. Exactly one of the three vectors is populated,
// selected by dtype.
struct TensorBlob
{
  TensorDType dtype{TensorDType::Float32};
  std::vector<uint32_t> dims;
  std::vector<float> f32;
  std::vector<uint8_t> u8;
  std::vector<int64_t> i64;

  std::size_t elementCount() const;
};

using TensorBundle = std::map<std::string, TensorBlob>;

// Read a gzipped bundle written by wato_mtr_eval.tensor_io.write_bundle.
// Throws std::runtime_error on a missing file, bad magic, unsupported version,
// unknown dtype or truncation.
TensorBundle readTensorBundle(const std::string & path);

}  // namespace prediction_ml

#endif  // PREDICTION_ML__TENSOR_IO_HPP_
```

- [ ] **Step 6: Write `tensor_io.cpp`**

```cpp
// Copyright (c) 2025-present WATonomous. All rights reserved.
// ... (Apache-2.0 header)

#include "prediction_ml/tensor_io.hpp"

#include <zlib.h>

#include <cstring>
#include <stdexcept>

namespace prediction_ml
{
namespace
{
constexpr char kMagic[4] = {'W', 'M', 'T', 'R'};
constexpr uint32_t kVersion = 1U;
constexpr std::size_t kChunk = 1U << 16U;

std::vector<uint8_t> readGzip(const std::string & path)
{
  gzFile file = gzopen(path.c_str(), "rb");
  if (file == nullptr) {
    throw std::runtime_error("tensor bundle: cannot open " + path);
  }

  std::vector<uint8_t> out;
  std::vector<uint8_t> chunk(kChunk);
  while (true) {
    const int got = gzread(file, chunk.data(), static_cast<unsigned>(chunk.size()));
    if (got < 0) {
      gzclose(file);
      throw std::runtime_error("tensor bundle: gzip read error in " + path);
    }
    if (got == 0) {
      break;
    }
    out.insert(out.end(), chunk.begin(), chunk.begin() + got);
  }
  gzclose(file);
  return out;
}

// Bounds-checked little-endian reader over the decompressed blob.
class Cursor
{
public:
  explicit Cursor(const std::vector<uint8_t> & data) : data_(data) {}

  void require(std::size_t count) const
  {
    if (offset_ + count > data_.size()) {
      throw std::runtime_error("tensor bundle: truncated");
    }
  }

  template <typename T>
  T read()
  {
    require(sizeof(T));
    T value{};
    std::memcpy(&value, data_.data() + offset_, sizeof(T));
    offset_ += sizeof(T);
    return value;
  }

  std::string readString(std::size_t length)
  {
    require(length);
    std::string value(reinterpret_cast<const char *>(data_.data() + offset_), length);
    offset_ += length;
    return value;
  }

  void readInto(void * dest, std::size_t bytes)
  {
    require(bytes);
    std::memcpy(dest, data_.data() + offset_, bytes);
    offset_ += bytes;
  }

private:
  const std::vector<uint8_t> & data_;
  std::size_t offset_{0};
};
}  // namespace

std::size_t TensorBlob::elementCount() const
{
  std::size_t total = 1U;
  for (const auto dim : dims) {
    total *= static_cast<std::size_t>(dim);
  }
  return dims.empty() ? 0U : total;
}

TensorBundle readTensorBundle(const std::string & path)
{
  const auto blob = readGzip(path);
  Cursor cursor(blob);

  char magic[4];
  cursor.readInto(magic, sizeof(magic));
  if (std::memcmp(magic, kMagic, sizeof(magic)) != 0) {
    throw std::runtime_error("tensor bundle: bad magic in " + path);
  }

  const auto version = cursor.read<uint32_t>();
  if (version != kVersion) {
    throw std::runtime_error("tensor bundle: unsupported version in " + path);
  }

  const auto count = cursor.read<uint32_t>();
  TensorBundle bundle;
  for (uint32_t i = 0; i < count; ++i) {
    const auto name_length = cursor.read<uint32_t>();
    const auto name = cursor.readString(name_length);

    TensorBlob tensor;
    const auto dtype_code = cursor.read<uint8_t>();
    const auto ndim = cursor.read<uint8_t>();
    tensor.dims.resize(ndim);
    for (uint8_t d = 0; d < ndim; ++d) {
      tensor.dims[d] = cursor.read<uint32_t>();
    }

    const auto elements = tensor.elementCount();
    switch (dtype_code) {
      case 0:
        tensor.dtype = TensorDType::Float32;
        tensor.f32.resize(elements);
        cursor.readInto(tensor.f32.data(), elements * sizeof(float));
        break;
      case 1:
        tensor.dtype = TensorDType::UInt8;
        tensor.u8.resize(elements);
        cursor.readInto(tensor.u8.data(), elements * sizeof(uint8_t));
        break;
      case 2:
        tensor.dtype = TensorDType::Int64;
        tensor.i64.resize(elements);
        cursor.readInto(tensor.i64.data(), elements * sizeof(int64_t));
        break;
      default:
        throw std::runtime_error("tensor bundle: unknown dtype code in " + path);
    }

    bundle.emplace(name, std::move(tensor));
  }

  return bundle;
}

}  // namespace prediction_ml
```

- [ ] **Step 7: Run the test to verify it passes**

```bash
colcon build --packages-select prediction_ml
colcon test --packages-select prediction_ml --ctest-args -R test_tensor_io
colcon test-result --verbose
```

Expected: 6 tests pass.

- [ ] **Step 8: Commit**

```bash
git add src/world_modeling/prediction_ml/include/prediction_ml/tensor_io.hpp \
        src/world_modeling/prediction_ml/src/tensor_io.cpp \
        src/world_modeling/prediction_ml/test/test_tensor_io.cpp \
        src/world_modeling/prediction_ml/test/fixtures \
        src/world_modeling/prediction_ml/eval/tests/make_cpp_fixtures.py \
        src/world_modeling/prediction_ml/CMakeLists.txt \
        src/world_modeling/prediction_ml/package.xml
git commit -m "feat(prediction_ml): add C++ tensor bundle reader for eval fixtures"
```

---

### Task 12: GATE B — the rung-2 parity gtest

**The CI gate, and the highest-durability deliverable in this plan.** It replays a committed mcap through the real `SceneBuilder` and compares the packed tensors against Python-generated references, in-process, with no GPU and no Python. It is the only thing standing between a future SceneBuilder refactor and silent quality loss that nobody notices until the vehicle behaves oddly.

**Files:**
- Create: `src/world_modeling/prediction_ml/test/test_scene_builder_parity.cpp`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/reference_pack.py`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt` (test registration)
- Modify: `src/world_modeling/prediction_ml/package.xml` (rosbag2 test deps)

**Interfaces:**
- Consumes: `readTensorBundle` (Task 11), `mtr.pinned_target_ids` (Task 10), fixture bags (Task 9).
- Produces: `reference_pack.pack(scen, config) -> dict[str, np.ndarray]` — the Python implementation of the WATO representation, producing the same eight input tensors SceneBuilder does.

**Comparison rules** (from the design spec):

| Field | Tolerance |
|---|---|
| shapes, dtypes | exact |
| `*_mask` (uint8) | **exact** |
| `track_index_to_predict`, `center_type_ids` (int64) | exact |
| positions, sizes, velocities (f32) | atol 1e-4 |
| headings, directions (f32) | atol 1e-5 |

Masks are compared **first and structurally**. A mask mismatch fails immediately with its own message rather than emitting thousands of confusing value diffs from comparing padding against real data. Float comparison then runs only over unmasked elements, with padding asserted zero separately.

- [ ] **Step 1: Write `reference_pack.py`**

This is the Python implementation of the same representation `scene_builder.cpp` implements. It reads `representation.py` constants so the two cannot drift on capacities or column layout.

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Reference packing: the WATO representation, implemented in Python.

Deliberately independent of scene_builder.cpp. Two independent implementations
of one written contract is the whole point -- a shared helper would let a bug
agree with itself.

Pure numpy: imports neither ROS nor torch.
"""
from __future__ import annotations

import math

import numpy as np

from . import lanelet_query as lq, representation as rep, scenario

S = scenario.STATE_COL


def _resampled_slots(scen, step: int) -> np.ndarray:
    """The HISTORY_STEPS timestamps SceneBuilder resamples onto, oldest first."""
    dt = 1.0 / rep.HISTORY_RATE_HZ
    now = float(scen.timestamps_s[step])
    return np.array([now - (rep.HISTORY_STEPS - 1 - i) * dt for i in range(rep.HISTORY_STEPS)])


def pack(scen, config: lq.QueryConfig, step: int | None = None) -> dict[str, np.ndarray]:
    """Pack the eight MTR input tensors for one scenario frame."""
    step = scen.current_time_index if step is None else step
    targets = list(scen.tracks_to_predict)[: rep.TARGET_CAPACITY]

    obj_trajs = np.zeros(
        (rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, rep.HISTORY_STEPS, rep.OBJ_FEATURE_DIM),
        dtype=np.float32,
    )
    obj_mask = np.zeros(
        (rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, rep.HISTORY_STEPS), dtype=np.uint8
    )
    last_pos = np.zeros((rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, 3), dtype=np.float32)
    track_index = np.zeros(rep.TARGET_CAPACITY, dtype=np.int64)
    type_ids = np.zeros(rep.TARGET_CAPACITY, dtype=np.int64)

    sdc = scen.sdc_track_index
    ego = scen.tracks[sdc].states[step]
    context = _rank_context(scen, step, (ego[S["x"]], ego[S["y"]]))
    slots = _resampled_slots(scen, step)

    for t, target_idx in enumerate(targets):
        center = scen.tracks[target_idx].states[step]
        cx, cy, cz = center[S["x"]], center[S["y"]], center[S["z"]]
        heading = float(center[S["heading"]])
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        type_ids[t] = rep.MTR_TYPE_CODES[scen.tracks[target_idx].object_type]
        track_index[t] = context.index(target_idx) if target_idx in context else 0

        for c, track_idx in enumerate(context):
            track = scen.tracks[track_idx]
            rel_v = np.zeros((rep.HISTORY_STEPS, 2))
            present = np.zeros(rep.HISTORY_STEPS, dtype=bool)

            for s, t_want in enumerate(slots):
                src = int(round((t_want - scen.timestamps_s[0]) * rep.HISTORY_RATE_HZ))
                if not (0 <= src < len(track.states)) or track.states[src, S["valid"]] == 0.0:
                    continue
                st = track.states[src]
                present[s] = True
                rel_v[s] = (
                    st[S["vx"]] * cos_h + st[S["vy"]] * sin_h,
                    -st[S["vx"]] * sin_h + st[S["vy"]] * cos_h,
                )

            acc = np.zeros((rep.HISTORY_STEPS, 2))
            acc_valid = np.zeros(rep.HISTORY_STEPS, dtype=bool)
            for s in range(1, rep.HISTORY_STEPS):
                if present[s] and present[s - 1]:
                    acc[s] = (rel_v[s] - rel_v[s - 1]) * rep.HISTORY_RATE_HZ
                    acc_valid[s] = True
            # scene_builder.cpp:729-743: the EARLIEST present slot inherits the next
            # valid acceleration; later gaps keep zero. Only the first present slot.
            for s in range(rep.HISTORY_STEPS):
                if not present[s]:
                    continue
                if not acc_valid[s]:
                    nxt = np.flatnonzero(acc_valid[s + 1 :])
                    if nxt.size:
                        acc[s] = acc[s + 1 + nxt[0]]
                break

            for s, t_want in enumerate(slots):
                if not present[s]:
                    continue
                src = int(round((t_want - scen.timestamps_s[0]) * rep.HISTORY_RATE_HZ))
                st = track.states[src]
                obj_mask[t, c, s] = 1

                px, py = st[S["x"]] - cx, st[S["y"]] - cy
                rel_heading = _wrap_pi(float(st[S["heading"]]) - heading)
                row = obj_trajs[t, c, s]
                row[rep.OBJ_COL["position"]] = (
                    px * cos_h + py * sin_h, -px * sin_h + py * cos_h, st[S["z"]] - cz
                )
                row[rep.OBJ_COL["size"]] = (
                    st[S["length"]], st[S["width"]], st[S["height"]]
                )
                onehot = rep.MTR_TYPE_CODES[track.object_type]
                row[rep.OBJ_COL["type_onehot"]][onehot] = 1.0
                row[rep.OBJ_COL["is_center"]] = 1.0 if track_idx == target_idx else 0.0
                row[rep.OBJ_COL["is_sdc"]] = 1.0 if track_idx == sdc else 0.0
                row[rep.OBJ_COL["time_onehot"]][s] = 1.0
                row[rep.OBJ_COL["scalar_time"]] = t_want - float(scen.timestamps_s[step])
                row[rep.OBJ_COL["heading"]] = (math.sin(rel_heading), math.cos(rel_heading))
                row[rep.OBJ_COL["velocity"]] = rel_v[s]
                row[rep.OBJ_COL["acceleration"]] = acc[s]

            for s in range(rep.HISTORY_STEPS - 1, -1, -1):
                if not present[s]:
                    continue
                src = int(round((slots[s] - scen.timestamps_s[0]) * rep.HISTORY_RATE_HZ))
                st = track.states[src]
                px, py = st[S["x"]] - cx, st[S["y"]] - cy
                last_pos[t, c] = (
                    px * cos_h + py * sin_h, -px * sin_h + py * cos_h, st[S["z"]] - cz
                )
                break

    map_tensors = _pack_map(scen, step, targets, config)
    return {
        "obj_trajs": obj_trajs,
        "obj_trajs_mask": obj_mask,
        "obj_trajs_last_pos": last_pos,
        "track_index_to_predict": track_index,
        "center_type_ids": type_ids,
        **map_tensors,
    }


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _rank_context(scen, step: int, ego_xy) -> list[int]:
    """Closest CONTEXT_CAPACITY tracks by ego-frame distance, ties by id ascending."""
    scored = []
    for i, track in enumerate(scen.tracks):
        st = track.states[step]
        if st[S["valid"]] == 0.0 or track.object_type not in rep.MTR_TYPE_CODES:
            continue
        d = (st[S["x"]] - ego_xy[0]) ** 2 + (st[S["y"]] - ego_xy[1]) ** 2
        scored.append((d, str(track.object_id), i))
    scored.sort()
    return [i for _, _, i in scored[: rep.CONTEXT_CAPACITY]]


def _pack_map(scen, step, targets, config):
    """Target-centered map polylines, 9 columns per point (scene_builder.cpp:939)."""
    polylines = np.zeros(
        (rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, rep.POINTS_PER_POLYLINE, rep.MAP_FEATURE_DIM),
        dtype=np.float32,
    )
    mask = np.zeros(
        (rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, rep.POINTS_PER_POLYLINE), dtype=np.uint8
    )
    centers = np.zeros((rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, 3), dtype=np.float32)

    ego = scen.tracks[scen.sdc_track_index].states[step]
    kept, _ = lq.query(
        scen.map_polylines,
        (float(ego[S["x"]]), float(ego[S["y"]])),
        float(ego[S["heading"]]),
        config,
    )

    for t, target_idx in enumerate(targets):
        center = scen.tracks[target_idx].states[step]
        cx, cy, cz = center[S["x"]], center[S["y"]], center[S["z"]]
        heading = float(center[S["heading"]])
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        chunks = []
        for poly in kept:
            for start in range(0, len(poly.points), rep.POINTS_PER_POLYLINE):
                chunks.append(poly.points[start : start + rep.POINTS_PER_POLYLINE])

        for p, chunk in enumerate(chunks[: rep.NUM_SRC_POLYLINES]):
            local = np.zeros((len(chunk), 3))
            for k, pt in enumerate(chunk):
                dx, dy = pt[0] - cx, pt[1] - cy
                local[k] = (dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h, pt[2] - cz)

            for k in range(len(chunk)):
                prev = local[k - 1] if k > 0 else local[k]
                direction = local[k] - prev
                norm = np.linalg.norm(direction)
                direction = direction / norm if norm > 0 else direction
                polylines[t, p, k, rep.MAP_COL["position"]] = local[k]
                polylines[t, p, k, rep.MAP_COL["direction"]] = direction
                polylines[t, p, k, rep.MAP_COL["global_type"]] = rep.WATO_GLOBAL_TYPE
                polylines[t, p, k, rep.MAP_COL["pre_position"]] = prev[:2]
                mask[t, p, k] = 1
            centers[t, p] = local.mean(axis=0)

    return {
        "map_polylines": polylines,
        "map_polylines_mask": mask,
        "map_polylines_center": centers,
    }
```

- [ ] **Step 2: Generate the expected tensors for each fixture**

```bash
cd src/world_modeling/prediction_ml/eval
python - <<'PY'
import pathlib
from wato_mtr_eval import lanelet_query as lq, reference_pack, scenario, tensor_io

shard = pathlib.Path("../../../../uncompressed_scenario_validation_validation.tfrecord-00000-of-00150")
fixtures = pathlib.Path("../test/fixtures")
wanted = {p.name for p in fixtures.iterdir() if p.is_dir()}

for scen in scenario.load_scenarios(shard):
    if scen.scenario_id not in wanted:
        continue
    tensors = reference_pack.pack(scen, lq.QueryConfig(strategy="radius"))
    tensor_io.write_bundle(fixtures / scen.scenario_id / "expected_tensors.bin.gz", tensors)
    print("wrote", scen.scenario_id)
PY
```

- [ ] **Step 3: Write the failing parity test**

```cpp
// Copyright (c) 2025-present WATonomous. All rights reserved.
// ... (Apache-2.0 header)
//
// GATE B. Replays a committed mcap through the real SceneBuilder and compares
// the packed tensors against Python-generated references. No GPU, no Python.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "prediction_ml/scene_builder.hpp"
#include "prediction_ml/tensor_io.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"

namespace
{
constexpr float kPositionTolerance = 1e-4F;
constexpr float kAngleTolerance = 1e-5F;

std::string fixtureRoot()
{
  const char * root = std::getenv("PREDICTION_ML_FIXTURE_DIR");
  return root ? root : "test/fixtures";
}

std::vector<std::string> fixtureScenarios()
{
  std::vector<std::string> names;
  for (const auto & entry : std::filesystem::directory_iterator(fixtureRoot())) {
    if (entry.is_directory() && std::filesystem::exists(entry.path() / "expected_tensors.bin.gz")) {
      names.push_back(entry.path().filename().string());
    }
  }
  return names;
}

// Replay every frame of a fixture bag into a SceneBuilder, then build at the
// last frame -- the same point reference_pack.py packs.
prediction_ml::MtrInputTensors buildFromBag(
  const std::string & bag_dir, const std::vector<std::string> & pinned_ids);

class SceneBuilderParity : public ::testing::TestWithParam<std::string>
{
};
}  // namespace

TEST_P(SceneBuilderParity, ShapesMatchTheContract)
{
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});

  ASSERT_TRUE(actual.valid) << "SceneBuilder produced no valid tensors";
  EXPECT_EQ(actual.obj_trajs.size(), expected.at("obj_trajs").f32.size());
  EXPECT_EQ(actual.obj_trajs_mask.size(), expected.at("obj_trajs_mask").u8.size());
  EXPECT_EQ(actual.map_polylines.size(), expected.at("map_polylines").f32.size());
  EXPECT_EQ(actual.map_polylines_mask.size(), expected.at("map_polylines_mask").u8.size());
}

TEST_P(SceneBuilderParity, MasksMatchExactly)
{
  // Compared FIRST and structurally. A mask mismatch makes value comparison
  // meaningless -- you would be diffing padding against real data -- so this
  // fails on its own rather than emitting thousands of confusing value diffs.
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});
  ASSERT_TRUE(actual.valid);

  const auto & want_obj = expected.at("obj_trajs_mask").u8;
  ASSERT_EQ(actual.obj_trajs_mask.size(), want_obj.size());
  std::size_t mismatches = 0;
  std::size_t first_mismatch = 0;
  for (std::size_t i = 0; i < want_obj.size(); ++i) {
    if (actual.obj_trajs_mask[i] != want_obj[i]) {
      if (mismatches == 0) {
        first_mismatch = i;
      }
      ++mismatches;
    }
  }
  EXPECT_EQ(mismatches, 0U)
    << "obj_trajs_mask differs at " << mismatches << " of " << want_obj.size()
    << " elements; first at flat index " << first_mismatch
    << ". Fix masking before looking at any value diff.";

  const auto & want_map = expected.at("map_polylines_mask").u8;
  ASSERT_EQ(actual.map_polylines_mask.size(), want_map.size());
  EXPECT_TRUE(std::equal(want_map.begin(), want_map.end(), actual.map_polylines_mask.begin()))
    << "map_polylines_mask differs";
}

TEST_P(SceneBuilderParity, IntegerFieldsMatchExactly)
{
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});
  ASSERT_TRUE(actual.valid);

  EXPECT_EQ(actual.track_index_to_predict, expected.at("track_index_to_predict").i64);
  EXPECT_EQ(actual.center_type_ids, expected.at("center_type_ids").i64);
}

TEST_P(SceneBuilderParity, UnmaskedObjectFeaturesMatchWithinTolerance)
{
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});
  ASSERT_TRUE(actual.valid);

  const auto & want = expected.at("obj_trajs").f32;
  const auto & mask = expected.at("obj_trajs_mask").u8;
  constexpr std::size_t kFeatureDim = 29U;
  // Columns 23-24 are sin/cos of relative heading: tighter tolerance.
  constexpr std::size_t kHeadingBegin = 23U;
  constexpr std::size_t kHeadingEnd = 25U;

  ASSERT_EQ(actual.obj_trajs.size(), want.size());
  for (std::size_t slot = 0; slot < mask.size(); ++slot) {
    if (mask[slot] == 0U) {
      continue;
    }
    for (std::size_t f = 0; f < kFeatureDim; ++f) {
      const std::size_t i = slot * kFeatureDim + f;
      const float tolerance =
        (f >= kHeadingBegin && f < kHeadingEnd) ? kAngleTolerance : kPositionTolerance;
      ASSERT_NEAR(actual.obj_trajs[i], want[i], tolerance)
        << "obj_trajs mismatch at slot " << slot << " feature column " << f;
    }
  }
}

TEST_P(SceneBuilderParity, MaskedSlotsArePaddedWithZeros)
{
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});
  ASSERT_TRUE(actual.valid);

  const auto & mask = expected.at("obj_trajs_mask").u8;
  constexpr std::size_t kFeatureDim = 29U;
  for (std::size_t slot = 0; slot < mask.size(); ++slot) {
    if (mask[slot] != 0U) {
      continue;
    }
    for (std::size_t f = 0; f < kFeatureDim; ++f) {
      ASSERT_FLOAT_EQ(actual.obj_trajs[slot * kFeatureDim + f], 0.0F)
        << "padding must be zero at slot " << slot << " column " << f;
    }
  }
}

TEST_P(SceneBuilderParity, UnmaskedMapFeaturesMatchWithinTolerance)
{
  const auto expected =
    prediction_ml::readTensorBundle(fixtureRoot() + "/" + GetParam() + "/expected_tensors.bin.gz");
  const auto actual = buildFromBag(fixtureRoot() + "/" + GetParam(), {});
  ASSERT_TRUE(actual.valid);

  const auto & want = expected.at("map_polylines").f32;
  const auto & mask = expected.at("map_polylines_mask").u8;
  constexpr std::size_t kMapDim = 9U;
  // Columns 3-5 are the normalized direction vector: tighter tolerance.
  constexpr std::size_t kDirBegin = 3U;
  constexpr std::size_t kDirEnd = 6U;

  ASSERT_EQ(actual.map_polylines.size(), want.size());
  for (std::size_t point = 0; point < mask.size(); ++point) {
    if (mask[point] == 0U) {
      continue;
    }
    for (std::size_t f = 0; f < kMapDim; ++f) {
      const std::size_t i = point * kMapDim + f;
      const float tolerance =
        (f >= kDirBegin && f < kDirEnd) ? kAngleTolerance : kPositionTolerance;
      ASSERT_NEAR(actual.map_polylines[i], want[i], tolerance)
        << "map_polylines mismatch at point " << point << " column " << f;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Fixtures, SceneBuilderParity, ::testing::ValuesIn(fixtureScenarios()),
  [](const auto & info) { return info.param; });
```

Implement `buildFromBag` in the anonymous namespace: open the bag with `rosbag2_cpp::Reader`, deserialize each `Detection3DArray` / `PoseStamped` / `Odometry` / `LaneletAhead` with `rclcpp::Serialization<T>`, call `builder.addFrame(...)` per detection message, retain the latest pose/odom/lanelet, and after the final message call `builder.build(frame)` with `has_ego`, `has_ego_velocity` and `has_map` set true.

- [ ] **Step 4: Register the test in CMake**

```cmake
  find_package(rosbag2_cpp REQUIRED)
  ament_add_gtest(test_scene_builder_parity test/test_scene_builder_parity.cpp)
  target_link_libraries(test_scene_builder_parity prediction_ml_lib rosbag2_cpp::rosbag2_cpp)
  set_tests_properties(test_scene_builder_parity PROPERTIES
    ENVIRONMENT "PREDICTION_ML_FIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
```

Add to `package.xml`:

```xml
  <test_depend>rosbag2_cpp</test_depend>
```

- [ ] **Step 5: Run the test to verify it fails**

```bash
colcon build --packages-select prediction_ml
colcon test --packages-select prediction_ml --ctest-args -R test_scene_builder_parity
colcon test-result --verbose
```

Expected: FAIL. The first honest failure is likely `MasksMatchExactly` or `IntegerFieldsMatchExactly` — that is the gate doing its job.

- [ ] **Step 6: Reconcile discrepancies one at a time**

Work in this order, because each later comparison is meaningless until the earlier one passes:

1. **Shapes** — a mismatch means a capacity constant disagrees. Reconcile against `representation.py` and `contract_8_target.yaml`.
2. **Masks** — resampling or validity logic differs. Check `nearestSampleForSlot` against `_resampled_slots`.
3. **Integer fields** — context ranking or tie-breaking differs. `scene_builder.cpp:148` documents ties broken by `detection_id` ascending; `_rank_context` must match.
4. **Object features** — column ordering, frame convention, or sign errors.
5. **Map features** — chunking, direction convention, or the `pre_position` definition.

For each discrepancy, decide deliberately which side is wrong. `reference_pack.py` is a reading of the written contract; `scene_builder.cpp` is production code. Either can be the bug. **Record every reconciliation in `REPRESENTATION.md`** — an ambiguity that needed a decision is exactly what the contract document exists to pin down.

**Expected known divergence:** the acceleration first-step rule under masked history (`scene_builder.cpp:717-743`). Do not force it to zero here; Task 13 measures it. If `UnmaskedObjectFeaturesMatchWithinTolerance` fails *only* on columns 27–28 and *only* for agents with masked history, that is the anticipated case — note it and continue.

- [ ] **Step 7: Run the full test suite to verify nothing regressed**

```bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

Expected: every test passes, including the six pre-existing suites.

- [ ] **Step 8: Record Gate B and commit**

Append a "Gate B" section to `eval/README.md` with the date, the fixture scenario IDs, and any reconciliations made in step 6.

```bash
git add src/world_modeling/prediction_ml/test/test_scene_builder_parity.cpp \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/reference_pack.py \
        src/world_modeling/prediction_ml/test/fixtures \
        src/world_modeling/prediction_ml/CMakeLists.txt \
        src/world_modeling/prediction_ml/package.xml \
        src/world_modeling/prediction_ml/REPRESENTATION.md \
        src/world_modeling/prediction_ml/eval/README.md
git commit -m "test(prediction_ml): add rung 2 scene builder parity gate"
```

---

### Task 13: `scene_dump` CLI and rung-2 scoring

The gtest proves the tensors match. This scores the *consequence* — feeding SceneBuilder's tensors to PyTorch MTR and measuring the paired minFDE delta against rung 1. Also resolves the deferred acceleration decision.

**Files:**
- Create: `src/world_modeling/prediction_ml/tools/scene_dump.cpp`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung2.py`
- Modify: `src/world_modeling/prediction_ml/CMakeLists.txt`

**Interfaces:**
- Consumes: `readTensorBundle` format (writing side), `metrics.paired_delta`, `reference_pack.pack`.
- Produces:
  - `scene_dump <bag_dir> <out.bin.gz> [--pinned id1,id2,...]` — replays a bag, writes the packed tensors.
  - `rung2.run(fixture_dir, checkpoint, mtr_root) -> Rung2Report` with `paired_minfde: PairedDelta`, `paired_minfde_masked_history: PairedDelta`, `paired_minfde_clean_history: PairedDelta`

- [ ] **Step 1: Write `scene_dump.cpp`**

Reuse the bag-replay helper from Task 12 rather than duplicating it — extract it into `test/bag_replay.hpp` during this step and include it from both. Write the tensors using the mirror of `readTensorBundle`; add `writeTensorBundle` to `tensor_io.cpp` alongside a round-trip test in `test_tensor_io.cpp`:

```cpp
TEST(TensorIo, WriterRoundTripsThroughTheReader)
{
  prediction_ml::TensorBundle bundle;
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::Float32;
  blob.dims = {2, 3};
  blob.f32 = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
  bundle.emplace("x", blob);

  const std::string path = std::string(std::getenv("TEST_TMPDIR") ?: "/tmp") + "/rt.bin.gz";
  prediction_ml::writeTensorBundle(path, bundle);
  const auto back = prediction_ml::readTensorBundle(path);

  EXPECT_EQ(back.at("x").dims, blob.dims);
  EXPECT_EQ(back.at("x").f32, blob.f32);
}
```

Register in CMake:

```cmake
add_executable(scene_dump tools/scene_dump.cpp)
target_link_libraries(scene_dump prediction_ml_lib rosbag2_cpp::rosbag2_cpp)
install(TARGETS scene_dump DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 2: Verify `scene_dump` reproduces the gtest's tensors**

```bash
colcon build --packages-select prediction_ml
source install/setup.bash
ros2 run prediction_ml scene_dump \
  src/world_modeling/prediction_ml/test/fixtures/<scenario_id> /tmp/dumped.bin.gz
```

Then confirm from Python that the dump equals the fixture's expected tensors under the Task 12 tolerances:

```python
from wato_mtr_eval import tensor_io
import numpy as np
a = tensor_io.read_bundle("/tmp/dumped.bin.gz")
b = tensor_io.read_bundle(".../expected_tensors.bin.gz")
np.testing.assert_array_equal(a["obj_trajs_mask"], b["obj_trajs_mask"])
np.testing.assert_allclose(a["obj_trajs"], b["obj_trajs"], atol=1e-4)
print("scene_dump agrees with the parity fixture")
```

Expected: no assertion error. If this disagrees with the passing gtest, the dump path and the test path diverge — fix before scoring anything.

- [ ] **Step 3: Write `rung2.py`**

```python
# Copyright (c) 2025-present WATonomous. All rights reserved.
# ... (Apache-2.0 header)
"""Rung 2 — score the consequence of C++ packing.

The gtest proves the tensors match. This measures what the difference costs in
prediction quality, and splits the paired delta by whether the agent had masked
history -- which is how the deferred acceleration-rule decision gets made from
data rather than from argument.
"""
from __future__ import annotations

import dataclasses
import pathlib
import subprocess

import numpy as np

from . import metrics, model, reference_pack, tensor_io


@dataclasses.dataclass(frozen=True)
class Rung2Report:
    paired_minfde: metrics.PairedDelta
    paired_minfde_masked_history: metrics.PairedDelta
    paired_minfde_clean_history: metrics.PairedDelta

    def gate_ok(self) -> bool:
        """Gate B: clean-history agents must be essentially identical."""
        return self.paired_minfde_clean_history.near_zero_fraction >= 0.99

    def format(self) -> str:
        return "\n\n".join([
            metrics.format_paired_delta(self.paired_minfde, "R1 -> R2  minFDE  (all agents)"),
            metrics.format_paired_delta(
                self.paired_minfde_clean_history, "R1 -> R2  minFDE  (gap-free history)"
            ),
            metrics.format_paired_delta(
                self.paired_minfde_masked_history, "R1 -> R2  minFDE  (masked history)"
            ),
            f"GATE B: {'PASS' if self.gate_ok() else 'FAIL'}",
        ])
```

`run()` iterates the fixture bags: invoke `scene_dump` per bag via `subprocess`, load both the dumped and the reference bundles, forward each through MTR, compute `min_fde` on the deployed horizon for both, then split agents by whether `obj_trajs_mask` is fully set across their 11 history slots and call `paired_delta` on each cohort plus the union.

- [ ] **Step 4: Run rung 2 and resolve the acceleration decision**

```bash
python -m wato_mtr_eval.rung2 \
  --fixtures ../test/fixtures --checkpoint "$MTR_CHECKPOINT" --mtr-root ~/mtr
```

Expected: `GATE B: PASS` — ≥99% of gap-free-history agents under 1e-3 m.

Then decide the acceleration rule from the masked-history cohort:

- **Negligible** (near-zero fraction ≥ 0.99, max < 0.01 m): keep WATO's inherit-next-valid rule — it is better on gappy real-world tracks — and document the tolerance in `REPRESENTATION.md`.
- **Material** (otherwise): change `scene_builder.cpp:717-743` to MTR's rule (first step copies second, fixed 10 Hz), matching training-time preprocessing, then re-run Task 12 and tighten the gate to zero delta on all agents.

Record the number and the decision in `REPRESENTATION.md` and `eval/README.md`.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction_ml/tools/scene_dump.cpp \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung2.py \
        src/world_modeling/prediction_ml/CMakeLists.txt \
        src/world_modeling/prediction_ml/REPRESENTATION.md \
        src/world_modeling/prediction_ml/eval/README.md
git commit -m "feat(prediction_ml): add scene_dump CLI and rung 2 scoring"
```

---

### Task 14: Rung 1 and the ablation sweep

Prices the WATO input representation, then prices each discarded map layer separately. The sweep is a phase-1 deliverable rather than a research afterthought because `lanelet_msgs/Lanelet` already carries road edges, crosswalks and boundary types — SceneBuilder discards them. The sweep therefore prices a **SceneBuilder change**, not a new data source.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung1.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_rung1.py`

**Interfaces:**
- Consumes: `reference_pack.pack`, `lanelet_query.QueryConfig`, `metrics`, `model`.
- Produces:
  - `Ablation` dataclass: `name: str`, `keep_types: set[str]`, `collapse_global_type: bool`, `query_strategy: str`
  - `SWEEP: list[Ablation]` — the ordered degradation chain
  - `rung1.run(shard, checkpoint, mtr_root, ablation, limit=None) -> Rung1Report`
  - `rung1.sweep(...) -> list[Rung1Report]`

- [ ] **Step 1: Write the failing test**

```python
import pytest

from wato_mtr_eval import rung1


def test_sweep_starts_at_full_map_and_ends_at_wato_representation():
    assert rung1.SWEEP[0].name == "full_map"
    assert rung1.SWEEP[-1].name == "wato_representation"


def test_sweep_is_monotonically_degrading():
    """Each step keeps a subset of the previous step's map types."""
    for prev, curr in zip(rung1.SWEEP, rung1.SWEEP[1:]):
        assert curr.keep_types <= prev.keep_types, f"{curr.name} adds types back"


def test_final_ablation_matches_what_scene_builder_actually_packs():
    final = rung1.SWEEP[-1]
    assert final.keep_types == {"lane"}
    assert final.collapse_global_type is True


def test_full_map_ablation_preserves_type_information():
    assert rung1.SWEEP[0].collapse_global_type is False
    assert "road_edge" in rung1.SWEEP[0].keep_types


def test_every_ablation_has_a_unique_name():
    names = [a.name for a in rung1.SWEEP]
    assert len(names) == len(set(names))
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
pytest tests/test_rung1.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.rung1'`.

- [ ] **Step 3: Write `rung1.py`**

Define the sweep as an ordered degradation chain and a `run` that packs with `reference_pack` under each ablation, forwards through MTR, and scores:

```python
SWEEP = [
    Ablation("full_map", {"lane", "road_line", "road_edge", "crosswalk", "speed_bump"},
             collapse_global_type=False, query_strategy="radius"),
    Ablation("no_road_edges", {"lane", "road_line", "crosswalk", "speed_bump"},
             collapse_global_type=False, query_strategy="radius"),
    Ablation("no_crosswalks", {"lane", "road_line", "speed_bump"},
             collapse_global_type=False, query_strategy="radius"),
    Ablation("no_boundaries", {"lane"},
             collapse_global_type=False, query_strategy="radius"),
    Ablation("collapsed_types", {"lane"},
             collapse_global_type=True, query_strategy="radius"),
    Ablation("wato_representation", {"lane"},
             collapse_global_type=True, query_strategy="reachable"),
]
```

`run` must extend `reference_pack.pack` to accept an `Ablation`, honouring `keep_types` in the polyline filter and writing the Waymo feature-type code rather than `WATO_GLOBAL_TYPE` when `collapse_global_type` is false.

- [ ] **Step 4: Run the tests to verify they pass**

```bash
pytest tests/test_rung1.py -v
```

Expected: 5 passed.

- [ ] **Step 5: Run the sweep and produce the map budget report**

```bash
python -m wato_mtr_eval.rung1 --sweep \
  --shard ../../../../uncompressed_scenario_validation_validation.tfrecord-00000-of-00150 \
  --checkpoint "$MTR_CHECKPOINT" --mtr-root ~/mtr
```

Report each rung's deployed minADE/minFDE and the paired delta from the previous rung, so each discarded layer is priced individually. Write the table to `eval/README.md` under "Map budget".

This table is the deliverable that answers whether extending SceneBuilder to pack road edges and crosswalks is worth the engineering — a question nobody on the team can currently answer.

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung1.py \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/reference_pack.py \
        src/world_modeling/prediction_ml/eval/tests/test_rung1.py \
        src/world_modeling/prediction_ml/eval/README.md
git commit -m "feat(prediction_ml): add rung 1 ablation and map budget sweep"
```

---

## ⛔ TensorRT gate

**Do not start Task 15 until both gates have passed and are recorded in `eval/README.md`:**

- **Gate A** (Task 6): rung 0 within 3% of the published minADE/minFDE.
- **Gate B** (Tasks 12–13): parity gtest green, and ≥99% of gap-free-history agents under 1e-3 m Δ minFDE.

A TensorRT discrepancy is only interpretable once the weights, the converter and the C++ packing are each known-good. Starting here early means debugging four unknowns at once inside a build log.

---

### Task 15: Rung 3-pre — replace the custom CUDA ops

**The riskiest item in the plan.** MTR's `setup.py` compiles two hand-written CUDA extensions — `knn_cuda` and `attention_cuda` (sparse/local attention, with separate weight- and value-computation kernels). Neither is ONNX-exportable; `torch.onnx.export` fails on the first one it reaches. This is almost certainly why `tensorrt_backend.cpp` has never had an engine to load.

Correctness is proven **in PyTorch, before ONNX enters the picture**, because otherwise a rung-3 discrepancy has three candidate causes (bad op replacement, bad export, fp16 loss) with no way to separate them.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/ops_dense.py`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_ops_dense.py`

**Interfaces:**
- Produces:
  - `dense_knn(query: Tensor, source: Tensor, k: int, batch_mask: Tensor) -> Tensor` — index tensor matching `knn_cuda`'s output
  - `DenseAttention(nn.Module)` — accepts the same arguments `attention_cuda` is called with, returns the same shape
  - `patch_mtr(net) -> net` — swaps every custom-op call site for the dense equivalent

- [ ] **Step 1: Write the failing equivalence test**

```python
import os

import pytest

torch = pytest.importorskip("torch")

from wato_mtr_eval import ops_dense

needs_cuda_ops = pytest.mark.skipif(
    not (torch.cuda.is_available() and os.environ.get("MTR_ROOT")),
    reason="needs a CUDA GPU and the built MTR extensions",
)


@needs_cuda_ops
def test_dense_knn_matches_the_cuda_kernel():
    from mtr.ops.knn import knn_utils

    torch.manual_seed(0)
    query = torch.rand(1, 64, 3, device="cuda")
    source = torch.rand(1, 256, 3, device="cuda")
    mask = torch.ones(1, 256, dtype=torch.bool, device="cuda")

    reference = knn_utils.knn_batch(source, query, k=16, batch_mask=mask)
    actual = ops_dense.dense_knn(query, source, k=16, batch_mask=mask)
    torch.testing.assert_close(actual.sort(dim=-1).values, reference.sort(dim=-1).values)


@needs_cuda_ops
def test_dense_attention_matches_the_cuda_kernel_within_float_tolerance():
    from mtr.ops.attention import attention_utils

    torch.manual_seed(0)
    n, heads, dim = 128, 8, 64
    query = torch.rand(n, heads, dim, device="cuda")
    key = torch.rand(n, heads, dim, device="cuda")
    value = torch.rand(n, heads, dim, device="cuda")
    index_pair = torch.randint(0, n, (n, 16), device="cuda", dtype=torch.int32)
    counts = torch.full((n,), 16, device="cuda", dtype=torch.int32)

    weights = attention_utils.attention_weight_computation(
        counts, torch.arange(n, device="cuda", dtype=torch.int32), index_pair, query, key
    )
    reference = attention_utils.attention_value_computation(
        counts, torch.arange(n, device="cuda", dtype=torch.int32), index_pair, weights, value
    )
    actual = ops_dense.DenseAttention()(query, key, value, index_pair, counts)
    torch.testing.assert_close(actual, reference, rtol=1e-4, atol=1e-4)


@needs_cuda_ops
def test_patched_model_matches_the_original_end_to_end():
    """The gate: identical outputs on a real batch before any export is attempted."""
    import copy
    from wato_mtr_eval import model

    net = model.load_checkpoint(
        os.environ["MTR_CHECKPOINT"],
        os.environ["MTR_ROOT"] + "/tools/cfgs/waymo/mtr+100_percent_data.yaml",
        device="cuda",
    )
    batch = torch.load(os.environ["MTR_SAMPLE_BATCH"])
    original_scores, original_trajs = model.forward_batch(net, batch)

    patched = ops_dense.patch_mtr(copy.deepcopy(net))
    patched_scores, patched_trajs = model.forward_batch(patched, batch)

    torch.testing.assert_close(patched_scores, original_scores, rtol=1e-3, atol=1e-3)
    torch.testing.assert_close(patched_trajs, original_trajs, rtol=1e-3, atol=1e-3)


def test_dense_attention_is_exportable_to_onnx(tmp_path):
    """No custom op may survive into the graph."""
    module = ops_dense.DenseAttention()
    n, heads, dim = 16, 2, 8
    args = (
        torch.rand(n, heads, dim), torch.rand(n, heads, dim), torch.rand(n, heads, dim),
        torch.randint(0, n, (n, 4), dtype=torch.int32),
        torch.full((n,), 4, dtype=torch.int32),
    )
    path = tmp_path / "attn.onnx"
    torch.onnx.export(module, args, str(path), opset_version=17)
    assert path.exists()
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
pytest tests/test_ops_dense.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'wato_mtr_eval.ops_dense'`.

- [ ] **Step 3: Implement `ops_dense.py`**

`dense_knn` computes the full pairwise distance matrix and takes `topk`, masking invalid sources with `+inf`. `DenseAttention` gathers keys and values by `index_pair`, masks positions beyond each row's `count`, applies scaled softmax, and contracts — the same computation the two CUDA kernels perform, expressed in exportable ops. `patch_mtr` walks the module tree and replaces each call site.

- [ ] **Step 4: Run the tests to verify they pass**

```bash
pytest tests/test_ops_dense.py -v
```

Expected: 4 passed. `test_patched_model_matches_the_original_end_to_end` is the gate — **do not proceed to Task 16 until it passes.**

- [ ] **Step 5: Measure the cost of the replacement**

```bash
python -m wato_mtr_eval.ops_dense --benchmark --batch "$MTR_SAMPLE_BATCH"
```

Report custom-kernel vs dense forward time. The dense form **will** be slower — MTR's sparse kernels exist precisely because dense attention over 768 polylines is expensive. Record the number; do not act on it yet. TRT plugins are a fallback to be written only if the measured on-4080 latency budget demands it, never speculatively.

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/ops_dense.py \
        src/world_modeling/prediction_ml/eval/tests/test_ops_dense.py \
        src/world_modeling/prediction_ml/eval/README.md
git commit -m "feat(prediction_ml): add ONNX-expressible replacements for MTR custom CUDA ops"
```

---

### Task 16: ONNX export

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/export_onnx.py`
- Create: `src/world_modeling/prediction_ml/config/engine_build.yaml`
- Test: `src/world_modeling/prediction_ml/eval/tests/test_export_onnx.py`

**Interfaces:**
- Consumes: `ops_dense.patch_mtr`, `model.load_checkpoint`, `contract_8_target.yaml`.
- Produces: `export(checkpoint, cfg_path, out_path, contract_path) -> None`; artifacts `mtr.onnx` + `engine_build.yaml`.

- [ ] **Step 1: Write the failing test**

```python
import pathlib

import pytest

onnx = pytest.importorskip("onnx")

from wato_mtr_eval import export_onnx, representation as rep

ONNX_PATH = pathlib.Path("build/mtr.onnx")
needs_onnx = pytest.mark.skipif(not ONNX_PATH.exists(), reason="run export first")


@needs_onnx
def test_input_names_and_shapes_match_the_frozen_contract():
    graph = onnx.load(str(ONNX_PATH)).graph
    got = {
        i.name: [d.dim_value for d in i.type.tensor_type.shape.dim] for i in graph.input
    }
    assert got["obj_trajs"] == [
        rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, rep.HISTORY_STEPS, rep.OBJ_FEATURE_DIM
    ]
    assert got["map_polylines"] == [
        rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, rep.POINTS_PER_POLYLINE, rep.MAP_FEATURE_DIM
    ]
    assert got["track_index_to_predict"] == [rep.TARGET_CAPACITY]


@needs_onnx
def test_output_names_and_shapes_match_the_frozen_contract():
    graph = onnx.load(str(ONNX_PATH)).graph
    got = {
        o.name: [d.dim_value for d in o.type.tensor_type.shape.dim] for o in graph.output
    }
    assert got["pred_scores"] == [rep.TARGET_CAPACITY, rep.NUM_MODES]
    assert got["pred_trajs"] == [
        rep.TARGET_CAPACITY, rep.NUM_MODES, rep.NUM_FUTURE_FRAMES, rep.TRAJ_CHANNELS
    ]


@needs_onnx
def test_index_inputs_are_int64_matching_the_engine_binding_contract():
    """DEVELOPING.md:22-23: both index inputs are int64 engine bindings."""
    graph = onnx.load(str(ONNX_PATH)).graph
    dtypes = {i.name: i.type.tensor_type.elem_type for i in graph.input}
    assert dtypes["track_index_to_predict"] == onnx.TensorProto.INT64
    assert dtypes["center_type_ids"] == onnx.TensorProto.INT64


@needs_onnx
def test_all_shapes_are_static():
    """contract_8_target.yaml pins every dimension; dynamic axes buy nothing."""
    graph = onnx.load(str(ONNX_PATH)).graph
    for tensor in list(graph.input) + list(graph.output):
        for dim in tensor.type.tensor_type.shape.dim:
            assert dim.dim_value > 0, f"{tensor.name} has a dynamic dimension"


@needs_onnx
def test_no_custom_operators_survive():
    model_proto = onnx.load(str(ONNX_PATH))
    domains = {node.domain for node in model_proto.graph.node}
    assert domains <= {"", "ai.onnx"}, f"custom domains present: {domains}"


@needs_onnx
def test_model_passes_onnx_checker():
    onnx.checker.check_model(onnx.load(str(ONNX_PATH)))
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
pytest tests/test_export_onnx.py -v
```

Expected: all skipped (no ONNX yet). This is the intended pre-export state.

- [ ] **Step 3: Write `export_onnx.py` and run the export**

Load the checkpoint, apply `ops_dense.patch_mtr`, build dummy inputs at exactly the contract shapes, and call `torch.onnx.export` with explicit `input_names` / `output_names` drawn from `contract_8_target.yaml` and **no** `dynamic_axes`.

```bash
python -m wato_mtr_eval.export_onnx \
  --checkpoint "$MTR_CHECKPOINT" \
  --cfg ~/mtr/tools/cfgs/waymo/mtr+100_percent_data.yaml \
  --contract ../config/contract_8_target.yaml \
  --out build/mtr.onnx
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
pytest tests/test_export_onnx.py -v
```

Expected: 6 passed. `test_no_custom_operators_survive` failing means `patch_mtr` missed a call site — return to Task 15 rather than working around it here.

- [ ] **Step 5: Write `engine_build.yaml`**

```yaml
---
# Engine build configuration. Paired with mtr.onnx as the portable artifact.
# The engine itself is a build product: never committed, never copied between
# machines. TensorRT engines are architecture-specific (3060 Ti is sm_86, the
# vehicle's 4080 is sm_89) and will not load across that boundary.
precision: fp16
workspace_size_mb: 4096
# Static shapes throughout: contract_8_target.yaml pins every dimension.
optimization_profiles: []
tactic_sources: [CUBLAS, CUBLAS_LT, CUDNN]
builder_optimization_level: 3
```

- [ ] **Step 6: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/export_onnx.py \
        src/world_modeling/prediction_ml/eval/tests/test_export_onnx.py \
        src/world_modeling/prediction_ml/config/engine_build.yaml
git commit -m "feat(prediction_ml): add ONNX export and engine build configuration"
```

---

### Task 17: Engine build, rung 3, and rung 4

Split by machine. 3-dev on the 3060 Ti catches categorical export failures; 3-final and rung 4 run on the vehicle's 4080. Because TensorRT selects kernels by per-architecture tactic autotuning, **fp16 numerics are not guaranteed identical across sm_86 and sm_89** — so the numerical parity gate, not just latency, passes on the 4080.

**Files:**
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/build_engine.py`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung3.py`
- Create: `src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung4.py`

**Interfaces:**
- Produces:
  - `build_engine.build(onnx_path, config_path, out_path) -> EngineInfo` with `sm_arch, trt_version, build_seconds`
  - `rung3.run(fixtures, engine, checkpoint, mtr_root) -> Rung3Report` with `paired_minfde`, `paired_mode_probability`, `top_mode_agreement`, `latency_ms_p50/p99`, `peak_memory_mb`
  - `rung4.run(fixtures, node_config) -> Rung4Report`

- [ ] **Step 1: Build the engine on the 3060 Ti (rung 3-dev)**

```bash
python -m wato_mtr_eval.build_engine \
  --onnx build/mtr.onnx --config ../config/engine_build.yaml --out build/mtr.engine
```

Expected: an engine builds with no unsupported-layer errors. If TensorRT reports unsupported ops, return to Task 15 — a plugin is the last resort, not the first.

- [ ] **Step 2: Validate the bindings against the C++ contract**

This is the first time `tensorrt_backend.cpp`'s binding-validation path meets a real engine; it has only ever run against the null backend.

```bash
colcon build --packages-select prediction_ml --cmake-args -DPREDICTION_ML_ENABLE_TENSORRT=ON
colcon test --packages-select prediction_ml --ctest-args -R test_backend
colcon test-result --verbose
```

Then load the engine through the node:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml \
  engine_path:=$(pwd)/build/mtr.engine metadata_path:=$(pwd)/../config/contract_8_target.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
```

Expected: configure succeeds, with every binding name, dtype and shape matching `MtrModelContract`. A mismatch here means the export and the frozen contract disagree — reconcile the contract before proceeding, per `DEVELOPING.md:49-56`.

- [ ] **Step 3: Run rung 3-dev — correctness only**

```bash
python -m wato_mtr_eval.rung3 --engine build/mtr.engine --fixtures ../test/fixtures \
  --checkpoint "$MTR_CHECKPOINT" --mtr-root ~/mtr --correctness-only
```

Report the paired minFDE delta, per-mode probability deltas, and top-mode agreement rate. **Record no latency or memory number from this machine.**

Aggregate mAP can conceal a single weird numerical failure mode, which is why per-sample trajectory and mode-probability deltas are reported rather than a summary statistic.

- [ ] **Step 4: Rebuild natively on the vehicle's 4080 (rung 3-final)**

On the vehicle, with `mtr.onnx` and `engine_build.yaml` copied across — **not** the engine:

```bash
python -m wato_mtr_eval.build_engine \
  --onnx mtr.onnx --config engine_build.yaml --out mtr.engine
python -m wato_mtr_eval.rung3 --engine mtr.engine --fixtures fixtures \
  --checkpoint "$MTR_CHECKPOINT" --mtr-root ~/mtr
```

Gate: ≥99% of agents under 1e-3 m paired Δ minFDE, plus latency p50/p99 and peak memory for a batch of 8 targets. These are the only performance numbers that enter any report.

- [ ] **Step 5: Run rung 4 — full runtime replay**

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml \
  engine_path:=$(pwd)/mtr.engine metadata_path:=$(pwd)/contract_8_target.yaml \
  mtr.mode:=tensorrt
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate

ros2 bag record -o rung4_out /world_modeling/world_object_seeds &
ros2 bag play fixtures/<scenario_id> --clock
```

Score the recorded `world_object_seeds` against ground truth at the deployed 3 s horizon and compare paired against rung 3-final. This delta isolates the TTL cache, map-frame conversion, detection association, timestamping, horizon downsampling and fallback takeover.

Set `mtr.pinned_target_ids` to the scenario's `tracks_to_predict` (Task 10) — without it the node runs its heuristic and this delta is unattributable.

- [ ] **Step 6: Write the full ladder report**

Assemble every rung into one table in `eval/README.md`: deployed minADE/minFDE per rung, the paired delta from the previous rung with mean/median/p90/p99/max and near-zero fraction, and the worst-10 agents with their ledger flags. Note explicitly which numbers came from the 4080.

- [ ] **Step 7: Commit**

```bash
git add src/world_modeling/prediction_ml/eval/wato_mtr_eval/build_engine.py \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung3.py \
        src/world_modeling/prediction_ml/eval/wato_mtr_eval/rung4.py \
        src/world_modeling/prediction_ml/eval/README.md \
        src/world_modeling/prediction_ml/DEVELOPING.md
git commit -m "feat(prediction_ml): add engine build, rung 3 and rung 4 runtime replay"
```

---

## Deferred

Out of scope, recorded so nobody rebuilds the reasoning:

- **Rung 4b — plausibility on real WATO bags** (drift, off-road rate, kinematic feasibility, MTR-vs-fallback takeover). The only test of the actual deployment domain, but it yields no attributable number and needs a working on-vehicle engine first.
- **Closing the map gap.** Extending SceneBuilder to pack road edges and crosswalks from `Lanelet.msg` is deliberately excluded — Task 14's sweep prices it first. Building the fix alongside the instrument that measures it leaves no way to know whether the fix worked.
- **Lanelet query calibration** against recorded-bag statistics. Recorded `LaneletAhead` can never be an oracle here (different map, different city, no shared frame); it can only supply distributions to tune the Waymo-side query toward.
- **TRT plugins** for the custom ops. Speculative until Task 17 measures 4080 latency.
- **Perception noise injection.** Confounds every rung at once and answers a different question. `waymo_to_ros.detections_at` is the hook if it is ever wanted.
- **MissRate**, **official Waymo mAP**, **a pybind11 live differential harness**, **the 5%/20% checkpoints**, **downloading more validation shards** — each rejected in the design spec's Scope section with reasoning.

