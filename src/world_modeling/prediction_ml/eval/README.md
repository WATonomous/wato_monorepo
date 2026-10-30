# MTR engine and parity fixtures

Python tooling for `prediction_ml`: it rebuilds the Waymo replay fixtures used
by the C++ parity test and exports the MTR checkpoint to the static ONNX graph
and TensorRT engine the node loads. Its Python environments are intentionally
split: ROS conversion modules and PyTorch/MTR modules must not be installed
together.

## WSL model test environment

The real-model tests run in the existing Ubuntu 22.04 WSL2 MTR environment:

- Python 3.10.12;
- official MTR checkout `sshaoshuai/MTR@a5ba7bd`;
- PyTorch `2.11.0+cu128` and the compiled MTR CUDA extensions;
- Waymo Open Dataset `1.6.5` for TensorFlow 2.12;
- RTX 3060 Ti; and
- isolated test venv `/root/mtr-harness-venv`, created with
  `virtualenv --system-site-packages` so it reuses the compiled CUDA stack.

The complete suite, including real checkpoint loading, runs with:

```bash
MTR_ROOT=/root/mtr-task0 \
MTR_CHECKPOINT=/root/mtr-task0/checkpoints/100_percent/MTR/checkpoint_epoch_30.pth \
/root/mtr-harness-venv/bin/python -m pytest \
  /mnt/c/Users/rylei/github/wato_monorepo/src/world_modeling/prediction_ml/eval/tests -v
```

## Gate B — C++ representation parity

The CPU-only `test_scene_builder_parity` gtest replays the MCAP fixtures
through the production `SceneBuilder` and compares all eight input tensors
with the independent NumPy reference (`reference_pack.py`):

- dense / overflow: `262112a66742fe8d`;
- sparse: `923a0c6bb90f4127`; and
- gappy history: `9f62a2970e4e8d4c`.

These fixtures are derived from the Waymo Open Dataset, whose terms allow
redistribution only to registered WOD users, so they are no longer committed.
Build them from a local copy of validation shard 0, then rerun the tests (they
skip while the fixtures are absent):

```bash
eval/regenerate_fixtures.sh /path/to/uncompressed_scenario_validation_validation.tfrecord-00000-of-00150
```

Shapes, dtypes, masks, target indices, and type IDs must match exactly;
unmasked values within the contract tolerances; padding must stay zero.
Replay rules both sides follow:

- replay stops at Waymo's `current_time_index` but calls `build()` at every
  earlier tracks callback to accumulate ego history;
- fixture records are ordered pose, odometry, lanelet, then tracks, and use a
  constant positive one-second ROS transport epoch;
- Waymo's SDC maps to the single synthetic `__ego__` context row, with explicit
  raw-to-packed target identity sidecars for ground-truth joins;
- selected targets are mandatory members of the 128-row context budget;
- context ranks retained histories by their latest retained sample; and
- map directions are computed before chunking, while chunks are ranked per
  target at the 30 m forward offset.

## Dense replacements for custom CUDA operators

`ops_dense` replaces pinned MTR's batch-local KNN and split attention weight/value
CUDA calls with ONNX-expressible PyTorch operations. Ragged batches, `-1`
sentinels, both attention implementations, and every runtime patch site are
covered. The real epoch-30 checkpoint matched end to end after patching.

## Static ONNX export

`export_onnx` exports the epoch-30 checkpoint through a tensor-only, static-shape
wrapper. The checked-in portable artifacts are the exporter and
`config/engine_build.yaml`; `build/mtr.onnx` is a 244 MB ignored build artifact.
The export tests check that:

- all eight inputs and both outputs match `contract_8_target.yaml`;
- every public dimension is static and both index bindings remain `int64`;
- no custom-domain, ONNX sequence, or loop operators survive;
- padded target slots receive zero-valued sentinel tokens without changing
  populated rows;
- the wrapper matches unmodified official MTR on a replay fixture; and
- ONNX Runtime matches the patched PyTorch graph within the configured
  tolerances.

Regenerate the portable model with:

```bash
python -m wato_mtr_eval.export_onnx \
  --checkpoint "$MTR_CHECKPOINT" \
  --cfg "$MTR_ROOT/tools/cfgs/waymo/mtr+100_percent_data.yaml" \
  --contract ../config/contract_8_target.yaml \
  --out build/mtr.onnx
```

## TensorRT engine

The development engine was built natively on the RTX 3060 Ti with TensorRT
10.13.2.6 and CUDA 12.8:

```bash
python -m wato_mtr_eval.build_engine \
  --onnx build/mtr.onnx \
  --config ../config/engine_build.yaml \
  --out build/mtr.engine
```

TensorRT parsed the entire graph without unsupported layers. The
TensorRT-enabled C++ backend then deserialized the real engine and validated
all ten bindings against `contract_8_target.yaml`.

`engine_build.yaml` builds FP32 with TF32 disabled: on the 3 replay fixtures it
matched PyTorch for 9/9 targets (below 1e-3 m minFDE delta), where TF32 (7/9)
and FP16 (1/9) did not. Engines are architecture-specific: rebuild natively on
the vehicle's `sm_89` GPU and measure latency and memory there.
