# Prediction quality harness

This package measures prediction quality at successive boundaries of the
`prediction_ml` pipeline. Its Python environments are intentionally split:
ROS conversion modules and PyTorch/MTR modules must not be installed together.

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

Result on 2026-07-26 after the full-validation Gate A hardening:
**63 passed**.

## Gate A — checkpoint validation

**Status: PASS / OPEN.**

The checkpoint was downloaded from
`Student-Xiaoji/LLM-Augmented-MTR-models/100_percent/MTR/checkpoint_epoch_30.pth`.
Its SHA256 is:

```text
f14858a1ce6b4098eef8230db39f014b337ef5e158b591e0febe722d7ae4667a
```

After building MTR's CUDA extensions and preprocessing all 150 Motion Dataset
v1.3.1 validation shards:

```bash
cd src/world_modeling/prediction_ml/eval
export MTR_ROOT=/root/mtr-task0
export MTR_CHECKPOINT=/root/mtr-task0/checkpoints/100_percent/MTR/checkpoint_epoch_30.pth
export MTR_PREPROCESSED_WAYMO=/root/mtr-task0/data/waymo/full_validation_v1_3_1
python -m wato_mtr_eval.rung0 \
  --shard /mnt/c/Users/rylei/waymo_motion_v1_3_1/validation \
  --checkpoint "$MTR_CHECKPOINT" \
  --mtr-root "$MTR_ROOT" \
  --preprocessed-data "$MTR_PREPROCESSED_WAYMO"
```

The gate-eligible full-validation report was:

```text
Rung 0 (golden) — 44097 scenarios, 192171 target agents

  official 8 s config  minADE 0.6055  (published 0.6046)
                       minFDE 1.2074  (published 1.2251)
  deployed 3 s  minADE 0.2901
                minFDE 0.5881

  per type (deployed):
    TYPE_CYCLIST minADE 0.3174  minFDE 0.6358
    TYPE_PEDESTRIAN minADE 0.1468  minFDE 0.3032
    TYPE_VEHICLE minADE 0.3075  minFDE 0.6228

  GATE A: PASS
```

The relative errors are 0.15% for minADE and 1.44% for minFDE, both below the
3% gate threshold. The earlier one-shard diagnostic (286 scenarios) was
intentionally ineligible because a single shard is too sensitive to shard
composition. Gate eligibility now requires one canonical, complete 150-shard
directory, no `--limit`, exact raw/preprocessed scenario-ID set equality, and
no duplicate scenario IDs.

## Gate B — C++ representation parity

**Task 12 status on 2026-07-26: PASS.** The CPU-only rung-2 gtest replayed
the committed MCAP fixtures through the production `SceneBuilder` and matched
the independent NumPy reference for all eight input tensors:

- dense / overflow: `262112a66742fe8d`;
- sparse: `923a0c6bb90f4127`; and
- gappy history: `9f62a2970e4e8d4c`.

Shapes and dtypes, masks, target indices, and type IDs matched exactly.
Unmasked object and map values matched within the contract tolerances, and all
padding remained zero. The three parameterized cases passed in 1.784 seconds.

Reconciliation against the initial plan pseudocode established these durable
rules:

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

Task 13 then forwarded both the independent reference bundle and the installed
C++ `scene_dump` output through the official checkpoint on CUDA:

```text
Rung 2 — 3 fixture scenarios, 9 accepted targets
Dropped without paired finite future: 0

all agents:                 9 / 9 below 1e-3 m delta minFDE (100%)
gap-free target history:   8 / 8 below 1e-3 m delta minFDE (100%)
masked target history:     1 / 1 below 1e-3 m delta minFDE (100%)
maximum R2 - R1 delta:     0.0000 m

GATE B: PASS
```

**Complete Gate B status: PASS / OPEN.** The clean-history requirement was
8/8, exceeding the required 99%, and no agent was dropped. “Clean” means the
selected target's own context row has all 11 history slots; gaps in neighboring
context rows do not move an agent into the masked cohort.

The acceleration-rule counterfactual remains deliberately deferred. Rung 1 and
rung 2 implement the same inherit-next-valid rule, so their zero delta proves
implementation parity but cannot compare that rule against official MTR's
alternative. That decision requires a separately generated counterfactual
bundle and must not be inferred from this gate.

## Map budget — raw Waymo information sensitivity

The rung-1 sweep ran on all 286 scenarios and 1,268 accepted targets in the
first validation shard, with zero targets dropped. It keeps WATO object
packing, map chunking/ranking, the 768-chunk budget, and the checkpoint fixed,
then removes one supported map axis at a time. Positive delta means the current
policy is worse than the preceding row.

| Policy | minADE (m) | minFDE (m) | Adjacent Δ minFDE (m) | Candidate chunks | Evicted target-chunks |
|---|---:|---:|---:|---:|---:|
| all supported, typed, radius | 0.5992 | 0.9138 | — | 211,459 | 154,027 |
| no driveways | 0.6086 | 0.9296 | +0.0158 | 209,013 | 150,986 |
| no speed bumps | 0.6088 | 0.9308 | +0.0013 | 208,482 | 149,539 |
| no road edges | 0.6515 | 1.0457 | +0.1149 | 160,106 | 51,506 |
| no crosswalks | 0.6502 | 1.0457 | -0.0000 | 159,095 | 50,093 |
| no road lines | 0.6562 | 1.0700 | +0.0243 | 106,500 | 744 |
| lane-only, collapsed types, radius | 0.6211 | 1.0542 | -0.0158 | 106,500 | 744 |
| lane-only, collapsed types, reachable approximation | 0.6282 | 1.0859 | +0.0316 | 74,580 | 0 |

Road edges carry the largest aggregate budget signal in this shard, followed
by the reachable-query approximation, road lines, and driveways. Speed bumps
and crosswalks are negligible in the aggregate table. Type collapse improves
the aggregate here, so retaining subtype tokens is not automatically beneficial
without retraining or broader validation. Per-agent tails are much larger than
the means; the CLI report includes paired percentiles and worst scenario/agent
IDs for each transition.

This is not a SceneBuilder-only price. It estimates MTR sensitivity to geometry
and subtype information retained from raw Waymo, so the deltas are upper bounds
on combined converter/schema/SceneBuilder work. Current fixture MCAPs contain
only untyped lane centerlines. A causal SceneBuilder-only experiment requires a
richer fixed `LaneletAhead` input before changing packing.

## Dense replacements for custom CUDA operators

Task 15 replaces pinned MTR's batch-local KNN and split attention weight/value
CUDA calls with ONNX-expressible PyTorch operations. Ragged batches, `-1`
sentinels, both attention implementations, and every runtime patch site are
covered. The real epoch-30 checkpoint matched end to end after patching.

On the sparse committed fixture (20 CUDA iterations), custom operators averaged
134.878 ms per forward and dense replacements averaged 122.727 ms (0.91x).
This is a development-GPU diagnostic, not the final TensorRT latency result.

## Static ONNX export

Task 16 exports the epoch-30 checkpoint through a tensor-only, static-shape
wrapper. The checked-in portable artifacts are the exporter and
`config/engine_build.yaml`; `build/mtr.onnx` is a 244 MB ignored build artifact.
The export gate passed all 13 focused checks:

- all eight inputs and both outputs match `contract_8_target.yaml`;
- every public dimension is static and both index bindings remain `int64`;
- no custom-domain, ONNX sequence, or loop operators survive;
- padded target slots receive zero-valued sentinel tokens without changing
  populated rows;
- the wrapper matches unmodified official MTR on a committed real fixture; and
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

## TensorRT rung 3 development gate

The development engine was built natively on the RTX 3060 Ti with TensorRT
10.13.2.6 and CUDA 12.8:

```bash
python -m wato_mtr_eval.build_engine \
  --onnx build/mtr.onnx \
  --config ../config/engine_build.yaml \
  --out build/mtr.engine
```

The FP16 build completed in 131.888 seconds for `sm_86`. TensorRT parsed the
entire graph without unsupported layers. The TensorRT-enabled C++ backend then
deserialized the real engine and validated all ten bindings against
`contract_8_target.yaml`.

Rung 3-dev is correctness-only; development-machine latency and memory are
intentionally omitted:

| Engine | targets below 1e-3 m paired delta minFDE | median delta | max absolute delta | top-mode agreement |
|---|---:|---:|---:|---:|
| FP16 deployment configuration | 1 / 9 (11.1%) | +0.0012 m | 1.1446 m | 100% |
| FP32 diagnostic | 7 / 9 (77.8%) | -0.0002 m | 0.0021 m | 100% |

The FP32 diagnostic isolates the large tail to unrestricted FP16 accumulation,
not an unsupported ONNX layer or binding mismatch. This is not the final gate:
the engine must be rebuilt on the vehicle's `sm_89` GPU, and only that native
build may supply final numerical, latency, or memory results.

Rung 3 emits a strict JSON per-agent ledger for rung 4:

```bash
python -m wato_mtr_eval.rung3 \
  --engine build/mtr.engine \
  --fixtures ../test/fixtures \
  --checkpoint "$MTR_CHECKPOINT" \
  --mtr-root "$MTR_ROOT" \
  --correctness-only \
  --report-json build/rung3_dev.json
```

## Rung 4 status

The rung-4 reader/scorer is implemented and unit tested, but the live replay
gate remains blocked until the vehicle run and one runtime ownership issue are
resolved. Fixture target pins include Waymo's SDC as `__ego__`, while
`MtrRuntime::selectOutput` currently attaches cached predictions only to
objects already present in the fallback detection array. The converter
deliberately omits a duplicate raw SDC detection, so `__ego__` cannot appear on
`world_object_seeds`. Rung 4 fails loudly on this missing pin instead of
silently weakening the paired comparison.

The final ladder therefore remains:

| Rung | Status | Result |
|---|---|---|
| R0 official checkpoint | PASS | deployed minADE 0.2901 m, minFDE 0.5881 m |
| R1 NumPy representation | measured | map-policy sweep recorded above |
| R2 C++ representation | PASS | 9/9 below 1e-3 m paired delta minFDE |
| R3-dev TensorRT (`sm_86`) | diagnostic FAIL | FP16 1/9 below 1e-3 m; no performance numbers |
| R3-final TensorRT (`sm_89`) | PENDING | rebuild and measure on vehicle |
| R4 full ROS replay | BLOCKED | requires R3-final and `__ego__` output ownership decision |
