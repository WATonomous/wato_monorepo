# Prediction ML (MTR) Skeleton — Design

**Date:** 2026-06-24
**Author:** Ryan (Prediction Lead, World Modeling)
**Status:** Approved design — pending implementation plan

## Goal

Stand up a new `prediction_ml` ROS 2 package as the skeleton for shifting WATO
prediction from hand-tuned physics to an ML approach (MTR — Motion Transformer).
The skeleton must let **3 people work in disjoint files with no merge conflicts**,
compile green from day one, and run as a safe constant-velocity fallback until the
ML path is filled in.

## Background: what is actually deployed (verified 2026-06-24)

Two prediction packages exist in the monorepo; only the simple one runs:

- `src/world_modeling/prediction/` (on `main`): elaborate physics/lanelet/intent
  predictor (~150 params). **Not deployed** — its node is commented out in
  `world_modeling_bringup/launch/world_modeling.launch.yaml`. It misbehaves when
  input tracking yaw "spins," because it builds per-object position history and
  lanelet matching that assume a stable heading.
- `src/world_modeling/simple_prediction/` (on `origin/current_main`, not merged to
  `main`): the **actually deployed** predictor. A ~150-line constant-velocity stub
  that reads each detection's yaw per frame (no history) and extrapolates a single
  straight line. Robust to spinning yaw.

The new ML node targets the **same I/O contract** so it is a drop-in replacement for
`simple_prediction`, and embeds that same CV logic as its always-safe fallback.

### Shared I/O contract

- **Input:** topic `tracks_3d` → `vision_msgs/Detection3DArray`
  (`/perception/tracked_detections_3d` on car, `/carla/tracked_detections_3d` in sim).
  Each `Detection3D`: `id`, `bbox` (center pose + size), `results[].hypothesis.class_id`.
- **Output:** topic `world_object_seeds` → `world_model_msgs/WorldObjectArray`.
  `WorldObject` = `detection` + `predictions[]`; `Prediction` = `header` + `conf` (float)
  + `poses` (`geometry_msgs/PoseStamped[]`). Consumed by world_model pre-enrichment.
- **Optional map context (for MTR only):** `ego_pose` (`geometry_msgs/PoseStamped`),
  `lanelet_ahead` (`lanelet_msgs/LaneletAhead`).

### MTR reference (official `github.com/sshaoshuai/MTR`, NeurIPS 2022)

Transformer multimodal motion predictor; GMM head, K≈6 modes, agent types
vehicle/pedestrian/cyclist. **Input tensor keys:** `obj_trajs`, `obj_trajs_mask`,
`obj_trajs_last_pos`, `track_index_to_predict`, `center_objects_type`, `map_polylines`,
`map_polylines_mask`, `map_polylines_center`. **Output keys:** `pred_scores`,
`pred_trajs`. There is **no official ONNX/TensorRT export**, so the TRT backend is
custom work — the most self-contained chunk, owned by one person.

## Design principles (from `action_plnan(1).md`)

- Fallback prediction stays synchronous and publishable every tick.
- MTR runs in the background and never blocks the tick.
- Cached MTR replaces fallback only when fresh, valid, and state-consistent.
- Disabled or unavailable MTR behaves exactly like fallback-only prediction.
- GPU/TensorRT code is isolated behind a compile flag so the package builds on
  CPU-only machines (CI, laptops) using the null backend.

## Architecture

A single lifecycle node (`prediction_ml_node`) owns three collaborators that meet only
at two frozen headers:

```
tracks_3d ─┐
ego_pose ──┼─▶ SceneBuilder ──MtrInputTensors──▶ MtrRuntime ──▶ IMtrInferenceEngine
lanelet ───┘   (Person A)        + sidecar       (Person C)        (Person B)
                                                     │  ▲ MtrOutputTensors
                                                     ▼  │
                                              OutputConverter (Person C)
                                                     │
                              selectOutput(fallback, cached_mtr, now)
                                                     │
                                                     ▼
                                          world_object_seeds
```

- **Fallback path (always):** node builds the CV straight-line prediction every tick
  and publishes it immediately.
- **ML path (async):** `MtrRuntime` submits the latest frame to a background worker
  running the inference engine; results are cached per-object with a TTL.
  `selectOutput` starts from the fallback and replaces only objects that have a fresh,
  valid cached MTR prediction.

## File layout & ownership

```
src/world_modeling/prediction_ml/
├── include/prediction_ml/
│   ├── mtr_types.hpp              # SHARED — frozen day 1, then read-only
│   ├── mtr_inference_engine.hpp   # SHARED — IMtrInferenceEngine interface
│   ├── scene_builder.hpp          # Person A
│   ├── mtr_backend.hpp            # Person B (factory decls for null + trt engines)
│   ├── output_converter.hpp       # Person C
│   ├── mtr_runtime.hpp            # Person C
│   └── prediction_ml_node.hpp     # Person C
├── src/
│   ├── scene_builder.cpp          # Person A
│   ├── null_backend.cpp           # Person B  (always built)
│   ├── tensorrt_backend.cpp       # Person B  (behind PREDICTION_ML_ENABLE_TENSORRT)
│   ├── output_converter.cpp       # Person C
│   ├── mtr_runtime.cpp            # Person C
│   └── prediction_ml_node.cpp     # Person C  (+ main)
├── config/params.yaml             # Person C owns header; A/B append own blocks
├── launch/prediction_ml.launch.yaml
├── CMakeLists.txt                 # Person C owns; A/B add their .cpp at marked lines
├── package.xml
├── README.md
└── test/
    ├── test_scene_builder.cpp     # Person A
    ├── test_backend.cpp           # Person B
    └── test_runtime.cpp           # Person C
```

### The two shared headers (the only cross-person coupling)

`mtr_types.hpp` defines plain structs with no behavior, plus the free functions that
parse/load them (declarations only; bodies live in their owner's `.cpp`). Symbol names
are kept identical to `action_plnan(1).md` so the team can cross-reference:

- `MtrConfig`, `MtrMode` (Disabled / Null / TensorRT) — runtime configuration;
  `parseMtrMode(std::string)` and `loadMtrConfig(node)` build it from ROS params.
- `MtrModelContract`, `MtrTensorSpec` — binding names, dtypes, shapes;
  `loadMtrModelContract(metadata_path)` and `validateMtrModelContract(...)` load/validate.
- `MtrFrameContext` — the per-tick bundle (detections + ego pose + map context +
  timestamp) handed to the scene builder.
- `MtrInputTensors` — the packed `obj_trajs*` / `map_polylines*` / `track_index_to_predict`
  / `center_objects_type` buffers (flat `std::vector<float>` + shape metadata).
- `MtrBatchSidecar`, `MtrTargetSidecar` — index↔detection-id map and map-frame pose
  needed to convert outputs back.
- `MtrOutputTensors` — raw `pred_scores` / `pred_trajs` buffers + shapes.
- `MtrObjectPrediction`, `MtrInferenceResult` — per-object converted result + status.

`mtr_inference_engine.hpp` defines the one interface both backends implement:

```cpp
class IMtrInferenceEngine {
 public:
  virtual ~IMtrInferenceEngine() = default;
  virtual bool ready() const = 0;
  virtual std::string lastError() const = 0;
  virtual MtrOutputTensors infer(const MtrInputTensors & input) = 0;
};
std::unique_ptr<IMtrInferenceEngine> createNullMtrInferenceEngine(const MtrConfig &);
std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig &);
```

These two headers are written and agreed **first**. After that, each person edits only
their own files; nobody touches another's `.cpp`.

### Ownership by pipeline stage

- **Person A — Input / Scene → Tensor** (`scene_builder.*`, `test_scene_builder.cpp`):
  maintain MTR-only per-object history keyed by detection id (pose, dims, heading,
  velocity, type, timestamp, validity); resample to a fixed step and mask missing
  samples; select target agents up to a limit; pack target + context agents into
  `obj_trajs*` and `track_index_to_predict` / `center_objects_type`; convert lanelet
  context into `map_polylines*`; populate the sidecar. Depends only on `mtr_types.hpp`.

- **Person B — Inference backend** (`null_backend.cpp`, `tensorrt_backend.cpp`,
  `mtr_backend.hpp`, `test_backend.cpp`): implement `IMtrInferenceEngine`. Null engine
  is always built and returns an empty/invalid result so the runtime falls back.
  TensorRT engine loads the `.engine` from `engine_path`, validates bindings against
  `MtrModelContract`, runs `infer`. Compiled only when `PREDICTION_ML_ENABLE_TENSORRT`
  is set. Depends only on the two shared headers — can develop against the null engine
  with zero input from A or C.

- **Person C — Orchestration / Output** (`prediction_ml_node.*`, `mtr_runtime.*`,
  `output_converter.*`, `config`, `launch`, `CMakeLists.txt`, `package.xml`,
  `test_runtime.cpp`): the lifecycle node (subs/pubs/params/wiring); the CV fallback
  predictor (ported from `simple_prediction`); `MtrRuntime` (latest-only async worker,
  per-object TTL cache, `submitFrame`, `selectOutput`, `ready`/`lastError`);
  `OutputConverter` (validate `pred_scores`/`pred_trajs`, rotate/translate target frame
  → map frame, infer yaw from consecutive points, emit `world_model_msgs/Prediction`).
  Owns the build/config/launch integration spine.

## Build & configuration

- `CMakeLists.txt`: always builds `scene_builder.cpp`, `null_backend.cpp`,
  `output_converter.cpp`, `mtr_runtime.cpp`, `prediction_ml_node.cpp` into a lib +
  executable. `tensorrt_backend.cpp` is added only under
  `option(PREDICTION_ML_ENABLE_TENSORRT)`. A/B add their files at clearly commented lines
  so edits don't collide.
- **Link safety (CPU-only build).** Everything referenced by always-built code must have
  an always-built definition. So Person B's always-built `null_backend.cpp` provides:
  `createNullMtrInferenceEngine`, `NullMtrInferenceEngine`, and — guarded by
  `#ifndef PREDICTION_ML_ENABLE_TENSORRT` — a stub `createTensorRtMtrInferenceEngine` that
  returns the null engine. `tensorrt_backend.cpp` provides the real
  `createTensorRtMtrInferenceEngine` plus `loadMtrModelContract` / `validateMtrModelContract`,
  which are **only referenced from the TensorRT path**, so their definitions can live in the
  gated TU without breaking the CPU build.
- `params.yaml`: top-level node params (horizon, time_step) plus an `mtr:` group
  (`mode`, `engine_path`, `metadata_path`, `cache_ttl_s`, `selected_target_agent_limit`,
  `history_steps`, `history_rate`). Person C owns the file skeleton; A and B append their
  own clearly-delimited sub-blocks.
- `launch/prediction_ml.launch.yaml`: mirrors `simple_prediction.launch.yaml` remaps
  (`tracks_3d` → tracked detections, `world_object_seeds` → `/world_modeling/...`) plus
  optional `ego_pose` / `lanelet_ahead` remaps, with `engine_path` / `metadata_path`
  injectable at launch (perception asset-path pattern).

## Error handling

- Disabled mode or a not-ready engine ⇒ pure fallback output; node never errors.
- `infer` failures and contract-validation failures are caught in the async worker,
  surfaced via `lastError()`, and never propagate to the tick.
- Invalid `pred_scores`/`pred_trajs` (NaN, wrong shape, low score) are dropped per-object
  in `OutputConverter`; that object keeps its fallback prediction.
- Stale cache entries (age > `cache_ttl_s`) are ignored by `selectOutput`.

## Testing

One test file per person, runnable independently:
- `test_scene_builder.cpp`: feed synthetic `Detection3DArray` sequences, assert
  `MtrInputTensors` shapes/masks/sidecar mapping.
- `test_backend.cpp`: null engine returns not-ready/invalid; (when TRT enabled) contract
  validation accepts/rejects known-good/bad bindings.
- `test_runtime.cpp`: `selectOutput` returns fallback when no cache; replaces only fresh
  valid objects; respects TTL; never blocks.

## Mapping to `action_plnan(1).md` work packages

The action plan's 6 work packages are preserved one-to-one; this design only re-homes
them into owner-scoped files. Every named surface symbol is kept verbatim.

| Action-plan WP | Surface symbols | Owner | File(s) |
|---|---|---|---|
| WP1 Plumbing & repo setup | `MtrConfig`, `parseMtrMode`, `loadMtrConfig`, `MtrRuntime`, `createNullMtrInferenceEngine`, `createTensorRtMtrInferenceEngine` | C | `prediction_ml_node.*`, `CMakeLists.txt`, `config`, `launch` |
| WP2 Metadata & contract validation | `MtrTensorSpec`, `MtrModelContract`, `loadMtrModelContract`, `validateMtrModelContract` | B | `mtr_backend.hpp`, `tensorrt_backend.cpp` (structs in shared `mtr_types.hpp`) |
| WP3 Runtime skeleton & disabled path | `IMtrInferenceEngine`, `NullMtrInferenceEngine`, `MtrInferenceResult`, `MtrRuntime::submitFrame/selectOutput/ready/lastError` | B (null engine) + C (runtime no-op) | `null_backend.cpp` (B), `mtr_runtime.*` (C) |
| WP4 Scene history & tensor building | `MtrFrameContext`, `MtrInputTensors`, `MtrBatchSidecar`, `MtrTargetSidecar` | A | `scene_builder.*` |
| WP5 TensorRT backend | `IMtrInferenceEngine`, `MtrInputTensors`, `MtrOutputTensors`, `MtrInferenceResult`, `createTensorRtMtrInferenceEngine` | B | `tensorrt_backend.cpp` |
| WP6 Async cache, conversion & merge | `MtrOutputTensors`, `MtrObjectPrediction`, `MtrRuntime::selectOutput` | C | `mtr_runtime.*`, `output_converter.*` |

## Differences from `action_plnan(1).md` (and why)

1. **New `prediction_ml` package, not extending `prediction`.** The plan put MTR inside
   the existing `prediction` node (`include/prediction/mtr.hpp`). Verification showed that
   node is *not deployed* (commented out in bringup); the live predictor is the separate
   `simple_prediction` package. Tying MTR to dead code adds 150 params of baggage and no
   reuse benefit, so MTR gets a clean package. Architecture (async backend + sync fallback)
   is unchanged.
2. **Fallback = CV logic ported from `simple_prediction`, not the heavy lanelet predictor.**
   The plan's "existing fallback predictor" was the heavy `prediction` path; since that
   isn't what runs, the always-safe path is the trivial constant-velocity logic that *is*
   deployed. Behaviourally this is what the plan wanted (a synchronous, always-publishable
   fallback) — just sourced from the real one.
3. **Files split per owner instead of 3 monolithic files.** The plan's `mtr.hpp` /
   `mtr.cpp` / `tensorrt_mtr.cpp` would put all three people in `mtr.cpp` at once. Same
   symbols, more files, drawn on the pipeline seam so the three owners never touch the same
   `.cpp`. This is the explicit reason the skeleton exists.
4. **`selectOutput(fallback, now)` → `selectOutput(fallback, cached_mtr, now)`.** Same
   semantics; the cache is passed explicitly rather than read from a member, which keeps
   the merge unit-testable in isolation (Person C's `test_runtime.cpp`).
5. **CMake flag renamed `PREDICTION_ENABLE_TENSORRT` → `PREDICTION_ML_ENABLE_TENSORRT`.**
   Cosmetic, to match the new package name. Identical behaviour (always build null +
   `scene_builder` + node; gate `tensorrt_backend.cpp`).
6. **Added per-owner test files.** The plan didn't specify tests; one test file per person
   keeps test edits conflict-free too.

Everything else — the optional async backend, never blocking the tick, contract
validation, TensorRT isolation, latest-only async worker, per-object TTL cache,
target→map frame conversion, and the disabled-path-equals-fallback guarantee — follows the
plan directly. (The plan's note "vehicle runtime uses TensorRT plus metadata" is honored:
the TensorRT engine + `MtrModelContract` metadata is the vehicle/agent runtime; pedestrians
and cyclists ride the same MTR batched output.)

## Out of scope (YAGNI for the skeleton)

- Actual MTR training, ONNX export pipeline, and a real `.engine` asset.
- Interaction-aware / multi-agent joint prediction beyond MTR's batched per-target output.
- Replacing `simple_prediction` in the deployed bringup launch (separate follow-up once
  the ML path is validated).

## Acceptance criteria for the skeleton

1. `colcon build --packages-select prediction_ml` succeeds CPU-only (TensorRT off).
2. `prediction_ml_node` runs and publishes valid `world_object_seeds` using only the CV
   fallback (MTR disabled), matching `simple_prediction` behavior.
3. The three owners' files are disjoint; only `mtr_types.hpp` and
   `mtr_inference_engine.hpp` are shared and are frozen first.
4. Each `test_*.cpp` builds and passes against the stubs.
