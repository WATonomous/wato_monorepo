# Prediction Quality Harness — Design

**Date:** 2026-07-26
**Branch:** `codex/mtr-lifecycle`
**Package:** `src/world_modeling/prediction_ml`

## Problem

`prediction_ml` is fully scaffolded — SceneBuilder, null + TensorRT backends, async runtime with
TTL cache, OutputConverter, lifecycle node, CV fallback — and every existing test is a *plumbing*
test. Nothing measures whether a predicted trajectory is any good.

Scoring MTR in Python on Waymo answers "is the model good," and that is already known. The open
question on this branch is different:

> **How much prediction quality does our integration destroy, and which component destroys it?**

The harness must therefore produce not one number but a ladder of attributable deltas.

## Findings that shaped this design

1. **A usable checkpoint exists.** No official MTR weights are published — Waymo challenge terms
   prohibit redistributing challenge-trained weights, which is why `sshaoshuai/MTR` ships code only.
   But `Student-Xiaoji/LLM-Augmented-MTR-models` publishes
   `100_percent/MTR/checkpoint_epoch_30.pth` (777 MB, MIT-labeled), whose own eval record reports
   mAP 0.4182 against the official README's 0.4164 — a faithful MTR reproduction. Training our own
   on a 3060 Ti was the alternative and is infeasible (config is batch-10-per-GPU × 8 GPUs × 30
   epochs on ~487k scenarios; 8 GB VRAM forces batch 2–4; multi-week at 20% data, months at 100%).
   *Risk:* the weights are Waymo-derived and the MIT label is the uploader's assertion. Fine for
   internal eval; needs a license read before anything ships on-vehicle.

2. **MTR ships two custom CUDA extensions.** `setup.py` compiles `knn_cuda` and `attention_cuda`
   (sparse/local attention, separate weight- and value-computation kernels). Neither is
   ONNX-exportable. This is the largest single work item and almost certainly why
   `tensorrt_backend.cpp` has never had an engine to load.

3. **The map representation collapses 20 types into 1.** `scene_builder.cpp:939` packs map rows as
   `[x, y, z, dir_x, dir_y, dir_z, global_type, pre_x, pre_y]`, and line 50 defines a single
   *"generic lane-centerline global_type token"*. MTR trained on Waymo maps with ~20 polyline types
   (road edges, crosswalks, stop signs, speed bumps, lane boundaries). `lanelet_ahead` supplies
   centerlines only. Silent quality loss with no error surfaced.

4. **The deployed horizon is 3 s, not 8 s.** `output_converter.cpp:135-148` computes
   `emitted_steps = min(model_steps/model_stride, requested_steps)` = `min(80/2, 3.0/0.2)` = **15
   waypoints over 3.0 s**. Metrics scored at MTR's native 8 s are not comparable to anything
   measured downstream of the node.

5. **Target selection has no Waymo analogue.** Waymo scores exactly `tracks_to_predict`.
   SceneBuilder picks its own 8 by forward-hemisphere + distance heuristic. Left unpinned, the
   C++-vs-Python delta silently absorbs "we chose different agents."

## The ladder

Each rung's delta from the prior rung names exactly one culprit.

| Rung | Pipeline | Delta isolates | Runs on |
|---|---|---|---|
| **0** Golden | Waymo → official Python preprocessing → PyTorch MTR | *(anchor)* eval-environment correctness | any GPU |
| **1** Representation | Official tensors degraded to the WATO representation → PyTorch MTR | Cost of our input representation | any GPU |
| **2** Parity | Waymo → converter → mcap → **C++ SceneBuilder** → tensors → PyTorch MTR | C++ implementation bugs, nothing else | CI + any GPU |
| **3-pre** Op swap | Custom CUDA ops replaced by ONNX-expressible equivalents, still PyTorch | Correctness of the op replacement | 3060 Ti |
| **3-dev** Export | ONNX → engine, correctness only | Export/op-coverage/shape failures | 3060 Ti |
| **3-final** Engine | Same C++ tensors → TRT engine | fp16/tactic loss + latency + memory | **4080** |
| **4** Runtime | mcap → ROS node → SceneBuilder → TRT → OutputConverter → `world_object_seeds` | TTL cache, transforms, association, timestamps, downsampling, fallback takeover | **4080** |

Rung 3-pre exists because without it a rung-3 discrepancy has three candidate causes (bad op
replacement, bad export, fp16 loss) and no way to separate them. Prove the dense-attention
equivalent correct in PyTorch, with a debugger attached, before ONNX enters the picture.

## Components

### Converter (keystone)

Waymo Scenario proto → **one mcap bag per scenario**, at 10 Hz across all 91 steps.

| Waymo source | ROS message | Topic |
|---|---|---|
| `tracks[]` @ step t | `vision_msgs/Detection3DArray` | `/perception/detections_3D_tracked` |
| `tracks[sdc_track_index]` | `geometry_msgs/PoseStamped` | `/localization/pose` |
| SDC velocity, rotated to body frame | `nav_msgs/Odometry` (`child_frame_id: base_footprint`) | `/world_modeling/transform/odometry` |
| `map_features[]`, lanes only | `lanelet_msgs/LaneletAhead` | `lanelet/lanelet_ahead` |

mcap is chosen so **one artifact feeds both rung 2 (rosbag2 C++ read) and rung 4
(`ros2 bag play`)**, with no custom serialization schema and Python/C++ agreeing by construction.

Mappings pinned against actual code:

- **Type** → `results[].hypothesis.class_id` string. `scene_builder.cpp:238-246` matches
  `vehicle|car|truck|bus`, `pedestrian|person`, `cyclist|bicycle|bike`. Waymo's three types map
  1:1. `TYPE_OTHER`/`TYPE_UNSET` have no mapping, so `parseObjectClass` returns `nullopt` and the
  sample is dropped — correct, but the converter must **count and report** drops.
- **Velocity** uses an unusual encoding that must be replicated exactly: `scene_builder.cpp:118-127`
  reads the `results[]` entry whose `class_id == "linear_velocity"` and takes vx/vy/vz from that
  hypothesis's **pose position**. Waymo has no `velocity_z` → 0.
- **Ego twist** must be body-frame; SceneBuilder rotates it *into* map using `ego_pose`, so the
  converter applies the inverse rotation. That pair round-trips — a clean unit test.
- **IDs**: Waymo track int → stable decimal string. SceneBuilder keys history by `detection.id`,
  so cross-frame stability is mandatory.

**Lanelet query strategy — parameterized.** `LaneletAhead` is a query result, not a map: *"legally
reachable lanelets ahead of ego within a configurable radius… BFS through the routing graph."*
Implement radius-only as the deterministic upper-context baseline, plus an optional
Waymo-connectivity traversal as an explicit *approximation* of the forward-reachable query — not a
claimed Lanelet2 replica. The delta between them prices the query's aggressiveness.

Recorded real `LaneletAhead` can **not** later become an oracle: it queries the Ontario lanelet map,
while Waymo scenarios are in different cities with their own geometry and their own ground-truth
futures. There is no shared frame or road network. Recorded output is useful only for
**distributional calibration** — lanelets per query, centerline points per lanelet, effective
radius, branching factor — which the Waymo-side query can then be tuned to match.

### Representation spec

The information contract that rungs 1 and 2 must implement identically, or their delta means
nothing: history depth/rate, context and target budgets, centerline-only map, the single
`global_type` collapse, the 768×20 polyline budget, the 29 `obj_trajs` columns, and traffic-light
handling (to be **enumerated, not assumed** — the packed 9-column map row has no light channel, so
signal state is dropped at the tensor level regardless).

Delivered as `REPRESENTATION.md` plus a Python constants module — **not** a shared YAML. Only the
Python side reads it programmatically; C++ implements it implicitly through SceneBuilder. A YAML
pretending to be a machine-shared contract would be ceremony.

### Metrics

**Primary, every run:** deployed **3 s @ 0.2 s** (15 waypoints, 6 modes) — `minADE₆`, `minFDE₆`,
broken out per type (vehicle / pedestrian / cyclist). Per-type is near-free and catches the failure
mode where a pedestrian regression hides inside a vehicle-dominated average.

**One-time only:** native 8 s @ 0.1 s, to anchor rung 0 against the published 0.6046 / 1.2251.
Anchor on minADE/minFDE only — those are unambiguous — targeting ~2–3% agreement, not
bit-exactness, since reproducing the published figures requires replicating Waymo's per-type and
per-bucket averaging protocol. Once rung 0 passes, the 8 s metric is not recomputed per run.

**Target pinning.** For all ladder runs, SceneBuilder's targets are pinned to Waymo's
`tracks_to_predict` so R1→R2 measures only packing. The heuristic's *would-have-picked* set is
recorded alongside, making the target-selection ablation a free byproduct of every run.

Mechanism: a real config parameter, `mtr.pinned_target_ids` (empty list = use the heuristic, which
is the production default). A test-only seam was considered and rejected because rung 4 replays the
**full lifecycle node**, which reads config and would otherwise still run its heuristic — leaving
rung 4's delta unattributable, defeating the purpose of the last rung. The cost is one production
parameter that only evaluation sets; that is cheaper than an unattributable rung 4.

### Paired delta distribution

Rung-to-rung comparison is **paired** — the same scenarios and the same target agents through both
pipelines — so the per-agent difference is directly available. Reporting only the difference of
aggregates discards nearly all of that signal, and it is the difference that matters here: a mean
Δ minFDE of 0.01 m could be 1268 agents each at 0.01 (uniform float precision loss, benign) or 1200
agents at 0.00 and 68 at 0.19 (one code path broken). Those demand opposite responses, and no
aggregate can tell them apart.

So for every rung transition, record the **distribution of paired per-agent deltas**, not another
formal model metric:

```
R1 → R2   Δ minFDE (m), paired over 1268 target agents
  mean      +0.004
  median    +0.000
  p90       +0.001
  p99       +0.087
  max       +0.412
  |Δ| < 1e-3:  1194 / 1268  (94.2%)

  worst 10 scenarios:
    scenario_id            agent_id   Δ minFDE   ledger flag
    a3f2...c81             1042       +0.412     context overflow (141 → 128)
    b7e1...09d              318       +0.388     history masked 6/11
    ...
```

Three properties this buys:

- **The worst-N list is the debugging entry point.** Each row is a directly reproducible case —
  scenario id plus agent id — rather than a number to go hunting behind.
- **Cross-referencing worst-N against the loss ledger usually names the cause immediately.** If the
  worst rows all carry `context overflow`, the bug is in ranking or truncation, not in packing.
- **The CI gate becomes a shape check, not a threshold on a mean.** "≥99% of agents have
  |Δ minFDE| < 1e-3" is far sharper than "mean Δ < 0.01", which a small number of catastrophic
  outliers can pass unnoticed.

Applies to every transition where elementwise tensor comparison is unavailable — R0→R1, R1→R2 at
the metric level, R3-final, and R4. Rung 2's tensor gtest remains a separate exact/near-exact
check.

### Loss ledger

Emitted per scenario alongside metrics. Every drop point already exists in code, so instrumenting
is cheap; the value is turning "the metric moved" into "and here is exactly what information
disappeared."

```
AGENTS   waymo tracks 54 → type-mapped 48 (6 dropped: TYPE_OTHER)
         → context capacity 128, used 48, overflow 0
TARGETS  candidates 48 → tracks_to_predict 8 → selected 8 (pinned)
         [heuristic would have selected 5/8 of these]
HISTORY  11 slots × 48 tracks = 528 → valid 491, masked 37 (7.0%)
MAP      radius 80 m; nearby lanes 37 → radius_only 37 → reachable 16
         centerline points 611 → 284
         polyline capacity 768, used 16, overflow 0
         omitted semantics: road_edge 18, road_line 24, crosswalk 3, stop_sign 1
         global_type: 20 Waymo types → 1 WATO token
OUTPUT   80 steps @ 0.1 s → stride 2 → 40 → horizon clamp → 15 @ 0.2 s
```

Scoped to drop points that **can change** (budgets, filters, policies). Static facts belong in
`REPRESENTATION.md`, not in per-run output.

### Rung-2 parity mechanics

Two consumers of SceneBuilder:
- **CI gtest** (`test_scene_builder_parity.cpp`) links SceneBuilder directly, replays the mcap,
  compares in-process. No GPU, no Python, no subprocess.
- **`scene_dump` CLI** (dev-only, not installed) writes tensors so Python can score rungs 2–3.

| Field | Tolerance |
|---|---|
| shapes, dtypes | exact |
| `*_mask` (uint8) | **exact** — any mismatch is a real bug |
| `track_index_to_predict`, `center_type_ids` (int64) | exact |
| positions, sizes, velocities (f32) | atol 1e-4 |
| headings, directions (f32) | atol 1e-5 |

Masks are compared **first and structurally**. A mask mismatch fails immediately with its own
message rather than emitting thousands of confusing value diffs from comparing padding against real
data. Float comparison then runs only over unmasked elements, with padding asserted zero separately.

**Fixtures: 3 scenarios**, chosen to exercise different code paths rather than for variety — one
dense intersection (>128 agents → context overflow), one sparse highway (mostly-empty buffers), one
with history gaps producing masked steps, collectively covering pedestrian and cyclist types.
Deliberately include budget overflow; a fixture set of only well-behaved scenes tests nothing.

Tensors are ~5.7 MB/frame uncompressed (`obj_trajs` 1.3 MB + `map_polylines` 4.4 MB) but >85% zeros,
since `lanelet_ahead` fills maybe 50 of 768 polyline slots and ~20 of 128 context rows. Gzipped,
three scenarios is a couple of MB — committable, no LFS, no CI download step.

### Export path

```
checkpoint_epoch_30.pth
  → strip optimizer state, load into MTR built from official config
  → swap knn_cuda / attention_cuda for ONNX-expressible equivalents   [rung 3-pre gate]
  → torch.onnx.export (static shapes, per contract_8_target.yaml)
  → mtr.onnx + engine_build.yaml            ← PORTABLE ARTIFACT, versioned
       ├─ 3060 Ti: build engine; export correctness, op coverage, shape validation
       └─ 4080:    build engine; parity gate + latency + memory   [sign-off]
```

Static shapes throughout — `contract_8_target.yaml` already pins every dimension, so dynamic-shape
optimization profiles buy nothing. `engine_build.yaml` carries precision flags, workspace size, and
tactic sources so both machines build comparably. **Engines are build products: never committed,
never copied between machines.**

The ONNX-expressible attention will be *slower* than MTR's custom kernels — the sparse kernels exist
precisely because dense attention over that many polylines is expensive. Sequence: prove the dense
equivalent **correct** first, measure its cost second, write TRT plugins only if the measured
latency budget demands it. No speculative plugin work.

This rung also finally exercises `tensorrt_backend.cpp`'s binding-validation path against a real
engine; today it has only ever run against the null backend.

## Hardware

TensorRT engines are not portable across GPU architectures — 3060 Ti is Ampere (sm_86), the vehicle's
4080 is Ada (sm_89). An engine built on one will not load on the other. Further, TRT selects kernels
by per-architecture tactic autotuning, so **fp16 numerics are not guaranteed identical between the
two**.

- **3060 Ti** — development builds, TensorRT correctness and debugging. Catches the categorical
  failures (unsupported ops, plugin requirements, shape mistakes, gross numerical divergence),
  which is the bulk of the work.
- **RTX 4080 (vehicle)** — native engine rebuild for final rung-3 quality/parity, latency, and
  memory measurement, and for rung-4 deployment replay. 16 GB is comfortable for batch-8 inference.
- **No 3060 Ti latency number enters any report.** The numerical parity *gate* also passes on the
  4080, not just perf.

## Scope

### In

- Waymo → ROS mcap converter, with counted drops and a configurable lanelet query
- Converter losslessness test (what makes R1→R2 a clean attribution)
- `REPRESENTATION.md` + Python constants
- Rungs 0, 1, 2, 3-pre, 3-dev, 3-final, 4
- Deployed 3 s minADE/minFDE, per type; 8 s once for rung-0 anchoring
- Loss ledger
- Rung-2 parity gtest in CI, 3 committed fixtures
- ONNX export with op replacement + `engine_build.yaml`
- Target-pinning via `mtr.pinned_target_ids`, with heuristic selection recorded as a free ablation
- **Rung-1 ablation sweep** (full map → drop road edges → drop crosswalks → drop boundaries →
  collapse types → centerlines only), promoted to a phase-1 deliverable. Originally deferred as a
  research tool; the `Lanelet.msg` finding changed its economics, since it now prices a
  SceneBuilder change rather than a new data source.

### Out — considered and rejected

- **Training MTR.** Infeasible on available hardware; the HF checkpoint removes the need.
- **Perception noise injection** (ID switches, dropouts, false positives). Waymo tracks are ground
  truth; injecting noise confounds every rung simultaneously. It answers a *different* question —
  "what does our perception cost us" — and belongs in a separate study. The converter keeps a
  per-frame passthrough hook so this stays possible.
- **MissRate in v1.** minFDE already captures endpoint error; MR adds a threshold parameter and a
  definitional debate for a number nothing downstream consumes. Add later only if averages prove
  misleading.
- **`representation.yaml` as a machine-shared contract.** Only Python reads it; C++ implements it
  implicitly. A doc plus constants is the honest form.
- **Full Waymo val download (~150 shards, ~1 TB).** The ladder measures **paired** deltas over
  identical scenarios, so scenario-difficulty variance largely cancels and a single shard suffices.
  The already-present `validation.tfrecord-00000-of-00150` yields 286 scenarios / ~1268 target
  agents (per `DEVELOPING.md`), which is also adequate for rung-0 absolute anchoring at the ~2–3%
  target. Download more shards only if rung-0 anchoring proves ambiguous.
- **The 5% and 20% HF checkpoints.** One reference model is enough.
- **TRT plugins for the custom ops.** Speculative until latency is measured.
- **Official Waymo mAP.** `DEVELOPING.md:47` records a `metrics_ops.so` ABI mismatch already
  blocking that environment. Self-computed minADE/minFDE are deterministic and dependency-free.
- **pybind11 live differential harness.** Faster iteration and no fixture staleness, but adds a
  binding target to maintain and cannot run in CI. Kept in reserve as a debugging tool if rung-2
  failures prove hard to localize.

### Deferred to a later phase

- **Rung 4b — plausibility on real WATO bags** (drift, off-road rate, kinematic feasibility,
  latency, MTR-vs-fallback takeover). The only test of the actual deployment domain, but it
  produces no attributable number and needs a working on-vehicle engine first.
- **Closing the map gap itself.** Extending SceneBuilder to emit real `global_type` tokens from
  `boundary_type` / `lanelet_type` and to pack road-edge and crosswalk polylines is deliberately
  *not* in scope: the sweep prices it first. Building the fix and the instrument that measures it
  at the same time would leave no way to know whether the fix worked.
- **Lanelet query calibration** against recorded-bag statistics.

## Sequencing

1. `REPRESENTATION.md` + constants — everything downstream implements this contract.
2. Converter + losslessness test + ledger. **Keystone**; unblocks every rung.
3. Rung 0. Validates the eval environment before it is used to judge anything else.
4. Rung 1. First real answer: what does our representation cost?
5. Rung-1 ablation sweep. Prices each discarded map layer separately, producing the
   map-interface budget decision.
6. Rung 2 + CI gtest + fixtures. The regression gate; the highest-durability deliverable.
7. Rung 3-pre. Op replacement correctness, in PyTorch.
8. Rung 3-dev → 3-final. Export, then engine on the 4080.
9. Rung 4. Full runtime replay.

Steps 1–6 need no GPU beyond rung 0/1's PyTorch runs and deliver both the CI gate and the map
budget decision. Steps 7–9 are gated on the custom-op replacement, the riskiest item — so the
highest-value outputs do not sit behind the highest-risk work.

## Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Custom CUDA ops not replaceable without accuracy loss | Blocks rungs 3–4 entirely | Rung 3-pre gates this in PyTorch before any export work |
| Dense attention too slow on 4080 | TRT path misses latency budget | Measure before writing plugins; plugins are the fallback |
| HF checkpoint licensing | Cannot ship on-vehicle | Fine for internal eval now; license read before deployment |
| Lanelet query approximation diverges from real WATO | Rungs 1–2 optimistic vs deployment | Report radius-only vs reachable delta; calibrate against bag statistics later |
| Single val shard too noisy for rung-0 anchor | Weak harness self-validation | Paired deltas are unaffected; download more shards only if needed |

## Resolved from code

### The 29 `obj_trajs` columns

From `scene_builder.cpp:763-781`:

| Cols | Content |
|---|---|
| 0–5 | `rx, ry, rz` (target-centered position), `dx, dy, dz` (box dims) |
| 6–10 | one-hot: vehicle, pedestrian, cyclist, `is_center`, `is_sdc` |
| 11–21 | time one-hot over the 11 history steps |
| 22 | scalar time, `desired_times[s] - current_time` |
| 23–24 | `sin(rel_heading)`, `cos(rel_heading)` |
| 25–26 | `rel_vx`, `rel_vy` (target-frame velocity) |
| 27–28 | `acc_x`, `acc_y` |

6 + 5 + 11 + 1 + 2 + 2 + 2 = 29 ✓. The layout appears to mirror official MTR's
`obj_trajs_data` concatenation, which is reassuring but **must be verified by rung 2, not assumed** —
column ordering is exactly the kind of thing that is silently wrong.

One concrete divergence candidate already visible: acceleration. Official MTR computes a fixed
first-step copy (`acce[:, :, 0, :] = acce[:, :, 1, :]`) and hardcodes the 10 Hz rate. WATO uses
`config_.history_rate_hz` and a more elaborate rule where the earliest present slot inherits the
next *valid* acceleration (`scene_builder.cpp:717-743`). These agree on gap-free histories and
diverge when slots are masked — which Waymo data does produce. A prime rung-2 finding.

### Traffic light / map semantics — better than assumed

`lanelet_msgs/Lanelet` carries far more than centerlines:

- `RegulatoryElement[]` with subtypes `traffic_light`, `stop_sign`, `yield`, `right_of_way`,
  including stop-line geometry (`ref_lines`). **Static infrastructure is present; dynamic signal
  state is not** — there is no current-light-colour field anywhere in the message.
- `left_boundary` / `right_boundary` geometry with `left_boundary_type` / `right_boundary_type`,
  whose constants include **`BOUNDARY_ROAD_EDGE`** and dashed/solid/solid-solid.
- `lanelet_type` with values including **`"crosswalk"`**, plus `turn_direction`, `is_intersection`,
  `speed_limit_mps`, and `conflicting_lanelet_ids`.

This materially changes finding 3. The map gap is **not** "WATO lacks road edges and crosswalks" —
WATO *has* them and `scene_builder.cpp` discards them, packing only centerlines under one generic
`global_type` token. Closing most of the gap is a SceneBuilder change, not a perception or mapping
project. That raises the expected return on the rung-1 ablation sweep considerably: it is pricing a
fix that is cheap, rather than one that needs a new data source.

Dynamic signal state remains genuinely absent, and the 9-column map row has no channel for it
regardless.

## Deferred decision

**The acceleration first-step rule.** Whether to change `scene_builder.cpp:717-743` to match
official MTR exactly (first step copies second, fixed rate) or keep WATO's inherit-next-valid rule
is deliberately **not decided up front** — the divergence only manifests on masked history, and its
size on Waymo's actual masking patterns is unknown.

Rung 2 therefore reports it rather than gating on it in the first iteration: the paired delta
distribution is broken out for masked-history agents specifically, and the decision is made from
that number. If it is negligible, keep WATO's rule (better on gappy real-world tracks) and document
the tolerance. If it is material, adopt MTR's rule — matching training-time preprocessing is
defensible on its own terms — and the gate tightens to zero delta.

This is the one place the rung-2 gate is provisional. Every other field compares exactly or at
float tolerance from day one.
