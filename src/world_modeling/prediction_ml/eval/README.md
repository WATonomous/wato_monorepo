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
