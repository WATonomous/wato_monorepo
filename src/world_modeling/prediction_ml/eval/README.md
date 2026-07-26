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
