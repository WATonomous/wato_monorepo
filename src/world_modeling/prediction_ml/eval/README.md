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

The complete suite, including real checkpoint loading and all 286 scenarios in
the local shard, runs with:

```bash
MTR_ROOT=/root/mtr-task0 \
MTR_CHECKPOINT=/root/mtr-task0/checkpoints/100_percent/MTR/checkpoint_epoch_30.pth \
/root/mtr-harness-venv/bin/python -m pytest \
  /mnt/c/Users/rylei/github/wato_monorepo/src/world_modeling/prediction_ml/eval/tests -v
```

Result on 2026-07-26: **46 passed**.

## Gate A — checkpoint validation

**Status: RUN / FAIL / CLOSED.**

The checkpoint was downloaded from
`Student-Xiaoji/LLM-Augmented-MTR-models/100_percent/MTR/checkpoint_epoch_30.pth`.
Its SHA256 is:

```text
f14858a1ce6b4098eef8230db39f014b337ef5e158b591e0febe722d7ae4667a
```

After building MTR's CUDA extensions and preprocessing the validation data:

```bash
cd src/world_modeling/prediction_ml/eval
export MTR_ROOT=~/mtr
export MTR_CHECKPOINT=/absolute/path/to/checkpoint_epoch_30.pth
export MTR_PREPROCESSED_WAYMO=/absolute/path/to/official/preprocessed/waymo
python -m wato_mtr_eval.rung0 \
  --shard /mnt/c/Users/rylei/github/wato_monorepo/uncompressed_scenario_validation_validation.tfrecord-00000-of-00150 \
  --checkpoint "$MTR_CHECKPOINT" \
  --mtr-root "$MTR_ROOT" \
  --preprocessed-data "$MTR_PREPROCESSED_WAYMO"
```

The full local-shard report was:

```text
Rung 0 (golden) — 286 scenarios, 1268 target agents

  official 8 s config  minADE 0.5821  (published 0.6046)
                       minFDE 1.1335  (published 1.2251)
  deployed 3 s  minADE 0.2787
                minFDE 0.5531

  per type (deployed):
    TYPE_CYCLIST    minADE 0.2799  minFDE 0.6136
    TYPE_PEDESTRIAN minADE 0.1451  minFDE 0.3008
    TYPE_VEHICLE    minADE 0.2920  minFDE 0.5762

  GATE A: FAIL
```

The checkpoint's own epoch-30 log records full-validation metrics of minADE
`0.5950`, minFDE `1.2094`, and mAP `0.4182` over 44,097 agents. Those
full-validation values are within 3% of the published `0.6046` / `1.2251`, but
the single committed shard's minFDE is not. This shows that the plan's
single-shard 3% gate is too sensitive to shard composition; it does **not**
justify opening Gate A from metadata alone. Either establish a shard-specific
reference or evaluate enough validation shards to make the published aggregate
a valid comparison.

Do not begin Task 7 or any TensorRT rung while this status remains closed.
