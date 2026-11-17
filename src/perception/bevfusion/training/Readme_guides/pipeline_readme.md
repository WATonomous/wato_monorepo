# BEVFusion frozen-backbone training: pipeline guide

## 1. Overview

This pipeline fine-tunes the ResNet-50 camera+LiDAR BEVFusion model on nuScenes with most of the model frozen. Only the detection head (`heads.object`) is trained: 1,034,910 of 37,361,053 parameters (2.8%).

It runs on the WATcloud SLURM cluster, on one RTX 3090, inside the `wato-bevfusion-train` Docker image. The upstream BEVFusion training code is used unchanged. Our scripts wrap it.

Three runs were done with it:

| Run | Data | Start weights |
|---|---|---|
| F0 | nuScenes mini | published checkpoint, and the same checkpoint with the head removed |
| F1 | nuScenes trainval | published checkpoint |
| F2 | nuScenes trainval | published checkpoint with the head removed |

## 2. Instructions

The data is already prepared for trainval and mini. For a new data folder, run this on the login node from `src/perception/bevfusion/training`:

```bash
scripts/link_nuscenes.sh full /mnt/wato-drive2/bevfusion_training_image/nuscenes-full
```

Then submit `job_full.sh`, which runs `scripts/prepare_data.sh full` in the container. The job log must end with `train 28130` and `val 6019` (mini: 323 and 81).

Training is submitted as a job script. This is the one for F1:

```bash
#!/bin/bash
set -euo pipefail
RUNS=/mnt/wato-drive2/maxqiu/runs
RUN_NAME=f1-head-1gpu-lr2.5e-5
mkdir -p "$RUNS"

slurm-start-dockerd.sh > /dev/null
export DOCKER_HOST=unix:///tmp/run/docker.sock
docker load -i /mnt/wato-drive2/bevfusion_training_image/wato-bevfusion-train.tar

docker run -i --rm --gpus all --shm-size 32g \
  -v "$HOME/wato_monorepo":/workspace -v /mnt/wato-drive2:/mnt/wato-drive2 \
  -e NUSC_DIR=/mnt/wato-drive2/bevfusion_training_image/nuscenes-full -e RUNS="$RUNS" -e NGPUS=1 \
  -e RUN_NAME="$RUN_NAME" -e RESUME=1 \
  -w /workspace/src/perception/bevfusion/training wato-bevfusion-train:latest bash -s <<'IN'
set -euo pipefail
bash scripts/setup_container.sh
source scripts/env.sh
scripts/train.sh head-only "$RUN_NAME" --max_epochs 2 --optimizer.lr 2.5e-5 --data.workers_per_gpu 16 </dev/null
IN
```

For F2, set `RUN_NAME=f2-head-reinit-1gpu-lr2.5e-5` and add this to the `docker run` line:

```
-e INIT_CKPT=/mnt/wato-drive2/bevfusion_training_image/bevfusion-det-nohead.pth
```

For F0, use `job_f0.sh`.

Submit the job:

```bash
mkdir -p ~/bf_logs
```

```bash
sbatch --gres=gpu:rtx_3090:1,tmpdisk:40960 --cpus-per-task=20 --mem=80G --time=1-00:00:00 --job-name=bf-f1 --output=$HOME/bf_logs/%j-bf-f1.out < job_f1.sh
```

Check its state and follow its log:

```bash
squeue -u $USER
```

```bash
tail -f ~/bf_logs/<job id>-bf-f1.out
```

The log must contain `Trainable parameters: 1,034,910 / 37,361,053`. A full F1 or F2 run takes about 15 h 45 min.

When the job is done, everything is in `$RUNS/$RUN_NAME`:

| File | What it is |
|---|---|
| `epoch_2.pth`, `latest.pth` | The trained weights |
| `configs.yaml` | Every setting that was used |
| `*.log`, `*.log.json` | Training log, including the scores |
| `tf_logs/` | TensorBoard logs |
| `job.sh` | The job script that was submitted |

## 3. Files

All paths are relative to `src/perception/bevfusion/training/`.

| File | What it is |
|---|---|
| `docker/hanlab.Dockerfile` | Builds the base image with the libraries the upstream code needs. |
| `docker/Dockerfile` | Adds the BEVFusion code to the base image and compiles its GPU code. |
| `scripts/env.sh` | Defines the shared paths used by all other scripts. |
| `scripts/setup_container.sh` | Links the checkpoint, configs and data to where the upstream code expects them. |
| `scripts/link_nuscenes.sh` | Builds a writable nuScenes folder from the read-only copy. Runs on the login node. |
| `scripts/prepare_data.sh` | Runs the upstream `create_data.py` to write the info files. |
| `scripts/train.sh` | Chooses the trainable layers and the start weights, then starts training. |
| `tools/train_frozen.py` | Wraps the upstream `train.py`: freezes all other layers and retries failed data reads. |
| `tools/strip_head.py` | Removes the head's weights from a checkpoint. Makes the start weights for F2. |

The job scripts (`job_f0.sh`, `job_f1.sh`, `job_f2.sh`, `job_full.sh`) are not in the repo. A copy is saved as `job.sh` in each run folder.

## 4. Results

nuScenes val, 6,019 samples, after epoch 2:

| Run | mAP | NDS | Criterion | Met? |
|---|---|---|---|---|
| Published (NVIDIA) | 67.93 | 70.97 | | |
| F1 | 67.26 | 70.74 | mAP within 0.3 of published | No (−0.67) |
| F2 | 66.17 | 68.01 | mAP within about 2 of published | Yes (−1.76) |

Checkpoints:

- F1: `/mnt/wato-drive2/maxqiu/runs/f1-head-1gpu-lr2.5e-5/epoch_2.pth`
- F2: `/mnt/wato-drive2/maxqiu/runs/f2-head-reinit-1gpu-lr2.5e-5/epoch_2.pth`
