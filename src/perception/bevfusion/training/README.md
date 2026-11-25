# BEVFusion training

This pipeline fine-tunes ResNet-50 camera+LiDAR BEVFusion on nuScenes with most layers frozen. Head-only training updates 1,034,910 of 37,361,053 parameters. The scripts wrap the upstream training code without editing it, then evaluate the model and export ONNX files for `bevfusion_node`.

## How the pipeline works

1. Build the training image.
2. Set paths in `params.env`.
3. Link the dataset on the host and prepare its info files in the container.
4. Train the selected layers.
5. Evaluate the checkpoint.
6. Export ONNX, optionally applying post-training quantization (PTQ).
7. Deploy using [Running BEVFusion](../Running_BEVFusion.md).

All paths below are relative to `src/perception/bevfusion/training/`.

| File | Purpose |
| --- | --- |
| `params.env` | Configures storage, dataset and output paths, and data-read retries. |
| `docker/hanlab.Dockerfile` | Builds the upstream dependency image. |
| `docker/Dockerfile` | Adds the pinned BEVFusion fork and compiles its GPU code. |
| `scripts/env.sh` | Loads shared paths and derives container settings. |
| `scripts/setup_container.sh` | Connects weights, configs and data to the upstream checkout. |
| `scripts/link_nuscenes.sh` | Creates a writable dataset root from source data on the host. |
| `scripts/prepare_data.sh` | Writes dataset info files and the ground-truth database. |
| `scripts/train.sh` | Selects trainable layers and starts training. |
| `scripts/eval.sh` | Evaluates a checkpoint on the validation split. |
| `scripts/export.sh` | Calibrates PTQ, exports ONNX and writes a model card. |
| `tools/train_frozen.py` | Freezes unselected layers and retries failed data reads. |
| `tools/strip_head.py` | Removes detection-head weights for training the head from scratch. |
| `tools/eval_ptq.py` | Evaluates a PTQ model with fake quantization in PyTorch. |
| `tools/onnx_regen_trt11.py` | Patches ONNX interfaces for TensorRT 11 FP16 buffers. |
| `tools/check_onnx_io.py` | Checks exported ONNX interfaces. |
| `tools/onnx_io_reference.json` | Records the expected ONNX interfaces. |
| `MODEL_CARD.md` | Template for export metadata and evaluation results. |

## Requirements

Use an NVIDIA GPU with 24 GB of memory; the recorded runs used one RTX 3090. The image compiles for A100 and RTX 30-series GPUs by default. For another GPU, set the appropriate CUDA architectures when building, for example `--build-arg TORCH_CUDA_ARCH_LIST="7.5;8.0;8.6+PTX"`.

## Setup

Run host commands from `src/perception/bevfusion/training`:

```bash
docker build -f docker/hanlab.Dockerfile -t bevfusion:upstream docker
docker build -f docker/Dockerfile -t wato-bevfusion-train docker
source params.env
# Optional: save the image for cluster jobs.
docker save -o "$SHARED/wato-bevfusion-train.tar" wato-bevfusion-train
```

Edit `params.env` for your machine, or set variables before sourcing it. Pass overrides into the container with `docker run -e NAME=value`. Mount `$STORAGE` at the same path inside the container.

| Variable | Default | Purpose |
| --- | --- | --- |
| `STORAGE` | `/mnt/wato-drive2` | Storage mount shared with the container. |
| `SHARED` | `$STORAGE/bevfusion_training_image` | Weights, saved image and default outputs. |
| `NUSC_MINI_SRC` | `$STORAGE/nuscenes-1.0-mini` | Source mini dataset. |
| `NUSC_FULL_SRC` | `$STORAGE/wato-datasets/nuscenes/full-dataset` | Source full dataset. |
| `NUSC_MAP_DIR` | `$STORAGE/wato-datasets/nuscenes/map-expansion` | Map expansion archive directory. |
| `NUSC_DIR` | `$SHARED/nuscenes-mini` | Writable root; use `$SHARED/nuscenes-full` for full runs. |
| `RUNS` | `$SHARED/runs` | Training, evaluation and export outputs. |
| `LOAD_RETRIES` | `5` | Retries after a failed sample read; `0` disables retries. |

Prepare this shared layout. `cuda-bevfusion-model/` contains NVIDIA's unzipped `model.zip`:

```text
$SHARED/
    cuda-bevfusion-model/
        model/
    runs/
    wato-bevfusion-train.tar
```

Link the full dataset on the host:

```bash
source params.env
scripts/link_nuscenes.sh full "$SHARED/nuscenes-full"
```

### Open a training container

Scripts marked "in the container" run in a shell like this. On WATcloud, do this inside a SLURM GPU session after `slurm-start-dockerd.sh` and `docker load` (see Train).

```bash
source params.env
docker run -it --rm --gpus all --shm-size 32g \
  -v "$(git rev-parse --show-toplevel)":/workspace -v "$STORAGE:$STORAGE" \
  -e NUSC_DIR="$SHARED/nuscenes-full" \
  -w /workspace/src/perception/bevfusion/training wato-bevfusion-train bash
# inside the container:
bash scripts/setup_container.sh && source scripts/env.sh
```

Inside the container (opened with NUSC_DIR set to the full root), prepare its info files. The full dataset takes a while with 16 workers, so run it as a job on a cluster:

```bash
scripts/prepare_data.sh full
```

Expect 28,130 training and 6,019 validation samples. For mini, use `mini` instead of `full` and `$SHARED/nuscenes-mini`; expect 323 training and 81 validation samples.

## Train

Save this as `job_f1.sh`. It fine-tunes the head from published weights for two epochs. On WATcloud, `slurm-start-dockerd.sh` starts the job's Docker daemon and `docker load` loads the saved image.

```bash
#!/bin/bash
set -euo pipefail
source params.env
RUN_NAME=f1-head-1gpu-lr2.5e-5
mkdir -p "$RUNS"

slurm-start-dockerd.sh > /dev/null
export DOCKER_HOST=unix:///tmp/run/docker.sock
docker load -i "$SHARED/wato-bevfusion-train.tar"

docker run -i --rm --gpus all --shm-size 32g \
  -v "$(git rev-parse --show-toplevel)":/workspace -v "$STORAGE:$STORAGE" \
  -e STORAGE="$STORAGE" -e SHARED="$SHARED" \
  -e NUSC_DIR="$SHARED/nuscenes-full" -e RUNS="$RUNS" -e NGPUS=1 \
  -e LOAD_RETRIES="$LOAD_RETRIES" -e RUN_NAME="$RUN_NAME" -e RESUME=1 \
  -w /workspace/src/perception/bevfusion/training wato-bevfusion-train:latest bash -s <<'IN'
set -euo pipefail
bash scripts/setup_container.sh
source scripts/env.sh
scripts/train.sh head-only "$RUN_NAME" --max_epochs 2 --optimizer.lr 2.5e-5 --data.workers_per_gpu 16 </dev/null
IN
```

Submit from the training directory on WATcloud:

```bash
mkdir -p ~/bf_logs
sbatch --gres=gpu:rtx_3090:1,tmpdisk:40960 --cpus-per-task=20 --mem=80G --time=1-00:00:00 --job-name=bf-f1 --output=$HOME/bf_logs/%j-bf-f1.out < job_f1.sh
```

The log must show `Trainable parameters: 1,034,910 / 37,361,053`. A full two-epoch head-only run takes about 15 h 45 min. `RESUME=1` continues from the run's `latest.pth` if it exists.

To train the head from scratch, create start weights inside the configured training container:

```bash
source scripts/env.sh
python tools/strip_head.py "$CB/model/resnet50/bevfusion-det.pth" "$SHARED/bevfusion-det-nohead.pth"
```

Set `RUN_NAME=f2-head-reinit-1gpu-lr2.5e-5` in the job and add `-e INIT_CKPT="$SHARED/bevfusion-det-nohead.pth"` to `docker run`.

Training writes to `$RUNS/$RUN_NAME`:

| Output | Contents |
| --- | --- |
| `epoch_2.pth`, `latest.pth` | Trained weights. |
| `configs.yaml` | Settings used for the run. |
| `*.log`, `*.log.json` | Training progress and scores. |
| `tf_logs/` | TensorBoard logs. |

Keep a copy of the job script with each run (the recorded runs have it as `job.sh`).

## Evaluate

Run inside the training container after `scripts/setup_container.sh`, from the training directory. Set `NUSC_DIR` to the full writable dataset root before setup to evaluate all validation samples.

```bash
source scripts/env.sh
RUN_NAME=f1-head-1gpu-lr2.5e-5   # the run to score
scripts/eval.sh "$RUNS/$RUN_NAME/epoch_2.pth"
# Published NVIDIA weights:
scripts/eval.sh "$CB/model/resnet50/bevfusion-det.pth"
```

`scripts/eval.sh CHECKPOINT [CONFIG]` reports nuScenes bounding-box mAP and NDS. The published checkpoint scored 67.93 mAP on the full validation set. For a PTQ model, use the fake-quantization evaluator:

```bash
(cd "$CB" && python "$TRAINING_DIR/tools/eval_ptq.py" --ckpt "$RUNS/ptq/r50-nusc-f2-v1.pth")
```

## Export

Run `scripts/export.sh CHECKPOINT VERSION` inside the configured training container, from the training directory:

```bash
source scripts/env.sh
RUN_NAME=f2-head-reinit-1gpu-lr2.5e-5   # the run to export
scripts/export.sh "$RUNS/$RUN_NAME/epoch_2.pth" r50-nusc-f2-v1
# Export an existing PTQ model without recalibration:
PTQ=0 scripts/export.sh "$CB/model/resnet50int8/bevfusion_ptq.pth" r50-nusc-published
```

`PTQ=1` is the default and calibrates with `CALIB_BATCHES=300` training batches. Set `CALIB_BATCHES` to change the count. `PTQ=0` requires an already quantized checkpoint. Set `EXAMPLE_DATA` to a directory containing `example-data.pth` to supply export example data; otherwise the script generates shape-only inputs.

The script patches ONNX interfaces for TensorRT 11 and checks them against the reference. It writes five ONNX files and `MODEL_CARD.md` to `$RUNS/export/<version>/`. With PTQ enabled, it also saves `$RUNS/ptq/<version>.pth`. Choose a new version for each export; an existing destination is rejected.

Copy the ONNX files into the monorepo's `models/bevfusion/resnet50_trt11_int8/` directory, complete the model card with evaluation and deployment results, and follow [Running BEVFusion](../Running_BEVFusion.md).

## Results

nuScenes validation results on 6,019 samples; both training runs were evaluated after epoch 2:

| Model | mAP | NDS |
| --- | --- | --- |
| Published (NVIDIA) | 67.93 | 70.97 |
| Head fine-tune from published | 67.26 | 70.74 |
| Head trained from scratch | 66.17 | 68.01 |

Recorded checkpoints:

- Published: `$CB/model/resnet50/bevfusion-det.pth`
- Head fine-tune: `/mnt/wato-drive2/maxqiu/runs/f1-head-1gpu-lr2.5e-5/epoch_2.pth`
- Head from scratch: `/mnt/wato-drive2/maxqiu/runs/f2-head-reinit-1gpu-lr2.5e-5/epoch_2.pth`

The head fine-tune came within 0.67 mAP of the published model and the head trained from scratch within 1.76 mAP, so the pipeline trains a working detection head. These runs used the training image from before the fork was pinned; the only later fork change is in runtime code that training does not use.

Export and on-car testing of these trained checkpoints have not been completed yet.
