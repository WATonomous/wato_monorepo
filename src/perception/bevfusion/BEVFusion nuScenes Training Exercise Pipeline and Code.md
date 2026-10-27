# BEVFusion nuScenes Training Exercise: Pipeline and Code

Oct 23, 2026 · @Shourrya

## Goal

**Train the same ResNet-50 camera+LiDAR BEVFusion we deploy, on nuScenes, end to end, and finish with 5 ONNX files that run in our ROS node.** The exercise is a dry run for training on Eve data. Every script built here gets reused when we swap nuScenes for our own dataset. It also clears gate G1 (reproduce the checkpoint) from the Eve training plan.

Done means:

- [ ] The pipeline runs on **v1.0-mini** (10 scenes) from a clean checkout in under a day. This is the smoke test.
- [ ] The published checkpoint, re-evaluated in our environment, matches its published nuScenes val mAP/NDS. This proves the environment is right before we spend GPU days.
- [ ] A model **we trained** on v1.0-trainval reaches mAP/NDS within about 1 point of the published one.
- [ ] Our model is exported to `camera.backbone`, `camera.vtransform`, `fuser`, `head.bbox` and `lidar.backbone.xyz` (ONNX), loads in `bevfusion_node` through `model_dir`, and draws sensible boxes on a nuScenes scene and on an Eve bag in Foxglove.

## Pipeline at a glance

&#91;embedded content: nuScenes training pipeline · 8 steps, 2 shortcuts via published weights\]

Dashed boxes are published weights. Tracks 1–2 skip step 3 by starting fusion from the published LiDAR-only model. The published ResNet-50 checkpoint is evaluated first to prove the environment works.

## Environment

**Use our fork, `wato-cuda-bevfusion`, as the single source.** It contains `CUDA-BEVFusion/bevfusion`, a git submodule pinned to mit-han-lab/bevfusion commit `db75150`, plus the `qat/` export scripts that produce exactly the ONNX files our node loads. Training with any other bevfusion commit risks an export that won't load.

```bash
git clone --recursive https://github.com/WATonomous/wato-cuda-bevfusion
cd wato-cuda-bevfusion && git submodule status   # expect db75150… CUDA-BEVFusion/bevfusion
```

Pinned versions (old, so don't upgrade anything piecemeal):

| Component | Version | Source |
| --- | --- | --- |
| Python | 3.8 | bevfusion README |
| PyTorch | 1.9 – 1.10.2 | bevfusion README |
| mmcv / mmdetection | 1.4.0 / 2.20.0 | bevfusion README |
| OpenMPI / mpi4py | 4.0.4 / 3.0.3 (for `torchpack dist-run`) | bevfusion README |
| Pillow | 8.4.0 | bevfusion README |
| onnx / onnxsim / onnxruntime | 1.12.0 / 0.4.10 / 1.13.1 | `CUDA-BEVFusion/tool/requirements.txt` |
| pytorch-quantization, onnx-graphsurgeon | NGC index | `CUDA-BEVFusion/tool/requirements.txt` |

Build the upstream image once, add the export requirements on top (see Code), and save it as a tar on shared storage. Every SLURM job then loads the same image.

**WATcloud SLURM:**

- GPUs: `--gres shard:<MiB>` shares a GPU (fine for mini and export). `--gres gpu:N` reserves whole GPUs and is needed for multi-GPU training, which is scarce, so book ahead. Example nodes have RTX 3090s. Check with `scontrol show nodes`.
- Time limits: the `compute` partition allows 1 day, `compute_dense` allows 7 days. Full runs either go on `compute_dense` or chain 1-day jobs with `--resume_from`.
- Docker: start the rootless daemon with `slurm-start-dockerd.sh`. Its data lives in `/tmp`, so request `tmpdisk` big enough for the image (about 20 GB is a safe start).

## Data

**Download nuScenes once to shared storage, keep mini and trainval as separate roots, and generate info files with this codebase.** The bevfusion authors advise against reusing info files made by stock mmdetection3d.

1. Make an account at [nuscenes.org](https://www.nuscenes.org/nuscenes#download) and accept the terms. Download **v1.0-mini** (a few GB) for the smoke test, and **v1.0-trainval** (metadata + 10 blob parts, a few hundred GB; approximate) for real runs. The map expansion is only needed for segmentation, so skip it.
2. Extract into one folder per version, e.g. `$NUSC_ROOT/v1.0-mini/` and `$NUSC_ROOT/full/`, then symlink the one in use to `bevfusion/data/nuscenes`. The path must be absolute so Docker mounts it.
3. Generate the info files and the GT-sampling database:

```bash
cd CUDA-BEVFusion/bevfusion
# mini (10 scenes): --version must be passed explicitly
python tools/create_data.py nuscenes --root-path ./data/nuscenes \
  --out-dir ./data/nuscenes --extra-tag nuscenes --version v1.0-mini
# full trainval + test (default --version v1.0, --max-sweeps 10)
python tools/create_data.py nuscenes --root-path ./data/nuscenes \
  --out-dir ./data/nuscenes --extra-tag nuscenes
```

Expected layout afterwards:

```
data/nuscenes/
├── maps/  samples/  sweeps/  v1.0-trainval/  (v1.0-test/)
├── nuscenes_infos_train.pkl
├── nuscenes_infos_val.pkl
├── nuscenes_dbinfos_train.pkl
└── nuscenes_gt_database/
```

The configs read `dataset_root: data/nuscenes/` and the `nuscenes_infos_{train,val}.pkl` names. They load 10 LiDAR sweeps per sample (`sweeps_num: 9` previous + current), 5 point dims, and use 704×256 images. Run `create_data.py` on a CPU node with many workers (`--workers`). It doesn't need a GPU, and on trainval it takes hours.

**Check before training:** use the devkit to render 3 samples with GT boxes on LiDAR and on all 6 cameras. Confirm that `len(train infos)` is 28,130 and `len(val infos)` is 6,019 for trainval.

## Training

**BEVFusion trains in two stages: a LiDAR-only TransFusion model, then the camera+LiDAR model initialized from it.** For the exercise, run three tracks of increasing cost, and only start a track after the previous one works.

| Track | What | Config | Epochs | Starting weights |
| --- | --- | --- | --- | --- |
| 1. Smoke test | Fusion on v1.0-mini, 1 epoch | ResNet-50 `convfuser.yaml` | 1 | Published `lidar-only-det.pth` |
| 2. Fusion stage | Fusion on trainval | ResNet-50 `convfuser.yaml` | 6 (LR 2e-4) | Published `lidar-only-det.pth` |
| 3. Full recipe (stretch) | LiDAR-only, then fusion | `lidar/voxelnet_0p075.yaml`, then `convfuser.yaml` | 20 (GT-paste off after 15), then 6 | ImageNet / nuImages backbone only |

**Weights to download first:**

- `CUDA-BEVFusion/bevfusion/tools/download_pretrained.sh` downloads `lidar-only-det.pth`, `bevfusion-det.pth` (Swin-T) and the others into `pretrained/`.
- CUDA-BEVFusion's `model.zip` (Google Drive link in its README) contains `model/resnet50/bevfusion-det.pth`, `model/resnet50int8/` and a `configs/` folder with the **ResNet-50** configs. Run `cp -r configs bevfusion` so that `configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml` exists. The upstream repo only ships Swin-T fusion configs.

**Commands** (run inside the container from `CUDA-BEVFusion/bevfusion`; `N` = GPUs):

```bash
# Stage 1: LiDAR-only (Track 3 only)
torchpack dist-run -np $N python tools/train.py \
  configs/nuscenes/det/transfusion/secfpn/lidar/voxelnet_0p075.yaml \
  --run-dir runs/lidar-only

# Stage 2: camera + LiDAR fusion, ResNet-50
torchpack dist-run -np $N python tools/train.py \
  configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml \
  --load_from pretrained/lidar-only-det.pth \
  --run-dir runs/fusion-r50

# Resume after a SLURM time limit
# add: --resume_from runs/fusion-r50/latest.pth
```

Check the camera backbone's `init_cfg` in the ResNet-50 config. If it points at a local checkpoint, override it with `--model.encoders.camera.backbone.init_cfg.checkpoint <path>`, the same way upstream does for Swin-T. Config keys can be overridden this way from the command line (torchpack style).

**Budget:** trainval has 28,130 training samples. At the default `samples_per_gpu: 4`, one epoch is about 7,030 iterations on 1 GPU, 1,760 on 4 and 880 on 8. Time 50 iterations on mini first, then multiply. That gives a real wall-clock estimate before booking GPUs. If memory runs out on a 24 GB card, lower `samples_per_gpu` to 2 and scale the LR down linearly.

Checkpoints save every epoch, and only the last one is kept (`max_keep_ckpts: 1`). Raise that to keep every epoch if you want to compare them. Logs go to TensorBoard in the run dir.

## Evaluation and visualization

**Evaluate the published ResNet-50 checkpoint first. If our environment can't reproduce 67.9 mAP with weights we didn't train, our own numbers mean nothing.**

```bash
# Published checkpoint: environment check (gate G1)
torchpack dist-run -np $N python tools/test.py \
  configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml \
  ../model/resnet50/bevfusion-det.pth --eval bbox

# Our checkpoint
torchpack dist-run -np $N python tools/test.py \
  configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml \
  runs/fusion-r50/latest.pth --eval bbox

# Draw GT (data check) or predictions to images
torchpack dist-run -np 1 python tools/visualize.py <config> --mode gt --out-dir viz/gt
torchpack dist-run -np 1 python tools/visualize.py <config> --mode pred \
  --checkpoint runs/fusion-r50/latest.pth --bbox-score 0.1 --out-dir viz/pred
```

The evaluator reads the dataset version from the info files, so the same command works on mini. Mini's val split is tiny, so treat its scores as "the pipeline runs", not as accuracy.

Reference numbers on nuScenes val:

| Model | Runtime | mAP | NDS |
| --- | --- | --- | --- |
| ResNet-50 fusion | PyTorch | 67.93 | 70.97 |
| ResNet-50 fusion | TensorRT FP16 | 67.89 | 70.98 |
| ResNet-50 fusion, PTQ | TensorRT FP16 + INT8 | 67.66 | 70.81 |
| LiDAR-only (TransFusion-L) | PyTorch | 64.68 | 69.28 |

Pass bars for the exercise: the published checkpoint is within 0.3 mAP of 67.93. Track 2 lands within about 1 mAP. After PTQ, we lose no more than about 0.5 mAP, matching NVIDIA's 0.27 drop. To check the exported engines themselves, `qat/test-mAP-for-cuda.py` runs the same evaluation through the CUDA pipeline.

## Export to ONNX and TensorRT

**Run the export on the published checkpoint first and load it in `bevfusion_node`. Once that path is proven, run it on ours.** The scripts run from `CUDA-BEVFusion/` (paths inside them are relative), and PTQ needs nuScenes at `bevfusion/data/nuscenes` for calibration. Because ptq.py runs from CUDA-BEVFusion/, it resolves data/nuscenes/ there too, so setup\_container.sh links the dataset in both places.

```bash
cd CUDA-BEVFusion
CKPT=bevfusion/runs/fusion-r50/latest.pth   # or model/resnet50/bevfusion-det.pth

# 1. PTQ: fuse BN, calibrate INT8 on 300 train batches -> qat/ckpt/bevfusion_ptq.pth
python qat/ptq.py \
  --config=bevfusion/configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml \
  --ckpt=$CKPT --calibrate_batch 300

# 2. Export (INT8 build; add --fp16 to the first two for an FP16 set in qat/onnx_fp16/)
python qat/export-camera.py     --ckpt=qat/ckpt/bevfusion_ptq.pth   # camera.backbone + camera.vtransform
python qat/export-transfuser.py --ckpt=qat/ckpt/bevfusion_ptq.pth   # fuser + head.bbox
python qat/export-scn.py        --ckpt=qat/ckpt/bevfusion_ptq.pth \
  --save=qat/onnx_int8/lidar.backbone.onnx                          # writes lidar.backbone.xyz.onnx
```

The output in `qat/onnx_int8/` is the same set of 5 files as `models/bevfusion/resnet50_trt11_int8/` today.

**Drop-in to the ROS node:**

1. Copy them to `wato_monorepo/models/bevfusion/<model-version>/`, e.g. `r50-nusc-ours-v1`, and add a `MODEL_CARD.md` with the checkpoint, git SHA, mAP/NDS before and after PTQ, and the date.
2. Point `model_dir` in `perception_bringup.yaml` at that folder.
3. Launch perception. `BEVFusionCore` builds the missing `.plan` engines into `build/` on the first start, which takes several minutes. Engines are tied to the GPU and TensorRT version, so never copy `build/` between machines.
4. Play an Eve bag and compare against the old model in Foxglove (`/perception/bev_detection_markers`).

CUDA-BEVFusion's own `tool/build_trt_engine.sh` does the same engine build offline with `trtexec`. Use it to catch build errors outside ROS.

## Code

**One small repo, `wato-bevfusion-train`, wraps the upstream scripts.** It has no forked training code, just pinned dependencies, path setup and one script per pipeline step. The same scripts will later run on the Eve dataset by pointing `NUSC_DIR` at it.

```
wato-bevfusion-train/
├── README.md
├── docker/Dockerfile
├── scripts/
│   ├── env.sh               # shared paths and config names
│   ├── setup_container.sh   # build CUDA ops, link configs + data
│   ├── prepare_data.sh      # create_data.py for mini | full
│   ├── train.sh             # lidar | fusion, with resume
│   ├── eval.sh
│   └── export.sh            # PTQ + 5 ONNX files + model card
├── slurm/job.sbatch         # runs any script above inside Docker
└── third_party/wato-cuda-bevfusion   # git submodule (recursive)
```

One-time setup (needs `model.zip` unzipped into `CUDA-BEVFusion/`, see Training):

```bash
git submodule add https://github.com/WATonomous/wato-cuda-bevfusion third_party/wato-cuda-bevfusion
git submodule update --init --recursive
docker build -t bevfusion:upstream third_party/wato-cuda-bevfusion/CUDA-BEVFusion/bevfusion/docker
docker build -t wato-bevfusion-train:latest -f docker/Dockerfile .
docker save wato-bevfusion-train:latest -o $SHARED/wato-bevfusion-train.tar
(cd third_party/wato-cuda-bevfusion/CUDA-BEVFusion/bevfusion && ./tools/download_pretrained.sh)
mkdir -p logs
```

**`docker/Dockerfile`**

```dockerfile
# Base: the upstream bevfusion image (Python 3.8, PyTorch 1.10, mmcv 1.4.0, mmdet 2.20.0),
# built from third_party/wato-cuda-bevfusion/CUDA-BEVFusion/bevfusion/docker
FROM bevfusion:upstream

# Quantization and ONNX tooling used by CUDA-BEVFusion's qat/ scripts
COPY third_party/wato-cuda-bevfusion/CUDA-BEVFusion/tool/requirements.txt /tmp/qat-requirements.txt
RUN pip install --no-cache-dir -r /tmp/qat-requirements.txt

# Compile CUDA ops for A100 / RTX 3090 even when no GPU is visible during the build
ENV TORCH_CUDA_ARCH_LIST="8.0;8.6"
WORKDIR /workspace
```

**`scripts/env.sh`**

```bash
# Shared paths for every script. Override any variable before running.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CB="${CB:-$REPO_ROOT/third_party/wato-cuda-bevfusion/CUDA-BEVFusion}"
export BF="$CB/bevfusion"
export NUSC_DIR="${NUSC_DIR:-/data/nuscenes}"   # mounted by slurm/job.sbatch
export RUNS="${RUNS:-$REPO_ROOT/runs}"
export NGPUS="${NGPUS:-1}"
export LIDAR_CFG="configs/nuscenes/det/transfusion/secfpn/lidar/voxelnet_0p075.yaml"
export R50_FUSION_CFG="configs/nuscenes/det/transfusion/secfpn/camera+lidar/resnet50/convfuser.yaml"
```

**`scripts/setup_container.sh`**

```bash
#!/usr/bin/env bash
# Run at the start of every container. CUDA ops compile into the mounted source tree,
# so only the first run is slow.
set -euo pipefail
source "$(dirname "$0")/env.sh"
(cd "$BF" && python setup.py develop > /dev/null)
# ResNet-50 configs ship in CUDA-BEVFusion's model.zip, not in upstream bevfusion
cp -rn "$CB/configs/." "$BF/configs/"
# train/test run from $BF, qat/ptq.py runs from $CB; both read data/nuscenes/
for d in "$BF" "$CB"; do
  mkdir -p "$d/data" && ln -sfn "$NUSC_DIR" "$d/data/nuscenes"
done
```

**`scripts/prepare_data.sh`**

```bash
#!/usr/bin/env bash
# Usage: scripts/prepare_data.sh mini|full
# Writes nuscenes_infos_*.pkl, nuscenes_dbinfos_train.pkl and the GT database into $NUSC_DIR.
set -euo pipefail
source "$(dirname "$0")/env.sh"
case "${1:?mini or full}" in
  mini) VERSION_ARGS=(--version v1.0-mini) ;;
  full) VERSION_ARGS=() ;;   # default v1.0 = trainval + test
  *) echo "unknown split: $1" >&2; exit 1 ;;
esac
cd "$BF"
python tools/create_data.py nuscenes --root-path ./data/nuscenes \
  --out-dir ./data/nuscenes --extra-tag nuscenes --workers "${WORKERS:-16}" "${VERSION_ARGS[@]}"
# Sanity check: trainval should print 28130 / 6019
python - <<'EOF'
import pickle
for split in ("train", "val"):
    with open(f"data/nuscenes/nuscenes_infos_{split}.pkl", "rb") as f:
        print(split, len(pickle.load(f)["infos"]))
EOF
```

**`scripts/train.sh`**

```bash
#!/usr/bin/env bash
# Usage: scripts/train.sh lidar|fusion RUN_NAME [config overrides...]
#   fusion initializes from $LIDAR_CKPT (default: published lidar-only-det.pth)
#   RESUME=1 continues from $RUNS/RUN_NAME/latest.pth (e.g. after a SLURM time limit)
set -euo pipefail
source "$(dirname "$0")/env.sh"
STAGE="${1:?lidar or fusion}"; RUN_NAME="${2:?run name}"; shift 2
RUN_DIR="$RUNS/$RUN_NAME"
ARGS=(--run-dir "$RUN_DIR")
case "$STAGE" in
  lidar)  CFG="$LIDAR_CFG" ;;
  fusion) CFG="$R50_FUSION_CFG"
          ARGS+=(--load_from "$(realpath "${LIDAR_CKPT:-$BF/pretrained/lidar-only-det.pth}")") ;;
  *) echo "unknown stage: $STAGE" >&2; exit 1 ;;
esac
if [[ "${RESUME:-0}" == 1 && -e "$RUN_DIR/latest.pth" ]]; then
  ARGS+=(--resume_from "$RUN_DIR/latest.pth")
fi
cd "$BF"
torchpack dist-run -np "$NGPUS" python tools/train.py "$CFG" "${ARGS[@]}" "$@"
```

**`scripts/eval.sh`**

```bash
#!/usr/bin/env bash
# Usage: scripts/eval.sh CHECKPOINT [CONFIG]
set -euo pipefail
source "$(dirname "$0")/env.sh"
CKPT="$(realpath "${1:?checkpoint}")"; CFG="${2:-$R50_FUSION_CFG}"
cd "$BF"
torchpack dist-run -np "$NGPUS" python tools/test.py "$CFG" "$CKPT" --eval bbox
```

**`scripts/export.sh`**

```bash
#!/usr/bin/env bash
# Usage: scripts/export.sh CHECKPOINT MODEL_VERSION
# PTQ + ONNX export into $RUNS/export/MODEL_VERSION/, ready to copy to models/bevfusion/.
set -euo pipefail
source "$(dirname "$0")/env.sh"
CKPT="$(realpath "${1:?checkpoint}")"; VERSION="${2:?model version, e.g. r50-nusc-ours-v1}"
OUT="$RUNS/export/$VERSION"
cd "$CB"
python qat/ptq.py --config="bevfusion/$R50_FUSION_CFG" --ckpt="$CKPT" --calibrate_batch 300
python qat/export-camera.py --ckpt=qat/ckpt/bevfusion_ptq.pth
python qat/export-transfuser.py --ckpt=qat/ckpt/bevfusion_ptq.pth
python qat/export-scn.py --ckpt=qat/ckpt/bevfusion_ptq.pth --save=qat/onnx_int8/lidar.backbone.onnx
mkdir -p "$OUT"
cp qat/onnx_int8/{camera.backbone,camera.vtransform,fuser,head.bbox,lidar.backbone.xyz}.onnx "$OUT/"
cat > "$OUT/MODEL_CARD.md" <<EOF
# $VERSION
- checkpoint: $CKPT
- training repo commit: $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)
- exported: $(date -u +%Y-%m-%dT%H:%MZ), PTQ INT8, 300 calibration batches
- nuScenes val mAP / NDS, PyTorch: TODO
- nuScenes val mAP / NDS, after PTQ (qat/test-mAP-for-cuda.py): TODO
EOF
ls -l "$OUT"
```

**`slurm/job.sbatch`**

```bash
#!/bin/bash
#SBATCH --job-name=bevfusion
#SBATCH --partition=compute_dense
#SBATCH --gres=gpu:4,tmpdisk:40960
#SBATCH --cpus-per-task=16
#SBATCH --mem=64G
#SBATCH --time=2-00:00:00
#SBATCH --output=logs/%j-%x.out
# Usage: sbatch slurm/job.sbatch <script> [args...]
# Needs IMAGE_TAR (saved training image) and NUSC_HOST_DIR (nuScenes root) in the environment.
set -euo pipefail
: "${IMAGE_TAR:?set IMAGE_TAR}" "${NUSC_HOST_DIR:?set NUSC_HOST_DIR}"

slurm-start-dockerd.sh            # rootless dockerd, data dir in /tmp (tmpdisk)
docker load -i "$IMAGE_TAR"

# If --gpus is rejected by the rootless daemon, check WATcloud's Docker docs for the GPU flag.
docker run --rm --gpus all --shm-size 16g \
  -v "$SLURM_SUBMIT_DIR":/workspace -v "$NUSC_HOST_DIR":/data/nuscenes \
  -e NGPUS="${NGPUS:-$(nvidia-smi -L | wc -l)}" -e RESUME -e LIDAR_CKPT \
  -w /workspace wato-bevfusion-train:latest \
  bash -c "scripts/setup_container.sh && $*"
```

**Example submissions** (run from the repo root, so `SLURM_SUBMIT_DIR` is the repo):

```bash
export IMAGE_TAR=$SHARED/wato-bevfusion-train.tar

# Track 1: mini smoke test on a shared GPU
export NUSC_HOST_DIR=$NUSC_ROOT/v1.0-mini
sbatch --partition=compute --gres=shard:20480,tmpdisk:40960 --time=12:00:00 \
  slurm/job.sbatch scripts/prepare_data.sh mini
sbatch --partition=compute --gres=shard:20480,tmpdisk:40960 --time=12:00:00 \
  slurm/job.sbatch scripts/train.sh fusion mini-smoke --max_epochs 1

# Gate G1 + Track 2 on trainval
export NUSC_HOST_DIR=$NUSC_ROOT/full
sbatch slurm/job.sbatch scripts/eval.sh third_party/wato-cuda-bevfusion/CUDA-BEVFusion/model/resnet50/bevfusion-det.pth
sbatch slurm/job.sbatch scripts/train.sh fusion fusion-r50
RESUME=1 sbatch slurm/job.sbatch scripts/train.sh fusion fusion-r50   # if it hit the time limit

# Export
sbatch --gres=shard:20480,tmpdisk:40960 slurm/job.sbatch \
  scripts/export.sh runs/fusion-r50/latest.pth r50-nusc-ours-v1
```

These scripts haven't been run yet. Expect the first mini run to turn up path and permission fixes, and commit those back to the repo so the next person doesn't hit them.

## Exercise plan

**Each person owns one stage, matching their role in the Eve plan, and everyone runs Track 1 themselves so all four know the whole pipeline.** The roles below are the same A–D roles as the Eve training doc.

| Person | Owns in this exercise | Carries over to Eve |
| --- | --- | --- |
| A. Data | nuScenes download + checksums, `prepare_data.sh`, GT visual check | The bag → nuScenes converter uses these same info files and checks |
| B. Eval | `eval.sh`, reproducing the published numbers, a results table per run | The frozen Eve val set and the eval script |
| C. Training | Docker image, `train.sh`, SLURM jobs, Tracks 2 and 3 | Stage A/B/C fine-tuning configs |
| D. Export | `export.sh`, PTQ, TRT build, loading in `bevfusion_node` | Every model release |

Checkpoints, in order (each unblocks the next):

- [ ] Image built, saved to shared storage; `setup_container.sh` compiles the CUDA ops (C)
- [ ] Mini prepared and the GT visualizations look right (A)
- [ ] Everyone finishes the Track 1 mini smoke test from a clean clone (all)
- [ ] Trainval prepared, counts are 28,130 / 6,019 (A)
- [ ] Published ResNet-50 checkpoint evaluates within 0.3 of 67.93 mAP, which passes G1 (B)
- [ ] Published checkpoint exported, loads in `bevfusion_node`, boxes show in Foxglove on an Eve bag (D)
- [ ] Track 2 fusion training finishes; our model is within about 1 mAP of published (C, B)
- [ ] Our model exported and loaded with a model card; PTQ costs no more than about 0.5 mAP (D)
- [ ] Stretch: Track 3 LiDAR-only from scratch reaches about 64.7 mAP, and fusion from it lands near Track 2 (C)
- [ ] Write-up: what broke, how long each stage took, GPU-hours used. This feeds the Eve plan's budget (all)

The first export checkpoint can run alongside training, because it starts from the published checkpoint. That way D is never waiting on C.

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `setup.py develop` fails compiling CUDA ops | No GPU visible and no target arch set | `TORCH_CUDA_ARCH_LIST` is set in the Dockerfile. Rebuild the image |
| `ImportError` for mmcv / mmdet3d ops | A package was upgraded, or `setup.py develop` didn't run in this container | Never `pip install -U` inside the image. Run `setup_container.sh` |
| `FileNotFoundError: ...resnet50/convfuser.yaml` | `model.zip` configs not copied | Unzip `model.zip` in `CUDA-BEVFusion/`. `setup_container.sh` copies `configs/` |
| `nuscenes_infos_train.pkl` not found during `ptq.py` | PTQ resolves `data/nuscenes` from `CUDA-BEVFusion/` | Run `setup_container.sh`, which links data in both places |
| CUDA OOM at batch 4 | 24 GB card, 0.075 m voxels, 6 cameras | `--data.samples_per_gpu 2` and halve the LR |
| `torchpack dist-run` hangs or fails on MPI | `--shm-size` too small, or the MPI ranks don't match the GPUs | Keep `--shm-size 16g`. Set `NGPUS` equal to the GPUs requested |
| Job killed at the time limit | `compute` allows 1 day | Use `compute_dense` (7 days) or resubmit with `RESUME=1` |
| Published checkpoint scores well below 67.93 | Stale info files from stock mmdet3d, wrong dataset root, or mini infos left in place | Regenerate infos with `prepare_data.sh full`. Check the 28,130 / 6,019 counts |
| Node fails to deserialize engines | `.plan` built on another GPU or TensorRT version | Delete `build/` in the model folder and let the node rebuild it |
| Boxes in the wrong place after loading our export | Exported a non-ResNet-50 config, or image size differs from 704×256 | Export from the ResNet-50 `convfuser.yaml` only, and keep the image size unchanged |

## Sources

- [mit-han-lab/bevfusion README](https://github.com/mit-han-lab/bevfusion): prerequisites, data prep, train/test commands, Swin-T and LiDAR-only results
- [bevfusion `configs/nuscenes/default.yaml` @ db75150](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/configs/nuscenes/default.yaml): sweeps, image size, batch size, info-file names
- [bevfusion fusion `default.yaml` @ db75150](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/configs/nuscenes/det/transfusion/secfpn/camera%2Blidar/default.yaml): 6 epochs, LR 2e-4
- [bevfusion `configs/default.yaml` @ db75150](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/configs/default.yaml): 20 default epochs, checkpoint settings
- [bevfusion `tools/create_data.py` @ db75150](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/tools/create_data.py), [`tools/train.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/tools/train.py), [`tools/visualize.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/tools/visualize.py): CLI flags
- [NVIDIA CUDA-BEVFusion README](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/master/CUDA-BEVFusion): ResNet-50 accuracy table, pinned bevfusion commit, `model/resnet50int8` layout
- [CUDA-BEVFusion `qat/` README](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/master/CUDA-BEVFusion/qat) and scripts ([`ptq.py`](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/ptq.py), [`export-scn.py`](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/export-scn.py)): PTQ and export commands, output paths
- [WATcloud SLURM docs](https://cloud.watonomous.ca/docs/compute-cluster/slurm): GRES syntax, partitions and time limits, rootless Docker
- [WATonomous/wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion): our fork
- `wato_monorepo/src/perception/bevfusion/README.md` and `DEVELOPING.md`: model folder layout, engine auto-build
