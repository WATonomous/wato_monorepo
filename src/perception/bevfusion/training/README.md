# BEVFusion training

The guides are [BEVFusion Training Work Split](Readme_guides/BEVFusion%20Training%20Work%20Split.md), [BEVFusion nuScenes Training Exercise Pipeline and Code](Readme_guides/BEVFusion%20nuScenes%20Training%20Exercise%20Pipeline%20and%20Code.md), [BEVFusion on WATonomous Car Training, Data Pipeline & Team Plan](Readme_guides/BEVFusion%20on%20WATonomous%20Car%20Training,%20Data%20Pipeline%20%26%20Team%20Plan.md), and [pipeline_readme.md](Readme_guides/pipeline_readme.md). `pipeline_readme.md` is the training how-to.

## Setup

Sub-task 1: TODO (cluster access, storage paths, image, job.sbatch).

## Evaluation and export

Run these commands inside the training container, from `src/perception/bevfusion/training`. Set `CB` to the wato-cuda-bevfusion checkout and `TRAINING_DIR` to this directory.

```bash
NUSC_DIR=/mnt/wato-drive2/bevfusion_training_image/nuscenes-full scripts/eval.sh "$CB/model/resnet50/bevfusion-det.pth"
PTQ=0 scripts/export.sh "$CB/model/resnet50int8/bevfusion_ptq.pth" r50-nusc-published
scripts/export.sh <checkpoint> r50-nusc-f2-v1
cd "$CB" && python "$TRAINING_DIR/tools/eval_ptq.py" --ckpt <ptq.pth>
```

The first command runs Gate G1 on the full validation set; the target is 67.93 ± 0.3 mAP. The second exports without PTQ. The third writes five ONNX files and `MODEL_CARD.md` under `$RUNS/export/<version>/`, plus the PTQ model at `$RUNS/ptq/<version>.pth`. Submit jobs with the job script pattern in `pipeline_readme.md`, section 2.

## Results

| Run | Layers trained | Init | Epochs | mAP | NDS | INT8 mAP | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Published | — | — | — |  |  |  | Gate G1, pending |
| F1 | Head only | Published | 2 | 67.26 | 70.74 |  | From Sub-task 2 epoch-2 eval |
| F2 | Head only | Head removed | 2 | 66.17 | 68.01 |  | From Sub-task 2 epoch-2 eval |
| F3 | Head, fuser, decoder | Head removed |  |  |  |  | Optional, not run |
