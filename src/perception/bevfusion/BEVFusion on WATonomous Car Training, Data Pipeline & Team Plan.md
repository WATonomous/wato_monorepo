# BEVFusion on WATonomous Car: Training, Data Pipeline & Team Plan

Oct 23, 2026 · @Shourrya

## Overview and goals

We should **fine-tune the existing nuScenes checkpoint on labeled Eve data, in stages, starting with most layers frozen**. Training from scratch needs a nuScenes-sized dataset (about 40k labeled keyframes), which we can't build. Phase 2 then extends the fine-tuned model from 6 to all 8 roof cameras.

Today the deployed model (`resnet50_trt11_int8`) runs on Eve unchanged from nuScenes. It sees different camera angles, a different image shape and a much sparser LiDAR cloud than it was trained on. The jittery, flickering boxes we see mostly come from that gap.

| Phase | Goal | Done when |
| --- | --- | --- |
| 0. Baseline | Measure the current model on Eve data and fix cheap input mismatches | We have a frozen Eve val set and baseline scores for accuracy, stability and latency |
| 1. Eve fine-tune (6 cams) | Adapt the model to Eve's sensors with partial freezing, then full fine-tuning | Beats the baseline on Eve val, passes the on-car gates in Training and evaluation, and ships as a versioned TRT model |
| 2. 8 cameras | Add `camera_pano_ee` and `camera_pano_ww` | The 8-camera model beats the 6-camera model on side-region objects, with latency still in budget |

"Smoother" in this doc means measurable temporal stability: fewer dropped frames per object, less box jitter, fewer yaw flips and class switches. It is defined in Making it smoother on the car.

## Current state and the nuScenes gap

The repo only does **inference**. `src/perception/bevfusion` wraps [wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion), our fork of NVIDIA's CUDA-BEVFusion, and runs five exported pieces: `camera.backbone`, `camera.vtransform`, `fuser`, `head.bbox` (ONNX → TensorRT `.plan`) and `lidar.backbone.xyz` (sparse conv, INT8). Training happens in a separate Python stack, MIT's [bevfusion](https://github.com/mit-han-lab/bevfusion) on mmdet3d. Its output is converted with CUDA-BEVFusion's `qat/` export scripts, and nothing in the monorepo does this yet.

What the checkpoint learned from nuScenes, compared with what Eve gives it:

| Input | nuScenes (training) | Eve (today) | Why it matters |
| --- | --- | --- | --- |
| Camera layout | 6 cams at about 0°, ±55°, ±110°, 180° | 6 of 8 roof cams at 0°, ±45°, ±135°, 180°; `ee`/`ww` dropped | The camera-to-BEV lift uses our calibration, but learned features expect nuScenes viewpoints. About 27° on each side has no camera coverage |
| Camera FOV | about 65–70° (rear 110°) | about 63° (fx ≈ 1046 px at 1280 px wide) | Close enough. Not the main problem |
| Image shape | 1600×900 (16:9), resize 0.48, crop to 704×256 | 1280×1024 (5:4), resize 0.55, bottom crop to 704×256 | 307 of 563 resized rows (55%) are cropped off the top, versus 41% in nuScenes. The image statistics differ |
| LiDAR | HDL-32E, 20 Hz, **10 sweeps** stacked with a time-lag channel | VLP-32C (`lidar_cc`), 20 Hz, **1 sweep**, time lag fixed at 0 | The cloud is about 10× sparser than in training. This is likely the biggest gap and the main cause of low, unstable scores |
| Classes | 10 nuScenes classes | Same 10, hard-coded in `bevfusion_constants.hpp` | Keep them for Phase 1 so the head and the nuScenes eval need no changes |
| BEV range | ±54 m, 0.075 m voxels, 0.3 m BEV cells | Same | No change needed |
| Sync | Keyframes at 2 Hz, sensors closely aligned | `ApproximateTime` with `sync_max_time_diff_ms: 200` | At 15 m/s, 200 ms is 3 m of misalignment between camera and LiDAR features |

The C++ side already handles calibration correctly from TF and `MultiCameraInfo`. Any change to image size, crop or camera count must be made in **both** the training config and `params.yaml` / `perception_bringup.yaml`, then checked against the `img_aug_matrix` tests in `test_bevfusion_node.cpp`.

## Training strategy: scratch vs. fine-tune vs. freezing

**Recommendation: fine-tune from the nuScenes checkpoint, unfreezing in stages, and keep nuScenes in the training mix.** The image backbone's features (ResNet-50 + FPN) carry over to Eve. What needs relearning is geometry and density: depth priors, camera viewpoints and how sparse the LiDAR is. Those live in the view transform, the fuser and the head, so we unfreeze those first.

| Option | Data needed (estimate) | Compute | Risk | Use it when |
| --- | --- | --- | --- | --- |
| From scratch | nuScenes-scale: about 30k+ labeled keyframes | Days on multiple GPUs, plus the official recipe (LiDAR-only pretraining, then fusion) | Very likely worse than the checkpoint with our data volume | Only if we change the architecture so much that no weights transfer |
| Freeze backbones, train the rest | About 1–3k Eve keyframes | Hours on 1–2 GPUs | Low. Hard to overfit badly | **Stage A**: first Eve model |
| Full fine-tune | About 5–10k+ Eve keyframes | About a day on 2–4 GPUs | Forgetting and overfitting to campus routes | **Stage C**: once the dataset is big enough |

### Staged plan

1. **Stage 0: no training.** Fix inputs first, then re-measure. Stack the last 10 `lidar_cc` sweeps, ego-motion compensated with Novatel odometry, and fill the time-lag channel properly so the cloud looks like nuScenes. Tune `confidence_threshold` per class on Eve val. Tighten camera/LiDAR sync. This alone may remove much of the flicker.
2. **Stage A: frozen backbones.** Freeze the camera backbone + neck and the LiDAR sparse encoder, with BatchNorm layers in eval mode. Train the camera `vtransform` (depth net), the fuser, the BEV decoder and the TransFusion head. Use the nuScenes checkpoint LR × 0.1, about 10–20 epochs, with 50/50 Eve/nuScenes sampling.
3. **Stage B: unfreeze the camera neck and the upper backbone stages** (ResNet layer3–4) at 0.1× the head LR. Keep the LiDAR encoder frozen unless Stage 0 multi-sweep isn't enough.
4. **Stage C: full fine-tune**, including the LiDAR encoder, with layer-wise LR decay. Lower the nuScenes share (e.g. 25%) as Eve data grows.
5. **Re-quantize and export.** Run CUDA-BEVFusion's PTQ/QAT on the LiDAR backbone using **Eve** calibration frames, export the 5 ONNX files, build the TRT engines on the car's GPU, and check that PyTorch and TRT outputs match (see Training and evaluation).

Each stage is a separate experiment against the same frozen Eve val set. We move to the next stage only if the previous one stopped improving. Disable GT-sampling (LiDAR copy-paste) augmentation for the last few epochs of each stage, as the BEVFusion recipe does.

The data estimates in this section are planning guesses. Revise them after the Stage A learning curve: train on 25/50/100% of the dataset and see whether val is still improving.

## Sensors and calibration

Bad calibration silently caps accuracy, and no amount of training fixes it. **Every labeled frame must record which calibration it was captured with, and calibration must be checked before each collection day.**

Sensors used:

- **Cameras:** 8 roof cameras (`camera_pano_nn/ne/ee/se/ss/sw/ww/nw`), 1280×1024, hardware-triggered by `src/embedded/camera_sync`. The 4 `camera_lower_*` cameras are out of scope.
- **LiDAR:** `lidar_cc` (VLP-32C, 1200 rpm = 20 Hz) is the primary sensor. `lidar_ne` / `lidar_nw` (VLP-16) are optional extras through `lidar_aggregator`. Stick with `lidar_cc` alone in Phase 1. It matches nuScenes' single 32-beam sensor best.
- **Ego pose:** Novatel (`/novatel/oem7/odom`, `inspva`). Needed for multi-sweep stacking, velocity labels and nuScenes-format `ego_pose`.

Requirements:

| Item | Source today | Requirement |
| --- | --- | --- |
| Camera intrinsics | `interfacing/calibration/camera_calib/config/*.yaml` → `camera_info` | **Fix first:** BEVFusion receives `image_rect_compressed` but builds intrinsics from `CameraInfo.k` (`bevfusion_node.cpp:560`). For rectified images the right matrix is the top-left 3×3 of `P`. On `camera_pano_nn`, fx is 1046.4 in K vs 1021.7 in P, a 2.4% depth-scale error. Use the same matrix in training data and at runtime |
| Camera → LiDAR extrinsics | `eve_description` URDF → `/tf_static` | Check every camera with a LiDAR-on-image overlay before each collection day. Re-calibrate when a sensor is touched. Store a calibration version ID (e.g. `calib-2026-10-30`) |
| LiDAR → IMU/base | `lidar_imu_calib` | Needed for motion compensation and multi-sweep stacking |
| Time sync | Camera hardware trigger + `ApproximateTime` (200 ms) | Log the camera/LiDAR stamp difference per frame. In the dataset, drop keyframes over 50 ms (proposed). At runtime, tighten `sync_max_time_diff_ms` to match |

The overlay check should be a script: project `lidar_cc` points into each camera and save a PNG grid per bag. A misaligned camera then shows up in seconds, not after a week of training.

## Training data pipeline

**Convert Eve bags into the nuScenes devkit format.** mmdet3d's `create_data.py`, BEVFusion's data loaders and the nuScenes metrics then work with almost no code changes. Record all 8 roof cameras from day one. 3D box labels live in the LiDAR/ego frame and don't depend on which cameras are used, so Phase 2 reuses every Phase 1 label.

&#91;embedded content: Eve data pipeline · 8 steps, QA loop and failure-case loop\]

Bags flow left to right into nuScenes-format releases. Rejected QA batches go back to labeling, and on-car failures re-enter at recording.

1. **Record.** Use the `all_sensors_compressed` bag\_recorder profile (`perception_bringup/config/bag_recorder/`). It already captures the 8 pano cameras, `camera_info`, all 3 LiDARs and Novatel. Make sure `/tf_static` is in every bag. Plan drives for coverage: campus, arterial roads, parking lots, night, rain and, later, snow.
2. **Ingest.** Copy to `/mnt/wato-drive2/bevfusion/raw/`. Add one manifest row per bag: date, route, weather, time of day, calibration version, driver notes.
3. **Extract and sync.** Take keyframes at 2 Hz (as nuScenes does) on `lidar_cc` stamps. For each, pick the nearest image from each camera and drop the frame if any camera is over the sync limit. Save the 9 previous sweeps for multi-sweep input, plus the ego pose from odometry.
4. **Convert.** Write the nuScenes tables (`scene`, `sample`, `sample_data`, `calibrated_sensor`, `ego_pose`, `sample_annotation`, `instance`). Use our own sensor channel names (`CAM_PANO_NN` …). A scene is a 20 s clip, like nuScenes.
5. **Pre-label.** Run an offline model on the keyframes: the current checkpoint with multi-sweep input and test-time augmentation, later our own fine-tuned model. Import its boxes as drafts.
6. **Human labeling.** Annotators correct the drafts in a 3D tool with LiDAR + camera views and track IDs across frames (candidates: [Xtreme1](https://github.com/xtreme1-io/xtreme1), [CVAT](https://github.com/cvat-ai/cvat)). Write a one-page guideline that follows nuScenes box conventions: tight box, yaw along heading, the same 10 classes, and a minimum LiDAR point count.
7. **QA.** A second person reviews a random 10% of each batch and every frame the model and the annotator disagree on. Reject the batch if more than a set share of boxes need fixes (e.g. 5%).
8. **Split and release.** Split **by scene and route, never by frame**, so near-duplicate frames can't leak into val. Keep a frozen val set (about 15% of scenes) and a frozen test set that nobody trains on. Release as an immutable version (`eve-bev-v0.1`) with generated `info.pkl` files.

Proposed volumes: v0.1 has about 2k keyframes (about 100 scenes) for Stage A. v0.3+ has 5–10k for full fine-tuning. Velocity labels come free once track IDs exist: box displacement between keyframes.

## Training and evaluation

**Gate 1: before any Eve training, reproduce the deployed checkpoint end to end.** Load it in our training repo, match its nuScenes val mAP, re-export it, and get the same boxes from TRT as from today's `.plan` files. Until that works, no fine-tuned model can be shipped.

**Setup**

- New repo `wato-bevfusion-train`: a fork of mit-han-lab/bevfusion plus CUDA-BEVFusion's `qat/` scripts, with a pinned Docker image (its PyTorch, mmcv, mmdet3d and spconv versions are old and break easily).
- Compute: WATcloud SLURM GPU jobs. Stage A fits on 1–2 GPUs. Request 2–4 for Stage C.
- Configs: one per stage, inheriting from the nuScenes ResNet-50 fusion config. They change dataset paths, camera names and count, `sweeps_num`, image aug `resize_lim` around 0.55, which modules are frozen, and the LR.
- Tracking: log every run to W&B or MLflow, tagged with dataset version, config and git SHA. A model nobody can trace back to its data is not deployable.

**Metrics** (all on the frozen Eve val/test sets)

- Accuracy: nuScenes mAP and NDS, per-class AP, and AP by range (0–20, 20–40, 40–54 m).
- Stability: see Making it smoother on the car. Run on 5–10 fixed replay bags.
- Latency: end-to-end on the car GPU, p50 and p99.
- Parity: PyTorch vs. TRT on 200 frames. Box centers within 0.1 m and scores within 0.02 (proposed tolerances).

**Acceptance gates**

| Gate | Check | Blocks |
| --- | --- | --- |
| G1 Reproduce | Checkpoint mAP matches the published number; TRT parity on nuScenes frames | All training work |
| G2 Beats baseline | Fine-tuned mAP/NDS above the Stage 0 baseline on Eve val, with no class more than 2 AP worse | Export |
| G3 Parity | Exported INT8/FP16 engines pass the parity check | On-car testing |
| G4 On-car | Stability metrics better than baseline on replay bags; p99 latency within the frame budget; a supervised on-car run with no new failure types | Merging the new `model_dir` into `perception_bringup.yaml` |

The test set is only used at G4. Choose thresholds and models on val, never on test.

## Making it smoother on the car

Smoothness comes from three things: **confident detections** (fine-tuning plus multi-sweep), **clean inputs** (sync and calibration), and **temporal filtering downstream** in the `tracking` node. BEVFusion sees one frame at a time, so the model alone will never be perfectly steady.

| Symptom | Likely cause | Fix | Owner |
| --- | --- | --- | --- |
| Boxes blink on and off | Scores hover around the very low `confidence_threshold: 0.08` because of the single-sweep domain gap | Multi-sweep input, fine-tuning, per-class thresholds tuned on Eve val | Training + Deployment |
| Box position jumps | Camera/LiDAR up to 200 ms apart; calibration error | Tighter sync, P-matrix fix, extrinsics check | Data |
| Yaw flips 180° | Symmetric shapes, no motion cue | Velocity from multi-sweep; tracker keeps heading consistent with velocity | Deployment |
| Class switches (car ↔ truck) | Ambiguous classes, little Eve data | Fine-tuning; tracker takes a majority vote over the track's history | Training + Deployment |
| Ghost boxes on the ego car | Self-returns | `ego_suppression_radius` already handles this; check after retraining | Deployment |

**Stability metrics** (computed from tracks on replay bags, using a reference tracker so model changes are compared fairly):

- **Flicker rate:** fraction of frames where a confirmed track has no matching detection.
- **Center jitter:** RMS of the detection-to-smoothed-track residual, in metres, per class.
- **Yaw flips** and **class switches** per 100 track-seconds.
- **Latency spread:** p99 − p50. Uneven frame timing looks like jitter downstream.

On the tracking side, use a 3D constant-velocity Kalman filter. Confirm a track only after N hits and drop it only after M misses (hysteresis). Fixing this in the tracker is cheap and helps regardless of model quality, so it can start in Phase 0.

## Phase 2: 6 → 8 cameras

**The network weights barely change. The real work is in the fixed-size plumbing and in training data with all 8 views.** The camera backbone is shared across cameras, and the LSS view transform lifts each camera independently before BEV pooling. Adding `camera_pano_ee` and `camera_pano_ww` therefore adds no new layers.

What has to change:

| Layer | Change |
| --- | --- |
| Training config | `num_cams = 8`, 8 channels in the dataset, 8 image aug matrices |
| Training data | nuScenes has only 6 cameras, so 8-camera batches must be Eve-only. Either keep nuScenes and Eve in separate batches, or pad nuScenes to 8 with masked dummy cameras |
| Initialization | Start from the Phase 1 Eve model. Fine-tune the `vtransform`, fuser and head first (as in Stage A), then unfreeze more if needed |
| Export | Re-export `camera.backbone` and `camera.vtransform` with N = 8 in the input shape. TRT engines are built for a fixed camera count |
| wato-cuda-bevfusion | Remove any hard-coded 6: camera buffers, the `6×4×4` calibration tensors, BEV-pool geometry setup |
| ROS node + bringup | `camera_names` with 8 entries, uncomment `ee`/`ww` in `multi_camera_sync` in `perception_bringup.yaml`, update DEVELOPING.md |
| Latency | The camera branch grows about 8/6 = 1.33×. If that breaks the budget, try FP16/INT8 camera engines or a smaller input |

Success criteria: AP on objects at 60–120° azimuth (the side regions that 6 cameras miss) clearly beats the 6-camera model, overall mAP is not worse, and G3/G4 pass. Keep the 6-camera model deployable as the fallback until then.

## Pipeline maintenance

**Treat datasets and models as versioned releases, and retrain on purpose rather than ad hoc.** The pipeline has to survive team turnover. Every step is a script in `wato-bevfusion-train` with a README, and the owner table below assigns a person to each one.

**Versioning**

- **Datasets:** immutable `eve-bev-vX.Y` folders on `/mnt/wato-drive2/bevfusion/datasets/`. Each has a manifest listing bags, calibration versions, label export, converter git SHA and split files. Use checksums or DVC so a version can't change in place.
- **Models:** `models/bevfusion/<model-version>/` holding the ONNX files plus a model card: dataset version, config, metrics, TRT version, GPU it was built on. `model_dir` in `perception_bringup.yaml` points at an exact version. Move off the single Google Drive folder.
- **Calibration:** each re-calibration gets a dated ID that is stored with every frame captured under it.

**When to retrain**

- A sensor is moved, replaced or re-calibrated.
- A new operating domain arrives (first snow, night driving, a new test site).
- About 1–2k new labeled keyframes have built up since the last release.
- A failure pattern repeats in on-car logs.

**Failure-case loop**

Safety drivers flag bad perception moments during runs. The disengage monitor from #486 is a natural trigger. Clip the bag ±10 s around each flag. Also auto-mine frames with low confidence or track/detection disagreement. All of these go to the front of the labeling queue, so the dataset grows where the model is weakest.

**Regression checks**

- Run the existing `colcon test --packages-select bevfusion` on every node change. It covers the `img_aug_matrix` math.
- A GPU replay job (manual or nightly) runs the candidate model on the frozen replay bags and compares accuracy and stability with the current release.
- Run the parity test on every export.
- Run the calibration overlay script before each collection day.

## Team split (4 people)

**Split by pipeline stage, so each person owns one hand-off and can work in parallel from week 1.** Labeling is the bottleneck, so in the busy weeks everyone labels about 2 hours a week, not just the data owners.

| Role | Owns | Key deliverables | Hands off to |
| --- | --- | --- | --- |
| **A. Data and calibration** | Recording, calibration, extraction, nuScenes converter, dataset versioning | Overlay check script; K→P intrinsics fix; tighter sync; `bag → nuScenes` converter; `eve-bev-v0.1` release | B (frames to label), C (datasets) |
| **B. Labeling and evaluation data** | Labeling tool, guidelines, pre-labeling, QA, splits, replay-bag set | Labeling tool on WATcloud; guideline doc; frozen val/test + replay bags; QA reports per batch | A (QA'd labels for release), C/D (eval sets) |
| **C. Training** | `wato-bevfusion-train`, configs, experiments, PTQ/QAT, Phase 2 model | G1 reproduction; Stage A/B/C runs; learning-curve study; 8-camera config | D (ONNX + model card) |
| **D. Deployment and on-car** | Export to TRT, wato-cuda-bevfusion + ROS node changes, multi-sweep at runtime, tracking hysteresis, metrics harness | Parity test; stability/latency harness; multi-sweep in `processLidar()`; 8-camera C++ changes; G4 on-car runs | Everyone (release) |

Agree on the interfaces in week 1 so nobody waits on anyone else:

- A → C: the dataset folder layout and `info.pkl` schema. C can start on a 100-frame hand-labeled toy set before v0.1 exists.
- C → D: the ONNX names and shapes stay the same as the 5 files deployed today.
- B → everyone: one evaluation script that everyone uses for metrics.

&#91;embedded content: Roadmap · 4 phases, 4 lanes, gates G1–G4\]

Weeks are counted from kickoff and are a proposal. Shift them around exams and co-op terms. No phase starts until the gate before it passes.

## Risks and open questions

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Labeling is slower than planned | Stage C slips | Pre-label and correct instead of labeling from scratch; Stage A needs only about 2k frames; everyone labels |
| The old training/export stack won't reproduce (mmcv/spconv versions, custom LayerNorm plugin) | Nothing ships | G1 is the first gate; pin a Docker image; ask NVIDIA's CUDA-BEVFusion issues for help early |
| Overfitting to campus routes | Works on test day, fails elsewhere | Split by route; keep nuScenes in the mix; deliberately plan diverse drives |
| Silent calibration drift | Accuracy slowly drops | Daily overlay check; calibration versions on every frame |
| Latency goes over budget at 8 cameras | Phase 2 can't deploy | Measure the camera branch cost in week 2; keep the 6-camera fallback |

Open questions:

- [ ] Which GPU is on the car, and what is the frame-rate target? These set the latency budget and where engines get built.
- [ ] Which labeling tool, and is there any budget for outsourced labeling?
- [ ] Keep all 10 nuScenes classes, or merge or drop some (construction vehicle, trailer)?
- [ ] Phase 1 with `lidar_cc` only, or the merged 3-LiDAR cloud?
- [ ] Is the K-vs-P intrinsics issue confirmed on all 8 cameras? (Data owner, week 1)
- [ ] Which weeks are the team away (exams, co-op), so the roadmap can be adjusted?
