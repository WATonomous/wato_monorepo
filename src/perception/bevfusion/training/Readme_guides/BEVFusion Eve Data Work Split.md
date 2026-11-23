# BEVFusion Eve Data: Work Split

Nov 22, 2026 · @Shourrya and Claude Opus 5.5

This document splits the work of making Eve data plug into the existing training pipeline into three independent sub-tasks. Background is in **[BEVFusion on WATonomous Car: Training, Data Pipeline & Team Plan](./BEVFusion%20on%20WATonomous%20Car%20Training,%20Data%20Pipeline%20%26%20Team%20Plan.md)** (the "team plan"). The pipeline being plugged into is described in **[pipeline_readme.md](./pipeline_readme.md)**.

| | **Sub-task 1: Bags → nuScenes format** | **Sub-task 2: Labels and splits** | **Sub-task 3: Pipeline adapter and 6 → 8 cameras** |
| --- | --- | --- | --- |
| **In one line** | Learn the nuScenes format and convert our rosbags to it | Put 3D boxes on the converted frames | Make training, export and the ROS node accept Eve data with 6 or 8 cameras |
| **Input** | Eve rosbags | Any dataset in nuScenes format | Any dataset in nuScenes format |
| **Output** | Sensor files + 8 tables | 5 annotation tables + `splits.json` | Code and configs, no data |
| **Develops against** | One real Eve bag | nuScenes mini | "Fake Eve", made from nuScenes mini |
| **Team plan roles** | A | B | C and D |

**Out of scope:** recording drives, the training stages themselves (0, A, B, C), multi-sweep input at runtime, tracking.

## Contents

1. [How the pieces fit](#how-the-pieces-fit)
2. [The contract](#the-contract)
3. [Sub-task 1: Bags → nuScenes format](#sub-task-1-bags--nuscenes-format)
4. [Sub-task 2: Labels and splits](#sub-task-2-labels-and-splits)
5. [Sub-task 3: Pipeline adapter and 6 → 8 cameras](#sub-task-3-pipeline-adapter-and-6--8-cameras)
6. [Putting it together](#putting-it-together)
7. [Corrections to the team plan](#corrections-to-the-team-plan)
8. [Sources](#sources)

---

## How the pieces fit

```
 Eve bags ──► [1] converter ──► sensor files + 8 tables ──┐
                                                          ├──► Eve dataset root ──► [3] adapter ──► train / eval / export
 LiDAR + images ──► [2] labels ──► 5 tables + splits.json ┘    (nuScenes format)     (6 or 8 cams)
```

Three choices keep the sub-tasks independent:

- **No shared files.** The nuScenes format has 13 JSON tables. Sub-task 1 writes 8 and Sub-task 2 writes 5. Sub-task 3 writes none.
- **Everyone has stand-in data from day one.** Sub-tasks 2 and 3 start on nuScenes mini, which is already on the cluster, so neither waits for the converter.
- **Camera count is not a property of the data.** The dataset always holds all 8 cameras. Sub-task 3's config picks which ones to load, so going from 6 to 8 needs no reconversion and no relabeling.

Labels are drawn on converted Eve frames, so real Eve labeling starts after Sub-task 1's first output. Building and testing the labeling workflow does not.

---

## The contract

Agree on this section before splitting up, and change it only with all three owners present. It is the shape of the puzzle piece.

### Decisions to make first

| # | Decision | Recommendation | Why it matters |
| --- | --- | --- | --- |
| **D1** | LiDAR axes | Use nuScenes axes (**x right, y forward, z up**) in the dataset **and** in the ROS node | From the URDF and the node code, `lidar_cc` is about x-forward (REP-103) and the node feeds it in unrotated. That is about 90° from what the checkpoint was trained on, and rotation augmentation only covers ±45°. **Confirm on a bag first.** Whatever is chosen, the data and the node must match. |
| **D2** | Intrinsics | Top-left 3×3 of `CameraInfo.P` | Bags hold rectified images. The node uses `K` today (`bevfusion_node.cpp:560`). In all 8 pano configs, `fx` in `K` is 1.4–2.4% larger than in `P`. |
| **D3** | Global frame | Local metric frame, with an origin subtracted per site | `NuScenesDataset` casts `ego2global` to float32, which is only accurate to about 0.5 m at UTM northings. |
| **D4** | Sync limit | Drop a keyframe if any of the 8 cameras is more than 50 ms from the LiDAR stamp | Same value as the team plan. |

If D1 is accepted, the dataset's LiDAR sensor is a virtual frame: `lidar_cc` with its axes swapped (`x_new = -y`, `y_new = x`). Points, `calibrated_sensor` and label boxes all use it.

### Dataset layout

```
$SHARED/eve/<version>/                    # immutable once released, e.g. eve-bev-v0.1
├── samples/LIDAR_TOP/*.pcd.bin           # [1] keyframes
├── samples/CAM_PANO_{NN,NE,EE,SE,SS,SW,WW,NW}/*.jpg   # [1]
├── sweeps/LIDAR_TOP/*.pcd.bin            # [1] every LiDAR frame between keyframes
├── maps/empty.png                        # [1] placeholder, see step 1.5
├── splits.json                           # [2]
├── manifest.json                         # [1] bags, calibration IDs, converter git SHA
└── v1.0-trainval/                        # folder name is fixed by upstream
    ├── sensor, calibrated_sensor, ego_pose, log, scene, sample, sample_data, map  (.json)  # [1]
    └── category, attribute, visibility, instance, sample_annotation               (.json)  # [2]
```

| Item | Rule |
| --- | --- |
| LiDAR channel | `LIDAR_TOP`. Hard-coded in both upstream BEVFusion and the devkit. |
| LiDAR file | float32 rows of `x, y, z, intensity, ring`, extension `.pcd.bin`. The loader overwrites column 5 with the sweep time lag. |
| Camera channels | `CAM_PANO_*`, all 8, always. 1280×1024 JPEG, rectified. |
| Keyframes | 2 Hz on `lidar_cc` stamps. Scenes are 20 s clips. |
| Units | Timestamps in integer microseconds, metres, quaternions as `[w, x, y, z]` |
| Tokens | **Deterministic**: a hash of bag name + topic + stamp. Labels refer to sample tokens, so rerunning the converter must not change them. |
| `splits.json` | `{"train": [scene names], "val": [...], "test": [...]}` |
| Label source of truth | Boxes in the **`LIDAR_TOP` frame**, keyed by sample token. Global coordinates are computed at export, so a calibration fix does not invalidate labels. |

### Code layout

```
src/perception/bevfusion/training/eve/
├── check_dataset.py     # contract checker (Sub-task 1 writes it, everyone runs it)
├── convert/             # Sub-task 1
├── labels/              # Sub-task 2
└── pipeline/            # Sub-task 3
```

As in the nuScenes exercise, upstream code is wrapped, not edited.

---

## Sub-task 1: Bags → nuScenes format

**Objective:** one command turns an Eve bag into a dataset root that the stock nuScenes devkit opens and renders correctly.

**Needs from others:** nothing. Use any short `all_sensors_compressed` bag that has `/tf_static`. Write empty lists (`[]`) for the 5 annotation tables.

### Learn the format

- [ ] **1.1** Work through the nuScenes devkit tutorial on v1.0-mini. Write a one-page note on the tables below and commit it in `eve/convert/README.md`.

| Table | Eve source |
| --- | --- |
| `sensor` | 9 rows: 8 cameras and `LIDAR_TOP` |
| `calibrated_sensor` | `/tf_static` (sensor → ego frame) + `CameraInfo.P` (D2). One set per calibration ID. |
| `ego_pose` | `/novatel/oem7/odom`, interpolated to **each** sensor stamp (D3) |
| `log` | One row per bag |
| `scene` | One row per 20 s clip |
| `sample` | One row per keyframe, linked with `prev`/`next` |
| `sample_data` | One row per file, linked with `prev`/`next` per channel. Sweeps have `is_key_frame: false`. |
| `map` | One placeholder row listing every log token |

### Build the converter

- [ ] **1.2 Inspect a bag.** Record for the target bags: topic rates, point fields, stamp offsets between cameras and LiDAR, the odometry frame IDs, and whether `/tf_static` is present. The Velodyne config asks for 1200 rpm (20 Hz) and stamps scans at the last packet, but a DEVELOPING.md sample shows about 10 Hz, so measure it. **Settle D1 here.**
- [ ] **1.3 Extract and sync.** Choose keyframes, pick the nearest image per camera, apply D4. Log the reason for every dropped frame.
- [ ] **1.4 Write sensor files.** Copy JPEG bytes straight from `CompressedImage.data` (no re-encoding). Write LiDAR `.pcd.bin`, dropping NaN/Inf points.
- [ ] **1.5 Write the 8 tables.** The devkit needs all 13 tables to exist. It also crashes if `map` is empty, if a log has no map row, or if the map's PNG is missing, hence `maps/empty.png`.
- [ ] **1.6 Write `check_dataset.py`.** Checks: all 13 tables load; every keyframe has 9 channels; links are unbroken; files exist; stamps are within D4; tokens are unchanged on a second run.

### Check geometry

- [ ] **1.7 LiDAR-on-image overlay.** Project LiDAR into all 8 cameras with the devkit and save one PNG grid per bag. Include `EE` and `WW`, which BEVFusion has never used on Eve. The pano extrinsics in `eve_roof_mount.xacro` are an ideal 45° ring, not individually calibrated, so expect to find misalignment.
- [ ] **1.8 Sweep stacking.** Stack 10 sweeps using the tables. Parked cars and walls must stay sharp while Eve is driving. Smearing means the ego pose or LiDAR-to-ego transform is wrong.

**Done when:** `check_dataset.py` passes on at least 3 bags, the overlays line up on all 8 cameras, and static objects are sharp in the 10-sweep stack.

---

## Sub-task 2: Labels and splits

**Objective:** a repeatable path from unlabeled frames to the 5 annotation tables, with quality control and a fixed train/val/test split.

**Needs from others:** nothing to start. The test is a **round trip on nuScenes mini**: export mini's ground truth to the labeling tool's format, import it, export it back, and compare with the original.

- [ ] **2.1 Pick the labeling tool.** It needs LiDAR + multi-camera view, 3D cuboids and track IDs across frames. Candidates: Xtreme1, CVAT. Try both on one mini scene, then host the winner on WATcloud.
- [ ] **2.2 Importer.** nuScenes format → tool format. Must read the camera list from `sensor.json`, so 6 (mini) and 8 (Eve) both work.
- [ ] **2.3 Pre-labels.** Run the published checkpoint offline and load its boxes as drafts.
  > Yaw trap: the training code stores yaw as `-heading - π/2`. The ROS node publishes `-z_rotation` with `size.x = width`, so its published yaw is 90° off the true heading. If you pre-label from the node's topic, use `heading = published_yaw - π/2`.
- [ ] **2.4 Guideline (one page).** nuScenes box rules: tight box, size as `width, length, height`, yaw along heading, the 10 existing classes, minimum LiDAR points. Labels go all the way round Eve, including the sides that only `EE`/`WW` see, so Phase 2 needs no relabeling.
- [ ] **2.5 Exporter.** Tool format → 5 tables:
  - Boxes are converted `LIDAR_TOP` → ego → global using the tables in the dataset root.
  - `num_lidar_pts` is counted from the point cloud, and `num_radar_pts` is 0. Training drops boxes with no points.
  - Category names are the full nuScenes ones (`vehicle.car`, `human.pedestrian.adult`, …).
  - One `instance` per track, with `prev`/`next` links. Velocity is derived from these, so a broken track ID gives a wrong velocity.
  - Attributes are optional, but without them mAAE and NDS are not comparable with nuScenes. Report mAP instead.
- [ ] **2.6 QA.** A second person reviews a random 10% of each batch. Reject the batch if more than 5% of boxes need fixes (proposed). Script the obvious checks: zero-point boxes, implausible sizes, speed jumps, class changes within a track.
- [ ] **2.7 Splits.** Split by route and scene, never by frame. About 15% of scenes each for frozen val and frozen test. Report the per-class box count per split. Sub-task 3 needs to know which classes are empty.

**Done when:** the mini round trip reproduces the original boxes (centre within 1 cm, yaw within 0.5°, same tracks), the guideline is approved, and one real Eve scene is labeled, reviewed and exported once Sub-task 1 delivers it.

---

## Sub-task 3: Pipeline adapter and 6 → 8 cameras

**Objective:** `prepare_data.sh`, `train.sh`, evaluation, export and `bevfusion_node` all accept an Eve dataset root, with the camera list as a config setting.

**Needs from others:** nothing. Write `make_fake_eve.py`, which turns mini into a contract-shaped dataset:

- Rename the 6 cameras to their Eve equivalents, and resize to 1280×1024, scaling the intrinsics to match.
- Copy `CAM_FRONT_RIGHT` / `CAM_FRONT_LEFT` as `EE` / `WW`, with their calibration.
- Write `splits.json`, and remove one class entirely (see 3.3).

### Part A: the adapter

- [ ] **3.1 Info files.** Upstream `create_data.py` cannot be used as is:

| Upstream behaviour | Eve fix |
| --- | --- |
| `_fill_trainval_infos` hard-codes 6 nuScenes camera names | Own version of that function, reusing `obtain_sensor2top`. Always stores 8 cameras. |
| Splits come from hard-coded nuScenes scene names | Read `splits.json` |
| Any scene not in train goes into **val** | Handle test explicitly, or the test set leaks into val |
| Runs trainval, then test (no `v1.0-test` exists for Eve) | Call the info and GT-database functions directly |

- [ ] **3.2 Dataset class.** `EveDataset(NuScenesDataset)`, with a `cameras` list that selects channels at load time. Evaluation must also use `splits.json`: the devkit's `create_splits_scenes` is hard-coded to the 1,000 nuScenes scenes.
- [ ] **3.3 Configs.** One Eve config inheriting from the ResNet-50 `convfuser.yaml`:

| Setting | Problem | Fix |
| --- | --- | --- |
| `LoadBEVSegmentation` | Looks up a nuScenes city map, so it fails on Eve | Remove it and `gt_masks_bev` from both pipelines |
| `augment2d.resize` | Test value 0.48 makes Eve images 614 px wide, so the 704 px input is padded with black | About `[0.55, 0.65]` for train. Exactly 0.55 for test, to match the node. |
| `CBGSDataset` | Divides by zero if a class has no boxes | Drop it, or skip empty classes |
| `ObjectPaste` | `KeyError` if a class has no database entry | Remove empty classes from `filter_by_min_points` and `sample_groups` |
| Eve + nuScenes mix | Different resize, and different camera counts at 8 | Per-dataset pipelines. Never mix camera counts in one batch. |

- [ ] **3.4 Runtime match.** Apply D1 and D2 in `bevfusion_node`: rotate the points and compose the same rotation into the camera and target-frame transforms (D1), and use `P` instead of `K` (D2). Update `test_bevfusion_node.cpp`.

### Part B: 6 → 8 cameras

The network needs no new layers: the backbone is shared, and no fixed 6 was found in the view transform or fusion model code. The work is in the plumbing around it, and **all of it can be tested before any Eve data exists**.

- [ ] **3.5 Train with 8.** Run a short head-only job on fake Eve with 6 cameras, then 8. Same checkpoint, only the `cameras` list changes. Record GPU memory.
- [ ] **3.6 Export with 8.** `export-camera.py` traces `example-data/example-data.pth`, which fixes N at 6. Generate an 8-camera example and re-export `camera.backbone` and `camera.vtransform`. The other three files keep their shapes.
- [ ] **3.7 Fork.** In `wato-cuda-bevfusion`, the core library already uses `num_camera`. The 6 is hard-coded only in `src/python.cpp` (used by `test-mAP-for-cuda.py`) and `src/main.cpp` (demo).
- [ ] **3.8 Node and bringup.** `num_cameras` already comes from `camera_names`. Add `ee`/`ww` to it in `params.yaml` and `perception_bringup.yaml`, uncomment them in `multi_camera_sync` and `all_cameras_composed.yaml`, and update DEVELOPING.md.
- [ ] **3.9 Latency.** Measure 6 vs 8 on the car's GPU using nuScenes weights. Accuracy does not matter for this. The camera branch should cost about 1.33×. Do this early, because it decides whether Phase 2 is possible.

**Done when:** on fake Eve, the 6- and 8-camera configs both prepare, train, evaluate and export; the 8-camera engines load in `bevfusion_node`; the 6-camera nuScenes results are unchanged; and 6 vs 8 latency is reported.

---

## Putting it together

| # | Check | Pieces |
| --- | --- | --- |
| I1 | Sub-task 3's adapter prepares and loads a real converted bag (no labels) | 1 + 3 |
| I2 | One real Eve scene goes through the labeling tool and back | 1 + 2 |
| I3 | Ground truth drawn on LiDAR and on all 8 cameras lines up, from a full dataset root | 1 + 2 |
| I4 | Overfit: training on about 100 labeled Eve frames drives the loss down, and predicted boxes match the labels on those frames | all |
| I5 | The same frames give matching boxes in PyTorch and in `bevfusion_node` playing the bag (proves D1 and D2). Use single-sweep input in both, since the node does not stack sweeps. | all |

After I5, the dataset is ready for the team plan's Stage A.

---

## Corrections to the team plan

Found while checking the plan against the repo and the pinned upstream code:

| Team plan says | What the code shows |
| --- | --- |
| Our own channel names need "almost no code changes" | Camera names, splits and evaluation sets are hard-coded upstream. Sub-task 3 Part A covers it. |
| Write 7 nuScenes tables | The devkit needs 13 |
| *(not mentioned)* | `lidar_cc` axes appear to be 90° from nuScenes (D1). To be confirmed on a bag. |
| Is K vs P confirmed on all 8 cameras? | All 8 calibration files differ by 1.4–2.4% in `fx`. Whether the rectifier outputs exactly `P` still needs the overlay check. |
| Remove any hard-coded 6 in wato-cuda-bevfusion | Only in `python.cpp` and `main.cpp`. The core is already general. |
| New repo `wato-bevfusion-train` | Superseded. The code lives in `training/`. |

## Sources

- Upstream at `db75150`: [`nuscenes_converter.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/tools/data_converter/nuscenes_converter.py), [`create_data.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/tools/create_data.py), [`nuscenes_dataset.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/mmdet3d/datasets/nuscenes_dataset.py), [`loading.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/mmdet3d/datasets/pipelines/loading.py), [`dataset_wrappers.py`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/mmdet3d/datasets/dataset_wrappers.py), [`configs/nuscenes/default.yaml`](https://github.com/mit-han-lab/bevfusion/blob/db75150717a9462cb60241e36ba28d65f6908607/configs/nuscenes/default.yaml)
- nuScenes devkit (read from `master`; check against the version in the training image): [`nuscenes.py`](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/nuscenes.py), [`loaders.py`](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/eval/common/loaders.py), [schema](https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/schema_nuscenes.md)
- [wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion), read at `4da4989`
- This repo: `bevfusion_node.cpp`, `params.yaml`, `perception_bringup.yaml`, `eve_roof_mount.xacro`, `camera_calib/config/`, `velodyne_lidar_config.yaml`, `bag_recorder/all_sensors_compressed.yaml`
