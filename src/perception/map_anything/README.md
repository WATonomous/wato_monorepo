# map_anything

[MapAnything](https://github.com/facebookresearch/map-anything) (Meta, feed-forward metric multi-view 3D) served over ROS for eidos' visual plugins:

- `~/estimate_relative_pose` (`eidos_msgs/srv/EstimateRelativePose`): the camera rig's motion between two sets of images, from one MapAnything pass over all of them. With `return_cloud` it also returns the second set's dense coloured points in the rig frame (`map_anything/cloud.py`), about 3× slower.
- `~/compute_place_descriptors` (`eidos_msgs/srv/ComputePlaceDescriptors`): one [MegaLoc](https://github.com/gmberton/MegaLoc) place-recognition descriptor (MIT) per image.

Its clients are `eidos::VisualLoopClosureFactor`, `eidos::VisualRelocalization` and `eidos::MultiViewOdometryFactor` (`src/world_modeling/eidos/docs/plugins/`). They stay free of GPU and image libraries and send the images still compressed.

This is an **experiment**. Read [Results](#results) before adding its factors to a graph.

## How it works

```text
eidos visual plugin ── images of instant a + images of instant b (compressed, frame_id = camera) ──►
  decode ─ crop: cut the roof rack (crop_bottom), centre the principal point
         ─ intrinsics from CameraInfo.P of each camera
         ─ MapAnything (images + intrinsics only) ─► one metric pose per image, in the first image's frame
         ─ rig fit: each camera's own motion a→b, made rig motion through its TF extrinsic, averaged
◄── relative pose of rig_frame, camera-to-camera spread, confidence
```

- **Images and intrinsics go into the model, and optionally metric depth per image** (`depth_a` / `depth_b`: points in each image's camera frame, e.g. lidar moved to the image's time by `eidos::MultiViewOdometryFactor`). The server projects them into sparse depth maps (`map_anything/depth.py`); pixels without a point are 0. No poses or IMU go in. The extrinsics are used after the model, to turn camera motion into rig motion.
- **Principal point.** MapAnything's poses behave as if the principal point were at the image centre. Off-centre by some angle, every pose is tilted by that angle. Cutting 470 rows from only the bottom of the Boreas images tilted the translation direction by 9.3°. The server therefore always crops symmetrically about the principal point (`model.centred_view`).
- **Rig fit.** `fit: per_camera` averages each camera's own motion and is robust to the model placing cameras with little overlap badly relative to each other. It needs the vehicle to face roughly the same way at both instants. `fit: joint` averages each instant's rig pose over its cameras.
- **Spread.** `trans_spread` and `rot_spread` are the RMS disagreement between the cameras' estimates. The plugins use them to reject bad measurements.
- **Points.** With `return_cloud`, each camera of the second set has its pixels moved into its own camera frame with its own predicted pose, then onto the rig with its TF extrinsic, the same choice as `per_camera`: the model's placement of the cameras relative to each other is not used. Sky and depth edges are masked out; `cloud_max_range` and `cloud_voxel` thin the rest (about 34k points for 8 cameras at 0.25 m and 30 m).

## Usage

```bash
# perception (GPU) container
ros2 launch map_anything map_anything.launch.yaml              # Apache-2.0 checkpoint
ros2 launch map_anything map_anything.launch.yaml checkpoint:=facebook/map-anything   # CC-BY-NC, more accurate
```

- **Weights.** About 4.6 GB per checkpoint, plus MegaLoc. They download on first start into `cache_dir`, which defaults to the mounted `/opt/watonomous/models/map_anything`. The services appear once the models are loaded, about 30 s with a warm cache.
- **Enabling the plugins.** Add `visual_loop_closure_factor` to `factor_plugins` in `eidos_slam.yaml`, and `visual_relocalization` to `relocalization_plugins` in `eidos_localization.yaml`. Both config blocks are already there.
- **Prior map.** `visual_relocalization` needs a prior map saved while `visual_loop_closure_factor` was loaded. That plugin stores every keyframe's images and descriptors, about 1.2 MB per keyframe with three cameras.

### Evaluating

| Tool | What it does |
|---|---|
| `tools/eidos_visual_eval.sh <bag> <run>` | Runs eidos with given parameter overrides over a bag (playback, recording, optional map save). See its header. |
| `tools/eidos_visual_eval.py slam <runs>` | Odometry, loop closure and final-map metrics against the INS. |
| `tools/eidos_visual_eval.py relocalization --map <run> <runs>` | Start pose and tracking error of relocalization runs. |
| `tools/map_anything_pairs_eval.py` | Model-only pair accuracy on a bag or a Boreas sequence (no eidos). |
| `tools/map_anything_reconstruct.py` + `tools/map_anything_viewer.html` | Dense reconstructions to look at. |
| `tools/camera_lidar_calibrate.py --bag <bag> --tf <tf.yaml> --out <overrides.yaml>` | Fits each camera's rotation and the camera time offset against the lidar, by aligning projected lidar with MapAnything's image-only depth; reports the fit on held-out instants. |
| `tools/tf_static_override.py` | A bag's static TF with transforms replaced (`extract`), published instead of the bag's during an eval run (`publish`, used by `eidos_visual_eval.sh` with `TF_STATIC`). |
| `tools/map_anything_bag_topics.py --bag <bag> --out <dir>` | Writes MapAnything topics for a recorded drive into a separate mcap, stamped at the camera times, to open next to the bag in Foxglove: local 3D cloud, map chunks and full map (placed by eidos LISO), and MapAnything's own odometry and path next to the LISO path. |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `checkpoint` | `facebook/map-anything-apache` | Hugging Face checkpoint. `facebook/map-anything` (CC-BY-NC 4.0) roughly halved translation error in our tests. |
| `device` | `cuda` | |
| `cache_dir` | `/opt/watonomous/models/map_anything` | Hugging Face and torch hub caches. |
| `camera_info_topic` | `/{camera}/camera_info` | `P` gives the intrinsics (the images are rectified). |
| `cameras` | the 8 pano cameras | Subscribed at start-up; others on first use. |
| `extrinsics_file` | `""` | Optional `visual_odometry` calibration overriding TF. |
| `crop_bottom` | `110` | Rows of roof rack at the bottom of every image. |
| `fit` | `per_camera` | `per_camera` or `joint` (see above). |
| `place_recognition` | `true` | Also serve MegaLoc descriptors. |

## Results

**Setup**
- **Bag:** `recording_20260530_144049`, daylight, 646 s, 3.96 km. It contains two laps of the same loop, so about 60% of the second lap revisits the first lap's road in the same direction.
- **Hardware and model:** RTX 4090, cameras nw/nn/ne, CC-BY-NC checkpoint.
- **Reference:** the Novatel INS without RTK (about 1 m absolute). A fitted INS time offset (−15 to 0 ms) and INS-to-body rotation (about 2.7°) are removed per run. Over pairs minutes apart, the INS itself is good to roughly 0.5–1 m, so smaller differences there cannot be resolved; section 3b compares against GICP instead.
- **Eidos runs** (playback at 0.5×):

| Run | Graph |
|---|---|
| `standard` | Default config: GPS + LISO + GICP loop closure |
| `no closure` | LISO only |
| `GICP closure` | LISO + GICP loop closure |
| `visual closure` | LISO + MapAnything loop closure (no gate, no robust kernel, like GICP) |

The other plugin always ran alongside in shadow and measured the same keyframes.

**1. Odometry over one keyframe step (5.2 m).** MapAnything ran in shadow on consecutive keyframes.

| Source | Translation error (med / p90) | % of step | Rotation error (med / p90) | Scale (median) |
|---|---|---|---|---|
| MapAnything | 0.28 / 0.60 m | 5.3 | 0.28 / 0.55° | 1.036 |
| eidos graph (LISO) | 0.08 / 0.20 m | 1.5 | 0.07 / 0.16° | 0.998 |

**2. Odometry chained over 100 m / 200 m (drift).**

| Source | 100 m | 200 m |
|---|---|---|
| MapAnything | 4.4–5.1% and 3.3–3.7° | 5.9–6.0% and 6.2–7.4° |
| eidos graph (LISO) | 0.5% and 0.27° | 0.5–0.6% and 0.45° |

As odometry, MapAnything is about 10× worse than LISO and consistently overestimates distance by about 4%.

**3. Loop-closure measurements against the INS** (accepted measurements, median / p90).

| Run | Plugin | Measured | Accepted | Translation error | Graph drift when measured |
|---|---|---|---|---|---|
| no closure | MapAnything, distance candidates | 228 | shadow | 1.26 / 1.83 m | 11.8 m |
| no closure | MapAnything, place recognition | 202 | shadow | 0.72 / 1.54 m | 12.1 m |
| no closure | GICP | 225 | shadow | 0.87 / 1.91 m | 11.8 m |
| GICP closure | GICP | 234 | 234 | 0.93 / 2.38 m | 0.9 m |
| visual closure | MapAnything | 234 | 234 | 0.77 / 1.61 m | 0.8 m |
| standard, before the GPS fix | GICP | 242 | 242 | **22.65 / 31.88 m** | 22.7 m |
| standard, GPS fix | GICP | 235 | 235 | 2.06 / 3.22 m | 2.1 m |

- Place-recognition candidates were all real revisits: 435 of 435 within 4.3 m by the INS.
- One pose request takes 0.30 s; a GICP alignment takes 0.05 s.

**3b. MapAnything against GICP** on the same keyframe pairs, where GICP agrees with the INS within 2 m.

| Run | Pairs | Median distance apart | Difference (med / p90 / max) | Rotation (med / p90) |
|---|---|---|---|---|
| no closure | 201 | 8.7 m | 0.60 / 1.10 / 2.05 m | 0.41 / 0.80° |
| visual closure | 230 | 1.0 m | 0.11 / 0.20 / 0.46 m | 0.18 / 0.42° |

**4. Final map.** Revisit consistency compares keyframes of different passes within 10 m. ATE is after a rigid alignment; its max includes an INS glitch.

| Run | Revisit error (med / p90 / max) | Revisit rotation (med / p90) | ATE RMSE |
|---|---|---|---|
| standard, before the GPS fix | 22.40 / 31.36 / 32.19 m | 5.95 / 15.07° | 13.55 m |
| standard, GPS fix | 2.08 / 3.29 / 5.19 m | 0.70 / 1.96° | 2.96 m |
| no closure | 11.86 / 16.49 / 17.15 m | 1.73 / 1.87° | 5.66 m |
| GICP closure | 0.92 / 2.04 / 3.63 m | 0.31 / 1.35° | 3.93 m |
| visual closure | **0.81 / 1.63 / 1.91 m** | **0.22 / 0.48°** | 3.99 m |

**5. Relocalization** in a map of lap 1 (GPS factors off, GICP closure; the GPS plugin still stores its UTM-to-map offset), starting localization at six points of lap 2. Run with the GPS fix below.

| Start (s into the bag) | GPS+ICP start error | MapAnything start error | Tracking afterwards: GPS+ICP / MapAnything |
|---|---|---|---|
| 400 | 11.2 m / 69° (wrong GICP match) | 1.18 m / 1.7° | 94 m (lost) / 0.66 m |
| 420 | 1.09 m / 1.3° | 2.44 m / 0.3° | 0.41 / 0.38 m |
| 440 | 0.24 m / 2.6° | 0.78 m / 9.2° | 0.39 / 0.41 m |
| 460 | 0.37 m / 1.0° | 2.31 m / 0.4° | 0.65 / 0.76 m |
| 480 | 0.66 m / 1.6° | 0.88 m / 1.5° | 1.59 / 1.63 m |
| 560 | 0.71 m / 2.5° | 0.89 m / 3.3° | 0.72 / 0.73 m |

GPS+ICP answers in 0.2–0.5 s of bag time; MapAnything took 6–7 s here, because each run started with a freshly loaded model (1–4 s with a warm one).
Before the GPS fix, GPS+ICP relocalized at none of the six points correctly (guesses 190–660 m away), while MapAnything did not depend on it.

**Findings**
- **The default config failed on this bag until `gps_factor` was fixed, for a reason unrelated to MapAnything.** It set the UTM-to-map rotation from the INS heading at the first accepted fix and assumed the map heading was zero at that moment; in this bag the car had turned about 180° in the parking lot by then. That caused the 22 m revisit errors of the `standard` run (GICP closures accepted wrong alignments), GPS+ICP relocalization guessing positions 190–660 m away, and one map run diverging. It now reads the INS heading and GPS position at the newest keyframe's time and subtracts that keyframe's map heading. With the fix the default config reaches 2.1 m revisit error and the lowest absolute error of all runs (2.96 m ATE).
- **MapAnything is not competitive as odometry:** about 5% per step, against 1.5% for LISO.
- **As loop closure, MapAnything matched GICP on this bag:**
  - its final map was 0.81 m revisit error against GICP's 0.92 m;
  - it is within 0.11 m of GICP on the pairs both closed;
  - its place recognition finds revisits independently of the graph's drift.
- **As relocalization without GPS**, it succeeded at all six points (0.8–2.4 m), including the one where GPS+ICP took a wrong match. With working GPS, GPS+ICP is faster and slightly more accurate where it succeeds (0.2–1.1 m).

**Caveats**
- **One bag, daylight.** A night drive (`pano_lidar_cc_20261029_190612`) gave much worse model accuracy.
- **Checkpoint licence.** The CC-BY-NC checkpoint is not licensed for commercial use. The Apache one was about twice as far off in the pair tests (`tools/map_anything_pairs_eval.py`).
- **Speed.** MapAnything was tested at 0.5× playback. At full speed with keyframes every 5 m and 0.3 s per request, it keeps up with loop candidates but not with shadow odometry as well.
- **Memory.** Keyframe images cost about 1.2 MB each in memory and in the map (0.9 GB of images for 755 keyframes).

## Results: camera + IMU + GPS graph without lidar

`eidos::MultiViewOdometryFactor` creates the keyframes and adds MapAnything's motion between them; `eidos::ImuFactor` (full preintegration with velocity and bias states) and `eidos::GpsFactor` correct it.

**Setup**
- **Bag:** `sensors_2026_07_07-23_01_24_2`, daylight, 306 s, 3.1 km, no revisits (so no loop closures).
- **Model:** all 8 pano cameras, CC-BY-NC checkpoint, RTX 4090, playback at 0.2×. One request takes 0.7 s, or 3.5 s with the dense points (asked for every 3rd keyframe).
- **Graph:** keyframes every 5 m of GPS travel or 1 s; IMU from `/novatel/oem7/imu/data_raw` with `axes_rotation_rpy: [0, 0, -90]` (that topic is in the IMU's own axes, see `imu_factor.md`); one GPS fix per 20 m (`min_radius`). Parameters: `/mnt/data/bags/ma_work/params/{cam_imu_gps,imu_gps,cam_gps}.yaml`.
- **Reference:** the INS. The GPS fixes come from the same receiver, so the absolute errors (ATE) flatter every graph with GPS in it; the per-step and per-100 m errors do not.
- **Runs:**

| Run | Graph |
|---|---|
| `cam_imu_gps` | MapAnything + IMU + GPS |
| `imu_gps` | IMU + GPS on keyframes from the same plugin, MapAnything in shadow |
| `cam_gps` | MapAnything + GPS, no IMU |

**1. One keyframe step (5.2 m)**

| Run | Graph translation error (med / p90) | % | Graph rotation error (med / p90) | MapAnything alone |
|---|---|---|---|---|
| cam_imu_gps | 0.14 / 0.30 m | 2.7 | 0.12 / 0.30° | 0.26 m (5.2 %), 0.33°, scale 0.974 |
| imu_gps | 0.08 / 0.25 m | 1.4 | 0.02 / 0.06° | 0.28 m (5.2 %), 0.31° |
| cam_gps | 0.26 / 0.61 m | 5.0 | 0.35 / 0.72° | 0.27 m (5.3 %), 0.36° |

**2. As a pose source** (final keyframes and the live pose eidos published at each keyframe; ATE after one rigid alignment)

| Run | Final ATE RMSE / max | Live ATE RMSE / max (median) | Drift per 100 m (med / p90) | Rotation per 100 m (med / p90) |
|---|---|---|---|---|
| cam_imu_gps | 0.79 / 2.18 m | 1.84 / 4.25 m (1.46 m) | 1.48 / 1.99 % | 0.31 / 1.01° |
| imu_gps | **0.47** / 2.18 m | 1.51 / 11.66 m (**0.48 m**) | **0.94** / 1.52 % | **0.14** / 0.48° |
| cam_gps | 0.89 / 3.27 m | 2.68 / 7.68 m | 1.69 / 2.97 % | 1.37 / 2.46° |
| *context: earlier LISO + GPS run in this bag (live pose, older code)* | – | 1.25 / 4.93 m | 0.94 / 3.28 % | 0.42 / 1.31° |

**Findings**
- **Added to IMU + GPS, MapAnything made the pose worse while moving.** Its rotation (0.3° per step) is about 15× worse than the gyro's, and its distances are about 2.6 % short, a bias that averaging cannot remove. With the noise model used (0.46° and 0.4 m per 5 m step), it pulls the IMU's rotation and velocity off.
- **It does help at a standstill.** With one GPS fix per 20 m of travel, a parked car gets no GPS factors, and IMU + GPS alone drifted up to 11.7 m live while parked (20–31 s after tracking began). MapAnything's "no motion" measurements prevented that.
- **Without the IMU it is no better than MapAnything alone** in rotation: 1.4° per 100 m.
- **Next things to try** (not done): MapAnything as translation-only or with a much looser rotation, correcting its scale bias, a camera time offset (the camera stamps appear 60–90 ms off INS time, unresolved), and GPS outages, which is where MapAnything + IMU should beat IMU alone.

**Caveats**
- **One bag**, driven mostly straight.
- **Keyframes differ between runs** (671 / 551 / 536): when the plugin falls behind the 4 s image buffer it skips ahead. The `cam_gps` run ended its keyframes 2.5 km in.
- **Absolute errors are flattered** by GPS coming from the INS's own receiver, as noted above.

### With lidar depth and corrected extrinsics

Same drive and graph, with `lidar_cc` points moved to each image's time (`depth.enabled`) and given to MapAnything as sparse metric depth. Static TF corrected (`TF_STATIC`):
- **lidar_cc:** yaw 6.0 → 7.2° and pitch 2.42 → 0.54°, measured against the direction of travel. `base_link -> imu_link` was kept except for its pitch, now 0.
- **cameras:** each camera's rotation fitted against that lidar with `tools/camera_lidar_calibrate.py`. Corrections are about ±1–1.7°, with no common tilt left once the lidar pitch was fixed. On held-out instants, 54–65 % of lidar points agree with MapAnything's depth within 5 %, against 37–56 % with the nominal rotations.
- **camera time offset:** +18 ms.

The IMU + GPS baseline was rerun with the same TF.

| Run | MapAnything per step (translation, rotation, scale) | Graph per step | Graph drift per 100 m | Final ATE RMSE / max | Live ATE median / p95 / max |
|---|---|---|---|---|---|
| cam_imu_gps (no depth, old TF) | 0.26 m (5.2 %), 0.33°, 0.974 | 2.7 %, 0.12° | 1.48 %, 0.31° | 0.79 / 2.18 m | 1.46 / – / 4.25 m |
| **cam_imu_gps_depth** | **0.23 m (4.4 %)**, 0.32°, 0.963 | 1.9 %, 0.17° | **0.89 %**, 0.33° | 0.91 / 2.40 m | 1.08 / 2.24 / 3.58 m |
| imu_gps_fixedtf | (shadow) | **1.4 %, 0.02°** | 1.02 %, **0.14°** | **0.31 / 1.03 m** | **0.43** / 1.90 / 12.14 m |

MapAnything alone, chained over 100 m: 6.4 % and 4.3° without depth, 4.6 % and 2.6° with it.

**Findings**
- **Depth cut MapAnything's random error two- to three-fold:** per-step lateral scatter 0.18 → 0.085 m, vertical 0.051 → 0.039 m.
- **It did not remove its biases.** Distances stay short (median −3.65 %), and yaw keeps a left bias (+0.17° per 5 m step). With less noise, the biases now dominate: the distance bias is 0.8 of the per-step spread.
- **The graph treats the factor as unbiased.** It absorbs the yaw bias into the gyro bias (−0.47 against −0.31 mrad/s without the camera) and the distance bias into the velocity. So with depth the camera graph matches IMU + GPS in drift per 100 m, but is still worse in rotation and in absolute error while moving.
- **The fixed extrinsics also helped the IMU + GPS baseline:** final ATE 0.47 → 0.31 m.
- **Next:** estimate the biases in the graph (below).

### With bias states

The camera factor became `eidos::BiasedRelativePoseFactor`. Every keyframe gets three bias states (translation scale, yaw per metre, rotation scale), each a slow random walk; the IMU and GPS make them observable (`bias.*` in `multiview_odometry_factor.md`). Lidar depth and the corrected extrinsics were kept. The model followed the errors measured above:
- yaw lean exists only while moving (+0.001° per step stopped, +0.14° moving);
- turns are under-reported (right turns +0.25°, left +0.02°, straight +0.13° per step);
- the scale bias drifted from −2 % to −5 % over the drive.

| Run | Graph per step | Drift per 100 m | Final ATE RMSE / max | Live ATE median / p95 / max |
|---|---|---|---|---|
| **cam_imu_gps_depth_bias** | 1.3 %, 0.04°, scale 0.999 | **0.54 %**, 0.20° | 0.39 / **0.95** m | 0.52 / **1.16** / **2.30** m |
| cam_imu_gps_depth | 1.9 %, 0.17°, scale 0.985 | 0.89 %, 0.33° | 0.91 / 2.40 m | 1.08 / 2.24 / 3.58 m |
| imu_gps_fixedtf | 1.4 %, 0.02°, scale 1.000 | 1.02 %, **0.14°** | **0.31** / 1.03 m | **0.43** / 1.90 / 12.14 m |

**Findings**
- **The camera graph now beats IMU + GPS** on drift (0.54 % against 1.02 % per 100 m) and on the live pose's tail: p95 1.16 m, max 2.30 m, with no 12 m excursion while parked. It is close on median live error (0.52 against 0.43 m) and on final ATE (0.39 against 0.31 m). IMU + GPS keeps a slightly better rotation (0.14° against 0.20° per 100 m).
- **Estimated biases:** scale settled at −3.5 % to −5 %, and yaw at +1.3 to +3.2° per 100 m (both as measured offline). The rotation scale started near −11 % and settled near −1 % once there were turns to tell it apart from the yaw lean.
- **Caveats:** the noise model was not retuned; one drive; GPS from the INS's own receiver flatters absolute errors.

### Against eidos as configured today

`jul7_eidos_default` is `eidos_slam.yaml` unchanged: LISO + GPS (one fix per 150 m) + GICP loop closure. It ran on the same drive with the same corrected extrinsics. There were no loop closures, because the drive has no revisits.

| Run | Final ATE RMSE / max | Live ATE median / p95 / max | Drift per 100 m | Rotation per 100 m |
|---|---|---|---|---|
| cam_imu_gps_depth_bias | **0.39 / 0.95 m** | **0.52 / 1.16 / 2.33 m** | **0.54 %** | **0.20°** |
| eidos default (LISO) | 0.93 / 2.03 m | 1.12 / 2.08 / 3.63 m | 0.75 % | 0.57° |

- The camera graph's live pose was closer to the INS 78 % of the time. The two live poses were 1.19 m apart at the median and 4.4 m at most.
- **The setups differ in more than the odometry:**
  - **GPS:** the camera graph had a fix every 20 m (139 factors), the default every 150 m (20 factors). Since the GPS shares the INS's receiver, part of the absolute-error gap comes from that. IMU + GPS alone, also at 20 m, reached a 0.43 m live median.
  - **IMU:** the default graph has no IMU factor.
- **Drift and rotation are measured relative to the INS over 100 m, which the extra GPS affects much less.** There the camera graph also leads: 0.54 % against 0.75 %, and 0.20° against 0.57° per 100 m.
