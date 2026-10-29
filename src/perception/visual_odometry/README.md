# visual_odometry

GPU multicamera visual odometry on the roof pano cameras, using NVIDIA cuVSLAM (the PyCuVSLAM wheel, v17). It publishes `nav_msgs/Odometry` so eidos can use it as an extra odometry factor (`eidos::OdometryFactor`; see `src/world_modeling/eidos/docs/plugins/factors/odometry_factor.md`).

This is an **experiment**. The cameras are not hardware-synchronized, and the pair extrinsics are self-calibrated. Read [Limitations](#limitations) before trusting its output.

## How it works

```text
/camera_pano_{nn,nw,...}/image_rect_compressed   (rectified JPEG, 1280x1024, 20 Hz, free-running)
        │  FrameGrouper: nearest frame to the reference camera (first in `cameras`) within sync_slop_ms
        ▼
  frame set ── JPEG decode (gray) ── RigPreprocessor: rectify each stereo pair into a shared view
        ▼
  cuVSLAM Multicamera odometry (rig = base_link)  ──►  /perception/visual_odometry/odometry
                                                        vo_odom -> base_link, reference camera stamp
```

**Why the pairs are rectified.** Neighbouring pano cameras look 45° apart with only about 18° of overlap and a 0.184 m baseline. cuVSLAM's cross-camera matching gets **0 landmarks** on the raw nn/nw pair. A synthetic sweep shows why: its matching works up to about 20° of relative yaw and fails from 30°. `rectify.py` therefore re-renders each overlapping pair (`stereo_pairs`) as a rectified stereo pair. Both virtual cameras look along the bisector of the two optical axes (e.g. +22.5° for nw/nn), with x along the baseline. Pixels that have no source data, and the roof rack at the bottom of the images (`border`), are masked for cuVSLAM.

**Extrinsics.** The URDF pano rotations are nominal xacro geometry and are 1–2° off. After rectification that leaves 12–15 px of row misalignment, and the rectified pair then finds almost no matches. `calibrate_pairs` fits the pair rotations from the images themselves. The result is `config/pano_extrinsics.yaml`, which overrides TF for those cameras in this package only; the URDF is not changed. See [Calibration](#calibration).

**Output contract.**
- Odometry is published only while tracking is valid. A gap in the stream (tracking loss) is the reset signal for consumers.
- The pose is the rig (`base_link`) in `vo_odom`, which is `base_link` at the start of the tracking session.
- Pose covariance is cuVSLAM's. For the single-pair rig it is uninformative (σ ≈ 1).
- The twist is a finite difference in the body frame.
- No TF is broadcast.

## Camera sets

| Config | Cameras / pairs | Sync within each pair | Parked split-2 result (49.6 s) |
|---|---|---|---|
| `nn_nw.yaml` (default) | nn, nw / nw:nn | 3 ms | 993/994 sets tracked, ~60 landmarks, drift 10.9 cm / 0.14° |
| `front3.yaml` | + ne / nw:nn, nn:ne | 3 ms, 21 ms | 993/994, ~63 landmarks, drift 2.8 cm / 0.14° |
| `all6.yaml` | nn, nw, ne, ss, se, sw / nw:nn, nn:ne, se:ss, ss:sw | 3, 21, 23, 22 ms | 992/993, ~174 landmarks, drift 2.3 cm / 0.07° |

Parked, more pairs means less drift. While driving, a pair whose images are ~20 ms apart has a moving baseline (see [Limitations](#limitations)). `nn_nw` stays the default until a moving bag shows which set is better.

Processing on an RTX 4090 (offline, including JPEG decode and remap): `nn_nw` 50 s of data in 9 s; `all6` in 28 s. cuVSLAM tracking alone takes 0.6 ms (`nn_nw`) or 4.5 ms (`all6`) per set.

## Usage

The node runs in the perception container. `docker/perception.Dockerfile` pip-installs the pinned cuVSLAM wheel.

**Live, or over a bag played with `--clock`:**

```bash
ros2 launch visual_odometry visual_odometry.launch.yaml [config_file:=.../all6.yaml] [use_sim_time:=true]
```

**Offline, frame by frame.** This path is reproducible because no frames are dropped and async bundle adjustment is off. The output bag is logged at camera stamps, so it can be replayed next to the input:

```bash
ros2 run visual_odometry offline_vo --bag <bag dir or .mcap> --config <config.yaml> --out <out_dir> \
  [--start S] [--duration S]
```

**Into eidos, offline.** Enable the factor in a params override, then run the eidos runner with the VO bag:

```yaml
# vo_on.yaml
/**/eidos_node:
  ros__parameters:
    factor_plugins: ["gps_factor", "liso_factor", "euclidean_distance_loop_closure_factor", "visual_odometry_factor"]
    visual_odometry_factor:
      add_factors: false   # shadow mode: residuals only
```

```bash
BAG_DIRECTORY=<dir with both bags> EIDOS_VO_BAG=<offline_vo out_dir> EIDOS_EXTRA_PARAMS=vo_on.yaml \
  tools/eidos_offline_to_bag.sh <input.mcap> 0.5
```

**Evaluate:**

```bash
tools/vo_eval.py --vo <offline_vo out_dir> [--ref <..._eidos bag>] [--imu <sensor bag>]
```

It reports uptime and gaps, drift per session, the VO-to-IMU time offset (which needs turning), relative pose error and scale against `liso/odometry` (which needs driving), and the eidos factor residuals.

## Calibration

```bash
ros2 run visual_odometry calibrate_pairs --bag <parked bag> --config config/all6.yaml --check      # report only
ros2 run visual_odometry calibrate_pairs --bag <parked bag> --config config/all6.yaml --out pano_extrinsics.yaml
```

The tool proceeds per pair:
1. It collects SIFT matches over many frame sets and removes mismatches (such as repetitive hedges) with a RANSAC epipolar check.
2. It keeps the anchor cameras (`calibration_anchors`: nn and ss) nominal and rotates the other camera of each pair about the pair's rectified axes. Translations stay nominal.
   - The rotations about the baseline and viewing axes are **fitted** so that rectified rows align. On the parked bag the residual is about 0.15 px (MAD 0.27 px).
   - The rotation **along the epipolar lines** is not observable from two-view matches without known depth. Any error in it becomes a stereo depth bias, and therefore VO scale error. It is set by a heuristic: the farthest matches (2nd percentile) sit at 0.5 px of disparity.

The current file came from `sensors_2026_09_13-20_27_23_2` (parked):

| Camera (vs anchor) | Rotation change | Vertical disparity before → after |
|---|---|---|
| nw (vs nn) | 1.28° | +15.4 → +0.14 px |
| ne (vs nn) | 0.95° | −12.5 → +0.17 px |
| se (vs ss) | 1.06° (almost all along the epipolar lines) | −2.2 → −0.18 px |
| sw (vs ss) | 0.26° | −1.2 → 0.00 px |

Recalibrate whenever a camera is remounted. On a moving bag, check the `vo_eval.py` scale. If it is consistently ≠ 1, the along-epipolar rotation is biased: adjust `--far-disparity` and recalibrate, or do a target-based calibration.

## Parameters

Each config is a ROS 2 params file. `offline_vo` and `calibrate_pairs` read the same file.

| Parameter | Default | Description |
|---|---|---|
| `cameras` | nn, nw | Physical cameras. The first one is the reference, and its stamp is the frame-set stamp. |
| `stereo_pairs` | `[]` | `"cam_a:cam_b"` overlapping pairs to rectify. When empty, the raw cameras go to cuVSLAM, which fails on this rig. |
| `rectified_stereo_camera` | `true` | Tell cuVSLAM the pairs are rectified. |
| `extrinsics_file` | `pano_extrinsics.yaml` | Calibrated `rig_from_camera` overrides, relative to the config dir. `""` means TF only. |
| `calibration_anchors` | nn | Cameras `calibrate_pairs` keeps nominal. |
| `rig_frame` / `odom_frame` | `base_link` / `vo_odom` | Rig frame (the extrinsics' target) and the odometry frame. |
| `sync_slop_ms` | 10 (nn_nw), 30 | Largest allowed \|stamp − reference stamp\| for a camera to join a set. |
| `max_wait_ms` | 300 | Finalize a set without a camera that stopped publishing. |
| `allow_incomplete` | `false` | Track sets with a missing camera (it is passed to cuVSLAM as empty). |
| `downscale` | 1 | Decode JPEGs at 1/2, 1/4 or 1/8 resolution. |
| `border` | `[0, 110, 0, 0]` | Pixels ignored at the top, bottom, left and right (the roof rack is at the bottom). |
| `multicam_mode` | `precision` | cuVSLAM `performance`, `precision` or `moderate`. |
| `async_sba`, `use_motion_model`, `use_denoising` | `true`, `true`, `false` | cuVSLAM odometry options. `offline_vo` forces `async_sba` off. |
| `queue_size` | 5 | Frame sets waiting for the GPU before the oldest is dropped (counted as `dropped`). |
| `verbosity` | 0 | cuVSLAM log level. |

`visual_odometry/status` (`diagnostic_msgs/DiagnosticArray`, ~1 Hz) carries:
- counts of tracked, lost, incomplete and dropped sets, and resets;
- `track_ms`, landmark counts and observation counts;
- each camera's measured offset to the reference camera.

## Limitations

**No hardware sync.**
- The Hikrobot cameras free-run at 20 fps with `TriggerMode Off`. An STM32 trigger generator exists (`src/embedded/camera_sync`) but is not connected, and the Hikrobots do not support PTP.
- The offsets to nn are stable within a recording, and can change from one power-up to the next: nw +3.0 ms, sw −3.3, se −4.1, ss +18.5, ne +21.2.
- cuVSLAM expects the frames in a set to be within 1 ms of each other.
- Inside a stereo pair, a stamp offset Δt at speed v shifts the baseline by v·Δt:
  - nw/nn: 3 ms × 15 m/s = 4.5 cm, or 25% of the 0.184 m baseline.
  - ne/nn and the rear pairs: about 20 ms, or 0.3 m, which is more than the baseline itself.

  Expect depth, and so scale, errors that grow with speed. Parked data cannot show this.
- The fix is hardware: trigger the cameras (`TriggerMode On` / `Line0`) from the camera_sync board.

**Stamps are host arrival time** (aravis system timestamp), not exposure time.
- Auto-exposure (0.1–15 ms) and GigE transfer add a variable delay, which is not modelled.
- The stamp-to-bag delay is 30–47 ms.
- eidos' `time_offset` corrects only the mean. Estimate it with `vo_eval.py --imu` on a drive with turns.

**Self-calibrated, not measured, extrinsics.**
- Only the relative rotation of each pair is fitted, and its along-epipolar part is a heuristic.
- The rig-to-`base_link` orientation and all translations are nominal URDF values.
- The front and rear groups are calibrated independently: ee and ww are not recorded, so no pair links them.

**Small overlap and baseline.**
- Each pair shares about 18° of view.
- A 0.184 m baseline gives about 19 px of disparity at 10 m and 2 px at 100 m, so far features carry little depth.

**No camera–IMU sync, so no VIO.** cuVSLAM's inertial mode needs synchronized clocks. In the Sept 2026 bags `/novatel/oem7/imu/data` also has zero angular velocity and acceleration, so use `imu/data_raw` for gyro.

**Image quality.** JPEG q90 artifacts; rectification is done on the car with its own calibration; night, glare and rain are untested.

**Dynamic objects are not masked.** Pedestrians and cars in view can bias tracking. The parked test bag is a busy plaza.

**Validated parked only.** Accuracy while driving (relative pose error and scale against LISO/INS) has not been measured yet.

**Requirements.**
- An NVIDIA GPU. Unit tests run without one, using a fake `cuvslam`.
- cuVSLAM is under the NVIDIA Community License. The wheel is downloaded at image build time and is not vendored.

## Tests

```bash
colcon test --packages-select visual_odometry   # geometry, grouping, TF tree, rectification, calibration, VoCore (fake cuvslam)
```
