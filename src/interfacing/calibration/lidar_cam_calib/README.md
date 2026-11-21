# lidar_cam_calib

Target-less extrinsic calibration of the pano cameras against the roof LiDAR (`lidar_cc`), using [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) (Koide et al., "General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox", ICRA 2023).

## Overview

The toolbox aligns LiDAR intensity with image brightness (normalized information distance), so it needs no checkerboard. It assumes intrinsics are already calibrated: those come from [camera_calib](../camera_calib/README.md).

This package automates the parts around it:

1. **Find segments**: scan a folder of bags, estimate the car's motion from the LiDAR (scan-to-scan GICP, cross-checked with IMU, CAN speed or odometry when recorded), and find:
   - **creep** segments: the car at rest for 3 s, then moving slowly for up to 5 m. The toolbox uses the first image and builds a dense cloud from the motion (`preprocess -d`).
   - **static** segments: the car parked. Sparser clouds, used without `-d`.
2. **Cut**: copy each segment into a small bag with the LiDAR cleaned up (NaN points removed, per-point time rebuilt when the recording has none), plus a preview of each camera's first frame.
3. **Calibrate repeatedly**: per camera, one run on all segments, one per segment left out, and several from a randomly perturbed initial guess. The initial guess is the current URDF.
4. **Summarize**: spread across runs, change from the URDF, position on the roof ring, a verdict, and URDF joints to paste.

Everything runs in one Docker image, locally or as WATcloud batch jobs. The cameras are the 6 pano cameras perception uses (`config/calib.yaml`).

## Quick start (local)

```bash
cd ~/wato_monorepo/src/interfacing/calibration/lidar_cam_calib
```

```bash
scripts/docker_run.sh python3 /ws/src/interfacing/calibration/lidar_cam_calib/scripts/extract_segments.py /bags /out
```

Look at `bags/lidar_cam_calib/previews/`, then disable segments or cameras in `bags/lidar_cam_calib/manifest.yaml` (`enabled: false`, `skip_cameras: [camera_pano_nw]`).

```bash
scripts/docker_run.sh python3 /ws/src/interfacing/calibration/lidar_cam_calib/scripts/run_calibration.py /out --run run1
```

```bash
scripts/docker_run.sh python3 /ws/src/interfacing/calibration/lidar_cam_calib/scripts/summarize_results.py /out --run run1
```

Results: `bags/lidar_cam_calib/calib/run1/results.md` and `urdf_snippet.xacro`. On WATcloud, use `scripts/watcloud_calib.sbatch` (see [DEVELOPING.md](DEVELOPING.md)).

## Recording calibration bags

The detector finds segments in any recording, but dedicated stations give the best data. Record everything in one bag and let `extract_segments.py` find the stations:

```bash
watod bag record -o calib_session -e '/(lidar_cc/velodyne_(points|packets)|camera_pano_[a-z]+/(image_rect_compressed|camera_info)|tf_static|novatel/oem7/imu/data|interfacing/can_state_estimator/body_velocity)$'
```

At each station: stop with texture 5–30 m away (painted lines, curbs, poles, walls; no glass), keep everyone still for 3 s, creep 3–5 m over 15–20 s, stop. Turn the car about 45° between stations; about 8 stations gives each camera 5–6 usable views.
