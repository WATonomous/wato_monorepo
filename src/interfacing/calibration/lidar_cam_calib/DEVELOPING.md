# Developing lidar_cam_calib

## Layout

| Path | Purpose |
|------|---------|
| `config/calib.yaml` | Cameras, topics, detection thresholds, trial settings |
| `docker/Dockerfile` | `koide3/direct_visual_lidar_calibration:jazzy` + small_gicp, xacro, xvfb |
| `scripts/extract_segments.py` | Find and cut creep/static segments from a folder of bags |
| `scripts/run_calibration.py` | Preprocess + repeated `calibrate` trials per camera |
| `scripts/summarize_results.py` | Results table, verdicts, URDF joints |
| `scripts/docker_run.sh` | Run a command in the image with the repo, bags and output mounted |
| `scripts/watcloud_calib.sbatch` | The same three steps as Slurm jobs |
| `lidar_cam_calib/` | Python modules: bag I/O, cloud clean-up, motion, segments, cutting, transforms, toolbox wrapper |

The scripts run from the source tree inside the image (`/ws/src/...`), like `slam_benchmark`. The package only installs `config/` and `docker/`. It is in `bloom/*.ignored` and is not released.

## Output folder

```text
OUT_DIR/
  manifest.yaml            every segment; edit `enabled` / `skip_cameras` here
  creep/<segment>/         cut bags for preprocess -d
  static/<segment>/        cut bags for static preprocess
  previews/<segment>.jpg   first frame of each camera (the frame the toolbox uses)
  motion/<bag>.csv         motion timeline: speed, rest flag, per-signal values
  calib/<run>/<camera>/
    bags/                  links to the selected segments
    base/                  preprocess output + initial guess
    trials/<trial>/        one calibrate run each (calib.json, calibrate.log)
    trials.yaml
  calib/<run>/results.md, results.yaml, urdf_snippet.xacro
```

Re-running a step skips finished work: processed bags, existing trials. `--force` redoes it. A new `--run` name keeps earlier results.

## How the pieces work

**Motion.** Every 5th LiDAR scan (0.25 s apart) is registered to the previous one with small_gicp (0.25 m voxels, 2–80 m range). Speed = translation / time. On the May 30 bag, GICP read a median of 0.005 m/s (max 0.03 m/s) while parked and tracked the recorded odometry while driving. When the bag also has the IMU, CAN `body_velocity` or `/tf` odometry, each must agree the car is at rest. CAN or odometry also supplies the speed where GICP is unreliable (converged with < 50% inliers).

**Segments.** Rest = speed < 0.05 m/s and rotation < 0.5 deg/s for at least 3 s. A creep segment keeps the last 3 s of rest, then the motion until 5 m, 20 s, a speed above 1.5 m/s, or a new stop; shorter than 1 m is dropped. A static segment is up to 15 s of a rest period.

**Cloud clean-up.** Recordings made before the launch fix in `sensor_interfacing` have `time = 0` on every point, which breaks the toolbox's dynamic integration (it divides by the last point's time). The organized cloud holds one firing per row, so time = row / rows × 0.05 s. NaN points are dropped. A valid time field is kept as is.

**Intrinsics.** The recorded images are rectified (`image_rect_compressed`), but `camera_info` carries the raw K and distortion. The toolbox gets P from `camera_calib/config/<camera>.yaml` with zero distortion. That's why `-a` (auto topics/intrinsics) is not used.

**Trials.** `all` uses every segment and is the estimate. `loo_<segment>` leaves one out (with 3+ segments). `perturb_<k>` starts from the URDF guess offset by N(0, 3 cm) per axis and N(0, 1 deg) per axis. The default BFGS refinement moves at most 0.2 m / 2 deg per outer iteration (10 iterations), so a guess more than a few degrees off may not recover.

## After Launching

1. Read the `extract_segments.py` log: one line per bag (used or skipped, with cameras) and per segment (times, distance, peak speed, end reason).
2. Open `previews/`. Disable segments with people next to the car, plain walls or glass in front of a camera, or bad light, using `enabled: false` or `skip_cameras` in `manifest.yaml`.
3. Run `run_calibration.py`. Each trial logs how far it moved from the URDF guess.
4. Run `summarize_results.py` and read `results.md`.
5. Check a camera visually with the toolbox viewer (needs a display):

```bash
scripts/docker_run.sh ros2 run direct_visual_lidar_calibration viewer /out/calib/run1/camera_pano_nn/trials/all
```

## Definition of Good Result

| Check | Good | Redo |
|-------|------|------|
| Spread: largest difference between `all` and any leave-one-out / perturbed trial | ≤ 2 cm and ≤ 0.2 deg | > 5 cm or > 0.5 deg |
| Viewer: LiDAR intensity edges on image edges, near and far, across the frame | aligned everywhere | offset at range or at the edges |
| Roof ring (`results.md`): radius about 0.24 m, heights equal, headings about 45 deg apart | all 6 consistent | one camera off the pattern |

For reference, the paper reports about 4 cm / 0.4 deg from one shot and about 3 cm / 0.4 deg from several with a 64-beam Ouster. The VLP-32C is sparser.

If a camera fails:

- Few or poor segments: record more stations facing its direction
- Large perturbed spread but small leave-one-out spread: the initial guess is outside the convergence basin. Get a better one with `initial_guess_manual` on `base/`, then rerun.
- Large leave-one-out spread: one segment disagrees. Find it in `trials.yaml` and disable it for that camera.

## Applying the result

`urdf_snippet.xacro` has one joint per camera with parent `lidar_cc`, the frame the toolbox calibrates against. In `eve_description/urdf/eve_roof_mount.xacro`, replace the computed `create_camera` poses with these joints. With `lidar_cc` as the parent, like `imu_link` and the mirror LiDARs, the extrinsics stay valid when the `base_link -> lidar_cc` leveling changes. Then rebuild `eve_description` and check:

```bash
ros2 run tf2_ros tf2_echo lidar_cc camera_pano_nn
```

Restart `spatial_association`, which caches camera extrinsics.

## WATcloud

`scripts/watcloud_calib.sbatch` starts rootless Docker (`slurm-start-dockerd.sh`), loads the image from `/mnt/wato-drive2/$USER/images/lidar_cam_calib.tar` (built and saved by the first non-array job), and runs the steps through `docker_run.sh`. Bags default to `~/wato_monorepo/bags`, output to `~/wato_monorepo/bags/lidar_cam_calib`. The submit commands are at the top of the script. The calibrate step is a job array with one task per camera.

## Known issues

Verified on WATcloud (2026-11-18): `calibrate --auto_quit --background` runs headless under xvfb and exits on its own, and a trial loads only the segments in its `meta.bag_names`.

- `preprocess` sometimes exits 1 after writing all its outputs. `run_calibration.py` therefore judges it by its output (`calib.json` listing every selected segment).
- Some nodes (`thor-slurm1`) set `DISPLAY` in batch jobs with no reachable X server, and `calibrate` then segfaults ("failed to initialize GLFW"). The sbatch script unsets `DISPLAY`.
- IMU stillness thresholds (`gyro_still`, `accel_std_still`) are untuned.

First results (run `creep1` / `static1` on the May 30, Feb 17 and ring-road bags: 1 creep segment, 5 static): every camera got **redo**, with spreads of 12–54 cm and 0.3–4.7 deg. Starting from slightly different guesses lands on different poses, so these recordings don't constrain the pose. Static segments barely move the estimate, because 32 beams of a parked scan are too sparse. Dedicated stations are needed (README, "Recording calibration bags").

## Dependencies

- direct_visual_lidar_calibration, small_gicp (pip), xacro, xvfb: provided by `docker/Dockerfile`
- rosbag2_py, sensor_msgs_py, numpy, scipy, OpenCV, PyYAML
