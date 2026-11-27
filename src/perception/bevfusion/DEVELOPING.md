# BEVFusion developer guide
## Overview

BEVFusion is a ROS 2 lifecycle composable node that combines camera and LiDAR data for 3D object detection. The current C++ wrapper publishes boxes only; map segmentation is not implemented here.

The model projects both sensors into a shared bird's-eye-view grid before fusing them. This gives the model camera semantics and LiDAR geometry in the same representation.

For each LiDAR scan, the node picks one image per camera (the one taken closest to when the LiDAR swept that camera's direction) and then:

1. **Preprocesses cameras** — on the car, uses each camera's GPU image as is; from bags, decompresses the JPEGs and converts BGR→RGB; then resizes and normalizes each image on the GPU
2. **Preprocesses LiDAR** — converts points to FP16 for the GPU voxelization kernels
3. **Runs TensorRT inference** — using [WATonomous/wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion), a fork of NVIDIA-AI-IOT's CUDA-BEVFusion, vendored via `cuda_bevfusion_vendor`
4. **Publishes 3D bounding boxes** — for detected objects (vehicles, pedestrians, etc.)
5. **Publishes visualization markers** — for debugging in Foxglove

## Architecture

```
Sensor Inputs (one camera topic per entry in camera_names)
──────────────────────────────────────────────────────────
/<camera_name>/image_rect            NITROS, GPU memory (the car)                          ──┐
  or /<camera_name>/image_rect_compressed   JPEG (a bag); auto picks per camera             │
/<camera_name>/camera_info           intrinsics                                              ├──► BEVFusionNode (lifecycle composable)
/lidar_cc/velodyne_points            one Velodyne sweep (input_lidar_topic)                ──┘          │
                                                                                                        ▼
                                                                   CameraMatcher: per-camera frame buffers, LiDAR-anchored
                                                                   (each camera's frame nearest when the LiDAR swept its direction)
                                                                                                        │ matched set: scan + 1 frame per camera
                                                                                                        ▼
                                                                   worker thread → BEVFusionCore
                                                                   ├── Camera input (GPU pointer as is, or JPEG decode + BGR→RGB)
                                                                   ├── LiDAR preprocessing (float→FP16)
                                                                   └── TensorRT inference (CUDA-BEVFusion)
                                                                                                        │
                                                                                 ┌──────────────────────┴──────────────┐
                                                                                 ▼                                     ▼
                                                                          Detection3DArray                       MarkerArray
                                                                    (3D boxes, LiDAR scan stamp)               (Foxglove debug)
```

### Node / Core Separation

- **`BEVFusionNode`** — ROS 2 lifecycle composable node. Handles per-camera subscriptions, camera matching, the inference worker thread, parameter loading, diagnostics, and publishing. Contains no inference logic.
- **`CameraMatcher`** (`camera_matcher.hpp`) and **`sweepTargets()`** (`lidar_sweep_timing.hpp`) — header-only, ROS-free camera matching, unit tested on their own.
- **`BEVFusionCore`** — Pure C++ class with no ROS dependencies. Owns the TensorRT engine, preprocessing pipeline, and inference. Independently testable.

## Sensors

### Cameras

Eve has 12 cameras total — 8 panoramic and 4 lower. BEVFusion uses **6 of the 8 panoramic cameras**, selected to match the nuScenes 6-camera training layout (front, front-left, front-right, back, back-left, back-right):

| nuScenes position | Eve camera | ROS frame ID |
|---|---|---|
| front | `camera_pano_nn` | `camera_pano_nn` |
| front-right | `camera_pano_ne` | `camera_pano_ne` |
| front-left | `camera_pano_nw` | `camera_pano_nw` |
| back | `camera_pano_ss` | `camera_pano_ss` |
| back-right | `camera_pano_se` | `camera_pano_se` |
| back-left | `camera_pano_sw` | `camera_pano_sw` |

`camera_pano_ee` (pure right) and `camera_pano_ww` (pure left) are excluded — they have no equivalent in the nuScenes layout the model was trained on. The 4 lower cameras are also excluded as they point downward and are not useful for 3D object detection.

Nothing in the node is tied to 6 cameras: subscriptions, matching and calibration are all sized from `camera_names`. For a model trained on all 8 panoramic cameras, add `camera_pano_ee` and `camera_pano_ww` to `camera_names` **in the order the model was trained with** (images, intrinsics and extrinsics are all indexed by that order), rebuild the engines for the new model, and enable both cameras in `perception_bringup/launch/all_cameras_composed.yaml`.

### LiDAR

Eve has 3 Velodyne LiDARs (`lidar_cc`, `lidar_ne`, `lidar_nw`), which can be pre-merged by a `lidar_aggregator` node into `/lidar/all/points_merged`. **BEVFusion currently defaults to the single `lidar_cc` sensor** (`/lidar_cc/velodyne_points`, set via `input_lidar_topic`), not the merged cloud. `lidar_frame_id` (default `lidar_cc`) must match whichever LiDAR topic is actually configured, since it's used as the TF target for camera extrinsics.

`lidar_cc` spins at 1200 RPM (one sweep every 50 ms). The driver stamps each sweep with its last packet (`timestamp_first_packet: false`) and stores points in firing order, and because `cut_angle` is not set, the direction a sweep starts from drifts from scan to scan. So the laser passes a given camera's direction anywhere from 0 to 50 ms before the scan's stamp. The node accounts for this when matching (see [Camera matching](#camera-matching)). The per-point `time` field is all zeros in the recorded bags, so the timing is worked out from the point order and the rotation period instead.

Neither the speed nor the stamp convention is fixed in the code, so a spinning LiDAR at another speed, or one stamped at the start of its turn, needs no code change:

- **Rotation period:** measured from the scan stamps (one scan is one turn: the median of the last 20 stamp steps, once 5 are known). `lidar_rotation_period_ms` is only the starting value, and the node warns once if the measured period differs from it by more than 10%. A LiDAR at 600 RPM is timed correctly after its first few scans.
- **Stamp position:** `lidar_stamp_position` says whether the stamp is the end of the turn (`"end"`, the Velodyne driver's `timestamp_first_packet: false`, Eve's setting) or its start (`"start"`). A bag cannot tell the two apart, so this is a parameter. On live input the node compares it with how long after their stamp the scans arrive (about 0 for an end stamp, about one turn for a start stamp) and warns once if it looks wrong.

## Topics

### Subscribed — Camera Images

Each camera in `camera_names` has two image topics:

| Topic | Type | Where it comes from |
|---|---|---|
| `/<camera_name>/image_rect` | NITROS image (RGB8, GPU memory) | On the car: the rectified image from that camera's `RectifyNode`, handed over in the same process without a copy |
| `/<camera_name>/image_rect_compressed` | `sensor_msgs/CompressedImage` (JPEG) | The car's JPEG copy, and the only image a bag holds |

With `camera_input: auto` (the default) the node listens to both and chooses **per camera**:

- A camera's NITROS images are used whenever they arrive; its JPEGs are ignored while it has sent a NITROS image within the last second, and are used otherwise. Until the JPEG topics are dropped (next point), a camera whose NITROS stream stops falls back to JPEG on its own. The node logs each camera's source when it changes.
- On the car every camera sends NITROS, so after two checks a second apart the node stops listening to the JPEG topics and decodes nothing on the CPU. They are only subscribed again when the node is re-activated.
- From a bag no camera sends NITROS, so every camera uses JPEG. Nothing needs to be set for bag replay (see [Replaying a bag](#replaying-a-bag)).
- A matched set can mix the two, for example while cameras switch over.

`camera_input: nitros` or `compressed` listens to that one topic only. If a camera sends no frames for over a second, its scans cannot be matched, and the node warns which cameras and topics.

- On the NITROS path, images must be `image_width x image_height` packed RGB8, which is what the camera driver and `RectifyNode` produce; anything else is dropped with an error. The node takes a read handle for each image on the inference CUDA stream, so the stream waits for the camera's write on the GPU, and passes the device pointers to `BEVFusionCore::infer()`. CUDA-BEVFusion copies camera images with `cudaMemcpyDefault`, which needs `wato-cuda-bevfusion` at `4da4989` or later (pinned in `docker/perception.Dockerfile`).
- On the JPEG path, only the frames chosen for a matched set are decoded, in parallel (`cv::imdecode`), then converted BGR→RGB.
- Each kept frame on the NITROS path holds one of its rectifier's GPU buffers (40 per camera by default, shared with the H.264 and JPEG encoders), so `sync_frames_per_camera` stays small.

### Camera matching

The cameras run freely at 20 Hz, so their frames are never taken at the same moment, and an image reaches the node about 30 ms after it was taken while a LiDAR scan arrives almost at once. For every LiDAR scan the node therefore keeps the last few frames of each camera and picks, per camera, the frame nearest **the moment the LiDAR swept that camera's viewing direction** (`sync_target: sweep`; `scan_stamp` uses the scan stamp for every camera). This matches how the nuScenes training data was recorded, where each camera fired as the LiDAR passed it.

- A camera is decided as soon as its choice is certain: a frame after the target has arrived, or the newest frame before the target is so close that the camera's next frame cannot be nearer (bounded from its last two frames and its learned frame interval, with room for stamp jitter). A camera's choice is final once made.
- Otherwise the node waits, but no longer than `sync_max_wait_ms` after the scan arrived, then uses the newest frame before the target if it is within `sync_tolerance_ms`. A scan whose best frame for some camera is further off is skipped.
- Waits are in stream time (the time of the stamps), not wall-clock time, so a bag gives the same matches at any playback rate. The node learns how fast scan stamps advance against their arrival (exactly 1 live, 2 for a bag played at `--rate 2`) and scales each scan's wait by it. Replaying the drive bag's recorded stamps and arrival times through the matcher offline at 0.25×, 0.5×, 2× and 4× changed 0 of 60,996 camera picks compared with 1×.
- Matching never holds up the LiDAR: inference runs on a worker thread, and if a newer set is ready before the worker is free, it replaces the older one.
- Detections are stamped with the LiDAR scan's stamp.

The rules are in [bevfusion/DEVELOPING.md](bevfusion/DEVELOPING.md#2-camera-matching-cameramatcher-sweeptargets); the ROS-free code is `camera_matcher.hpp` and `lidar_sweep_timing.hpp`.

### Replaying a bag

No parameter or code changes are needed. Start the perception launch as usual and play a bag at any rate (extra options go to `ros2 bag play`):

```sh
./watod bag play <bag_folder>/<file>.mcap
./watod bag play <bag_folder>/<file>.mcap --rate 2
```

- **Cameras:** every camera uses its JPEGs, since a bag holds no NITROS images. `live_cameras:=false` ([camera_bringup.md](../perception_bringup/docs/camera_bringup.md)) only stops the camera drivers from starting; BEVFusion does not depend on it.
- **Timing:** waits follow the stamps and the rotation period is measured from them, so the matches are the same at any rate. The first few scans after the node starts wait in wall-clock time while the rate is learned. A bag that restarts or loops (stamps jump back more than 200 ms) starts the matching over and keeps the learned rate.
- **Above 1×:** scans arrive faster than inference runs (about 40 ms per set from JPEGs on an RTX 4070, against 25 ms between scans at 2×). The worker then skips sets: each set it does run is matched exactly as at 1×, but fewer are published.
- **Diagnostics** (`/diagnostics`, `BEVFusion Status`): `Stream rate` (the learned playback rate, 1 live), `LiDAR rotation period in use (ms)`, `Cameras on NITROS images` / `Cameras on JPEG images`, and `Sets replaced before inference` (sets skipped because a newer one was ready).

### Subscribed — Other

| Topic (param) | Type | Description |
|---|---|---|
| `input_lidar_topic` (default `/lidar_cc/velodyne_points`) | `sensor_msgs/PointCloud2` | LiDAR point cloud, must contain `x`, `y`, `z`, and `intensity` fields; one sweep with points in firing order for the sweep timing (otherwise the scan stamp is used) |
| `/<camera_name>/camera_info`, one per camera | `sensor_msgs/CameraInfo` | Camera intrinsics (cached once every camera has reported, not time-synced) |

`processLidar()` extracts `x, y, z, intensity` and appends a 5th feature of `0`. The nuScenes-trained model expects the multi-sweep time lag there (0 for the current sweep), **not** the `ring` index.

**Camera extrinsics via TF:** The physical mounting position and orientation of each camera (extrinsics) are looked up at runtime from the ROS 2 TF tree using `tf2_ros::Buffer` and `tf2_ros::TransformListener`. `computeCalibrationMatrices()` requests the transform from each camera's frame ID (e.g. `camera_pano_nn`) to the configured `lidar_frame_id` (default `lidar_cc`) — **not** `base_link`. Since camera mounts are fixed, these transforms are *static* — they are published once on `/tf_static` by the sensor launch infrastructure. The node does not subscribe to `/tf_static` directly; `tf2_ros::TransformListener` creates that subscription internally and caches all available transforms in the `Buffer`. Calibration (camera intrinsics, camera→lidar extrinsics, lidar→image projection, and the image augmentation matrix) is computed once every camera's `CameraInfo` has arrived (or immediately in `on_activate()` if they were already cached), and again is *not* recomputed per-frame.

**Output frame transform:** `createDetections3D()` looks up the LiDAR-to-target transform once per callback and applies it to each box. Published detections use `target_frame` (normally `base_link`).

### Detection filtering

There are two filters, at different points in the pipeline:

- The TensorRT detection head drops boxes below `confidence_threshold` and boxes whose centers fall outside `post_center_range_start` and `post_center_range_end`.
- The ROS node applies `ego_suppression_radius` in `createDetections3D()`. A box is dropped when its XY distance from the LiDAR origin is strictly less than this radius. Z and the box size do not affect this check. Set the radius to `0.0` to disable it.

The second filter protects against detections on the vehicle itself and against occasional zero-position outputs. It runs before the LiDAR-to-target-frame transform, so the radius is always measured in the LiDAR frame.

### Published

| Topic | Type | Description |
|---|---|---|
| `/perception/detections_3d_bev` | `vision_msgs/Detection3DArray` | 3D bounding boxes for detected objects |
| `/perception/bev_detection_markers` | `visualization_msgs/MarkerArray` | Foxglove visualization markers |

## Parameters

Parameters are declared in `declareParameters()` / `on_configure()` (`bevfusion_node.cpp`), defaulted in the node's own `bevfusion/config/params.yaml`, and overridden for the vehicle in `perception_bringup/config/perception_bringup.yaml` under `/**/bevfusion_node`.

| Parameter | Default (`params.yaml`) | Description |
|---|---|---|
| `lidar_frame_id` | `"lidar_cc"` | Frame TF extrinsics are resolved against (camera→this frame) |
| `target_frame` | `"base_link"` | Frame ID stamped on output `Detection3DArray` / `MarkerArray` messages |
| `camera_input` | `"auto"` | `"auto"`: per camera, GPU images on `/<camera_name>/image_rect` when they arrive (the car), else JPEGs on `/<camera_name>/image_rect_compressed` (a bag); `"nitros"` or `"compressed"`: that topic only |
| `intrinsics_source` | `"projection"` | Camera matrix from `CameraInfo` `P` (`"projection"`), the matrix of the rectified images BEVFusion receives, or `K` (`"camera_matrix"`), the raw distorted camera's. On Eve's cameras K and P differ by about 2% in focal length, which moves projected points up to 14-22 px at the image corners (8-12 px in the 704x256 network input) |
| `input_lidar_topic` | `/lidar_cc/velodyne_points` | LiDAR input topic |
| `model_dir` | `/opt/watonomous/models/bevfusion/resnet50_trt11_int8` | Directory with `.onnx` files and the LiDAR backbone |
| *(derived)* `build_dir` | `<model_dir>/build` | Not a parameter: where compiled `.plan` TensorRT engines are read from/written to, always `<model_dir>/build` |
| *(derived)* precision | `"int8"` | Not a parameter: `"int8"` if `model_dir` contains `int8`, otherwise `"fp16"` |
| `camera_names` | 6 `camera_pano_*` frame IDs | Frame IDs of the cameras to use, in nuScenes order; also determines `num_cameras` |
| `confidence_threshold` | `0.08` | Minimum detection score passed to the detection head |
| `ego_suppression_radius` | `2.5` | LiDAR-frame XY radius in which detections are discarded; `0.0` disables it |
| `image_width` / `image_height` | `1280` / `1024` | Input camera resolution |
| `resize_lim` | calculated | Resize ratio used for calibration; derived from the configured image and network dimensions |
| `norm_output_width` / `norm_output_height` | `704` / `256` | Network input resolution after resize+crop |
| `min_range` / `max_range` | `[-54,-54,-5]` / `[54,54,3]` | LiDAR voxelization range (x,y,z) in meters |
| `voxel_size` | `[0.075, 0.075, 0.2]` | Voxel size (x,y,z) in meters |
| `max_points_per_voxel` / `max_points` / `max_voxels` | `10` / `300000` / `160000` | LiDAR point cloud caps (GPU memory vs. coverage tradeoff) |
| `xbound` / `ybound` / `zbound` / `dbound` | see `params.yaml` | BEV grid bounds `[min, max, step]` used for camera-to-BEV projection |
| `post_center_range_start` / `post_center_range_end` | `[-61.2,-61.2,-10]` / `[61.2,61.2,10]` | Discards detections whose center falls outside this volume |
| `sync_target` | `"sweep"` | Match each camera to when the LiDAR swept its direction (`"sweep"`), or to the scan stamp (`"scan_stamp"`) |
| `sync_max_wait_ms` | `60.0` | Longest wait after a scan arrives for each camera's best frame, in stream time (the same at any bag playback rate); `< 0` waits until certain (1 s limit) |
| `sync_tolerance_ms` | `450.0` | Largest gap between a camera frame and its target that can still be used |
| `sync_early_fraction` | `0.5` | Decide early on the frame before the target when it is within this fraction of the time until the camera's next frame can come at the earliest (`0` to `0.5`; up to `0.5` only decides sooner, see [Camera matching](bevfusion/DEVELOPING.md#2-camera-matching-cameramatcher-sweeptargets) for the exceptions) |
| `sync_frames_per_camera` | `6` | Frames kept per camera (`2` to `16`; on the NITROS path each holds one of the camera rectifier's 40 GPU buffers) |
| `sync_max_pending_scans` | `4` | Scans that can wait for their frames at the same time; beyond this the oldest is decided with what it has (`1` to `16`) |
| `lidar_rotation_period_ms` | `50.0` | One LiDAR rotation (60 s / RPM): only the starting value, until the period is measured from the scan stamps (after 5 scans) |
| `lidar_stamp_position` | `"end"` | Whether a scan's stamp is the time of the end (`"end"`, Velodyne `timestamp_first_packet: false`) or the start (`"start"`) of its turn; checked against live arrival times, with a warning |
| `qos_subscriber_reliability` / `qos_subscriber_depth` | `"best_effort"` / `2` | Subscriber QoS (each camera's image and `camera_info` topic, and the LiDAR) |
| `qos_publisher_reliability` / `qos_publisher_durability` / `qos_publisher_depth` | `"reliable"` / `"transient_local"` / `5` | Publisher QoS |

Camera intrinsics are *not* a declared parameter — they come from each camera's `camera_info` topic at runtime, and camera extrinsics come from TF (see above).

## Implementation Notes

### TensorRT precision: FP16 vs INT8

`BEVFusionInputConfig::precision` (derived from `model_dir`: `"int8"` if the path contains `int8`) controls whether the LiDAR SCN backbone is built/run at `"fp16"` or `"int8"` — the deployed default is `"int8"`. This only affects the LiDAR sparse-conv backbone (`::bevfusion::lidar::Precision`); the camera backbone, fuser, and detection head engines are separately compiled `.plan` files built once (via `BEVFusionCore::buildTRTEngines()`) if not already present in `build_dir`. Model weights (`.onnx` sources and compiled `.plan` engines) live under `/opt/watonomous/models/bevfusion/`, which is mounted into the container by docker-compose.

### What BEVFusion does NOT do

- **Map Segmentation** — while the BEVFusion architecture supports it, this C++ TensorRT wrapper is currently only implemented for 3D object detection (bounding boxes).
- **Floor removal** — the model's encoder learns to ignore the road internally
- **Clustering** — would discard geometry the model needs
- **Tracking** — handled by the `tracking` node downstream

## Building

```bash
colcon build --packages-select cuda_bevfusion_vendor bevfusion
```

`cuda_bevfusion_vendor` must be built first — it compiles [WATonomous/wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion) (a WATonomous fork of NVIDIA-AI-IOT's CUDA-BEVFusion), which `docker/perception.Dockerfile` clones and pins into the workspace, into the `cuda_bevfusion_vendor::bevfusion_core` CMake target. `bevfusion` links against that target. Colcon resolves this build order automatically when both packages are specified.

`bevfusion` itself builds two libraries plus an executable: `bevfusion_wato_core` (the ROS-free `BEVFusionCore` GPU/TensorRT wrapper), `bevfusion_node` (the ROS 2 lifecycle node, registered as a component via `rclcpp_components_register_node`), and the `bevfusion_node` executable (`main.cpp`, for standalone/non-composed runs).

Dependencies: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `isaac_ros_managed_nitros`, `isaac_ros_nitros_image_type`, `vision_msgs`, `visualization_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`, `eigen3_cmake_module`, `diagnostic_updater`, `OpenCV`, CUDA Toolkit (`nvinfer`, `nvinfer_plugin`, `nvonnxparser`), `cuda_bevfusion_vendor`.

### TensorRT and spconv versions

The perception image installs two TensorRT major versions, pinned via the `TRT10_VERSION` and `TRT11_VERSION` build args in `docker/perception.Dockerfile`:

- **TensorRT 10** runtime libraries (`libnvinfer10`, `libnvinfer-plugin10`, `libnvonnxparsers10`) for `deep_ros`.
- **TensorRT 11** development packages (`libnvinfer-dev`, `libnvinfer-plugin-dev`, `libnvonnxparsers-dev`) for BEVFusion. These are `+cuda13.4` builds, while the base image is CUDA 12.8.1; the CUDA 13 toolkit they build against comes from `ros-jazzy-isaac-ros-common`.

`cuda_bevfusion_vendor` defaults `SPCONV_CUDA_VERSION` to `13.0` to match. Change the TRT 11 pin and `SPCONV_CUDA_VERSION` together, and rebuild engines (`.plan` files) after a TensorRT upgrade, since serialized engines are tied to the TensorRT version that built them.

### CUDA architecture (`CUDASM`)

`cuda_bevfusion_vendor` compiles CUDA kernels for a single GPU compute capability. It defaults to `CUDASM=89`, which targets Ada GPUs such as the RTX 40-series. Building for one architecture does not produce binaries that run on an older one: `89` code cannot run on an 8.6 GPU, and attempting it fails with `cudaErrorNoKernelImageForDevice` in `create_frustum_kernel`.

Check your GPU's compute capability with:

```bash
nvidia-smi --query-gpu=compute_cap --format=csv,noheader
```

Remove the decimal point and set the result in `watod-config.local.sh` before building. For example, compute capability `8.6` (Ampere / RTX 30-series) becomes:

```bash
export CUDASM=86
```

Then rebuild with `./watod build`. `watod up --build` does not rebuild the source and dependency pre-build stages, so it will keep the previous architecture. PTX is forward-compatible but not backward-compatible: an image built with `CUDASM=86` can run on 8.6 and newer GPUs, while an image built with `CUDASM=89` cannot run on 8.6.

### Testing

```bash
colcon build --packages-up-to bevfusion --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select bevfusion
colcon test-result --verbose
```

Tests live under `bevfusion/test/` and use `wato_test`/Catch2. The core tests mock `::bevfusion::Core`, so inference and calibration forwarding can be checked without a real GPU pipeline. The node tests cover lifecycle setup, parameter overrides, image and detection conversion, calibration math, and ego-zone filtering. They use `#define private public` to inspect node internals where that is useful.

### Launch

`bevfusion.launch.yaml` in this package can run the node standalone (`camera_input:=auto`, the default, `nitros` or `compressed`). In practice, it's loaded as a composable node into the shared `perception_container` from `perception_bringup/launch/perception.launch.yaml`, in the same process as the cameras, with `output_detections` and `output_markers` remapped to `/perception/detections_3d_bev` and `/perception/bev_detection_markers`, and parameters sourced from `perception_bringup/config/perception_bringup.yaml`. It's also listed in the `detection_lifecycle_manager`'s `node_names` for autostart. BEVFusion no longer uses `multi_camera_sync`; that node stays in the launch file for `spatial_association`.
