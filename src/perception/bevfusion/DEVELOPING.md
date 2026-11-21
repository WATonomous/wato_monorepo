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
/<camera_name>/image_rect            NITROS, GPU memory (car, camera_input: nitros)        ──┐
  or /<camera_name>/image_rect_compressed   JPEG (bag replay, camera_input: compressed)     │
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

## Topics

### Subscribed — Camera Images

Each camera in `camera_names` is subscribed on its own topic. `camera_input` selects which:

| `camera_input` | Topic | Type | When |
|---|---|---|---|
| `nitros` | `/<camera_name>/image_rect` | NITROS image (RGB8, GPU memory) | On the car: the rectified image from that camera's `RectifyNode`, handed over in the same process without a copy |
| `compressed` (default) | `/<camera_name>/image_rect_compressed` | `sensor_msgs/CompressedImage` (JPEG) | Bag replay: the bags only hold the JPEG copy of each camera |

`perception.launch.yaml` sets `camera_input` from its `live_cameras` argument (this overrides the value in `perception_bringup.yaml`), so bag replay with `live_cameras:=false`, as [camera_bringup.md](../perception_bringup/docs/camera_bringup.md) describes, uses `compressed`. There is no automatic switching at runtime: if a camera delivers no frames, every scan is dropped and the node warns which cameras and topic, with this hint.

- On the NITROS path, images must be `image_width x image_height` packed RGB8, which is what the camera driver and `RectifyNode` produce; anything else is dropped with an error. The node takes a read handle for each image on the inference CUDA stream, so the stream waits for the camera's write on the GPU, and passes the device pointers to `BEVFusionCore::infer()`. CUDA-BEVFusion copies camera images with `cudaMemcpyDefault`, which needs `wato-cuda-bevfusion` at `4da4989` or later (pinned in `docker/perception.Dockerfile`).
- On the JPEG path, only the frames chosen for a matched set are decoded, in parallel (`cv::imdecode`), then converted BGR→RGB.
- Each kept frame on the NITROS path holds one of its rectifier's GPU buffers (40 per camera by default, shared with the H.264 and JPEG encoders), so `sync_frames_per_camera` stays small.

### Camera matching

The cameras run freely at 20 Hz, so their frames are never taken at the same moment, and an image reaches the node about 30 ms after it was taken while a LiDAR scan arrives almost at once. For every LiDAR scan the node therefore keeps the last few frames of each camera and picks, per camera, the frame nearest **the moment the LiDAR swept that camera's viewing direction** (`sync_target: sweep`; `scan_stamp` uses the scan stamp for every camera). This matches how the nuScenes training data was recorded, where each camera fired as the LiDAR passed it.

- A camera is decided as soon as its choice is certain: a frame after the target has arrived, or the newest frame before the target is so close (`sync_early_fraction` ≤ 0.5 of the camera's frame interval, less 10% for stamp jitter) that the next one cannot be nearer. A camera's choice is final once made.
- Otherwise the node waits, but no longer than `sync_max_wait_ms` after the scan arrived, then uses the newest frame before the target if it is within `sync_tolerance_ms`. A scan whose best frame for some camera is further off is skipped.
- Matching never holds up the LiDAR: inference runs on a worker thread, and if a newer set is ready before the worker is free, it replaces the older one.
- Detections are stamped with the LiDAR scan's stamp.

The rules are in [bevfusion/DEVELOPING.md](bevfusion/DEVELOPING.md#2-camera-matching-cameramatcher-sweeptargets); the ROS-free code is `camera_matcher.hpp` and `lidar_sweep_timing.hpp`.

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
| `camera_input` | `"compressed"` | `"compressed"`: per-camera JPEGs on `/<camera_name>/image_rect_compressed` (bags); `"nitros"`: per-camera GPU images on `/<camera_name>/image_rect` (the car) |
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
| `sync_max_wait_ms` | `60.0` | Longest wait after a scan arrives for each camera's best frame; `< 0` waits until certain (1 s limit) |
| `sync_tolerance_ms` | `450.0` | Largest gap between a camera frame and its target that can still be used |
| `sync_early_fraction` | `0.5` | Decide early on the frame before the target when it is within this fraction of the camera's frame interval, less 10% for stamp jitter (`0` to `0.5`; up to `0.5` never changes which frame is used) |
| `sync_frames_per_camera` | `6` | Frames kept per camera (`2` to `64`) |
| `sync_max_pending_scans` | `4` | Scans that can wait for their frames at the same time; beyond this the oldest is decided with what it has (`1` to `64`) |
| `lidar_rotation_period_ms` | `50.0` | One LiDAR rotation (60 s / RPM) |
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

`bevfusion.launch.yaml` in this package can run the node standalone (`camera_input:=nitros` or `compressed`). In practice, it's loaded as a composable node into the shared `perception_container` from `perception_bringup/launch/perception.launch.yaml`, in the same process as the cameras, with `output_detections` and `output_markers` remapped to `/perception/detections_3d_bev` and `/perception/bev_detection_markers`, `camera_input` set from `live_cameras`, and parameters sourced from `perception_bringup/config/perception_bringup.yaml`. It's also listed in the `detection_lifecycle_manager`'s `node_names` for autostart. BEVFusion no longer uses `multi_camera_sync`; that node stays in the launch file for `spatial_association`.
