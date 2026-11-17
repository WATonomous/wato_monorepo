# BEVFusion developer guide
## Overview

BEVFusion is a ROS 2 lifecycle composable node that combines camera and LiDAR data for 3D object detection. The current C++ wrapper publishes boxes only; map segmentation is not implemented here.

The model projects both sensors into a shared bird's-eye-view grid before fusing them. This gives the model camera semantics and LiDAR geometry in the same representation.

Given synchronized camera images and a merged LiDAR point cloud, the node:

1. **Preprocesses cameras** — decompresses, converts BGR→RGB, resizes, and normalizes each camera image
2. **Preprocesses LiDAR** — converts points to FP16 for the GPU voxelization kernels
3. **Runs TensorRT inference** — using [WATonomous/wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion), a fork of NVIDIA-AI-IOT's CUDA-BEVFusion, vendored via `cuda_bevfusion_vendor`
4. **Publishes 3D bounding boxes** — for detected objects (vehicles, pedestrians, etc.)
5. **Publishes visualization markers** — for debugging in Foxglove

## Architecture

```
Sensor Inputs
─────────────
/multi_camera_sync/multi_image_compressed  ──┐
(6 cameras, JPEG-compressed, CPU)            │
                                             ├──► BEVFusionNode (lifecycle composable)
/lidar_cc/velodyne_points                    │          │
(configurable LiDAR topic)                 ──┘          │
                                                        ▼
                                              BEVFusionCore
                                              ├── Camera preprocessing (JPEG decode + BGR→RGB)
                                              ├── LiDAR preprocessing (float→FP16)
                                              └── TensorRT inference (CUDA-BEVFusion)
                                                         │
                                          ┌──────────────┴──────────────┐
                                          ▼                             ▼
                                   Detection3DArray               MarkerArray
                                   (3D bounding boxes)            (Foxglove debug)
```

### Node / Core Separation

- **`BEVFusionNode`** — ROS 2 lifecycle composable node. Handles subscriptions, message synchronization, parameter loading, diagnostics, and publishing. Contains no inference logic.
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

### LiDAR

Eve has 3 Velodyne LiDARs (`lidar_cc`, `lidar_ne`, `lidar_nw`), which can be pre-merged by a `lidar_aggregator` node into `/lidar/all/points_merged`. **BEVFusion currently defaults to the single `lidar_cc` sensor** (`/lidar_cc/velodyne_points`, set via `input_lidar_topic`), not the merged cloud. `lidar_frame_id` (default `lidar_cc`) must match whichever LiDAR topic is actually configured, since it's used as the TF target for camera extrinsics.

## Topics

### Subscribed — Camera Images

BEVFusion has two camera inputs and picks between them per frame. While the cameras are running it uses their NITROS images straight from GPU memory (see the next section). Otherwise, e.g. during bag replay where only compressed images exist, it uses the pre-batched, pre-synced multi-camera topic used by other perception nodes (`attribute_assigner`, `spatial_association`):

| Topic (param) | Type | Description |
|---|---|---|
| `input_multi_image_topic` (default `/multi_camera_sync/multi_image_compressed`) | `deep_msgs/MultiImageCompressed` | All 6 cameras batched into one message, JPEG-compressed |

Each callback filters the incoming image list to the frame IDs in `camera_names`, puts them in that order, then decompresses each image with `cv::imdecode` and converts BGR to RGB before calling `BEVFusionCore::infer()`.

**Time synchronization:** The node uses `message_filters::ApproximateTimeSynchronizer` to match the multi-image message and the LiDAR point cloud by timestamp — ensuring each inference call receives a synchronized camera+LiDAR frame. The queue depth is `sync_queue_size` and the maximum allowed time difference is `sync_max_time_diff_ms`.

**Important — message_filters and remapping:** `input_multi_image_topic` and `input_lidar_topic` are read directly from `params.yaml` at parameter-declaration time and passed to the `message_filters::Subscriber` constructors, because ROS 2 topic remapping does not reliably apply to `message_filters` subscribers created this way. Change these two topics via the params file, not via launch-time `remap`.

### Subscribed — NITROS (GPU) Camera Images

When `use_nitros_images` is `true` (the default), the node also subscribes to each camera's rectified NITROS image, which is already in GPU memory:

| Topic | Type | Description |
|---|---|---|
| `/<camera_name>/image_rect`, one per entry in `camera_names` | `NitrosImage` (RGB8) | Rectified image published by that camera's `RectifyNode` |

- The number of cameras follows `camera_names`. There is no separate camera-count or topic-list parameter.
- Per-camera topics are used because there is no batched message type for GPU images.
- The images are RGB8, which is what the model expects, so no color conversion is done. Images with any other encoding are dropped.
- There is no batched message to synchronize, so each LiDAR scan is matched with the newest image from every camera. The frame is skipped unless all of them are within `sync_max_time_diff_ms` of the scan.
- The GPU buffers are passed to `BEVFusionCore::infer()` as they are, with no JPEG decode and no copy through CPU memory.

**Choosing the input:** nothing has to be switched by hand. These topics only carry data while the cameras are running. While frames arrive this way, the batched callbacks do nothing; when they stop for more than a second, or never start, the batched topic is used. The node logs `Camera input: ...` whenever the input in use changes, so a fall back from GPU images is visible. Set `use_nitros_images` to `false` to always use the batched topic.

**Requirement:** CUDA-BEVFusion must accept GPU pointers for its camera images, i.e. copy them with `cudaMemcpyDefault` in `camera-normalization.cu`. Perception images built from an older `wato-cuda-bevfusion` assume host memory and have to be rebuilt.

### Subscribed — Other

| Topic (param) | Type | Description |
|---|---|---|
| `input_lidar_topic` (default `/lidar_cc/velodyne_points`) | `sensor_msgs/PointCloud2` | LiDAR point cloud, must contain `x`, `y`, `z`, and `intensity` fields |
| `camera_info` (remappable, wired to `/multi_camera_sync/multi_camera_info` in `perception.launch.yaml`) | `deep_msgs/MultiCameraInfo` | Camera intrinsics for all cameras (cached once, not time-synced) |

`processLidar()` extracts `x, y, z, intensity` and appends a 5th feature of `0`. The nuScenes-trained model expects the multi-sweep time lag there (0 for the current sweep), **not** the `ring` index.

**Camera extrinsics via TF:** The physical mounting position and orientation of each camera (extrinsics) are looked up at runtime from the ROS 2 TF tree using `tf2_ros::Buffer` and `tf2_ros::TransformListener`. `computeCalibrationMatrices()` requests the transform from each camera's frame ID (e.g. `camera_pano_nn`) to the configured `lidar_frame_id` (default `lidar_cc`) — **not** `base_link`. Since camera mounts are fixed, these transforms are *static* — they are published once on `/tf_static` by the sensor launch infrastructure. The node does not subscribe to `/tf_static` directly; `tf2_ros::TransformListener` creates that subscription internally and caches all available transforms in the `Buffer`. Calibration (camera intrinsics, camera→lidar extrinsics, lidar→image projection, and the image augmentation matrix) is computed once when the first `MultiCameraInfo` message arrives (or immediately in `on_activate()` if camera info was already cached), and again is *not* recomputed per-frame.

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
| `input_multi_image_compressed_topic` | `/multi_camera_sync/multi_image_compressed` | Compressed camera input topic |
| `input_multi_image_raw_topic` | `/multi_camera_sync/multi_image_raw` | Raw camera input topic |
| `use_nitros_images` | `true` | Use per-camera NITROS (GPU) images from `/<camera_name>/image_rect` whenever they are published; the batched topics are used otherwise |
| `input_lidar_topic` | `/lidar_cc/velodyne_points` | LiDAR input topic (same remap caveat as above) |
| `model_dir` | `/opt/watonomous/models/bevfusion/resnet50_trt11_int8` | Directory with `.onnx` files and the LiDAR backbone |
| `build_dir` | `<model_dir>/build` | Directory where compiled `.plan` TensorRT engines are read from/written to |
| `precision` | `"int8"` | Model precision — `"fp16"` or `"int8"` |
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
| `sync_queue_size` | `5` | Queue depth for `ApproximateTimeSynchronizer` |
| `sync_max_time_diff_ms` | `200.0` | Maximum timestamp difference for ApproximateTime sync (ms) |
| `qos_subscriber_reliability` / `qos_subscriber_depth` | `"best_effort"` / `10` | Subscriber QoS |
| `qos_publisher_reliability` / `qos_publisher_durability` / `qos_publisher_depth` | `"reliable"` / `"transient_local"` / `10` | Publisher QoS |

Camera intrinsics are *not* a declared parameter — they come from the `camera_info` topic (`deep_msgs/MultiCameraInfo`) at runtime, and camera extrinsics come from TF (see below).

## Implementation Notes

### TensorRT precision: FP16 vs INT8

`BEVFusionInputConfig::precision` (ROS param `precision`) controls whether the LiDAR SCN backbone is built/run at `"fp16"` or `"int8"` — the deployed default is `"int8"`. This only affects the LiDAR sparse-conv backbone (`::bevfusion::lidar::Precision`); the camera backbone, fuser, and detection head engines are separately compiled `.plan` files built once (via `BEVFusionCore::buildTRTEngines()`) if not already present in `build_dir`. Model weights (`.onnx` sources and compiled `.plan` engines) live under `/opt/watonomous/models/bevfusion/`, which is mounted into the container by docker-compose.

### What BEVFusion does NOT do

- **Map Segmentation** — while the BEVFusion architecture supports it, this C++ TensorRT wrapper is currently only implemented for 3D object detection (bounding boxes).
- **Floor removal** — the model's encoder learns to ignore the road internally
- **Clustering** — would discard geometry the model needs
- **Tracking** — handled by the `tracking` node downstream

## Building

```bash
colcon build --packages-select cuda_bevfusion_vendor bevfusion
```

`cuda_bevfusion_vendor` must be built first — its `CMakeLists.txt` fetches [WATonomous/wato-cuda-bevfusion](https://github.com/WATonomous/wato-cuda-bevfusion) (a WATonomous fork of NVIDIA-AI-IOT's CUDA-BEVFusion) and compiles it into the `cuda_bevfusion_vendor::bevfusion_core` CMake target. `bevfusion` links against that target. Colcon resolves this build order automatically when both packages are specified.

`bevfusion` itself builds two libraries plus an executable: `bevfusion_wato_core` (the ROS-free `BEVFusionCore` GPU/TensorRT wrapper), `bevfusion_node` (the ROS 2 lifecycle node, registered as a component via `rclcpp_components_register_node`), and the `bevfusion_node` executable (`main.cpp`, for standalone/non-composed runs).

Dependencies: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `message_filters`, `vision_msgs`, `visualization_msgs`, `deep_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`, `eigen3_cmake_module`, `diagnostic_updater`, `OpenCV`, CUDA Toolkit (`nvinfer`, `nvinfer_plugin`, `nvonnxparser`), `cuda_bevfusion_vendor`.

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

`bevfusion.launch.yaml` in this package can run the node standalone. In practice, it's loaded as a composable node into the shared `perception_container` from `perception_bringup/launch/perception.launch.yaml`, with `camera_info`, `output_detections`, and `output_markers` remapped to `/multi_camera_sync/multi_camera_info`, `/perception/detections_3d_bev`, and `/perception/bev_detection_markers` respectively, and parameters sourced from `perception_bringup/config/perception_bringup.yaml`. It's also listed in the `perception_lifecycle_manager`'s `node_names` for autostart. Recall that `input_multi_image_topic` and `input_lidar_topic` must be changed via params, not launch-time `remap` (see the message_filters note above).

## GPU lidar preprocessing

`lidar_processing_backend` selects `auto` (default), `cuda`, or `cpu`.
During configuration, `auto` initializes the CUDA preprocessing workspace and
selects CPU preprocessing if that initialization fails. Subscription initialization
can also select the CPU route before activation completes. `cuda` requires both
steps to succeed; `cpu` uses the original PointCloud2 subscription and preprocessing.
The selected backend and initialization fallback reason appear in node diagnostics.
Backend selection is fixed while active: malformed frames and runtime CUDA errors
are reported without switching backends. The inference model always requires a GPU.

The driver continues publishing `/lidar_cc/velodyne_points` as standard PointCloud2,
so other consumers keep their CPU messages. The adapted NITROS subscription uploads
the point array once; an in-process NITROS GPU producer can supply a shared device
buffer instead. Field metadata and timestamp matching remain on the CPU.

The CUDA backend supports scalar FLOAT32 x/y/z/intensity fields with arbitrary valid
offsets, point/row padding, and either byte order. It stably removes points with
nonfinite features, truncates to `max_points` (also bounded by the vendor's 300000-point
capacity), and packs FP16 `[x,y,z,intensity,0]` features on the GPU. The fifth feature
is zero sweep time lag, not ring. `lidar_max_input_points` (default 1000000) bounds
preallocated filtering workspace; larger raw input clouds are rejected, not silently
cropped before invalid-point removal. Only the resulting scalar point count is
returned to the CPU. Device-input inference bypasses host staging and lidar upload.

PR #599's latest-image matching and batched-image fallback are retained. Both camera
paths use the same selected lidar backend. A lightweight message-filter envelope
retains the source cloud and timestamp without downloading GPU points. NITROS read
handles coordinate buffer lifetime with the core's nondefault CUDA stream.

### Building the three repositories together

The perception image must include matching versions of the NITROS point-cloud
package and the vendor's `lidar-preprocess.hpp` / `forward_device_lidar` interface.
Rebuild the image after changing its pinned dependency commits; rebuild consumers
together after changing the point-cloud type's ABI. Dependency commits must be
available in their origin repositories before a clean Docker clone can fetch them.

For local development before publishing dependency commits, build the checked-out
NITROS point-cloud package against the perception underlay, source that overlay,
and build the application with
`-DCUDA_BEVFUSION_SOURCE_DIR=/path/to/wato-cuda-bevfusion`. The vendor directory must
be writable for protobuf generation. The package environment hook adds the GXF serialization/compositor runtime paths
needed by the existing NITROS camera libraries. Legacy point-cloud GXF conversion
is disabled by default to avoid its bundled-protobuf conflict with BEVFusion.

### Validation and performance comparison

1. Verify PointCloud2 to NITROS round-trips preserve field metadata, intensity,
   organized row padding, headers, and payload bytes. Check empty/malformed inputs,
   shared ownership, and rejection of XYZI by legacy XYZ/XYZRGB-only GXF.
2. Build the vendor with `-DBUILD_LIDAR_PREPROCESS_TESTS=ON` and run
   `ctest -R lidar_preprocess --output-on-failure`. This model-independent GPU test
   compares FP16 bytes against a CPU reference and exercises truncation, invalid
   values, unaligned offsets, padded rows, both byte orders, and capacity errors.
3. Run the monorepo's BEVFusion tests. Backend selection tests inject an initialization
   failure: `auto` selects CPU and records the reason, while `cuda` throws. Additional
   checks cover CPU subscription cleanup and timestamp matching for both envelopes.
4. On deployment hardware, replay the same bag with identical models, calibration,
   camera settings, QoS, and sync tolerance. Compare `cpu` and `cuda`, warming up first
   and alternating repeated trials. Hold PR #599's camera configuration constant.
   Compare prepared features and detections for identical sensor timestamp pairs;
   use the CPU baseline's repeatability to establish detection tolerances.
5. Record p50/p95/p99 processing and receipt-to-publication latency, CPU utilization,
   GPU memory, output rate, and unmatched/dropped scans at normal and increased replay
   rates. Capture a short Nsight Systems CUDA/NVTX trace to inspect copies and kernels;
   collect final throughput without tracing overhead. There should be one raw-cloud
   upload from a CPU publisher, scalar count downloads, and no full-cloud download or
   second upload before inference. The existing callback profiler excludes receipt
   and sync wait; add receive/publish tracepoints for end-to-end measurements.
6. Exercise configure/activate/deactivate/cleanup/reactivation with CPU and CUDA lidar,
   each with NITROS cameras and batched cameras. Check `auto` failure selection before
   activation using an intentionally oversized workspace on a test deployment.

A lower lidar preparation time is not by itself proof of higher whole-pipeline FPS:
GPU contention, raw upload size, camera processing, and model inference may dominate.

### Local implementation validation (2026-11-17)

- CUDA 13 / SM 89 vendor library and ROS 2 Jazzy BEVFusion package builds passed.
- All 58 BEVFusion core/node test cases passed with the package's sourced runtime setup.
- All six point-cloud regression cases passed, including a real CPU PointCloud2
  publisher to Managed NITROS subscriber and GPU payload round-trip.
- The standalone CUDA preprocessing test passed on the available GTX 1080 using
  CUDA 12.8 / SM 61. Production CUDA 13 does not compile for this older GPU.
- An actual incompatible CUDA 13 / SM 89 binary verified initialization fallback:
  `auto` selected CPU and strict `cuda` rejected initialization.
- The optional legacy point-cloud GXF configuration compiled separately.

Full-model detection equivalence and end-to-end latency/FPS remain unmeasured:
this checkout has neither BEVFusion model assets nor a representative recorded bag,
and the available GPU is older than the production architecture. The checks above
establish transport/preprocessing correctness and build integration, not a measured
whole-pipeline performance gain.
