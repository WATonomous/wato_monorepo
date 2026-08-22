# BEVFusion - Developer Guide

## Overview

**BEVFusion** is a ROS 2 lifecycle composable node that fuses camera and LiDAR data into a unified Bird's-Eye View (BEV) representation for 3D object detection. *(Note: Map segmentation is supported by the architecture but not currently implemented in the CUDA-BEVFusion C++ wrapper).*

Rather than forcing cameras to see in 3D or LiDAR to see in 2D, both are converted into a top-down BEV grid where they are fused and processed together. This maintains both geometric structure and semantic density, and compensates for individual sensor weaknesses — cameras struggle in low light, LiDAR struggles in poor weather.

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
(6 cameras, JPEG-compressed, CPU)             │
                                              ├──► BEVFusionNode (lifecycle composable)
/lidar_cc/velodyne_points                     │         │
(configurable LiDAR topic)                 ──┘         │
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

Eve has 3 Velodyne LiDARs (`lidar_cc`, `lidar_ne`, `lidar_nw`), which can be pre-merged by a `lidar_aggregator` node into `/lidar/all/points_merged`. **BEVFusion currently defaults to the single `lidar_cc` sensor** (`/lidar_cc/velodyne_points`, set via `input_lidar_topic`), not the merged cloud. The `ring` field is made optional via the `has_ring` parameter (default `false`) — set it to `true` for `lidar_cc` (which includes a `ring` field) or keep it `false` for merged clouds that may lack one. `lidar_frame_id` (default `lidar_cc`) must match whichever LiDAR topic is actually configured, since it's used as the TF target for camera extrinsics.

## Topics

### Subscribed — Camera Images

BEVFusion subscribes to the pre-batched, pre-synced multi-camera topic used by other perception nodes (`attribute_assigner`, `spatial_association`), rather than 6 individual per-camera topics:

| Topic (param) | Type | Description |
|---|---|---|
| `input_multi_image_topic` (default `/multi_camera_sync/multi_image_compressed`) | `deep_msgs/MultiImageCompressed` | All 6 cameras batched into one message, JPEG-compressed |

Each callback (`syncedCallback` in `bevfusion_node.cpp`) filters the incoming `MultiImageCompressed.images` down to only the frame IDs listed in the `camera_names` parameter, reorders them to match `camera_names`, then decompresses each with `cv::imdecode` and converts BGR → RGB (`cv::cvtColor`) before handing raw pointers to `BEVFusionCore::infer()`.

*Note: an earlier design considered subscribing to per-camera Nitros GPU-memory topics (`/camera_pano_*/image_rect/nitros`) for a zero-copy path. This was not implemented — the current node exclusively uses the compressed-image path above, with a CPU JPEG decode + color conversion per frame.*

**Time synchronization:** The node uses `message_filters::ApproximateTimeSynchronizer` to match the multi-image message and the LiDAR point cloud by timestamp — ensuring each inference call receives a synchronized camera+LiDAR frame. The queue depth is `sync_queue_size` and the maximum allowed time difference is `sync_max_time_diff_ms`.

**Important — message_filters and remapping:** `input_multi_image_topic` and `input_lidar_topic` are read directly from `params.yaml` at parameter-declaration time and passed to the `message_filters::Subscriber` constructors, because ROS 2 topic remapping does not reliably apply to `message_filters` subscribers created this way. Change these two topics via the params file, not via launch-time `remap`.

### Subscribed — Other

| Topic (param) | Type | Description |
|---|---|---|
| `input_lidar_topic` (default `/lidar_cc/velodyne_points`) | `sensor_msgs/PointCloud2` | LiDAR point cloud, must contain `x`, `y`, `z`, `intensity`, and `ring` fields |
| `camera_info` (remappable, wired to `/multi_camera_sync/multi_camera_info` in `perception.launch.yaml`) | `deep_msgs/MultiCameraInfo` | Camera intrinsics for all cameras (cached once, not time-synced) |

`processLidar()` supports an **optional** `ring` field in the incoming `PointCloud2`. Set the `has_ring` parameter to `true` (default `false`) if the configured LiDAR source includes a `ring` field (e.g. `lidar_cc` Velodyne). When `false`, ring values are omitted and only `x, y, z, intensity` are extracted per point — ensure the model's `num_features` parameter matches.

**Camera extrinsics via TF:** The physical mounting position and orientation of each camera (extrinsics) are looked up at runtime from the ROS 2 TF tree using `tf2_ros::Buffer` and `tf2_ros::TransformListener`. `computeCalibrationMatrices()` requests the transform from each camera's frame ID (e.g. `camera_pano_nn`) to the configured `lidar_frame_id` (default `lidar_cc`) — **not** `base_link`. Since camera mounts are fixed, these transforms are *static* — they are published once on `/tf_static` by the sensor launch infrastructure. The node does not subscribe to `/tf_static` directly; `tf2_ros::TransformListener` creates that subscription internally and caches all available transforms in the `Buffer`. Calibration (camera intrinsics, camera→lidar extrinsics, lidar→image projection, and the image augmentation matrix) is computed once when the first `MultiCameraInfo` message arrives (or immediately in `on_activate()` if camera info was already cached), and again is *not* recomputed per-frame.

**Output frame transform:** `createDetections3D()` performs a `tf_buffer_->lookupTransform(target_frame_, lidar_frame_id_, stamp)` per callback and applies `tf2::doTransform` to each bounding box pose, so published detections are correctly expressed in `target_frame` (default `base_link`).

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
| `input_multi_image_topic` | `/multi_camera_sync/multi_image_compressed` | Camera input topic (remap does not work for message_filters — edit here instead) |
| `input_lidar_topic` | `/lidar_cc/velodyne_points` | LiDAR input topic (same remap caveat as above) |
| `model_dir` | `/opt/watonomous/models/bevfusion/resnet50int8_trt11` | Directory with `.onnx` files and the LiDAR backbone |
| `build_dir` | `<model_dir>/build` | Directory where compiled `.plan` TensorRT engines are read from/written to |
| `precision` | `"int8"` | Model precision — `"fp16"` or `"int8"` |
| `camera_names` | 6 `camera_pano_*` frame IDs | Frame IDs of the cameras to use, in nuScenes order; also determines `num_cameras` |
| `confidence_threshold` | `0.3` | Minimum detection score to keep a bounding box |
| `image_width` / `image_height` | `1280` / `1024` | Input camera resolution |
| `resize_lim` | `0.55` | Resize ratio applied before crop, used to build `img_aug_matrix` |
| `norm_output_width` / `norm_output_height` | `704` / `256` | Network input resolution after resize+crop |
| `min_range` / `max_range` | `[-54,-54,-5]` / `[54,54,3]` | LiDAR voxelization range (x,y,z) in meters |
| `voxel_size` | `[0.075, 0.075, 0.2]` | Voxel size (x,y,z) in meters |
| `max_points_per_voxel` / `max_points` / `max_voxels` | `10` / `300000` / `160000` | LiDAR point cloud caps (GPU memory vs. coverage tradeoff) |
| `xbound` / `ybound` / `zbound` / `dbound` | see `params.yaml` | BEV grid bounds `[min, max, step]` used for camera-to-BEV projection |
| `post_center_range_start` / `post_center_range_end` | `[-61.2,-61.2,-10]` / `[61.2,61.2,10]` | Discards detections whose center falls outside this volume |
| `sync_queue_size` | `2` | Queue depth for `ApproximateTimeSynchronizer` |
| `sync_max_time_diff_ms` | `100.0` | Max time difference for ApproximateTime sync (ms) |
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

### Testing

```bash
colcon build --packages-up-to bevfusion --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select bevfusion
colcon test-result --verbose
```

Tests live under `bevfusion/test/` and use `wato_test`/Catch2. `test_bevfusion_core.cpp` mocks `::bevfusion::Core` to exercise `BEVFusionCore::infer()` / `updateCalibration()` without a real GPU pipeline. `test_bevfusion_node.cpp` exercises lifecycle transitions, parameter overrides, and calibration matrix math (including the `img_aug_matrix` computation) by using `#define private public` to reach into node internals.

### Launch

`bevfusion.launch.yaml` in this package can run the node standalone. In practice, it's loaded as a composable node into the shared `perception_container` from `perception_bringup/launch/perception.launch.yaml`, with `camera_info`, `output_detections`, and `output_markers` remapped to `/multi_camera_sync/multi_camera_info`, `/perception/detections_3d_bev`, and `/perception/bev_detection_markers` respectively, and parameters sourced from `perception_bringup/config/perception_bringup.yaml`. It's also listed in the `perception_lifecycle_manager`'s `node_names` for autostart. Recall that `input_multi_image_topic` and `input_lidar_topic` must be changed via params, not launch-time `remap` (see the message_filters note above).
