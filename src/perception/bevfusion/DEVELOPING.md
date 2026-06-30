# BEVFusion - Developer Guide

## Overview

**BEVFusion** is a ROS 2 lifecycle composable node that fuses camera and LiDAR data into a unified Bird's-Eye View (BEV) representation for 3D object detection and map segmentation.

Rather than forcing cameras to see in 3D or LiDAR to see in 2D, both are converted into a top-down BEV grid where they are fused and processed together. This maintains both geometric structure and semantic density, and compensates for individual sensor weaknesses — cameras struggle in low light, LiDAR struggles in poor weather.

Given synchronized camera images and a merged LiDAR point cloud, the node:

1. **Preprocesses cameras** — resizes, normalizes, and builds camera matrices (intrinsic + extrinsic) for each of the 6 cameras
2. **Preprocesses LiDAR** — range-filters and voxelizes the merged point cloud
3. **Runs TensorRT inference** — using the CUDA-BEVFusion library (NVIDIA-AI-IOT)
4. **Publishes 3D bounding boxes** — for detected objects (vehicles, pedestrians, etc.)
5. **Publishes visualization markers** — for debugging in Foxglove

## Architecture

```
Sensor Inputs
─────────────
/camera_pano_[nn|ne|nw|ss|se|sw]/image_rect/nitros  ──┐
(6 cameras, NitrosImage, stored in GPU memory)         │
                                                       ├──► BEVFusionNode (lifecycle composable)
/lidar/all/points_merged                               │         │
(3 LiDARs pre-merged)                               ──┘         │
                                                         ▼
                                              BEVFusionCore
                                              ├── Camera preprocessing
                                              ├── LiDAR preprocessing
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

Eve has 3 Velodyne LiDARs (`lidar_cc`, `lidar_ne`, `lidar_nw`). They are pre-merged by the `lidar_aggregator` node into a single point cloud. BEVFusion subscribes to the merged output directly.

## Topics

### Subscribed — Camera Images (Nitros zero-copy)

BEVFusion subscribes to 6 individual Nitros topics, one per camera. These keep images stored in GPU memory from capture all the way through inference — no CPU copies or compression roundtrip.

| Topic | Type | Description |
|---|---|---|
| `/camera_pano_nn/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Front camera, rectified, stored in GPU memory |
| `/camera_pano_ne/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Front-right camera, rectified, stored in GPU memory |
| `/camera_pano_nw/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Front-left camera, rectified, stored in GPU memory |
| `/camera_pano_ss/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Back camera, rectified, stored in GPU memory |
| `/camera_pano_se/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Back-right camera, rectified, stored in GPU memory |
| `/camera_pano_sw/image_rect/nitros` | `nvidia::isaac_ros::nitros::NitrosImage` | Back-left camera, rectified, stored in GPU memory |

**Why `image_rect/nitros` and not other camera topics:**
Each camera also publishes `image_rect` (`sensor_msgs/Image`, CPU) and the batched `/multi_camera_sync/multi_image_compressed` (JPEG, CPU) used by other perception nodes. BEVFusion uses `image_rect/nitros` because it is both rectified (lens distortion removed, required for BEVFusion's camera matrix math) and stored in GPU memory (no CPU copy needed before feeding into TensorRT). The `/nitros` topic is published automatically by the Isaac ROS Nitros framework alongside every standard topic from a Nitros node.

**Time synchronization:** Because these arrive as 6 separate streams, the node uses `message_filters::ApproximateTimeSynchronizer` to match them by timestamp — ensuring each inference call receives 6 camera images and 1 LiDAR scan that all correspond to the same moment in time. The maximum allowed time difference between matched messages is configurable via `sync_max_time_diff_ms`.

**Fallback:** If Nitros subscriber integration proves too complex initially, subscribe to `/multi_camera_sync/multi_image_compressed` (`deep_msgs/MultiImageCompressed`) — all 6 cameras pre-batched and pre-synced in one message, already used by `attribute_assigner` and `spatial_association`, but at the cost of a JPEG CPU roundtrip.

### Subscribed — Other

| Topic | Type | Description |
|---|---|---|
| `/lidar/all/points_merged` | `sensor_msgs/PointCloud2` | All 3 LiDARs pre-merged into one point cloud |
| `/multi_camera_sync/multi_camera_info` | `deep_msgs/MultiCameraInfo` | Camera intrinsics for all cameras (cached, not time-synced) |

LiDAR has no Nitros equivalent — `PointCloud2` on CPU is the only option.

**Camera extrinsics via TF:** The physical mounting position and orientation of each camera (extrinsics) are looked up at runtime from the ROS 2 TF tree using `tf2_ros::Buffer` and `tf2_ros::TransformListener`. The node requests the transform from each camera's frame ID (e.g. `camera_pano_nn`) to `base_link`. Since camera mounts are fixed, these transforms are *static* — they are published once on `/tf_static` by the sensor launch infrastructure (from the camera calibration files in `camera_calib`). The node does not subscribe to `/tf_static` directly; `tf2_ros::TransformListener` creates that subscription internally and caches all available transforms in the `Buffer`.

### Published

| Topic | Type | Description |
|---|---|---|
| `/perception/detections_3d_bev` | `vision_msgs/Detection3DArray` | 3D bounding boxes for detected objects |
| `/perception/bev_detection_markers` | `visualization_msgs/MarkerArray` | Foxglove visualization markers |

## Parameters

When implemented, parameters will be set in `perception_bringup/config/perception_bringup.yaml` under `/**/bevfusion_node`. The node's own `config/params.yaml` will contain defaults. The table below lists the intended parameters — these are not yet declared in `params.yaml`.

| Parameter | Intended Default | Description |
|---|---|---|
| `model_path` | `""` | Path to TensorRT engine file (place in `/opt/watonomous/models/`) |
| `camera_names` | `["camera_pano_nn", "camera_pano_ne", "camera_pano_nw", "camera_pano_ss", "camera_pano_se", "camera_pano_sw"]` | Frame IDs of the 6 cameras to use |
| `sync_max_time_diff_ms` | `200.0` | Max time difference for ApproximateTime sync (ms) |
| `sync_queue_size` | `10` | Queue depth for message synchronization |
| `detection_score_threshold` | `0.3` | Minimum score to publish a bounding box |
| `qos_subscriber_reliability` | `"best_effort"` | Subscriber QoS reliability |
| `qos_publisher_reliability` | `"reliable"` | Publisher QoS reliability |

## Implementation Notes

### TensorRT + FP16

Inference uses TensorRT with FP16 precision. INT8 is deferred — it requires calibration data and can introduce localization noise. Model weights go in `/opt/watonomous/models/` which is already mounted into the container by docker-compose.

### What BEVFusion does NOT do

- **Floor removal** — the model's encoder learns to ignore the road internally
- **Clustering** — would discard geometry the model needs
- **Tracking** — handled by the `tracking` node downstream

## Building

```bash
colcon build --packages-select cuda_bevfusion_vendor bevfusion
```

`cuda_bevfusion_vendor` must be built first — it clones and compiles the CUDA-BEVFusion library from source. `bevfusion` depends on it. Colcon resolves this order automatically when both are specified.

Dependencies: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `message_filters`, `vision_msgs`, `visualization_msgs`, `deep_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `diagnostic_updater`, `OpenCV`, `cuda_bevfusion_vendor`
