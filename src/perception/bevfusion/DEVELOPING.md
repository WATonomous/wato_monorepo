# BEVFusion - Developer Guide

## Overview

**BEVFusion** is a ROS 2 lifecycle composable node that fuses camera and LiDAR data into a unified Bird's-Eye View (BEV) representation for 3D object detection and map segmentation.

Rather than forcing cameras to see in 3D or LiDAR to see in 2D, both are converted into a top-down BEV grid where they are fused and processed together. This compensates for individual sensor weaknesses — cameras struggle in low light, LiDAR struggles in rain.

Given synchronized camera images and a merged LiDAR point cloud, the node:

1. **Preprocesses cameras** — decompresses, resizes, normalizes, and builds camera matrices (intrinsic + extrinsic) for each of the 6 cameras
2. **Preprocesses LiDAR** — range-filters and voxelizes the merged point cloud
3. **Runs TensorRT inference** — using the CUDA-BEVFusion library (NVIDIA-AI-IOT)
4. **Publishes 3D bounding boxes** — for detected objects (vehicles, pedestrians, etc.)
5. **Publishes visualization markers** — for debugging in Foxglove

> BEVFusion does **not** output tracks. Tracking is handled downstream by the `tracking` node.

## Architecture

```
Sensor Inputs
─────────────
/multi_camera_sync/multi_image_compressed   ──┐
(6 cameras, pre-synced)                       │
                                              ├──► BEVFusionNode (lifecycle composable)
/lidar/all/points_merged                      │         │
(3 LiDARs pre-merged)                       ──┘         │
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

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/multi_camera_sync/multi_image_compressed` | `deep_msgs/MultiImageCompressed` | All 6 camera images bundled in one message, pre-synchronized |
| `/multi_camera_sync/multi_camera_info` | `deep_msgs/MultiCameraInfo` | Camera intrinsics for all cameras (cached, not time-synced) |
| `/lidar/all/points_merged` | `sensor_msgs/PointCloud2` | All 3 LiDARs merged into one point cloud |

Camera images and LiDAR point cloud are synchronized using `message_filters` ApproximateTime.

Camera extrinsics (physical mounting position and orientation of each camera relative to the car) are looked up from TF at runtime — frame IDs match camera names (e.g. `camera_pano_nn`) and are transformed to `base_link`.

### Published

| Topic | Type | Description |
|---|---|---|
| `/perception/detections_3d_bev` | `vision_msgs/Detection3DArray` | 3D bounding boxes for detected objects |
| `/perception/bev_detection_markers` | `visualization_msgs/MarkerArray` | Foxglove visualization markers |

## Parameters

All parameters are set in `perception_bringup/config/perception_bringup.yaml` under `/**/bevfusion_node`. The node's own `config/params.yaml` contains defaults.

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `""` | Path to TensorRT engine file (place in `/opt/watonomous/models/`) |
| `camera_names` | see above | List of 6 camera frame IDs to use |
| `sync_max_time_diff_ms` | `200.0` | Max time difference for ApproximateTime sync (ms) |
| `sync_queue_size` | `10` | Queue depth for message synchronization |
| `detection_score_threshold` | `0.3` | Minimum score to publish a bounding box |
| `qos_subscriber_reliability` | `"best_effort"` | Subscriber QoS reliability |
| `qos_publisher_reliability` | `"reliable"` | Publisher QoS reliability |

## Implementation Notes

### Why batched topics instead of individual camera topics

Each camera also publishes individually (e.g. `/camera_pano_nn/image_rect`). We subscribe to the pre-batched `/multi_camera_sync/multi_image_compressed` instead because:
- All 6 images arrive in one message with one shared timestamp — no need to sync 6 separate streams
- This is the same pattern used by every other perception node (`attribute_assigner`, `spatial_association`, etc.)

The camera pipeline is fully Nitros-based (GPU rectification via `isaac_ros_image_proc::RectifyNode`), outputting `image_rect` as a Nitros tensor and `image_rect_compressed` as JPEG. The `multi_camera_sync` node batches the JPEG outputs into `MultiImageCompressed`. Subscribing directly to individual Nitros `image_rect` topics is a future optimization if needed to hit 20 Hz.

### TensorRT + FP16

Inference uses TensorRT with FP16 precision. INT8 is deferred — it requires calibration data and can introduce localization noise. Model weights go in `/opt/watonomous/models/` which is already mounted into the container by docker-compose.

### What BEVFusion does NOT do

- **Floor removal** — the model's encoder learns to ignore the road internally
- **Clustering** — would discard geometry the model needs
- **Tracking** — handled by the `tracking` node downstream

## Building

```bash
colcon build --packages-select bevfusion
```

Dependencies: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `message_filters`, `vision_msgs`, `visualization_msgs`, `deep_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `diagnostic_updater`, `OpenCV`, CUDA-BEVFusion (NVIDIA-AI-IOT)
