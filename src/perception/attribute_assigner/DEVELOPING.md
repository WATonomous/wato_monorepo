# Attribute Assigner - Developer Guide

## Overview

The **Attribute Assigner** is a ROS 2 lifecycle composable node that enriches 2D object detections with semantic attributes using heuristic-based image analysis. It also estimates 3D positions for detected objects using camera intrinsics and TF transforms.

Given a stream of 2D detections (from YOLOv8 or similar) and synchronized camera images, the node:

1. **Classifies traffic light state** (red / yellow / green) via HSV color analysis
2. **Detects car behaviors** (braking, turning, hazard lights) via brake/signal light detection
3. **Estimates 3D bounding boxes** for traffic lights and vehicles using monocular depth estimation
4. **Publishes visualization markers** for debugging in Foxglove

## Architecture

```
                        ┌─────────────────────────────┐
  MultiImageCompressed ─┤                             ├─► MultiDetection2DArray (enriched)
                        │   ApproximateTime Sync      │
  MultiDetection2DArray ┤                             ├─► Detection3DArray
                        │   AttributeAssignerNode     │
  MultiCameraInfo ──────┤   (lifecycle composable)    ├─► ImageMarker (2D overlays)
       (cached)         │                             │
                        │                             ├─► MarkerArray (3D boxes)
                        └─────────────────────────────┘
```

### Node / Core Separation

- **`AttributeAssignerNode`** — ROS 2 lifecycle node handling subscriptions, message synchronization, TF lookups, 3D projection, and marker publishing. Contains no CV logic.
- **`AttributeAssignerCore`** — Pure C++ class with no ROS dependencies. Owns all OpenCV-based classification logic (HSV analysis, blob detection). Testable in isolation.

## Topics

### Subscribed (via ApproximateTime sync)

| Topic | Type | Description |
|-------|------|-------------|
| `/multi_camera_sync/multi_image_compressed` | `deep_msgs/MultiImageCompressed` | Synchronized compressed images from multiple cameras |
| `/perception/detections` | `deep_msgs/MultiDetection2DArray` | Per-camera 2D detections from object detector |

### Subscribed (cached, not synced)

| Topic | Type | Description |
|-------|------|-------------|
| `/multi_camera_sync/multi_camera_info` | `deep_msgs/MultiCameraInfo` | Camera intrinsics for 3D projection |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections_enriched` | `deep_msgs/MultiDetection2DArray` | Original detections with appended attribute hypotheses |
| `/perception/detections_3d` | `vision_msgs/Detection3DArray` | 3D bounding boxes for traffic lights and vehicles |
| `/perception/enriched_detection_markers` | `visualization_msgs/ImageMarker` | 2D bounding box overlays colored by state (toggle: `enable_image_markers`) |
| `/perception/detections_3d_markers` | `visualization_msgs/MarkerArray` | 3D cube markers colored by state (toggle: `enable_3d_markers`) |

## Enrichment Format

Attributes are appended as additional `ObjectHypothesisWithPose` entries in each detection's `results` array. The original detection hypothesis is preserved.

**Traffic lights** use the prefix `state:`:
- `state:red`, `state:yellow`, `state:green`
- Confidence scores are based on the ratio of bright pixels matching each hue bucket

**Cars** use the prefix `behavior:`:
- `behavior:braking`, `behavior:turning_left`, `behavior:turning_right`, `behavior:hazard_lights`
- Confidence is derived from the coverage ratio of red/amber pixels in relevant regions

## 3D Detection Pipeline

Monocular depth estimation using known physical dimensions:

1. **Depth estimation**: `depth = (real_width * focal_length) / bbox_width_pixels`
   - Traffic lights use height instead of width
   - Per-class dimensions for car, truck, and bus
2. **Projection clamping**: 3D box dimensions are clamped so their projection does not exceed the 2D bbox
3. **TF transform**: 3D point is transformed from camera frame to `target_frame` (default: `base_link`)
4. **Orientation**: Vehicle boxes are oriented to face the ego vehicle (yaw from origin to detection position)

### Accuracy: Traffic Lights vs Vehicles

Traffic light 3D positions are significantly more accurate than vehicle positions because traffic lights are a **standardized physical size** — their real-world dimensions are known and consistent. This means the monocular depth estimate (`depth = real_height * fy / bbox_height_pixels`) is reliable.

Vehicles, on the other hand, vary widely in size. A compact car and a full-size SUV both map to class `car` but differ by over a meter in every dimension. The configurable per-class sizes (car/truck/bus) are averages, so depth estimates will be over- or under-estimated for vehicles that deviate from the assumed dimensions.

### Known Limitations

- **Monocular depth is approximate** — accuracy degrades at long range and with partial occlusions. It assumes the full object is visible.
- **Fixed-size assumption** — all cars use the same dimensions (configurable per class: car/truck/bus), which is inaccurate for vehicles that don't match the assumed size.
- **No temporal tracking** — each frame is processed independently. There is no smoothing or tracking across frames, so 3D positions may jitter.

## Traffic Light Classification

The classifier uses HSV color analysis on a vertical (or horizontal) center strip of the detection crop:

1. **Strip margin**: configurable fraction (`traffic_light_strip_margin`) of the bounding box width is ignored on each side to avoid housing/frame pixels. Automatically rotated for horizontal traffic lights.
2. **Bright pixel mask**: pixels with `V >= min_value` and `S >= min_saturation` are considered "lit"
3. **Hue bucketing**: bright pixels are classified into red, yellow, or green based on configurable hue boundaries
4. **Confidence**: each color's score is `pixel_count / total_bright_pixels`
5. **Default to red**: if too few bright pixels are found (e.g., side-view traffic lights where the bulbs aren't visible), the classifier defaults to red. **This is intentional** — on an autonomous vehicle, an unreadable traffic light must be treated as red (stop). This means you may see false red classifications for traffic lights viewed at extreme angles, which is the desired safe behavior.

### Tuning Traffic Light Thresholds

The HSV thresholds are the most likely parameters you'll need to tune. OpenCV uses H: 0-180, S: 0-255, V: 0-255.

Key params in `perception_bringup.yaml`:
- `traffic_light_min_value` / `traffic_light_min_saturation` — raise these if too many non-light pixels pass the bright mask (housing reflections, sky)
- `traffic_light_red_hue_lo` / `traffic_light_red_hue_hi` — red wraps around the hue wheel: `H <= lo OR H >= hi`
- `traffic_light_strip_margin` — increase (up to 0.45) if side pixels from the housing are interfering

To debug, temporarily enable the pixel overlay by adding a POINTS ImageMarker in `createDetectionMarkers` (see git history for examples).

## Car Behavior Classification

Uses a single-pass HSV scan of the detection crop:

- **Braking**: red hue pixels in the lower portion of the bounding box
- **Turning**: amber hue pixels concentrated on the left or right side
- **Hazard lights**: amber pixels on both sides simultaneously

Key params: `car_brake_min_brightness`, `car_brake_min_saturation`, `car_red_hue_lo/hi`, `car_amber_*`

## Parameters

All parameters are set in `perception_bringup/config/perception_bringup.yaml` under `/**/attribute_assigner_node`. The node's own `config/params.yaml` contains defaults.

### Quick Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_detection_confidence` | 0.3 | Minimum YOLO score to process a detection |
| `traffic_light_strip_margin` | 0.25 | Fraction to ignore on each side of TL crop |
| `car_real_width/height/length` | 1.8/1.8/6.0 | Car dimensions for depth estimation (m) |
| `truck_real_width/height/length` | 2.5/3.5/8.0 | Truck dimensions (m) |
| `bus_real_width/height/length` | 2.5/3.2/12.0 | Bus dimensions (m) |
| `enable_image_markers` | true | Toggle 2D overlay markers |
| `enable_3d_markers` | true | Toggle 3D visualization markers |
| `sync_max_time_diff_ms` | 2000.0 | Max time difference for ApproximateTime sync (ms) |
| `target_frame` | "base_link" | TF frame for 3D detections |

## Common Issues

### No 3D detections appearing
1. Check that `MultiCameraInfo` is being published — the node will log: `3D detections skipped: no cached MultiCameraInfo`
2. Verify TF is available from camera frames (e.g., `camera_pano_nn`) to `target_frame`
3. The timestamp passed to `create3DDetections` must match the TF tree — use the detection array's stamp, not the multi-image stamp

### Traffic light always classified as red/wrong color
- The HSV thresholds may need tuning for your camera's color response
- Check `traffic_light_strip_margin` — if too small, housing pixels contaminate the analysis
- Side-view traffic lights with few bright pixels should default to red

### Car braking score too low
- Lower `car_brake_min_brightness` and `car_brake_min_saturation` — brake lights on white cars in daylight appear washed out
- The score represents the ratio of red pixels to total pixels in the analysis region

## Building

Built as part of the perception module via colcon:

```bash
colcon build --packages-select attribute_assigner
```

Dependencies: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `message_filters`, `vision_msgs`, `visualization_msgs`, `deep_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `diagnostic_updater`, `OpenCV`

## Future Development

Ideally in the future OpenCV Cuda is installed in the container then all OpenCV operations can be parallelized through the GPU.
