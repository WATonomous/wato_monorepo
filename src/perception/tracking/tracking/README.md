# Multi-Object Tracking

ROS2 Node for tracking 3D objects detected around the car.

## Overview

This tracker uses a modified version of ByteTrack to track 3D bounding boxes.
Core logic draws inspiration from ByteTrackV2 (Zhang et al., 2023). The implementation is built on top of [`ByteTrack-cpp`](https://github.com/Vertical-Beach/ByteTrack-cpp) with modifications made for 3D tracking.

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections_3D` | `vision_msgs/Detection3DArray` | Incoming 3D detections |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections_3D_tracked` | `vision_msgs/Detection3DArray` | Tracked detections, track velocities are stored in the results field (ObjectHypothesisWithPose[]) |

## Parameters

### Global Defaults

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_rate`| int | 30 | Frame rate of the input detections |
| `track_buffer` | int | 30 | Amount of consecutive frames a track can stay unmatched before getting removed |
| `track_thresh` | float | 0.5 | Threshold between high and low confidence detections |
| `high_thresh` | float | 0.6 | Minimum detection confidence score required to start a new track |
| `match_thresh` | float | 0.8 | Maximum IoU cost to still be considered a match |
| `use_maj_cls` | bool | true | Use most frequent class as track's class if true, use most recent class otherwise |
| `use_R_scaling` | bool | false | Scale R matrix in Kalman Filter according to detection confidence |
| `dist_metric` | string | "IOU" | Which distance metric to use (currently supports "IOU", "DIOU", "CIOU") |
| `output_frame` | string | "map" | Frame to output tracks in |

### Prediction (Kalman Filter)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `centroid_history_size` | int | 10 | Past centroids stored per track for velocity seeding |
| `prediction_time` | double | 5.0 | How far into the future to predict (seconds) |
| `prediction_dt` | double | 0.1 | Time step between predicted poses (seconds) |
| `process_noise` | double | 0.1 | KF process noise (higher = trust measurements more, noisier velocity) |
| `measurement_noise` | double | 0.5 | KF measurement noise (higher = smoother tracks, filters outliers) |

### Per-Class Tracking Overrides

When configured, separate `BYTETracker` instances run per class with independent thresholds and distance metrics. Detections are split by class before tracking and merged afterward with unique track ID offsets to avoid collisions. Classes not listed use the global defaults.

Empty arrays disable per-class tracking (single global tracker for all classes).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `class_tracking.classes` | string[] | [] | Class names (parallel with arrays below) |
| `class_tracking.match_thresholds` | double[] | [] | Max IoU cost per class (higher = more permissive matching) |
| `class_tracking.high_thresholds` | double[] | [] | Min confidence to create new track per class |
| `class_tracking.track_thresholds` | double[] | [] | High/low confidence split per class |
| `class_tracking.dist_metrics` | string[] | [] | Distance metric per class ("IOU", "DIOU", "CIOU") |

**Example** (cars use strict IOU, persons use permissive CIOU):

```yaml
class_tracking:
  classes:           ["car",  "person", "truck", "bus"]
  match_thresholds:  [0.95,   0.99,     0.95,    0.95]
  high_thresholds:   [0.40,   0.001,    0.10,    0.10]
  track_thresholds:  [0.10,   0.01,     0.10,    0.10]
  dist_metrics:      ["IOU",  "CIOU",   "IOU",   "IOU"]
```

**Why per-class?** Small objects (persons, bicycles) have tiny 3D bounding boxes where frame-to-frame BEV IoU is near zero even for the same physical object. CIOU with a high match threshold bridges this gap. Large objects (cars, trucks) have stable boxes where standard IOU works well and CIOU can cause ID switches.

## Acknowledgements

This package uses a modified version of [`ByteTrack-cpp`](https://github.com/Vertical-Beach/ByteTrack-cpp).
The original repository is licensed under MIT License (see [`THIRD_PARTY_LICENSES/BYTETRACK_LICENSE`](../THIRD_PARTY_LICENSES/BYTETRACK_LICENSE) for details).

ByteTrackV2 paper can be found at:<br>
Y. Zhang et al., “ByteTrackV2: 2D and 3D Multi-Object Tracking by Associating Every Detection Box,” arXiv, Mar. 2023, https://doi.org/10.48550/arXiv.2303.15334
