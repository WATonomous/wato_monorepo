# Multi-Object Tracking

ROS2 Node for tracking 3D objects detected around the car.

## Overview

This tracker uses a modified version of ByteTrack to track 3D bounding boxes.
Core logic follows that of ByteTrackV2 (Zhang et al., 2023). The implementation is built on top of [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp) with modifications made for 3D tracking.

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `[placeholder]` | `vision_msgs/Detection3DArray` | Incoming 3D detections |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `tracked_boxes` | `vision_msgs/Detection3DArray` | Tracked detections |

## Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_rate`| int | 30 | Frame rate of the input detections |
| `track_buffer` | int | 30 | Amount of consecutive frames a track can stay unmatched before getting removed |
| `track_thresh` | float | 0.5 | Threshold between high and low confidence detections |
| `high_thresh` | float | 0.6 | Minimum detection confidence score required to start a new track |
| `match_thresh` | float | 0.8 | Maximum IoU cost to still be considered a match |
| `output_frame` | string | "map" | Frame to output tracks in |

## Acknowledgements

This package uses a modified version of [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp).
The original repository is licensed under MIT License (see THIRD_PARTY_LICENSES/BYTETRACK_LICENSE for details).

ByteTrackV2 paper can be found at:<br>
Y. Zhang et al., “ByteTrackV2: 2D and 3D Multi-Object Tracking by Associating Every Detection Box,” arXiv, Mar. 2023, https://doi.org/10.48550/arXiv.2303.15334
