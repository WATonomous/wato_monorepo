# 2D Tracking
The two packages, tracking_2d and track_viz_2d, are for tracking and visualization respectively.

## Tracker Node
The tracking node uses ByteTrack to track 2D bounding boxes in the image plane.
### Tracker Parameters
- `detections_topic`: topic to subscribe to for incoming detections
- `track_topic`: topic to publish tracks to
- ByteTrack parameters:
  - `frame_rate`
  - `track_buffer`
  - `track_thresh`
  - `high_thresh`
  - `match_thresh`

## Visualization Node
The visualization node decodes and annotates compressed images with 2D bounding boxes for both the detections and the tracks.
### Visualization Parameters
- `detections_topic`: topic to subscribe to for detections
- `track_topic`: topic to subscribe to for tracks
- `image_sub_topic`: topic to subscribe to for incoming images
- `image_pub_topic`: topic to publish annotated images to
- `camera_frame`: frame of camera providing the images
- `color_dets`: color of detection bounding boxes
- `color_trks`: color of tracker bounding boxes
- `bbox_line_width`: bbox line width

#### ROS2 Distro: Humble