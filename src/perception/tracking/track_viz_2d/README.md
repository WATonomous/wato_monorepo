# Tracking Visualization Node
This node decodes and annotates compressed images with 2D bounding boxes for both the detections and the tracks.
## Subscribed Topics
- `detections_topic`: topic to subscribe to for detections
- `track_topic`: topic to subscribe to for tracks
## Published-to Topics
- `annotations_topic`: topic to publish foxglove annotations to
## Other Parameters
- [string] `camera_frame`: frame of camera providing the images
- [string] `color_dets`: color of detection bounding boxes
- [string] `color_trks`: color of tracker bounding boxes
- [int] `bbox_line_width`: bbox line width in pixels
