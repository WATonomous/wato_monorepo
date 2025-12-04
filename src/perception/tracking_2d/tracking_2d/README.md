# 2D Tracking Node
This node uses ByteTrack to track 2D bounding boxes in the image plane.
## Subscribed Topics
- `detections_topic`: topic to subscribe to for incoming detections
## Published-to Topics
- `track_topic`: topic to publish tracks to
## Other Parameters:
- [int] `frame_rate`: frame rate of the input detections
- [int] `track_buffer`: how much consecutive time a track is allowed to stay unmatched before getting removed
- [float] `track_thresh`: threshold between high and low confidence detections
- [float] `high_thresh`: min detection confidence score to start a new track
- [float] `match_thresh`: max IoU cost to be considered a match

Note: Intuitively, it might seem like track_thresh and high_thresh should have their purposes swapped, but this is how it was set up in the original ByteTrack repo.
