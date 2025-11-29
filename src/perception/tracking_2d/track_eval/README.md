# Tracking Evaluation Node
This node evaluates the tracker on-the-fly using MOTP, MOTA, IDF1, etc.
## Subscribed Topics
- `gts_topic`: topic to subscribe to for ground truths
- `track_topic`: topic to subscribe to for tracks
## Other Parameters
- [int] `precision`: number of decimal places to output for metric values
