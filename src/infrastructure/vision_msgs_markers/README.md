# vision_msgs_markers

Generic ROS2 node that converts `vision_msgs/msg/Detection3DArray` to `visualization_msgs/msg/MarkerArray` for visualization in Foxglove or RViz.

## Features

- Wireframe bounding boxes (LINE_LIST markers)
- Text labels showing class ID and confidence score
- Colors assigned per class ID (consistent hashing into 12-color palette)
- Semi-transparent markers (configurable alpha)

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `detections_in` | `vision_msgs/msg/Detection3DArray` | Subscribe | Input 3D detections |
| `markers_out` | `visualization_msgs/msg/MarkerArray` | Publish | Output visualization markers |

Use topic remappings to connect to your actual topics.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_alpha` | double | 0.6 | Transparency of box outlines (0.0-1.0) |
| `text_scale` | double | 0.3 | Size of text labels |
| `line_width` | double | 0.05 | Thickness of box edges |

## Usage

Launch from your module's bringup with topic remappings:

```yaml
- node:
    pkg: vision_msgs_markers
    exec: vision_msgs_markers_node
    name: detection_viz
    output: screen
    remappings:
      - from: detections_in
        to: /perception/detections_3d
      - from: markers_out
        to: /visualization/detection_markers
    param:
      - name: marker_alpha
        value: 0.7
```

## Color Palette

Classes are assigned colors via consistent hashing:

- Red, Green, Blue, Yellow, Magenta, Cyan
- Orange, Purple, Spring Green, Rose, Lime, Sky Blue
