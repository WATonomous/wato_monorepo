# Costmap Markers - Developer Guide

## Architecture

`CostmapMarkersNode` is a simple `rclcpp::Node` (not lifecycle) that:

1. Subscribes to a `PolygonStamped` topic (`footprint`).
2. On each message, builds a `LINE_STRIP` marker from the polygon vertices, closing the loop back to the first vertex.
3. Publishes the marker as a single-element `MarkerArray`.

The node is deliberately lightweight: it has no timers or TF dependencies, and publishes only when new footprint data arrives.

## Configuration

Color is a 4-element `[R, G, B, A]` array (each 0.0-1.0). If fewer than 4 elements are provided, defaults fill in (R=0, G=1, B=0, A=0.8).

Topic remapping is handled at launch time. In the full `world_modeling` launch:

- `footprint` is remapped to `/world_modeling/costmap_footprint`
- `markers` is remapped to `/world_modeling/costmap_footprint_markers`

## Building

```bash
colcon build --packages-select costmap_markers
```

## Testing

```bash
# Run standalone
ros2 launch costmap_markers costmap_markers.launch.yaml

# Verify output
ros2 topic echo /world_modeling/costmap_footprint_markers --once
```
