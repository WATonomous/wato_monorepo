# Costmap Markers

Converts the costmap footprint polygon into a `visualization_msgs/MarkerArray` for rviz2 / Foxglove.

## Overview

Subscribes to a `geometry_msgs/PolygonStamped` footprint (published by the costmap node) and re-publishes it as a closed `LINE_STRIP` marker. Color and line width are configurable.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"base_link"` | Frame ID stamped on markers |
| `footprint_color` | double[] | [0.0, 1.0, 0.0, 0.8] | RGBA color for the footprint outline |
| `footprint_line_width` | double | 0.05 | Line width in metres |

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `footprint` | `geometry_msgs/PolygonStamped` | Vehicle footprint polygon |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `markers` | `visualization_msgs/MarkerArray` | Footprint visualization markers |
