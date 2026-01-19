# lanelet_msgs

ROS2 message and service definitions for Lanelet2 map queries.

## Messages

- `Lanelet` - Complete per-lanelet data (geometry, semantics, connectivity)
- `Corridor` - Multi-lane sampled corridor for motion planning
- `CurrentLaneContext` - Real-time ego lane context (published as topic)
- `MapVisualization` - Subset of map for RViz visualization
- `TrafficLightInfo` - Traffic light position and stop line
- `StopLineInfo` - Stop line geometry
- `LanePath` - Single lane centerline and widths within a corridor
- `LaneDivider` - Lane divider with boundary types

## Services

- `GetRoute` - Get route as sequence of Lanelets
- `GetCorridor` - Get multi-lane corridor for planning
