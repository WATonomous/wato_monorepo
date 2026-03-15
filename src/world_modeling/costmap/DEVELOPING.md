# Costmap - Developer Guide

## Architecture

The `CostmapNode` is a lifecycle node that owns a list of `CostmapLayer` instances.
A factory map (`kLayerFactory`) maps layer name strings to constructors. On configure,
the node iterates the `layers` parameter, instantiates each layer, and calls
`layer->configure()`.

On each publish tick (`publishCostmap()`):

1. Look up `costmap_frame` -> `map_frame` transform.
2. Allocate a zeroed `OccupancyGrid` centred on the origin.
3. Call `layer->update(grid, transform)` for each layer; layers write costs using `std::max`.
4. Publish the composed grid.
5. If `footprint_front_left != footprint_rear_right`, publish the footprint polygon.

## Adding a New Layer

1. Create `include/costmap/layers/my_layer.hpp` inheriting `CostmapLayer`.
2. Implement `configure`, `activate`, `deactivate`, `cleanup`, `update`.
3. Add the source to `CMakeLists.txt`.
4. Register the layer in the `kLayerFactory` map in `costmap_node.cpp`.
5. Add the layer name to the `layers` parameter in config.

## Footprint

The vehicle footprint is defined by two opposite corners (`footprint_front_left` and
`footprint_rear_right`) in `[x, y]` format relative to `costmap_frame`. It serves two
purposes:

- Published as a `PolygonStamped` for downstream visualization (`costmap_markers`).
- Used by `PointCloudLayer` to exclude LiDAR returns from the vehicle body.

Set both to `[0.0, 0.0]` to disable.

## Building

```bash
colcon build --packages-select costmap
```

## Testing

```bash
# Run standalone
ros2 launch costmap costmap.launch.yaml

# Verify topics
ros2 topic echo /world_modeling/costmap --once
ros2 topic echo /world_modeling/costmap_footprint --once
```
