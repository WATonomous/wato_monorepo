# Developing carla_ros_bridge

## Architecture

This is a metapackage that serves as the root of the CARLA ROS 2 bridge workspace.

## Package Dependencies

The metapackage lists all component packages as dependencies in `package.xml`. This allows installing the entire bridge with a single package.

## Adding New Packages

1. Create the new package in the workspace
2. Add `<exec_depend>new_package</exec_depend>` to this package's `package.xml`

## Common Patterns

All CARLA bridge packages follow these patterns:

### Lifecycle Nodes

All nodes that connect to CARLA are lifecycle nodes:
- `on_configure`: Connect to CARLA, find ego vehicle
- `on_activate`: Start publishers/timers
- `on_deactivate`: Stop publishers/timers, brake vehicle
- `on_cleanup`: Release CARLA resources

### CARLA Connection

```python
self.carla_client = carla.Client(host, port)
self.carla_client.set_timeout(timeout)
world = self.carla_client.get_world()
```

### Ego Vehicle Discovery

```python
vehicles = world.get_actors().filter("vehicle.*")
ego = [v for v in vehicles if v.attributes.get("role_name") == role_name]
```

### Coordinate Systems

CARLA uses left-handed coordinates. Convert to ROS right-handed:
- Flip Y axis
- Flip yaw and pitch signs
