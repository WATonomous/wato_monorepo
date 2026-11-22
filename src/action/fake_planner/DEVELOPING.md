# Developing the fake planner

## Code structure

- `FakePlannerNode` owns ROS interfaces and playback state.
- `FakePlannerCore` expands maneuver segments, anchors the path, and selects the
  controller window.
- Headers contain declarations and Doxygen comments. Implementations live in
  their `.cpp` files.

The node has three main operations:

| Operation | Responsibility |
|-----------|----------------|
| `loadTrajectory()` | Read parameters, validate them, and load the JSON |
| `resetTrajectory()` | Reuse the loaded path and reset matching and completion |
| `publishTrajectory()` | Publish the current window while released |

Lifecycle and service callbacks call these operations. Topic routing stays in
launch files. There are no topic-name parameters in `params.yaml`.

## Geometry

Segments share a moving coordinate frame. Each appends points and advances that
frame. The path is resampled at `sample_spacing_m` before anchoring.

Open paths have a braking profile and zero-speed stop pad. Closed paths wrap at
the seam. The bounded nearest-point search follows path progress and avoids
jumping to an earlier crossing.

These operations belong in the helper. Keep ROS clocks, lifecycle transitions,
controller engagement, and visualization in the node.

## Tests

The core tests cover JSON validation, segment geometry, spacing, speed profiles,
anchoring, path matching, and open/closed windows.

From a prepared action environment:

```bash
colcon test --packages-select fake_planner --event-handlers console_direct+
colcon test-result --verbose
```

For runtime checks, activate with `start_on_activate:=false`. Confirm preview
markers appear before release. Start, stop, reset, and repeat the maneuver.
Check that reset waits for fresh odometry and cleanup releases all interfaces.
