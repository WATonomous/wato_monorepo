# simple_prediction

Constant-velocity prediction stub for World Modeling. Bridges tracked 3D
detections to `WorldObjectArray` seeds for the world model. It is the drop-in
stand-in for the (currently disabled) heavy `prediction` node.

For each detection it takes the bbox center + yaw, estimates speed from bbox
length (`> 3.5 m → 5 m/s` vehicle, else `1.4 m/s` pedestrian), and emits a
single straight-line hypothesis (`conf = 1.0`) over the prediction horizon.
No history and no lanelet matching, so it is robust to tracker yaw that spins.

## I/O contract

| Direction | Node topic | Type | Wired to (car) |
|-----------|-----------|------|----------------|
| Sub | `tracks_3d` | `vision_msgs/Detection3DArray` | `/perception/tracked_detections_3d` |
| Pub | `world_object_seeds` | `world_model_msgs/WorldObjectArray` | `/world_modeling/world_object_seeds` |

Runs as a **lifecycle node** named `simple_prediction_node` in namespace
`world_modeling`, managed by `world_modeling_lifecycle_manager` (autostart
configures + activates it).

Parameters (`config/params.yaml`):

- `prediction_horizon` (default `3.0` s) — how far ahead to predict.
- `prediction_time_step` (default `0.2` s) — spacing between predicted poses.

## Testing on the car

The node is wired into `world_modeling_bringup`, so it comes up with the full
World Modeling stack. All commands run **inside the world modeling dev
container** on the car / dev node.

### 1. Build

```bash
cd /tmp/wato_monorepo
./watod -t world_modeling_bringup_dev          # interactive shell (ROS sourced)
cd /ws
colcon build --packages-select simple_prediction world_modeling_bringup --symlink-install
source /ws/install/setup.bash
```

### 2. Confirm the tracker is actually publishing

The bringup subscribes to `/perception/tracked_detections_3d` by default. Make
sure the perception tracker is up and producing detections first — an empty
input is the most common reason for "no predictions":

```bash
ros2 topic list | grep tracked_detections_3d      # confirm the topic exists
ros2 topic hz /perception/tracked_detections_3d    # confirm it's publishing
```

If the car publishes on a different topic, override it at launch (see step 3).

### 3. Launch

```bash
# Car (default input topic):
ros2 launch world_modeling_bringup world_modeling.launch.yaml

# Override the input topic if the tracker publishes elsewhere:
ros2 launch world_modeling_bringup world_modeling.launch.yaml \
    tracks_topic:=/your/tracked_detections_topic

# Sim (CARLA):
ros2 launch world_modeling_bringup world_modeling.launch.yaml \
    tracks_topic:=/carla/tracked_detections_3d
```

### 4. Verify

```bash
# Node reached the active lifecycle state
ros2 lifecycle get /world_modeling/simple_prediction_node        # -> active

# Predictions are flowing (rate should roughly track the input rate)
ros2 topic hz /world_modeling/world_object_seeds
ros2 topic echo /world_modeling/world_object_seeds --once
```

Sanity check the output: each `objects[i].predictions[0].poses` should contain
~`horizon / step` poses (15 with defaults), stepping `speed * step` metres along
the detection's yaw (~1.0 m/step for a vehicle, ~0.28 m/step for a pedestrian),
with `z` held constant and `conf = 1.0`.

### 5. Visualize (Foxglove)

`world_modeling_bringup` publishes pre-enrichment markers on
`/world_modeling/world_object_seeds_markers` — enable that topic to see the
straight-line prediction paths overlaid on the tracked objects.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Node stuck in `unconfigured` / `inactive` | Lifecycle manager didn't activate it — check `world_modeling_lifecycle_manager` logs; confirm `/world_modeling/simple_prediction_node` is in its `node_names`. |
| Node `active` but `world_object_seeds` silent | No input — verify `/perception/tracked_detections_3d` (step 2) or fix `tracks_topic`. |
| Predictions point the wrong way | Detection yaw/quaternion from the tracker is off; this node trusts it verbatim. |
| Speeds look wrong | Speed is a bbox-length heuristic only (vehicle vs pedestrian), not measured velocity. |

## Notes

- Build + startup verified in a clean `ros:jazzy` container against `main`'s
  `world_model_msgs` / `lanelet_msgs`.
- This is a placeholder predictor; the ML (MTR) predictor under development is
  intended to eventually replace it behind the same `tracks_3d` /
  `world_object_seeds` contract.
