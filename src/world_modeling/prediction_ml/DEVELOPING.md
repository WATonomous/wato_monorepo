# Developing prediction_ml

## Ownership boundary

`prediction_ml` owns WATO-specific ROS adaptation, fallback generation, result validation, cache
selection, and final `world_model_msgs` output. It does not own model tensors or inference.

The inference owner implements temporal history, MTR tensor packing, ONNX model execution, and
output decoding in WATonomous/deep_ros/deep_mtr. Do not reintroduce backend or tensor code in
prediction_ml.

The world-modeling image pins an immutable Deep ROS revision in `config/deep_ros.ref`. Update that
file only to a reviewed, merged revision that contains the required `deep_msgs` and `deep_mtr`
contracts.

## Data flow

```text
tracked detections ─┐
ego pose ───────────┼─> MtrSceneAdapter ──> deep_msgs/MtrScene ──> deep_mtr
lanelet context ────┘                                             │
                                                                  │ MtrPredictionArray
tracked detections ──> constant-velocity fallback                 v
                                      └──────────────> MtrResultCache
                                                            │
                                                            v
                                               world_object_seeds
```

`deep_mtr` is deliberately non-inferencing today and publishes no result. The adapter therefore
continues to publish fallback predictions when the bridge is enabled but silent.

## Build and test

Use the world-modeling image so the pinned Deep ROS packages are present:

```bash
colcon build --packages-select prediction_ml --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select prediction_ml
colcon test-result --verbose
```

Focused targets cover:

- lanelet and optional-context conversion in `test_mtr_scene_adapter`;
- correlation, validation, expiry, and replacement in `test_mtr_result_cache`;
- fallback-only lifecycle behavior in `test_prediction_ml_node`;
- fallback speed selection in `test_fallback_prediction`.

The complete module check is:

```bash
./watod test world_modeling prediction_ml
```

## Runtime smoke checks

Fallback-only:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml
ros2 lifecycle set /world_modeling/prediction_ml_node configure
ros2 lifecycle set /world_modeling/prediction_ml_node activate
ros2 param get /world_modeling/prediction_ml_node mtr.enabled
ros2 topic echo /world_modeling/world_object_seeds --once
```

Opt-in skeleton:

```bash
ros2 launch prediction_ml prediction_ml.launch.yaml enable_mtr:=true
```

After configuring and activating both lifecycle nodes, scenes may appear on `/mtr/scenes`, but
`/mtr/predictions` must remain silent until inference is implemented. A missing model, failed Deep
ROS lifecycle transition, or expired result must never suppress fallback output.
