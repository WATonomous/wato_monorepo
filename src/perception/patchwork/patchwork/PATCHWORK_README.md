# How to run the Patchwork++ Wrapper ROS Node

1. build the patchwork container

`watod build patchwork`

1. attach a shell or open the container and run (in mode 'develop')

`colcon build patchworkpp`

1. launch the node

`ros2 launch patchworkpp ground_removal.launch.py`

## Key Parameters (config/params.yaml)

- `sensor_height`: Expected LIDAR height above ground; increase if the sensor sits higher, decrease if it is mounted lower.
- `num_iter`: Maximum refinement loops; higher yields smoother ground at more compute cost, lower is faster but rougher.
- `num_lpr`: Lowest points per ring kept for the seed plane; increase for robustness to outliers, decrease for responsiveness to local terrain.
- `num_min_pts`: Minimum points required in a bin; higher skips sparse regions, lower tries to classify sparse data.
- `th_seeds`: Vertical tolerance when picking seed points; higher includes more candidates, lower keeps the seed set tight.
- `th_dist`: Distance threshold to accept points as ground; higher swallows more obstacles, lower keeps obstacles but may leave rough ground.
- `th_seeds_v`: Angular tolerance in the variation-aware stage; higher broadens acceptance, lower keeps it strict.
- `th_dist_v`: Variation-stage distance threshold; higher labels more points as ground, lower is conservative.
- `max_range`: Furthest radius considered; raise to filter ground farther away, lower to focus near the ego vehicle.
- `min_range`: Minimum radius processed; raise to ignore close-in returns, lower to keep near-field points.
- `uprightness_thr`: Maximum deviation of the ground normal from vertical; higher allows steeper slopes, lower accepts only flat ground.
- `enable_RNR`: Ring-wise noise removal toggle; enable to prune isolated spikes, disable to keep raw data.
- `verbose`: Diagnostic logging toggle; enable for detailed logs, disable for quiet operation.
