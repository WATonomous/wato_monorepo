# carla_sample_description

Sample robot description (URDF) for CARLA vehicles.

This package provides a URDF that defines the TF frames for sensors on the ego vehicle. The `robot_state_publisher` reads this URDF and publishes static transforms between `base_link` and sensor frames (e.g., `front_camera_optical`, `lidar`).

Sensor publisher nodes (`camera_publisher`, `lidar_publisher`) look up these transforms at activation time to determine where to spawn sensors in CARLA. Frame names in the URDF must match the `frame_id` parameters configured for each sensor.

## Usage

```bash
ros2 launch carla_sample_description robot_description.launch.yaml
```

## Files

- `urdf/sample_vehicle.urdf.xacro` - Main vehicle description with sensor mounts
- `urdf/sensors.xacro` - Reusable sensor frame macros

## Customization

Modify the URDF to match your vehicle's sensor configuration. Add new sensor frames using the macros in `sensors.xacro`, ensuring frame names match those used in sensor publisher parameters.
