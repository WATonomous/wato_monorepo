# Developing carla_sample_description

## Architecture

Standard ROS 2 robot description package with URDF and launch file for `robot_state_publisher`.

## Package Structure

```
carla_sample_description/
├── launch/
│   └── robot_description.launch.yaml
├── urdf/
│   └── sample_vehicle.urdf
└── package.xml
```

## URDF Structure

Define the TF tree from `base_link` to sensor frames:

```xml
<robot name="carla_vehicle">
  <link name="base_link"/>

  <link name="front_camera_optical"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="front_camera_optical"/>
    <origin xyz="2.0 0 1.5" rpy="0 0 0"/>
  </joint>
</robot>
```

## Launch Integration

The launch file reads URDF and starts robot_state_publisher:
```yaml
- node:
    pkg: robot_state_publisher
    exec: robot_state_publisher
    param:
      - name: robot_description
        value: $(command 'cat $(var urdf_path)')
```

## Adding New Sensors

1. Add new link for sensor frame
2. Add fixed joint from `base_link` (or parent frame) to sensor frame
3. Set transform matching CARLA sensor attachment point
