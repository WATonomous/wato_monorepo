# carla_sample_description

Sample robot description (URDF) for CARLA vehicles.

## Overview

Provides a sample URDF defining the TF tree for sensors attached to the ego vehicle. This is used by `robot_state_publisher` to broadcast static transforms.

## Usage

Included automatically via:
```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

## Customization

Replace or modify the URDF in `urdf/` to match your vehicle's sensor configuration. Ensure frame names match those used by sensor publishers.
