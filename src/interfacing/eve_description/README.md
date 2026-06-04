# eve_description

URDF/Xacro robot model for Eve (Kia Soul EV). Publishes the vehicle TF tree via `robot_state_publisher` so all other nodes can look up sensor positions and frame relationships at runtime.

## Overview

All sensor extrinsics and wheel geometry are defined here in one place. Downstream packages (localization, perception, control) consume these frames via TF rather than hardcoding transforms.

## TF Frame Hierarchy

```
base_link
└── roof_mount
│   ├── camera_pano_nn
│   ├── camera_pano_ne
│   ├── camera_pano_ee
│   ├── camera_pano_se
│   ├── camera_pano_ss
│   ├── camera_pano_sw
│   ├── camera_pano_ww
│   └── camera_pano_nw
│   ├── camera_lower_ne
│   ├── camera_lower_se
│   ├── camera_lower_sw
│   └── camera_lower_nw
├── rear_axle
└── front_axle
```

**Panorama cameras** (`camera_pano_*`): 8 roof-mounted cameras arranged in a circle around `roof_mount`, each oriented to face outward. Positions are calibrated.

**Lower cameras** (`camera_lower_*`): 4 side-mounted cameras. These transforms are **nominal placeholders** — update them once you have measured or calibrated values (see DEVELOPING.md).

**Axle frames** (`rear_axle`, `front_axle`): Used by `can_state_estimator` to look up the wheelbase distance at runtime rather than hardcoding it.
