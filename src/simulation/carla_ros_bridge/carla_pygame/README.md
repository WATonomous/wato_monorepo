# carla_pygame

Web-based map visualization for CARLA simulation.

## Overview

Renders a bird's-eye view map of the CARLA world with vehicles, pedestrians, and traffic lights. Accessible via web browser.

## Nodes

### pygame_hud

Renders the map using pygame (headless) and streams frames to a web interface via WebSocket.

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Features

- Bird's-eye view of CARLA map with roads, lanes, and markings
- Real-time actor visualization (vehicles, pedestrians)
- Pan and zoom via mouse
- Color-coded actors (ego vehicle, other vehicles, pedestrians)
- HUD with simulation info

## Usage

Launched automatically via:

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

Access the visualization at `http://localhost:5000` (default port).

### Disable Visualization

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml pygame_hud_enabled:=false
```
