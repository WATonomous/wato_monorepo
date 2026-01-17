# carla_scenarios

Scenario management for CARLA simulation.

## Overview

Provides a scenario server that loads and manages CARLA scenarios. Scenarios define world setup including map, ego vehicle spawn, NPC traffic, and weather.

## Nodes

### scenario_server

Manages scenario lifecycle and provides services for switching scenarios.

**Publications:**
- `~/scenario_status` (carla_msgs/ScenarioStatus)

**Services:**
- `~/switch_scenario` (carla_msgs/SwitchScenario)
- `~/get_available_scenarios` (carla_msgs/GetAvailableScenarios)

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Built-in Scenarios

- `carla_scenarios.scenarios.default_scenario` - Spawns ego vehicle with NPC traffic

## Usage

Launched automatically via:

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

### Switch Scenario

```bash
ros2 service call /scenario_server/switch_scenario carla_msgs/srv/SwitchScenario \
  "{scenario_name: 'carla_scenarios.scenarios.default_scenario'}"
```
