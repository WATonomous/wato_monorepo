# carla_msgs

Custom ROS 2 message and service definitions for CARLA bridge.

## Overview

Defines messages and services used for scenario management and CARLA-specific data types.

## Messages

### MapsStatus
Status of the currently running map.

### ScenarioStatus
Status of the currently running scenario.

## Services

### GetAvailableMaps
List all available maps.

### SwitchScenario
Switch to a different scenario by module path, loading its declared map before
spawning entities. Returns `success`, `message`, `previous_scenario`,
`previous_map`, and `new_map` (the resulting CARLA map name when known). Select the desired map by
calling `SwitchScenario` with a scenario that declares it.

### GetAvailableScenarios
List all available scenarios with descriptions.

## Usage

```python
from carla_msgs.msg import MapsStatus, ScenarioStatus
from carla_msgs.srv import (
    SwitchScenario,
    GetAvailableScenarios,
    GetAvailableMaps
)
```
