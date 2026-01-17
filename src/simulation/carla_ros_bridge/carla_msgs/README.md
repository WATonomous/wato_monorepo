# carla_msgs

Custom ROS 2 message and service definitions for CARLA bridge.

## Overview

Defines messages and services used for scenario management and CARLA-specific data types.

## Messages

### ScenarioStatus
Status of the currently running scenario.

## Services

### SwitchScenario
Switch to a different scenario by module path.

### GetAvailableScenarios
List all available scenarios with descriptions.

## Usage

```python
from carla_msgs.msg import ScenarioStatus
from carla_msgs.srv import SwitchScenario, GetAvailableScenarios
```
