# carla_scenarios

Scenario management for CARLA simulation.

The `scenario_server` is the central coordinator for the CARLA simulation. It connects to the CARLA server, configures simulation timing settings, and loads scenarios that define the world state - which map to use, where to spawn the ego vehicle, and what traffic to generate.

When a scenario is switched via the `~/switch_scenario` service, the server first notifies the `lifecycle_manager` to clean up all managed nodes, then unloads the current scenario (destroying all actors), and finally loads the new scenario. This triggers the lifecycle manager to bring nodes back up with the new world state.

The server also publishes `/clock` from CARLA's simulation time.

## Nodes

### scenario_server

```bash
ros2 run carla_scenarios scenario_server
```

**Publications:** `~/scenario_status` (`carla_msgs/ScenarioStatus`), `/clock` (`rosgraph_msgs/Clock`)

**Services:** `~/switch_scenario` (`carla_msgs/SwitchScenario`), `~/get_available_scenarios` (`carla_msgs/GetAvailableScenarios`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `initial_scenario` | string | `carla_scenarios.scenarios.default_scenario` | Scenario module path to load on startup |
| `carla_fps` | double | `60.0` | Simulation FPS (sets `fixed_delta_seconds`) |
| `synchronous_mode` | bool | `false` | Server waits for client tick (requires `world.tick()` calls) |
| `no_rendering_mode` | bool | `false` | Disable rendering for faster simulation |
| `substepping` | bool | `true` | Enable physics substepping |
| `max_substep_delta_time` | double | `0.01` | Max physics substep time in seconds |
| `max_substeps` | int | `10` | Max physics substeps per frame |
| `lifecycle_manager_name` | string | `carla_lifecycle_manager` | Name of the lifecycle manager node |

See [CARLA synchrony and time-step docs](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/) for details on timing parameters.

## Built-in Scenarios

- `carla_scenarios.scenarios.empty_scenario` - Ego vehicle only, no NPCs
- `carla_scenarios.scenarios.default_scenario` - Ego + 20 vehicles + 40 pedestrians
- `carla_scenarios.scenarios.light_traffic_scenario` - Ego + 5 vehicles + 10 pedestrians
- `carla_scenarios.scenarios.heavy_traffic_scenario` - Ego + 50 vehicles + 100 pedestrians
