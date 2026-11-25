# CARLA playgrounds and scenarios

This package provides two YAML catalogs backed by shared Python worlds:

- Playgrounds are manual environments: map, ego, traffic, weather and bridge
  start immediately. Users launch their own application nodes; no evaluator or
  scored attempt is created.
- Scenarios are automated tests: a playground plus setup overrides, a reviewed
  stack, application inputs, events, deadlines and scoring requirements.

```text
config/playgrounds/             Public manual world definitions
config/scenarios/               Automated test definitions
carla_scenarios/worlds/          Shared CARLA world/actor operations
carla_scenarios/definitions.py   Loading, validation and legacy selector aliases
```

## Guides

- [Authoring](AUTHORING.md): copyable definitions, options/defaults, spawn
  selection, stack limitations, events, scoring, validation and extension points.
- [Manager commands](../../simulation_manager/README.md#commands): session use
  and the distinction between manual worlds and scored execution.
- [Shutdown](../../simulation_manager/README.md#shutdown),
  [recovery](../../simulation_manager/README.md#session-recovery) and
  [results](../../simulation_manager/README.md#results-and-diagnostics).

## Compatibility

The `scenario_server`, `simulation_runner`, existing ROS service names and
`initial_scenario` launch parameter remain. The seven historical selectors
`carla_scenarios.scenarios.{default,light_traffic,heavy_traffic,empty,custom,oval,ring_road}_scenario`
and their short forms resolve explicitly to the YAML playgrounds. Their traffic,
weather and spawn settings are preserved; the old Python catalog is removed.

The server accepts both `playground/NAME` and `scenario/NAME` through
`prepare_experiment`; a bare legacy selector remains a manual world request.

The manual `/carla/scenario_server/reset_ego` service remains available and is
serialized with world ticks. Managed sessions use `/sim/reset` so world,
application and evaluator state reset together.
