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

PR #487's `carla_scenarios.scenarios.oval_track_scenario` and its short form remain
manual selectors backed by the installed original plugin. Use the manual
launch argument or `switch_scenario` service; managed `prepare_experiment`
rejects this plugin before changing a world. They read
`/opt/watonomous/maps/osm/oval_track_4_lane.xodr` and `oval_track_spawn.json`
from that directory, preserving lane snapping and readiness polling. Pair with
world modeling's `map:=generated_oval`, optionally `map_only:=true`, and the
`oval_track` maneuver. Main's `oval` playground and world-model preset still use
the standard converted-map paths. No artifacts are moved or regenerated.

The original `/carla/scenario_server/reset_ego` service remains available for
manual sessions, serialized with world ticks. Managed sessions use `/sim/reset`
so actor, application and evaluator state reset together.
