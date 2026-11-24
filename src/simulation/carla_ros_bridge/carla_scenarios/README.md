# CARLA playgrounds and scenarios

This package provides two YAML catalogs backed by shared Python worlds:

- Playgrounds are manual environments: map, ego, traffic, weather and bridge
  start immediately. Users launch their own application nodes; no evaluator or
  scored attempt is created.
- Scenarios are automated tests: a playground plus setup overrides, a reviewed
  mode, trajectory source, actors, goal, completion and selected metric limits.

```text
config/playgrounds/             Public manual world definitions
config/scenarios/               Automated test definitions
carla_scenarios/worlds/          Shared CARLA world/actor operations
carla_scenarios/definitions.py   Loading and validation of named definitions
```

## Guides

- [Authoring](AUTHORING.md): copyable definitions, options/defaults, spawn
  selection, stack limitations, events, scoring, validation and extension points.
- [Manager commands](../../simulation_manager/README.md#commands): session use
  and the distinction between manual worlds and scored execution.
- [Shutdown](../../simulation_manager/README.md#shutdown),
  [recovery](../../simulation_manager/README.md#session-recovery) and
  [results](../../simulation_manager/README.md#results-and-diagnostics).

## Catalog IDs

Use the YAML filename stem, such as `town10hd_default`, `town10hd_light`,
`town10hd_heavy`, `empty`, `custom`, `oval`, or `ring_road`. Python module paths
are not accepted. Adding a definition needs no alias or registry entry.

Managed sessions mount the checkout's data directories. Add or edit YAML and JSON,
then reopen the scenario; no image rebuild is needed for data changes. Scenario
files use version 2; playground files use version 1. See the authoring guide for
coordinate frames, stack choices, and the parked-car goal flow.
