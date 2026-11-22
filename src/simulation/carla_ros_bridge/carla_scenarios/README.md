# CARLA playgrounds and scenarios

Playgrounds are manual development worlds. Opening one loads its map, ego,
traffic, weather and bridge, and leaves physics running. Launch application
nodes yourself. No stack, scored attempt, evaluator or result recorder starts.
World pause/resume and close remain available.

Scenarios are repeatable tests. They select a playground, setup overrides,
application inputs, stack, ordered events, deadlines, completion criteria and
thresholds. The runner prepares the complete stack, controls execution and
writes evaluation results through the [simulation manager](../../simulation_manager/README.md).
Both shipped scenarios use adaptive pure pursuit.

```text
config/playgrounds/     Public manual worlds, including traffic variants
config/scenarios/       controller_snake and parked_car_straight
carla_scenarios/worlds/  Shared CARLA world setup
carla_scenarios/definitions.py  YAML loading and legacy selector aliases
```

## Commands

Run on the host with prepared module images:

```bash
watod sim list
watod sim setup --verbose
watod sim open playground town10hd_default
# Launch your own ROS application nodes; no sim play is required.
watod sim pause
watod sim resume
watod sim close
watod sim open scenario controller_snake
watod sim play
watod sim reset
watod sim close
watod sim test parked_car_straight
watod sim stop --verbose
```

Results are created only for scenarios under `.cache/watod-sim/results/`.
`test` saves its outcome and closes the experiment while keeping the session.
`stop` removes recorded session containers and preserves maps, caches and results.
Image builds and runtime acceptance are operator tasks.

## Add a definition

See the [authoring guide](AUTHORING.md) for copyable playground/scenario
templates, supported fields and defaults, spawn selection, events, deadlines,
scoring, installation and extension points.

Add a version-1 YAML file with an `id` matching its filename. A playground
selects `map.town` or `map.xodr`, an ego blueprint, spawn settings, weather, seed
and traffic. Town traffic variants share one implementation; oval and ring road
share OpenDRIVE loading. Custom traffic props are configured under `traffic.props`.

A scenario references `playground`, puts test-only spawn/actor settings under
`setup`, selects a reviewed `stack` and its `inputs`, and declares `events`,
`deadlines`, `completion`, `thresholds` and `metrics`. Keep fixtures out of the
public playground catalog. Add reusable CARLA operations under `worlds/` only
when the existing primitives cannot express the world.

`setup.py` installs both catalogs under the package share directory and includes
`worlds` in the Python package. Rebuild the affected module images after edits.
Only `scenario_server` ticks CARLA and publishes `/clock`; worlds request setup
ticks through it. Cleanup releases owned actors, never a global actor sweep.

## Compatibility

The `scenario_server`, `simulation_runner`, existing ROS service names and
`initial_scenario` launch parameter remain. The seven historical selectors
`carla_scenarios.scenarios.{default,light_traffic,heavy_traffic,empty,custom,oval,ring_road}_scenario`
and their short forms resolve explicitly to the YAML playgrounds. Their traffic,
weather and spawn settings are preserved; the old Python catalog is removed.

The server accepts both `playground/NAME` and `scenario/NAME` through
`prepare_experiment`; a bare legacy selector remains a manual world request.
