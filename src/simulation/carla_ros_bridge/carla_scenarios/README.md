# CARLA playgrounds and scenarios

The scenario server installs YAML playground and scenario catalogs. Playgrounds
provide manual worlds with no application stack or scoring. Scenarios add reviewed
stack inputs and scoring definitions; execution is supplied by the manager in the
next stack layer. Shared Python worlds own their actors and the server owns all
ticks and `/clock` publication.

The seven historical module-style selectors and short forms resolve to YAML
playgrounds with their original traffic, spawn and weather settings. The default
manual launch still loads light traffic. See the
[manual simulation guide](../../SIMULATION_README.md).

PR #487's `carla_scenarios.scenarios.oval_track_scenario` and its short form remain
manual selectors backed by the installed original plugin. They read
`/opt/watonomous/maps/osm/oval_track_4_lane.xodr` and `oval_track_spawn.json`
from that directory, preserving lane snapping and readiness polling. Pair with
world modeling's `map:=generated_oval`, optionally `map_only:=true`, and the
`oval_track` maneuver. Main's `oval` playground and world-model preset still use
the standard converted-map paths. No artifacts are moved or regenerated.

The original `/carla/scenario_server/reset_ego` service remains available for
manual sessions, serialized with world ticks. Managed sessions use `/sim/reset`
so actor, application and evaluator state reset together.
