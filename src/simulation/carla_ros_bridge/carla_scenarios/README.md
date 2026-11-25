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

The original `/carla/scenario_server/reset_ego` service remains available for
manual sessions, serialized with world ticks. Managed sessions use `/sim/reset`
so actor, application and evaluator state reset together.
