# WATO lifecycle manager

The C++ lifecycle manager owns configure, activate, pause and shutdown operations
for a declared group of lifecycle nodes. Its existing interface is unchanged.

## Simulation application runtime

`ros2 run wato_lifecycle_manager application_runtime` runs a separate process
supervisor for a managed simulation container. Set `WATOD_SIM_RUNTIME_ROLE` to
`action` or `world` and `WATOD_SIM_SESSION_ID` to the owning session ID. The host
CLI supplies these values.

The runtime accepts the reviewed presets in
[`sim_presets.py`](wato_lifecycle_manager/sim_presets.py). It starts existing launch
files with autostart disabled. The simulation stack adapter then calls the
lifecycle manager's group startup service. Runtime stop terminates only its own
launch process group, with bounded signal escalation.

See the [simulation manager](../../simulation/simulation_manager/README.md#application-integration)
for ownership, reset ordering and ROS interfaces.
