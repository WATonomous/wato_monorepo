# Developing carla_lifecycle

## Design Rationale

All CARLA bridge nodes are lifecycle nodes so they can be started/stopped cleanly during scenario switches. Without lifecycle management, sensors would fail when their parent vehicle is destroyed, requiring a full system restart.

The lifecycle_manager centralizes this coordination rather than having each node watch for scenario changes independently.

## Architecture

The manager subscribes to `scenario_status` from the scenario_server. When the scenario changes:

1. scenario_server calls `prepare_for_scenario_switch` service
2. Manager deactivates and cleans up all managed nodes
3. scenario_server unloads old scenario, loads new one
4. Manager sees new scenario in status, brings nodes back up

This ordering ensures nodes don't try to use CARLA resources that no longer exist.

## Node Discovery

Managed nodes are specified by name in the `node_names` parameter. The manager creates lifecycle service clients for each node upfront. Nodes that aren't running yet will be brought up when they become available.

## Startup Behavior

With `autostart:=true`, the manager first brings up the scenario_server (which connects to CARLA and loads the initial scenario), then waits for scenario_status to indicate "running" before bringing up other nodes.

If CARLA isn't available, the startup timer retries at `startup_retry_interval` until it succeeds.

## Adding New Managed Nodes

Just add the node name to `node_names` parameter. The node must:
- Be a lifecycle node
- Handle configure/activate/deactivate/cleanup transitions properly
- Not hold CARLA resources after cleanup (connections, sensors, actors)
