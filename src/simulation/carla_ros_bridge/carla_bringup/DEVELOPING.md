# Developing carla_bringup

## Design Rationale

Centralized launch and configuration makes it easy to:
1. Start the entire bridge stack with one command
2. Configure all nodes from one YAML file
3. Ensure correct startup ordering

## Launch Order

The launch file starts nodes in dependency order:
1. scenario_server and lifecycle_manager (no dependencies)
2. robot_state_publisher (needs URDF, no CARLA dependency)
3. All other nodes as managed lifecycle nodes

The lifecycle_manager handles the actual activation sequencing based on scenario_server status.

## Configuration

All parameters are in `config/carla_bridge.yaml` using ROS 2 parameter file format. Each node has its own section with `ros__parameters`.

To add a new node's configuration, add a new top-level key matching the node name.

## Adding New Nodes to the Launch

1. Add node configuration to `carla_bridge.yaml`
2. Add node to launch file
3. If it's a lifecycle node that depends on CARLA, add its name to `lifecycle_manager.node_names`

Non-lifecycle nodes or nodes that don't depend on CARLA (like robot_state_publisher) don't need to be managed.
