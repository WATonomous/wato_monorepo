# Developing carla_bringup

## Architecture

carla_bringup serves as the orchestration layer for the CARLA ROS 2 bridge. It uses YAML-format launch files for declarative node configuration.

## Launch File Structure

The launch file follows this pattern:

1. **Launch Arguments** - Declared at the top with defaults
2. **Includes** - Robot description and other launch files
3. **Nodes** - Each node loads parameters from the shared config file

### Adding a New Node

1. Add node configuration to `config/carla_bridge.yaml`:

   ```yaml
   my_node:
     ros__parameters:
       carla_host: "localhost"
       carla_port: 2000
   ```

2. Add node entry to the launch file:

```yaml
   - node:
       pkg: my_package
       exec: my_node
       name: my_node
       param:
         - from: $(find-pkg-share carla_bringup)/config/carla_bridge.yaml
   ```

1. If the node is a lifecycle node, add it to the lifecycle manager's `node_names` list.

## Configuration Patterns

### Conditional Nodes

Use `if` to conditionally launch nodes:

```yaml
- node:
    pkg: my_package
    exec: my_node
    if: $(var my_node_enabled)
```

### Parameter Overrides

Override config file parameters with launch arguments:

```yaml
param:
  - from: $(find-pkg-share carla_bringup)/config/carla_bridge.yaml
  - name: web_port
    value: $(var pygame_web_port)
```

## Lifecycle Management

All CARLA nodes are lifecycle nodes managed by `carla_lifecycle_manager`. The manager:

1. Waits for `scenario_server` to connect to CARLA
2. Brings up `scenario_server` first (configure + activate)
3. When scenario is loaded, brings up all other managed nodes
4. Restarts managed nodes when scenarios change
