# carla_lifecycle

Lifecycle management for CARLA ROS 2 bridge nodes.

All CARLA bridge nodes are ROS 2 lifecycle nodes, managed centrally by the `lifecycle_manager`. When the `scenario_server` loads or switches scenarios, the lifecycle manager automatically brings up all managed nodes (configure -> activate). Before a scenario switch, it cleanly shuts them down (deactivate -> cleanup) to ensure sensors are destroyed and connections are released before the new scenario spawns actors.

This coordination ensures nodes don't try to attach sensors to vehicles that no longer exist, and that the system can recover cleanly from scenario changes without requiring a full restart.

## Nodes

### lifecycle_manager

```bash
ros2 run carla_lifecycle lifecycle_manager
```

**Subscriptions:** `/<scenario_server>/scenario_status` (`carla_msgs/ScenarioStatus`)

**Services:** `prepare_for_scenario_switch` (`std_srvs/Trigger`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `autostart` | bool | `true` | Automatically start managed nodes on startup |
| `scenario_server_name` | string | `scenario_server` | Name of the scenario server node to coordinate with |
| `node_names` | string[] | `[]` | List of lifecycle node names to manage |
| `service_timeout` | double | `10.0` | Timeout for lifecycle service calls in seconds |
| `startup_retry_interval` | double | `5.0` | Retry interval when waiting for nodes in seconds |
