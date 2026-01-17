# Developing carla_lifecycle

## Architecture

The lifecycle manager is implemented in C++ for performance and direct access to lifecycle service clients.

## Key Components

### NodeClients

Holds service clients for each managed node:

```cpp
struct NodeClients {
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state;
};
```

### TransitionStep

Represents a lifecycle transition to execute:

```cpp
struct TransitionStep {
    uint8_t transition_id;     // e.g., TRANSITION_CONFIGURE
    uint8_t required_state;    // State node must be in
    const char* name;          // For logging
};
```

## Startup Flow

1. `startupTimerCallback` - Retries bringing up scenario_server
2. `bringUpNode` - Configures then activates a single node
3. `scenarioStatusCallback` - Triggers node bringup/restart on scenario changes

## Parallel Transitions

Nodes are transitioned in parallel using async service calls:

```cpp
for (const auto& node_name : node_names_) {
    futures.emplace_back(node_name, client->async_send_request(req));
}
for (auto& [node_name, future] : futures) {
    future.wait_for(timeout);
}
```

## Node State Handling

- Nodes not ready (`service_is_ready() == false`) are skipped
- Nodes in wrong state for transition are skipped
- Failed transitions are logged but don't block other nodes

## Adding New Managed Nodes

Add the node name to `node_names` parameter in launch configuration. The manager will automatically create service clients and manage its lifecycle.
