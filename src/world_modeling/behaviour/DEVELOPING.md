# Developing the Behaviour Tree

## Key Design Decisions

### Blackboard as Parameter Bridge

ROS parameters flow through the blackboard to BT nodes:

- `params.yaml` → `behaviour_node` declares/reads → `blackboard_->set()` → XML reads via `{@param}`

This decouples BT nodes from ROS, making them easier to unit test. Nodes only read from blackboard, never directly from ROS parameters.

### Node Registrars by Category

BT nodes are grouped into registrar classes by functional area:

- `CommonNodeRegistrar` - Shared/common nodes used throughout subtrees
- `IntersectionNodeRegistrar` - Nodes specifically for intersection subtree
- `LaneNavigationNodeRegistrar` - Nodes specifically for lane navgiation subtree

Each registrar is self-contained and can be enabled/disabled or reused in other projects.

### Header-Only BT Nodes

All BT nodes are header-only (no `.cpp` files). This:

- Reduces build complexity (no library linking)
- Enables template-based port definitions
- Keeps node logic visible in one place

The tradeoff is longer compile times when nodes change.

### Thread-Safe Stores for Subscriptions

Subscription callbacks write to thread-safe "store" objects (`DynamicObjectStore`, `AreaOccupancyStore`). The tree reads snapshots from these stores during tick. This prevents data races between ROS callbacks and tree execution.

### XML Subtrees for Modularity

Complex behaviors are split into subtrees (`intersection.xml`, `lane_navigation.xml`). The main tree includes them via `<SubTree>`. This enables:

- Independent testing of subtrees
- Reuse across different main trees
- Easier code review (smaller diffs)

## Package Structure

```
behaviour/
├── behaviour/                  # Main ROS package
│   ├── config/params.yaml      # All configurable parameters
│   ├── trees/                  # Behavior tree XMLs
│   │   ├── main_tree.xml       # Entry point
│   │   └── subtrees/           # Modular subtrees
│   ├── include/behaviour/
│   │   ├── behaviour_node.hpp  # Main node
│   │   ├── behaviour_tree.hpp  # Tree wrapper
│   │   └── nodes/              # BT node implementations
│   │       ├── common/         # Shared nodes + registrar
│   │       ├── intersection/   # Traffic control nodes
│   │       └── lane_navigation/# Lane nodes
│   └── src/                    # Implementation files
└── behaviortree_ros2_vendor/   # BT.ROS2 from source
```

## Common Patterns

### Node Types

| Type         | Base Class               | Use Case                              |
| ------------ | ------------------------ | ------------------------------------- |
| Condition    | `BT::ConditionNode`      | Check state, return SUCCESS/FAILURE   |
| Action       | `BT::SyncActionNode`     | Perform computation, return result    |
| Service      | `BT::RosServiceNode`     | Call ROS service, wait for response   |
| Async Action | `BT::StatefulActionNode` | Multi-tick actions with RUNNING state |

### Blackboard Naming

- `{variable}` - Local tree blackboard keys (set by other nodes)
- `{@parameter}` - Global tree blackboard keys (set by behaviour_node from params)
- `out_` prefix - Output ports that write to blackboard

### Adding a New Parameter

1. Add to `config/params.yaml`
2. Declare in `behaviour_node.cpp` constructor
3. Read and set on blackboard in `init()`
4. Use in XML with `{@param_name}`

### Adding a New BT Node

1. Create header in `include/behaviour/nodes/<category>/`
2. Inherit from appropriate base class
3. Implement `providedPorts()` and `tick()`
4. Add include to category's `registrar.hpp`
5. Register in registrar's `register_nodes()`

## Testing Changes

The behaviour node depends on two nodes:

- costmap
- world_model

However, the behaviour node spews out a lot of logs to see debug and check if the tree is going through the right logic.

Therefore, if you don't want the console where you are running world model and the costmap to filled with hundreds of lines, it is better to run the bheaviour node on a different terminal.

```bash
# Build
colcon build --packages-up-to world_modeling_bringup

source install/setup.bash

# Run full stack
ros2 launch world_modeling_bringup world_modeling.launch.yaml

# Run behaviour standalone on a different terminal
ros2 launch behaviour behaviour.launch.yaml
```

Use `test_tree.xml` for isolated testing by setting `bt_tree_file: "test_tree.xml"` in params.

## Common Issues

| Issue                    | Solution                                                                  |
| ------------------------ | ------------------------------------------------------------------------- |
| Node not found           | Check if the node is registered in its registrar                          |
| Port not connecting      | Verify port name matches XML exactly                                      |
| Blackboard value missing | Add to `updateBlackboard()` in behaviour_node.cpp                         |
| Service timeout          | Increase `*_timeout_ms` param, check service is running                   |
| Build fails              | Rebuild vendor: `colcon build --packages-select behaviortree_ros2_vendor` |
