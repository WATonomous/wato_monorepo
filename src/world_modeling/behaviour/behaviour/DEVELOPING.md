# Developing the behaviour tree

## Setup

Attach `world_modeling:dev` in `ACTIVE_MODULES` under the watod config file:

```sh
export ACTIVE_MODULES="world_modeling:dev"
```

Build:

```sh
watod build
```

Run:

```sh
watod up
```

Enter the container:

```sh
watod -t world_modeling_bringup_dev
```

## Developing

After changes are made, check if it compiles:

```sh
colcon build --packages-up-to behaviour
```

Launch it:

```sh
source install/setup.bash
```

```sh
ros2 launch world_modeling_bringup world_modeling.launch.yaml
```

**NOTE**: The behaviourtree.cpp library does not raise every error during compile time. In fact, almost everytime you will encounter a BT:RuntimeError. Some example include:

- if a node port has a type mismatch
- if a node was not registered but used in the xml file
- if a node's port name is different than the name in the xml file

### Parameter convention

Use this split in `behaviour/config/params.yaml`:

- top-level params: node/runtime/shared values (e.g. `map_frame`, `base_frame`, `rate_hz`, timeouts)
- `bt.*` params: only values consumed by BT XML ports

In XML, reference those BT-only values with `{@bt.*}`.

### Adding a new node

1. Figure out the node type and location:
   - `ConditionNode` for boolean checks
   - `SyncActionNode` for fast compute/set logic
   - ROS wrappers (`RosServiceNode` / `RosTopicPubNode`) for ROS I/O
   - Place under `common/`, `lane_navigation/`, or `intersection/`.

2. Define `providedPorts()` first, then implement logic:
   - read inputs with `ports::tryGet(...)` / `ports::tryGetPtr(...)`
   - validate required inputs with `ports::require(...)`
   - for non-ROS nodes: log and return `FAILURE` on missing input
   - for ROS wrapper nodes: log with `RCLCPP_*` and set `error_message` when applicable.

3. Register the node in the correct registrar:
   - `include/behaviour/nodes/<domain>/registrar.hpp`.

4. Wire the node into XML:
   - add it to the right tree/subtree
   - ensure XML port names exactly match C++ port names
   - use `{@bt.*}` for BT-only config values.

5. Test:
   - build: `colcon build --packages-up-to behaviour`
   - launch: `ros2 launch world_modeling_bringup world_modeling.launch.yaml`
   - verify no BT runtime errors (port/type/registration mismatches are common).

Refer to docs https://www.behaviortree.dev/docs/intro.

## Visualizing

The behaviour node depends on two nodes:

- Costmap
- World Model

Attach `simulation` in the `ACTIVE_MODULES` under the watod config:

```sh
export ACTIVE_MODULES="world_modeling:dev simulation"
```

Build and enter the `world_modeling_bringup_dev` container.

Since the `world_modeling_bringup` already configures both the costmap and the world model, running the world modeling bringup will achieve everything the behaviour tree needs (including carla).

**NOTE**: the behaviour tree is also configured under the `world_modeling_bringup` but the behaviour tree node will log a lot of messages. To avoid flooding the `world_modeling_bringup` console, it is suggested to run the behaviour node in another terminal.

Build:

```sh
colcon build --packages-up-to world_modeling_bringup
```

Launch:

```sh
source install/setup.bash
```

```sh
ros2 launch world_modeling_bringup world_modeling.launch.yaml
```

To see the behaviour tree ouputs, either check:

1. The terminal logs
2. Foxglove behaviour markers

On launch the behaviour tree will be expecting a goal point published to the topic `/goal_point`.

It could achieved by using a ros command:

```sh
ros2 topic pub --once /goal_point geometry_msgs/msg/Point "{x: 1.0, y: 2.0, z: 0.0}"
```

or using the publish panel on Foxglove.
