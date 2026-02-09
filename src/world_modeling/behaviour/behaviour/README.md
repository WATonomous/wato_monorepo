# Behaviour

Behavior tree for Eve's decision making using [BehaviorTree.CPP](https://www.behaviortree.dev/) and ros wrappers from [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2).

## Overview

The three main behaviours the behaviour tree currently handles are:

- **Reaching goal point** (ensure global route, ensure car at goal)
- **Lane navigation** (following route, safe lane changes)
- **Intersection** (traffic light, stop sign, yield)

## Parameters

There is more comments and info under `/config/param.yaml`

| Parameter                                              | Type     | Default                        | Description                                       |
| ------------------------------------------------------ | -------- | ------------------------------ | ------------------------------------------------- |
| `bt_tree_file`                                         | string   | `main_tree.xml`                | Behavior tree XML file to load                    |
| `rate_hz`                                              | double   | 10.0                           | Tree tick frequency                               |
| `ego_state_rate_hz`                                    | double   | 20.0                           | Ego pose TF lookup rate                           |
| `map_frame`                                            | string   | `map`                          | Map frame ID                                      |
| `base_frame`                                           | string   | `base_link`                    | Robot base frame ID                               |
| `bt.traffic_light_state_hypothesis_index`              | int      | 1                              | Hypothesis index for traffic light state          |
| `world_objects_hypothesis_index`                       | int      | 0                              | Hypothesis index for object classification        |
| `bt.left_lane_change_areas`                            | string[] | `[left_lane_change_corridor]`  | Area names for left lane change safety            |
| `bt.right_lane_change_areas`                           | string[] | `[right_lane_change_corridor]` | Area names for right lane change safety           |
| `bt.intersection_wall_of_doom_width`                   | double   | 5.0                            | Virtual wall width at stop lines (m)              |
| `bt.intersection_wall_of_doom_length`                  | double   | 1.0                            | Virtual wall length at stop lines (m)             |
| `bt.ego_stopped_velocity_threshold`                    | double   | 0.1                            | Velocity threshold for "stopped" (m/s)            |
| `bt.intersection_lookahead_m`                          | double   | 40.0                           | Intersection detection lookahead (m)              |
| `bt.traffic_control_element_passed_lanelet_threshold`  | int      | 2                              | Lanelets past element to consider passed          |
| `bt.traffic_control_element_handled_lanelet_threshold` | int      | 1                              | Lanelets past element to consider handled         |
| `bt.stop_sign_car_detection_threshold_m`               | double   | 8.0                            | Distance to detect cars at stop sign (m)          |
| `bt.goal_reached_threshold_m`                          | double   | 1.0                            | Distance to goal to consider reached (m)          |
| `get_shortest_route_timeout_ms`                        | int      | 6000                           | Service timeout for get_shortest_route (ms)       |
| `set_route_timeout_ms`                                 | int      | 6000                           | Service timeout for set_route (ms)                |
| `get_lanelets_by_reg_elem_timeout_ms`                  | int      | 5000                           | Service timeout for get_lanelets_by_reg_elem (ms) |
| `wall_service_timeout_ms`                              | int      | 5000                           | Service timeout for spawn/despawn wall (ms)       |
| `enable_console_logging`                               | bool     | false                          | Enable BT console logging                         |

## Topics

### Subscribed

| Topic            | Type                                  | Description                       |
| ---------------- | ------------------------------------- | --------------------------------- |
| `goal_point`     | `geometry_msgs/PointStamped`          | Goal position                     |
| `lane_context`   | `lanelet_msgs/CurrentLaneContext`     | Current lane information          |
| `world_objects`  | `world_model_msgs/DynamicObjectArray` | Tracked dynamic objects           |
| `area_occupancy` | `world_model_msgs/AreaOccupancyArray` | Occupancy status of defined areas |

### Published

| Topic               | Type                                  | Description                                                                                                               |
| ------------------- | ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour.msg` | Sends lattice planner the message to execute the accurate behaviour and the preferred lanelets that the car should be on. |
|                     |

## Services Called

| Service                    | Type                                | Description                         |
| -------------------------- | ----------------------------------- | ----------------------------------- |
| `get_shortest_route`       | `lanelet_msgs/GetShortestRoute`     | Get route from ego to goal          |
| `set_route`                | `lanelet_msgs/SetRoute`             | Set the active route                |
| `get_lanelets_by_reg_elem` | `lanelet_msgs/GetLaneletsByRegElem` | Find lanelets by regulatory element |
| `spawn_wall`               | `costmap_msgs/SpawnWall`            | Create virtual wall at stop lines   |
| `despawn_wall`             | `costmap_msgs/DespawnWall`          | Remove virtual wall                 |

## Tree Structure

The main tree (`main_tree.xml`) includes modular subtrees:

| Subtree               | Description                              |
| --------------------- | ---------------------------------------- |
| `lane_navigation.xml` | Lane following and lane change logic     |
| `intersection.xml`    | Intersection detection and routing       |
| `stop_sign.xml`       | Stop sign behavior (stop, wait, proceed) |
| `traffic_light.xml`   | Traffic light state handling             |
| `yield.xml`           | Yield sign behavior                      |

## Diagrams

### Root Tree

![Root behaviour tree](docs/root.svg)

Source: [`docs/root.excalidraw`](docs/root.excalidraw)

### Intersections

![Intersection behaviour tree](docs/intersections.svg)

Source: [`docs/intersections.excalidraw`](docs/intersections.excalidraw)

### Lane Navigation

![Lane navigation behaviour tree](docs/lane_navigation.svg)

Source: [`docs/lane_navigation.excalidraw`](docs/lane_navigation.excalidraw)
