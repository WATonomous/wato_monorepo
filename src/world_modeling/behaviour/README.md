# Behaviour

ROS2 Node that executes a behavior tree for high-level autonomous driving decisions using [BehaviorTree.CPP](https://www.behaviortree.dev/).

## Overview

The behaviour node orchestrates driving decisions by ticking a behavior tree at a configurable rate. It handles:

- **Lane Navigation** - Lane following and safe lane changes
- **Intersection Handling** - Stop signs, traffic lights, and yield signs
- **Goal Tracking** - Route following and goal reached detection

The tree reads world state from subscribed topics and publishes decisions (e.g., virtual walls for stopping).

## Parameters

| Parameter                                           | Type     | Default                        | Description                                       |
| --------------------------------------------------- | -------- | ------------------------------ | ------------------------------------------------- |
| `bt_tree_file`                                      | string   | `main_tree.xml`                | Behavior tree XML file to load                    |
| `rate_hz`                                           | double   | 10.0                           | Tree tick frequency                               |
| `ego_pose_rate_hz`                                  | double   | 20.0                           | Ego pose TF lookup rate                           |
| `map_frame`                                         | string   | `map`                          | Map frame ID                                      |
| `base_frame`                                        | string   | `base_link`                    | Robot base frame ID                               |
| `traffic_light_state_hypothesis_index`              | int      | 1                              | Hypothesis index for traffic light state          |
| `world_objects_hypothesis_index`                    | int      | 0                              | Hypothesis index for object classification        |
| `left_lane_change_areas`                            | string[] | `[left_lane_change_corridor]`  | Area names for left lane change safety            |
| `right_lane_change_areas`                           | string[] | `[right_lane_change_corridor]` | Area names for right lane change safety           |
| `stop_line_wall_of_doom_width`                      | double   | 5.0                            | Virtual wall width at stop lines (m)              |
| `stop_line_wall_of_doom_length`                     | double   | 1.0                            | Virtual wall length at stop lines (m)             |
| `ego_stopped_velocity_threshold`                    | double   | 0.1                            | Velocity threshold for "stopped" (m/s)            |
| `intersection_lookahead_m`                          | double   | 40.0                           | Intersection detection lookahead (m)              |
| `traffic_control_element_passed_lanelet_threshold`  | int      | 2                              | Lanelets past element to consider passed          |
| `traffic_control_element_handled_lanelet_threshold` | int      | 1                              | Lanelets past element to consider handled         |
| `stop_sign_car_detection_threshold_m`               | double   | 8.0                            | Distance to detect cars at stop sign (m)          |
| `goal_reached_threshold_m`                          | double   | 1.0                            | Distance to goal to consider reached (m)          |
| `get_shortest_route_timeout_ms`                     | int      | 6000                           | Service timeout for get_shortest_route (ms)       |
| `set_route_timeout_ms`                              | int      | 6000                           | Service timeout for set_route (ms)                |
| `get_lanelets_by_reg_elem_timeout_ms`               | int      | 5000                           | Service timeout for get_lanelets_by_reg_elem (ms) |
| `wall_service_timeout_ms`                           | int      | 5000                           | Service timeout for spawn/despawn wall (ms)       |
| `enable_console_logging`                            | bool     | false                          | Enable BT console logging                         |

## Topics

### Subscribed

| Topic            | Type                                  | Description                       |
| ---------------- | ------------------------------------- | --------------------------------- |
| `goal_point`     | `geometry_msgs/PointStamped`          | Goal position                     |
| `lane_context`   | `lanelet_msgs/CurrentLaneContext`     | Current lane information          |
| `world_objects`  | `world_model_msgs/DynamicObjectArray` | Tracked dynamic objects           |
| `area_occupancy` | `world_model_msgs/AreaOccupancyArray` | Occupancy status of defined areas |

### Published

| Topic               | Type                             | Description                 |
| ------------------- | -------------------------------- | --------------------------- |
| `behaviour/markers` | `visualization_msgs/MarkerArray` | Debug visualization markers |

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

## License

Apache 2.0 - Copyright (c) 2025-present WATonomous
