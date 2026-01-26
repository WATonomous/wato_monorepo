# lanelet_msgs

ROS2 message and service definitions for Lanelet2 map queries.

**Note:** The `lanelet_msgs::msg::Lanelet` message is a ROS-serializable representation of lanelet data, distinct from `lanelet::ConstLanelet` in the Lanelet2 C++ library. The world_model node converts between these types internally using `LaneletHandler::toLaneletMsg()`.

## Messages

| Message | Description |
|---------|-------------|
| `Lanelet` | Complete per-lanelet data (geometry, semantics, connectivity, regulatory elements) |
| `RouteAhead` | Lanelets ahead of ego on the active route (enables BFS via successor_ids) |
| `CurrentLaneContext` | Real-time ego lane context with distances to events |
| `MapVisualization` | Subset of lanelets for RViz visualization |
| `TrafficLightInfo` | Traffic light position and associated stop line |
| `StopLineInfo` | Stop line geometry |

## Services

| Service | Description |
|---------|-------------|
| `SetRoute` | Set destination and resolve lanelet IDs for routing |
| `GetShortestRoute` | Get entire shortest route from ego to goal |
| `GetLaneletsByRegElem` | Find lanelets associated with a regulatory element |

## License

Apache 2.0 - Copyright (c) 2025-present WATonomous
