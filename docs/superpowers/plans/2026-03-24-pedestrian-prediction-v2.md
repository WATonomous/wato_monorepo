# Pedestrian Prediction V2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add crosswalk-proximity, road-crossing (jaywalking), and parking-lot-aware multi-hypothesis pedestrian prediction.

**Architecture:** New `GetNearbyLanelets` spatial query service (bypasses vehicle routing graph) feeds a per-pedestrian lanelet cache in the trajectory predictor. A context classifier dispatches to 4 context-specific hypothesis generators (road-adjacent, crosswalk, parking, open-area). Priority: road-adjacent jaywalking > crosswalk-following > parking/open fan.

**Tech Stack:** ROS2 Humble, C++17, lanelet2, rclcpp_lifecycle, lanelet_msgs, vision_msgs, world_model_msgs

**Spec:** `docs/superpowers/specs/2026-03-22-pedestrian-prediction-v2-design.md`

---

## File Map

### New Files
| File | Responsibility |
|------|---------------|
| `src/world_modeling/lanelet_msgs/srv/GetNearbyLanelets.srv` | Service definition: position + radius -> Lanelet[] |
| `src/world_modeling/world_model/include/world_model/interfaces/services/get_nearby_lanelets_service.hpp` | Service handler (wraps `getLaneletsInRadius` + `toLaneletMsg`) |

### Modified Files
| File | What Changes |
|------|-------------|
| `src/world_modeling/lanelet_msgs/CMakeLists.txt:26-41` | Add `GetNearbyLanelets.srv` to rosidl_generate_interfaces |
| `src/world_modeling/world_model/include/world_model/lanelet_handler.hpp:150` | Declare `getNearbyLanelets()` method |
| `src/world_modeling/world_model/src/lanelet_handler.cpp:280` | Implement `getNearbyLanelets()` (~10 lines) |
| `src/world_modeling/world_model/src/world_model_node.cpp:33-40,120` | Include + register new service |
| `src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp:231-235,357,510-531` | Expand PedestrianParams, add PedestrianContext enum, PedestrianLaneletEntry, new method declarations |
| `src/world_modeling/prediction/src/trajectory_predictor.cpp:1166-1193` | Replace generatePedestrianHypotheses + add all helper functions |
| `src/world_modeling/prediction/include/prediction/prediction_node.hpp:107,121-123` | Add nearby_lanelets_client_, pedestrian pending requests |
| `src/world_modeling/prediction/src/prediction_node.cpp:200-202,373` | Add pedestrian param loading + service client + query callback |
| `src/world_modeling/prediction/config/params.yaml:68-70` | Add new pedestrian parameter defaults |

---

## Task 1: GetNearbyLanelets Service Definition

**Files:**
- Create: `src/world_modeling/lanelet_msgs/srv/GetNearbyLanelets.srv`
- Modify: `src/world_modeling/lanelet_msgs/CMakeLists.txt:26-41`

- [ ] **Step 1: Create the service definition file**

```
# Request: spatial query — all lanelets within radius of a point.
# Unlike GetLaneletAhead, this does NOT use the routing graph BFS
# and returns ALL lanelet types (road, crosswalk, parking, etc.)
geometry_msgs/Point position
float64 radius_m
---
# Response
bool success
string error_message
lanelet_msgs/Lanelet[] lanelets
```

Write to `src/world_modeling/lanelet_msgs/srv/GetNearbyLanelets.srv`.

- [ ] **Step 2: Register in CMakeLists.txt**

In `src/world_modeling/lanelet_msgs/CMakeLists.txt`, add `"srv/GetNearbyLanelets.srv"` to the `rosidl_generate_interfaces` block, after line 40 (`"srv/GetLaneletAhead.srv"`).

- [ ] **Step 3: Commit**

```bash
git add src/world_modeling/lanelet_msgs/srv/GetNearbyLanelets.srv src/world_modeling/lanelet_msgs/CMakeLists.txt
git commit -m "feat(lanelet_msgs): add GetNearbyLanelets spatial query service"
```

---

## Task 2: World Model Service Provider

**Files:**
- Create: `src/world_modeling/world_model/include/world_model/interfaces/services/get_nearby_lanelets_service.hpp`
- Modify: `src/world_modeling/world_model/include/world_model/lanelet_handler.hpp:150`
- Modify: `src/world_modeling/world_model/src/lanelet_handler.cpp:280`
- Modify: `src/world_modeling/world_model/src/world_model_node.cpp:33-40,120`

- [ ] **Step 1: Add `getNearbyLanelets()` to LaneletHandler header**

In `src/world_modeling/world_model/include/world_model/lanelet_handler.hpp`, after the existing `getLaneletsInRadius` declaration (line 150), add:

```cpp
  /**
   * @brief Get all lanelets within radius as ROS messages (no routing graph).
   * Pure spatial query — returns ALL lanelet types (road, crosswalk, parking, etc.)
   */
  std::vector<lanelet_msgs::msg::Lanelet> getNearbyLanelets(
    const geometry_msgs::msg::Point & center, double radius) const;
```

- [ ] **Step 2: Implement `getNearbyLanelets()` in lanelet_handler.cpp**

In `src/world_modeling/world_model/src/lanelet_handler.cpp`, after `getLaneletsInRadius()` (after line 280), add:

```cpp
std::vector<lanelet_msgs::msg::Lanelet> LaneletHandler::getNearbyLanelets(
  const geometry_msgs::msg::Point & center, double radius) const
{
  std::vector<lanelet_msgs::msg::Lanelet> result;
  auto lanelets = getLaneletsInRadius(center, radius);
  result.reserve(lanelets.size());
  for (const auto & ll : lanelets) {
    result.push_back(toLaneletMsg(ll));
  }
  return result;
}
```

- [ ] **Step 3: Create the service handler**

Create `src/world_modeling/world_model/include/world_model/interfaces/services/get_nearby_lanelets_service.hpp`. Mirror the `GetLaneletAheadService` pattern exactly:

```cpp
// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...standard Apache 2.0 header...

#ifndef WORLD_MODEL__INTERFACES__SERVICES__GET_NEARBY_LANELETS_SERVICE_HPP_
#define WORLD_MODEL__INTERFACES__SERVICES__GET_NEARBY_LANELETS_SERVICE_HPP_

#include <string>

#include "lanelet_msgs/srv/get_nearby_lanelets.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

class GetNearbyLaneletsService : public InterfaceBase
{
public:
  GetNearbyLaneletsService(rclcpp_lifecycle::LifecycleNode * node, const LaneletHandler * lanelet_handler)
  : node_(node)
  , lanelet_(lanelet_handler)
  {
    srv_ = node_->create_service<lanelet_msgs::srv::GetNearbyLanelets>(
      "get_nearby_lanelets",
      std::bind(&GetNearbyLaneletsService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRequest(
    lanelet_msgs::srv::GetNearbyLanelets::Request::ConstSharedPtr request,
    lanelet_msgs::srv::GetNearbyLanelets::Response::SharedPtr response)
  {
    response->success = false;

    if (!lanelet_->isMapLoaded()) {
      response->error_message = "map_not_loaded";
      return;
    }

    response->lanelets = lanelet_->getNearbyLanelets(request->position, request->radius_m);
    response->success = true;
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  const LaneletHandler * lanelet_;

  rclcpp::Service<lanelet_msgs::srv::GetNearbyLanelets>::SharedPtr srv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__SERVICES__GET_NEARBY_LANELETS_SERVICE_HPP_
```

- [ ] **Step 4: Register service in world_model_node.cpp**

In `src/world_modeling/world_model/src/world_model_node.cpp`:

Add include after line 35 (`get_lanelet_ahead_service.hpp`):
```cpp
#include "world_model/interfaces/services/get_nearby_lanelets_service.hpp"
```

Add registration after line 120 (`GetLaneletAheadService`):
```cpp
  interfaces_.push_back(std::make_unique<GetNearbyLaneletsService>(this, lanelet_handler_.get()));
```

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/world_model/include/world_model/interfaces/services/get_nearby_lanelets_service.hpp \
        src/world_modeling/world_model/include/world_model/lanelet_handler.hpp \
        src/world_modeling/world_model/src/lanelet_handler.cpp \
        src/world_modeling/world_model/src/world_model_node.cpp
git commit -m "feat(world_model): implement GetNearbyLanelets spatial query service"
```

---

## Task 3: Expand PedestrianParams and Add Types

**Files:**
- Modify: `src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp:231-235`
- Modify: `src/world_modeling/prediction/config/params.yaml:68-70`
- Modify: `src/world_modeling/prediction/src/prediction_node.cpp:200-202`

- [ ] **Step 1: Add PedestrianContext enum and expand PedestrianParams**

In `src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp`:

Before `struct TrajectoryHypothesis` (before line 81), add:
```cpp
/**
 * @brief Spatial context for pedestrian prediction.
 */
enum class PedestrianContext
{
  CROSSWALK,       // Near a crosswalk lanelet
  PARKING,         // Near a parking lanelet
  ROAD_ADJACENT,   // Near a road/intersection lanelet (no crosswalk)
  OPEN_AREA        // No lanelets nearby
};
```

Replace the `PedestrianParams` struct (lines 231-235) with:
```cpp
  struct PedestrianParams
  {
    double default_speed;  // m/s - fallback when no velocity history
    double max_speed;  // m/s - clamp upper bound
    // Context classification
    double lanelet_proximity_threshold;  // meters - max distance to consider lanelet nearby
    double nearby_query_radius;  // meters - radius for GetNearbyLanelets service query
    double cache_invalidation_dist;  // meters - re-query after pedestrian moves this far
    // Crosswalk scoring
    double crosswalk_heading_tolerance;  // radians - reject crosswalk if heading too far off
    double heading_score_denominator;  // Gaussian denominator for heading scoring
    double lateral_score_denominator;  // Gaussian denominator for lateral scoring
    double crosswalk_prior;  // maneuver prior for crosswalk-following
    // Road-adjacent
    double forward_prior;  // maneuver prior for forward walking
    double road_crossing_prior;  // maneuver prior for jaywalking
    // Parking / open area fan
    double parking_fan_half_angle;  // radians - half-spread of directional fan
    int parking_fan_count;  // number of directional hypotheses
    double parking_forward_boost;  // boost for forward direction in fan
  };
```

- [ ] **Step 2: Add PedestrianLaneletEntry cache struct**

In `trajectory_predictor.hpp`, after the `VehicleLaneletEntry` struct (after line 528), add:

```cpp
  // Per-pedestrian nearby lanelet cache (from GetNearbyLanelets spatial query).
  // Separate from vehicle cache because it stores raw lanelet arrays, not
  // routing-graph-based LaneletAhead responses.
  struct PedestrianLaneletEntry
  {
    std::vector<lanelet_msgs::msg::Lanelet> lanelets;
    double last_x;
    double last_y;
    rclcpp::Time last_update;
  };

  mutable std::mutex pedestrian_cache_mutex_;
  std::unordered_map<std::string, PedestrianLaneletEntry> pedestrian_lanelet_cache_;
```

Also add a `PedestrianQueryFn` type alias and setter, near `LaneletQueryFn` (after line 307):

```cpp
  /**
   * @brief Callback type for requesting nearby lanelets around a pedestrian.
   * Fire-and-forget: response populates pedestrian_lanelet_cache_.
   */
  using PedestrianQueryFn =
    std::function<void(const std::string & ped_id, const geometry_msgs::msg::Point & position)>;

  void setPedestrianQueryFunction(PedestrianQueryFn fn);

  /**
   * @brief Update the per-pedestrian nearby lanelet cache with async query results.
   * Thread-safe: protected by pedestrian_cache_mutex_.
   */
  void updatePedestrianLaneletCache(
    const std::string & ped_id, std::vector<lanelet_msgs::msg::Lanelet> lanelets, double x, double y);
```

And add the member variable after `lanelet_query_fn_` (after line 515):
```cpp
  PedestrianQueryFn pedestrian_query_fn_;
```

- [ ] **Step 3: Add parameter defaults to params.yaml**

In `src/world_modeling/prediction/config/params.yaml`, replace lines 68-70 with:

```yaml
    # Pedestrian prediction defaults
    pedestrian_default_speed: 1.4  # m/s - average human walking speed
    pedestrian_max_speed: 3.0      # m/s - fast jog upper bound

    # Pedestrian context-aware prediction
    pedestrian_lanelet_proximity_threshold: 8.0  # meters - max distance to consider lanelet nearby
    pedestrian_nearby_query_radius: 15.0         # meters - radius for GetNearbyLanelets query
    pedestrian_cache_invalidation_dist: 3.0      # meters - re-query after moving this far
    pedestrian_crosswalk_heading_tolerance: 1.05  # radians (~60deg) - max heading diff for crosswalk
    pedestrian_heading_score_denominator: 1.5     # Gaussian denominator (wider than vehicle 0.2)
    pedestrian_lateral_score_denominator: 10.0    # Gaussian denominator (wider than vehicle 4.0)
    pedestrian_crosswalk_prior: 2.0               # maneuver prior for crosswalk-following
    pedestrian_forward_prior: 1.5                 # maneuver prior for forward walking
    pedestrian_road_crossing_prior: 0.15          # maneuver prior for jaywalking
    pedestrian_parking_fan_half_angle: 1.05       # radians (~60deg) - half-spread of fan
    pedestrian_parking_fan_count: 5               # number of directional hypotheses
    pedestrian_parking_forward_boost: 1.5         # boost for forward direction in fan
```

- [ ] **Step 4: Load new params in prediction_node.cpp**

In `src/world_modeling/prediction/src/prediction_node.cpp`, find the pedestrian params section (lines 200-202). The `declare_parameter` calls for the new params need to be added. First, find where parameters are declared (search for `declare_parameter.*pedestrian`). Then expand the pedestrian_params loading block:

After line 202, add:
```cpp
  pedestrian_params.lanelet_proximity_threshold =
    this->get_parameter("pedestrian_lanelet_proximity_threshold").as_double();
  pedestrian_params.nearby_query_radius =
    this->get_parameter("pedestrian_nearby_query_radius").as_double();
  pedestrian_params.cache_invalidation_dist =
    this->get_parameter("pedestrian_cache_invalidation_dist").as_double();
  pedestrian_params.crosswalk_heading_tolerance =
    this->get_parameter("pedestrian_crosswalk_heading_tolerance").as_double();
  pedestrian_params.heading_score_denominator =
    this->get_parameter("pedestrian_heading_score_denominator").as_double();
  pedestrian_params.lateral_score_denominator =
    this->get_parameter("pedestrian_lateral_score_denominator").as_double();
  pedestrian_params.crosswalk_prior =
    this->get_parameter("pedestrian_crosswalk_prior").as_double();
  pedestrian_params.forward_prior =
    this->get_parameter("pedestrian_forward_prior").as_double();
  pedestrian_params.road_crossing_prior =
    this->get_parameter("pedestrian_road_crossing_prior").as_double();
  pedestrian_params.parking_fan_half_angle =
    this->get_parameter("pedestrian_parking_fan_half_angle").as_double();
  pedestrian_params.parking_fan_count =
    this->get_parameter("pedestrian_parking_fan_count").as_int();
  pedestrian_params.parking_forward_boost =
    this->get_parameter("pedestrian_parking_forward_boost").as_double();
```

Also add the corresponding `declare_parameter` calls in the parameter declaration section (search for where `pedestrian_default_speed` and `pedestrian_max_speed` are declared, and add the new ones after).

Update the pedestrian RCLCPP_INFO log (around line 232) to include the new fields.

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp \
        src/world_modeling/prediction/config/params.yaml \
        src/world_modeling/prediction/src/prediction_node.cpp
git commit -m "feat(prediction): expand PedestrianParams with context-aware fields"
```

---

## Task 4: Pedestrian Lanelet Cache + Query Wiring

**Files:**
- Modify: `src/world_modeling/prediction/src/trajectory_predictor.cpp`
- Modify: `src/world_modeling/prediction/include/prediction/prediction_node.hpp:107,121-123`
- Modify: `src/world_modeling/prediction/src/prediction_node.cpp:373`

- [ ] **Step 1: Implement cache methods in trajectory_predictor.cpp**

Add after the `setLaneletQueryFunction` implementation (after line 113):

```cpp
void TrajectoryPredictor::setPedestrianQueryFunction(PedestrianQueryFn fn)
{
  pedestrian_query_fn_ = std::move(fn);
}

void TrajectoryPredictor::updatePedestrianLaneletCache(
  const std::string & ped_id, std::vector<lanelet_msgs::msg::Lanelet> lanelets, double x, double y)
{
  std::lock_guard<std::mutex> lock(pedestrian_cache_mutex_);
  auto & entry = pedestrian_lanelet_cache_[ped_id];
  entry.lanelets = std::move(lanelets);
  entry.last_x = x;
  entry.last_y = y;
  entry.last_update = node_->get_clock()->now();
}
```

Also add a private method `queryPedestrianNearbyLanelets`:

```cpp
std::optional<std::vector<lanelet_msgs::msg::Lanelet>> TrajectoryPredictor::queryPedestrianNearbyLanelets(
  const std::string & ped_id, const geometry_msgs::msg::Point & position)
{
  {
    std::lock_guard<std::mutex> lock(pedestrian_cache_mutex_);
    auto it = pedestrian_lanelet_cache_.find(ped_id);
    if (it != pedestrian_lanelet_cache_.end()) {
      double dx = position.x - it->second.last_x;
      double dy = position.y - it->second.last_y;
      if (dx * dx + dy * dy <
        pedestrian_params_.cache_invalidation_dist * pedestrian_params_.cache_invalidation_dist)
      {
        return it->second.lanelets;  // Cache hit
      }
    }
  }

  // Cache miss — fire async query
  if (pedestrian_query_fn_) {
    pedestrian_query_fn_(ped_id, position);
  }
  return std::nullopt;
}
```

Declare both in the header (private section, after `generatePedestrianHypotheses` declaration around line 357):
```cpp
  std::optional<std::vector<lanelet_msgs::msg::Lanelet>> queryPedestrianNearbyLanelets(
    const std::string & ped_id, const geometry_msgs::msg::Point & position);
```

- [ ] **Step 2: Add service client to prediction_node.hpp**

In `src/world_modeling/prediction/include/prediction/prediction_node.hpp`:

Add include (near the other lanelet_msgs includes):
```cpp
#include "lanelet_msgs/srv/get_nearby_lanelets.hpp"
```

After the existing `lanelet_ahead_client_` (line 107), add:
```cpp
  // Service client for per-pedestrian spatial lanelet queries
  rclcpp::Client<lanelet_msgs::srv::GetNearbyLanelets>::SharedPtr nearby_lanelets_client_;
```

After the existing `pending_vehicle_requests_` block (line 123), add:
```cpp
  // Pedestrian pending request tracking (separate from vehicle)
  std::mutex pending_pedestrian_requests_mutex_;
  std::unordered_set<std::string> pending_pedestrian_requests_;
  static constexpr size_t kMaxPendingPedestrianRequests = 8;
```

- [ ] **Step 3: Wire the pedestrian query callback in prediction_node.cpp**

In `src/world_modeling/prediction/src/prediction_node.cpp`, after the vehicle `setLaneletQueryFunction` block (after line 430), add:

```cpp
  // Create service client for pedestrian spatial lanelet queries
  nearby_lanelets_client_ = this->create_client<lanelet_msgs::srv::GetNearbyLanelets>("get_nearby_lanelets");

  // Wire per-pedestrian spatial query as async fire-and-forget.
  trajectory_predictor_->setPedestrianQueryFunction(
    [this](const std::string & ped_id, const geometry_msgs::msg::Point & position) {
      if (!nearby_lanelets_client_ || !nearby_lanelets_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "GetNearbyLanelets service not ready, using open-area fallback");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(pending_pedestrian_requests_mutex_);
        if (pending_pedestrian_requests_.count(ped_id) ||
            pending_pedestrian_requests_.size() >= kMaxPendingPedestrianRequests) {
          return;
        }
        pending_pedestrian_requests_.insert(ped_id);
      }

      auto request = std::make_shared<lanelet_msgs::srv::GetNearbyLanelets::Request>();
      request->position = position;
      request->radius_m = trajectory_predictor_->getPedestrianQueryRadius();

      double px = position.x, py = position.y;
      nearby_lanelets_client_->async_send_request(
        request,
        [this, ped_id, px, py](rclcpp::Client<lanelet_msgs::srv::GetNearbyLanelets>::SharedFuture future) {
          {
            std::lock_guard<std::mutex> lock(pending_pedestrian_requests_mutex_);
            pending_pedestrian_requests_.erase(ped_id);
          }
          auto response = future.get();
          if (response->success) {
            trajectory_predictor_->updatePedestrianLaneletCache(
              ped_id, response->lanelets, px, py);
          }
        });
    });
```

Add a simple public getter in `trajectory_predictor.hpp` (public section):
```cpp
  double getPedestrianQueryRadius() const { return pedestrian_params_.nearby_query_radius; }
```

- [ ] **Step 4: Add pedestrian cache to pruneStaleCaches**

In `trajectory_predictor.cpp`, find `pruneStaleCaches` and add after the existing vehicle cache pruning:
```cpp
  {
    std::lock_guard<std::mutex> lock(pedestrian_cache_mutex_);
    for (auto it = pedestrian_lanelet_cache_.begin(); it != pedestrian_lanelet_cache_.end();) {
      if ((now - it->second.last_update).seconds() > ttl_s) {
        it = pedestrian_lanelet_cache_.erase(it);
      } else {
        ++it;
      }
    }
  }
```

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp \
        src/world_modeling/prediction/src/trajectory_predictor.cpp \
        src/world_modeling/prediction/include/prediction/prediction_node.hpp \
        src/world_modeling/prediction/src/prediction_node.cpp
git commit -m "feat(prediction): wire pedestrian lanelet cache and GetNearbyLanelets client"
```

---

## Task 5: Context Classifier + Road-Adjacent Hypotheses (PRIORITY 1)

This is the highest-priority task: jaywalking prediction. Even without crosswalk lanelets, any road lanelet near a pedestrian triggers the road-crossing hypothesis.

**Files:**
- Modify: `src/world_modeling/prediction/src/trajectory_predictor.cpp:1166-1193`
- Modify: `src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp`

- [ ] **Step 1: Declare new private methods in header**

In `trajectory_predictor.hpp`, add in the private section (after `generatePedestrianHypotheses` declaration):

```cpp
  /**
   * @brief Classify spatial context around a pedestrian from nearby lanelets.
   * Returns the context and a pointer to the most relevant lanelet (if any).
   */
  struct PedestrianContextResult {
    PedestrianContext context;
    const lanelet_msgs::msg::Lanelet * lanelet;  // Nearest relevant lanelet (nullptr for OPEN_AREA)
    double distance;  // Distance to the nearest lanelet
  };

  PedestrianContextResult classifyPedestrianContext(
    const geometry_msgs::msg::Point & position,
    const std::vector<lanelet_msgs::msg::Lanelet> & nearby_lanelets) const;

  std::vector<TrajectoryHypothesis> generateRoadAdjacentPedestrianHypotheses(
    const KinematicState & state, double velocity,
    const lanelet_msgs::msg::Lanelet & road_lanelet);

  std::vector<TrajectoryHypothesis> generateCrosswalkPedestrianHypotheses(
    const KinematicState & state, double velocity,
    const lanelet_msgs::msg::Lanelet & crosswalk_lanelet);

  std::vector<TrajectoryHypothesis> generateOpenAreaPedestrianHypotheses(
    const KinematicState & state, double velocity);

  double computePedestrianCrosswalkScore(
    double heading_diff, double lateral_offset) const;
```

- [ ] **Step 2: Implement `classifyPedestrianContext`**

In `trajectory_predictor.cpp`, add after the current `generatePedestrianHypotheses` function:

```cpp
TrajectoryPredictor::PedestrianContextResult TrajectoryPredictor::classifyPedestrianContext(
  const geometry_msgs::msg::Point & position,
  const std::vector<lanelet_msgs::msg::Lanelet> & nearby_lanelets) const
{
  // Find nearest lanelet of each type by 2D distance to centerline
  struct TypeMatch {
    const lanelet_msgs::msg::Lanelet * lanelet = nullptr;
    double distance = std::numeric_limits<double>::max();
  };

  TypeMatch crosswalk_match, parking_match, road_match;

  for (const auto & ll : nearby_lanelets) {
    // Compute min distance from position to this lanelet's centerline
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & pt : ll.centerline) {
      double dx = pt.x - position.x;
      double dy = pt.y - position.y;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < min_dist) min_dist = d;
    }

    if (min_dist > pedestrian_params_.lanelet_proximity_threshold) continue;

    if (ll.lanelet_type == "crosswalk" && min_dist < crosswalk_match.distance) {
      crosswalk_match = {&ll, min_dist};
    } else if (ll.lanelet_type == "parking" && min_dist < parking_match.distance) {
      parking_match = {&ll, min_dist};
    } else if ((ll.lanelet_type == "road" || ll.lanelet_type == "intersection") &&
               min_dist < road_match.distance) {
      road_match = {&ll, min_dist};
    }
  }

  // Priority: crosswalk > parking > road > open
  if (crosswalk_match.lanelet) {
    return {PedestrianContext::CROSSWALK, crosswalk_match.lanelet, crosswalk_match.distance};
  }
  if (parking_match.lanelet) {
    return {PedestrianContext::PARKING, parking_match.lanelet, parking_match.distance};
  }
  if (road_match.lanelet) {
    return {PedestrianContext::ROAD_ADJACENT, road_match.lanelet, road_match.distance};
  }
  return {PedestrianContext::OPEN_AREA, nullptr, 0.0};
}
```

- [ ] **Step 3: Implement `generateRoadAdjacentPedestrianHypotheses`**

This is the jaywalking hypothesis generator — highest priority.

```cpp
std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateRoadAdjacentPedestrianHypotheses(
  const KinematicState & state, double velocity,
  const lanelet_msgs::msg::Lanelet & road_lanelet)
{
  std::vector<TrajectoryHypothesis> hypotheses;
  double stop_prob = computeStopProbability(velocity);

  // 1. Forward walking hypothesis
  auto forward_poses = constant_velocity_model_->generateTrajectory(state, prediction_horizon_, time_step_);
  auto forward_hyp = buildHypothesis(std::move(forward_poses), time_step_, Intent::CONTINUE_STRAIGHT);

  // 2. Road-crossing hypothesis: perpendicular to nearest road centerline
  // Find the heading of the road at the closest point
  double road_heading = 0.0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < road_lanelet.centerline.size(); ++i) {
    double cx = (road_lanelet.centerline[i].x + road_lanelet.centerline[i + 1].x) * 0.5;
    double cy = (road_lanelet.centerline[i].y + road_lanelet.centerline[i + 1].y) * 0.5;
    double dx = cx - state.x;
    double dy = cy - state.y;
    double d = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) {
      min_dist = d;
      road_heading = std::atan2(
        road_lanelet.centerline[i + 1].y - road_lanelet.centerline[i].y,
        road_lanelet.centerline[i + 1].x - road_lanelet.centerline[i].x);
    }
  }

  // Perpendicular: choose the direction closer to pedestrian's current heading
  double perp_left = road_heading + M_PI / 2.0;
  double perp_right = road_heading - M_PI / 2.0;
  double diff_left = std::abs(normalizeAngle(state.theta - perp_left));
  double diff_right = std::abs(normalizeAngle(state.theta - perp_right));
  double crossing_heading = (diff_left < diff_right) ? perp_left : perp_right;
  Intent crossing_intent = (diff_left < diff_right) ? Intent::TURN_LEFT : Intent::TURN_RIGHT;

  KinematicState crossing_state = state;
  crossing_state.theta = crossing_heading;
  auto crossing_poses = constant_velocity_model_->generateTrajectory(crossing_state, prediction_horizon_, time_step_);
  auto crossing_hyp = buildHypothesis(std::move(crossing_poses), time_step_, crossing_intent);

  // 3. Stop hypothesis
  KinematicState stopped_state = state;
  stopped_state.v = 0.0;
  auto stop_poses = constant_velocity_model_->generateTrajectory(stopped_state, prediction_horizon_, time_step_);
  auto stop_hyp = buildHypothesis(std::move(stop_poses), time_step_, Intent::STOP);

  // Assign probabilities: stop from sigmoid, rest proportional to priors
  stop_hyp.probability = stop_prob;
  double remaining = 1.0 - stop_prob;
  double total_prior = pedestrian_params_.forward_prior + pedestrian_params_.road_crossing_prior;
  forward_hyp.probability = remaining * pedestrian_params_.forward_prior / total_prior;
  crossing_hyp.probability = remaining * pedestrian_params_.road_crossing_prior / total_prior;

  hypotheses.push_back(std::move(forward_hyp));
  hypotheses.push_back(std::move(crossing_hyp));
  hypotheses.push_back(std::move(stop_hyp));

  return hypotheses;
}
```

- [ ] **Step 4: Replace `generatePedestrianHypotheses` with context dispatch**

Replace the existing function body (lines 1166-1193) with:

```cpp
std::vector<TrajectoryHypothesis> TrajectoryPredictor::generatePedestrianHypotheses(
  const vision_msgs::msg::Detection3D & detection, std::optional<double> velocity)
{
  double v = velocity.value_or(pedestrian_params_.default_speed);
  v = std::clamp(v, 0.0, pedestrian_params_.max_speed);

  auto state = stateFromDetection(detection.bbox.center.position, detection.bbox.center.orientation, v);

  // Try to get nearby lanelets for context classification
  auto nearby = queryPedestrianNearbyLanelets(detection.id, detection.bbox.center.position);

  if (nearby.has_value() && !nearby->empty()) {
    auto ctx = classifyPedestrianContext(detection.bbox.center.position, *nearby);

    std::vector<TrajectoryHypothesis> hypotheses;
    switch (ctx.context) {
      case PedestrianContext::CROSSWALK:
        hypotheses = generateCrosswalkPedestrianHypotheses(state, v, *ctx.lanelet);
        break;
      case PedestrianContext::PARKING:
        hypotheses = generateOpenAreaPedestrianHypotheses(state, v);  // Same as open area
        break;
      case PedestrianContext::ROAD_ADJACENT:
        hypotheses = generateRoadAdjacentPedestrianHypotheses(state, v, *ctx.lanelet);
        break;
      case PedestrianContext::OPEN_AREA:
        hypotheses = generateOpenAreaPedestrianHypotheses(state, v);
        break;
    }

    if (!hypotheses.empty()) {
      RCLCPP_DEBUG_ONCE(node_->get_logger(),
        "Pedestrian context-aware prediction: v=%.2f, context=%d, hypotheses=%zu",
        v, static_cast<int>(ctx.context), hypotheses.size());
      return hypotheses;
    }
  }

  // Fallback: open area fan
  RCLCPP_DEBUG_ONCE(node_->get_logger(), "Pedestrian open-area fallback: v=%.2f", v);
  return generateOpenAreaPedestrianHypotheses(state, v);
}
```

- [ ] **Step 5: Commit**

```bash
git add src/world_modeling/prediction/include/prediction/trajectory_predictor.hpp \
        src/world_modeling/prediction/src/trajectory_predictor.cpp
git commit -m "feat(prediction): add context classifier and road-adjacent jaywalking hypotheses"
```

---

## Task 6: Crosswalk Hypotheses (PRIORITY 2)

**Files:**
- Modify: `src/world_modeling/prediction/src/trajectory_predictor.cpp`

- [ ] **Step 1: Implement `computePedestrianCrosswalkScore`**

```cpp
double TrajectoryPredictor::computePedestrianCrosswalkScore(
  double heading_diff, double lateral_offset) const
{
  double heading_score = std::exp(
    -heading_diff * heading_diff / pedestrian_params_.heading_score_denominator);
  double lateral_score = std::exp(
    -lateral_offset * lateral_offset / pedestrian_params_.lateral_score_denominator);
  return heading_score * lateral_score;
}
```

- [ ] **Step 2: Implement `generateCrosswalkPedestrianHypotheses`**

```cpp
std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateCrosswalkPedestrianHypotheses(
  const KinematicState & state, double velocity,
  const lanelet_msgs::msg::Lanelet & crosswalk_lanelet)
{
  std::vector<TrajectoryHypothesis> hypotheses;
  double stop_prob = computeStopProbability(velocity);

  if (crosswalk_lanelet.centerline.size() < 2) {
    return generateOpenAreaPedestrianHypotheses(state, velocity);
  }

  // Compute crosswalk heading (forward and reverse)
  const auto & cl = crosswalk_lanelet.centerline;
  double cw_heading_fwd = std::atan2(
    cl.back().y - cl.front().y, cl.back().x - cl.front().x);
  double cw_heading_rev = normalizeAngle(cw_heading_fwd + M_PI);

  double diff_fwd = std::abs(normalizeAngle(state.theta - cw_heading_fwd));
  double diff_rev = std::abs(normalizeAngle(state.theta - cw_heading_rev));

  // Primary direction: the crosswalk direction closer to pedestrian heading
  double primary_heading = (diff_fwd <= diff_rev) ? cw_heading_fwd : cw_heading_rev;
  double primary_diff = std::min(diff_fwd, diff_rev);

  // Only generate crosswalk-following if heading is within tolerance
  if (primary_diff <= pedestrian_params_.crosswalk_heading_tolerance) {
    KinematicState cw_state = state;
    cw_state.theta = primary_heading;
    auto cw_poses = constant_velocity_model_->generateTrajectory(cw_state, prediction_horizon_, time_step_);
    auto cw_hyp = buildHypothesis(std::move(cw_poses), time_step_, Intent::CONTINUE_STRAIGHT);

    // Also consider reverse direction with lower score
    double secondary_heading = (primary_heading == cw_heading_fwd) ? cw_heading_rev : cw_heading_fwd;
    double secondary_diff = std::max(diff_fwd, diff_rev);

    double primary_score = computePedestrianCrosswalkScore(primary_diff, 0.0) * pedestrian_params_.crosswalk_prior;
    double secondary_score = 0.0;

    TrajectoryHypothesis secondary_hyp;
    bool has_secondary = secondary_diff <= pedestrian_params_.crosswalk_heading_tolerance;
    if (has_secondary) {
      KinematicState sec_state = state;
      sec_state.theta = secondary_heading;
      auto sec_poses = constant_velocity_model_->generateTrajectory(sec_state, prediction_horizon_, time_step_);
      secondary_hyp = buildHypothesis(std::move(sec_poses), time_step_, Intent::CONTINUE_STRAIGHT);
      secondary_score = computePedestrianCrosswalkScore(secondary_diff, 0.0) * pedestrian_params_.crosswalk_prior;
    }

    // Stop hypothesis
    KinematicState stopped_state = state;
    stopped_state.v = 0.0;
    auto stop_poses = constant_velocity_model_->generateTrajectory(stopped_state, prediction_horizon_, time_step_);
    auto stop_hyp = buildHypothesis(std::move(stop_poses), time_step_, Intent::STOP);

    // Assign probabilities
    stop_hyp.probability = stop_prob;
    double remaining = 1.0 - stop_prob;
    double total_score = primary_score + secondary_score;
    if (total_score < 1e-6) total_score = 1.0;  // Prevent division by zero

    cw_hyp.probability = remaining * primary_score / total_score;
    hypotheses.push_back(std::move(cw_hyp));

    if (has_secondary) {
      secondary_hyp.probability = remaining * secondary_score / total_score;
      hypotheses.push_back(std::move(secondary_hyp));
    }

    hypotheses.push_back(std::move(stop_hyp));
    return hypotheses;
  }

  // Heading too far off crosswalk — fall back to open area
  return generateOpenAreaPedestrianHypotheses(state, velocity);
}
```

- [ ] **Step 3: Commit**

```bash
git add src/world_modeling/prediction/src/trajectory_predictor.cpp
git commit -m "feat(prediction): add crosswalk-following pedestrian hypotheses"
```

---

## Task 7: Open Area / Parking Fan Hypotheses (PRIORITY 3)

**Files:**
- Modify: `src/world_modeling/prediction/src/trajectory_predictor.cpp`

- [ ] **Step 1: Implement `generateOpenAreaPedestrianHypotheses`**

This is the multi-directional fan used for parking lots, open areas, and as the fallback.

```cpp
std::vector<TrajectoryHypothesis> TrajectoryPredictor::generateOpenAreaPedestrianHypotheses(
  const KinematicState & state, double velocity)
{
  std::vector<TrajectoryHypothesis> hypotheses;
  double stop_prob = computeStopProbability(velocity);

  int fan_count = pedestrian_params_.parking_fan_count;
  double half_angle = pedestrian_params_.parking_fan_half_angle;

  // Generate evenly spaced directions within [-half_angle, +half_angle]
  std::vector<double> offsets;
  if (fan_count <= 1) {
    offsets.push_back(0.0);
  } else {
    double step = (2.0 * half_angle) / (fan_count - 1);
    for (int i = 0; i < fan_count; ++i) {
      offsets.push_back(-half_angle + i * step);
    }
  }

  double remaining = 1.0 - stop_prob;
  double total_prior = 0.0;
  std::vector<double> priors;

  for (double offset : offsets) {
    // Forward (offset=0) gets boost, others get 1.0
    double prior = (std::abs(offset) < 1e-3) ? pedestrian_params_.parking_forward_boost : 1.0;
    priors.push_back(prior);
    total_prior += prior;
  }

  for (size_t i = 0; i < offsets.size(); ++i) {
    KinematicState fan_state = state;
    fan_state.theta = normalizeAngle(state.theta + offsets[i]);
    auto poses = constant_velocity_model_->generateTrajectory(fan_state, prediction_horizon_, time_step_);

    Intent intent = Intent::CONTINUE_STRAIGHT;
    if (offsets[i] > 0.1) intent = Intent::TURN_LEFT;
    else if (offsets[i] < -0.1) intent = Intent::TURN_RIGHT;

    auto hyp = buildHypothesis(std::move(poses), time_step_, intent);
    hyp.probability = remaining * priors[i] / total_prior;
    hypotheses.push_back(std::move(hyp));
  }

  // Stop hypothesis
  KinematicState stopped_state = state;
  stopped_state.v = 0.0;
  auto stop_poses = constant_velocity_model_->generateTrajectory(stopped_state, prediction_horizon_, time_step_);
  auto stop_hyp = buildHypothesis(std::move(stop_poses), time_step_, Intent::STOP);
  stop_hyp.probability = stop_prob;
  hypotheses.push_back(std::move(stop_hyp));

  return hypotheses;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/world_modeling/prediction/src/trajectory_predictor.cpp
git commit -m "feat(prediction): add open-area/parking fan pedestrian hypotheses"
```

---

## Task 8: Verify and Clean Up

**Files:**
- Modify: `src/world_modeling/prediction/DEVELOPING.md`

- [ ] **Step 1: Verify no missing includes**

Check that `trajectory_predictor.cpp` has:
```cpp
#include <limits>  // for std::numeric_limits (used in classifyPedestrianContext)
```
This should already be present; verify.

Check that `prediction_node.hpp` has the new service include:
```cpp
#include "lanelet_msgs/srv/get_nearby_lanelets.hpp"
```

- [ ] **Step 2: Verify `normalizeAngle` is accessible**

`normalizeAngle` is in an anonymous namespace in `trajectory_predictor.cpp` (line 67). The new pedestrian functions use it and are in the `prediction` namespace (methods of `TrajectoryPredictor`), so they CAN access the anonymous-namespace function since it's in the same translation unit. Verify this by reading the file structure.

- [ ] **Step 3: Update DEVELOPING.md**

Add a section about pedestrian prediction to `src/world_modeling/prediction/DEVELOPING.md` describing the context-aware pipeline, the 4 contexts, and the `GetNearbyLanelets` service dependency.

- [ ] **Step 4: Final commit**

```bash
git add src/world_modeling/prediction/DEVELOPING.md
git commit -m "docs(prediction): update DEVELOPING.md with pedestrian prediction v2 architecture"
```

---

## Simulation Testing

After all tasks are committed, test in CARLA sim:

```bash
export ACTIVE_MODULES="simulation perception world_modeling action"
./watod up
```

Use `heavy_traffic` scenario (has `PEDESTRIAN_CROSS_FACTOR = 0.2` — pedestrians actually cross roads):

Verify in Foxglove:
1. **Road-adjacent context:** Pedestrians near roads show 3 hypotheses (forward + road-crossing + stop). The crossing hypothesis should be perpendicular to the nearest road.
2. **Crosswalk context:** Pedestrians on/near crosswalks show 2-3 hypotheses along the crosswalk direction.
3. **Open area:** Pedestrians far from roads show the 5-directional fan + stop.
4. **No regressions:** Vehicle and cyclist predictions unchanged. No crashes or service failures.
