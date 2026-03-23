# Pedestrian Prediction V2: Crosswalk-Proximity + Parking-Aware Context

**Date:** 2026-03-22
**Branch:** `prediction-pedestrian-v2`
**Scope:** Incremental — one new service definition, no new packages

## Problem

Pedestrian prediction currently produces exactly 2 hypotheses (constant-velocity forward walk + stop/yield) regardless of environment. Vehicles get 1-6 lanelet-aware hypotheses with curvature scoring, lane changes, and intersection handling. Cyclists get DFS path exploration with weighted probability distribution. Pedestrians get nothing map-aware.

This is insufficient for:
- **Crosswalks:** Pedestrian heading toward a crosswalk should be predicted to follow it
- **Parking lots:** Pedestrians move unpredictably — a single forward hypothesis misses lateral motion
- **Road-adjacent:** Jaywalking risk needs a perpendicular road-crossing hypothesis
- **Open areas:** Same unpredictability as parking lots

## Critical Design Constraint: Vehicle-Only Routing Graph

The existing `GetLaneletAhead` service uses BFS through a routing graph built with `Participants::Vehicle`. This means crosswalks (`lanelet_type == "crosswalk"`) and parking areas (`lanelet_type == "parking"`) are **never returned** by this service — they are not part of the vehicle routing graph.

Additionally, `findCurrentLaneletId()` (the BFS entry point) uses heading alignment scoring, which fails for pedestrians walking perpendicular to road lanelets.

**Solution:** Add a new `GetNearbyLanelets` service that wraps the existing `LaneletHandler::getLaneletsInRadius()` method — a pure spatial query (O(n) scan of `map_->laneletLayer`) that returns ALL lanelet types regardless of routing connectivity. This method already exists but has no service exposure.

## Approach

### New Service: GetNearbyLanelets

A lightweight spatial query service. No heading, no routing graph, no BFS. Just "give me everything within radius."

**Service definition** (`lanelet_msgs/srv/GetNearbyLanelets.srv`):
```
# Request
geometry_msgs/Point position
float64 radius_m
---
# Response
bool success
string error_message
lanelet_msgs/Lanelet[] lanelets
```

**Server-side implementation:**
1. New method on `LaneletHandler`: `getNearbyLanelets(position, radius)` — calls existing `getLaneletsInRadius()`, converts each result via existing `toLaneletMsg()`
2. New service handler: `GetNearbyLaneletsService` — same pattern as `GetLaneletAheadService` (15 lines of boilerplate)
3. Register in `world_model_node.cpp` — one `interfaces_.push_back()` line

**Note on connectivity fields:** `toLaneletMsg()` calls `populateLaneletConnectivity()` which queries the Vehicle routing graph. For crosswalk/parking lanelets, this returns empty connectivity (no successors, no left/right, no conflicting). This is correct — pedestrian prediction only needs geometry (centerline, boundaries) and `lanelet_type`, not routing connectivity.

### Prediction-Side: Context Classification

Add a **context classifier** in `generatePedestrianHypotheses` that determines the pedestrian's spatial context from nearby lanelet data, then dispatches to context-specific hypothesis generators.

**Flow:**
1. Prediction node creates a second async service client for `get_nearby_lanelets`
2. `generatePedestrianHypotheses` calls a new `queryPedestrianContext()` function (analogous to `queryVehicleLanelets()`) that caches per-pedestrian nearby lanelets
3. New `classifyPedestrianContext()` iterates cached lanelets, computes raw 2D distance from pedestrian to each lanelet centerline (NO heading rejection — pedestrians can face any direction relative to any lanelet)
4. Classify by nearest lanelet type within `pedestrian_lanelet_proximity_threshold` (8m)
5. Priority when multiple types nearby: **crosswalk > parking > road > open**

**Why a separate cache from vehicles:** The vehicle cache (`vehicle_lanelet_cache_`) stores `LaneletAhead` responses (routing-based). Pedestrians need spatial responses (non-routing). Mixing the two cache types would require type-erasure or variant. Cleaner to add `pedestrian_lanelet_cache_` with its own `std::unordered_map<string, PedestrianLaneletEntry>`.

### Hypothesis Generation Per Context

#### Crosswalk (pedestrian near `lanelet_type == "crosswalk"`)

| Hypothesis | Intent | Description |
|-----------|--------|-------------|
| Crosswalk-following | CONTINUE_STRAIGHT | Project along crosswalk centerline via CV model. Direction chosen by heading alignment (closer of forward/reverse centerline). |
| Stop/yield | STOP | Stationary. Probability from existing sigmoid. |

Scoring: New `computePedestrianCrosswalkScore()` function — Gaussian on heading alignment + lateral offset, with **pedestrian-specific denominators** (`pedestrian_heading_score_denominator`, `pedestrian_lateral_score_denominator`). Does NOT reuse `computeGeometricScore()` because vehicle denominators (`heading_score_denominator = 0.2`, `lateral_score_denominator = 4.0`) are far too tight for pedestrians.

Result: 2-3 hypotheses (forward along crosswalk, possibly reverse direction, stop).

#### Parking (pedestrian near `lanelet_type == "parking"`)

| Hypothesis | Intent | Description |
|-----------|--------|-------------|
| Fan directions (5x) | CONTINUE_STRAIGHT / TURN_LEFT / TURN_RIGHT | CV model at heading offsets: 0deg, +/-30deg, +/-60deg from current heading |
| Stop | STOP | Stationary. Sigmoid probability. |

Scoring: Forward direction gets `pedestrian_parking_forward_boost` (1.5x prior). Lateral directions get equal lower priors. Stop from sigmoid. Probabilities normalized to sum to 1.0.

Result: 6 hypotheses (5 directional + stop).

#### Road-adjacent (near `lanelet_type == "road"` or `"intersection"`, no crosswalk nearby)

| Hypothesis | Intent | Description |
|-----------|--------|-------------|
| Forward walking | CONTINUE_STRAIGHT | Current heading, CV model |
| Road-crossing | TURN_LEFT or TURN_RIGHT | Perpendicular to nearest road lanelet centerline heading. Low probability. |
| Stop | STOP | Stationary. Sigmoid probability. |

Scoring: Forward gets `pedestrian_forward_prior` (1.5). Road-crossing gets `pedestrian_road_crossing_prior` (0.15). Stop from sigmoid. Normalized.

Result: 3 hypotheses (forward, cross-road, stop).

#### Open area (no lanelets within proximity threshold)

Same as parking: multi-directional fan + stop. Pedestrians with no map context are unpredictable.

Result: 6 hypotheses (5 directional + stop).

### Fallback Chain (Production Safety)

```
nearby_lanelets service available AND returns data?
  YES -> classifyPedestrianContext -> context-specific hypotheses
  NO  -> open-area fan (5 directional + stop)

context-specific generator produces empty result?
  -> open-area fan fallback

all paths guarantee: valid hypotheses with probabilities summing to 1.0
```

Every code path produces valid hypotheses. The system never returns empty.

### Probability Normalization Strategy

Each context generator sets raw probability values using priors and sigmoid stop probability, then normalizes to sum to 1.0 before returning. This ensures the intent classifier's "preserve pre-set probabilities" path triggers (checks `existing_total > 1e-6`).

Explicit pattern for each context:
1. Compute `stop_prob` from sigmoid
2. Set stop hypothesis probability = `stop_prob`
3. Distribute `(1.0 - stop_prob)` among moving hypotheses proportional to their priors
4. Return normalized vector

This avoids the ambiguity of mixing geometric scores with confidence-split patterns.

## Parameters

### New `PedestrianParams` fields

```yaml
# Pedestrian lanelet-aware prediction
pedestrian_lanelet_proximity_threshold: 8.0       # meters - max distance to consider lanelet "nearby"
pedestrian_crosswalk_heading_tolerance: 1.05      # radians (~60deg) - max heading diff for crosswalk alignment
pedestrian_heading_score_denominator: 1.5         # Gaussian denominator for pedestrian heading scoring (wider than vehicle 0.2)
pedestrian_lateral_score_denominator: 10.0        # Gaussian denominator for pedestrian lateral scoring (wider than vehicle 4.0)
pedestrian_parking_fan_half_angle: 1.05           # radians (~60deg) - half-spread of directional fan
pedestrian_parking_fan_count: 5                   # number of directional hypotheses in fan
pedestrian_crosswalk_prior: 2.0                   # maneuver prior for crosswalk-following
pedestrian_forward_prior: 1.5                     # maneuver prior for forward walking
pedestrian_road_crossing_prior: 0.15              # maneuver prior for jaywalking
pedestrian_parking_forward_boost: 1.5             # boost for forward direction in parking fan
pedestrian_cache_invalidation_dist: 3.0           # meters - re-query nearby lanelets after pedestrian moves this far
pedestrian_nearby_query_radius: 15.0              # meters - radius for GetNearbyLanelets service query
```

### Unchanged

- `pedestrian_default_speed: 1.4` and `pedestrian_max_speed: 3.0`
- `stop_sigmoid_midpoint`, `stop_sigmoid_steepness` (shared with vehicles)
- `computeStopProbability()` reused as-is
- `buildHypothesis()` reused as-is

### Removed from previous spec revision

- `pedestrian_lanelet_confidence` / `pedestrian_cv_fallback_confidence` — replaced by cleaner per-context prior-based normalization
- Reuse of `computeGeometricScore()` — replaced by pedestrian-specific scoring with wider denominators

## Files Changed

### lanelet_msgs package (new service only)

| File | Change |
|------|--------|
| `lanelet_msgs/srv/GetNearbyLanelets.srv` | **New file.** Service definition (position + radius -> Lanelet[]) |
| `lanelet_msgs/CMakeLists.txt` | Add `GetNearbyLanelets.srv` to `rosidl_generate_interfaces()` |

### world_model package (service provider)

| File | Change |
|------|--------|
| `world_model/src/lanelet_handler.cpp` | Add `getNearbyLanelets()` method (~10 lines: call `getLaneletsInRadius` + `toLaneletMsg` loop) |
| `world_model/include/world_model/lanelet_handler.hpp` | Declare `getNearbyLanelets()` method |
| `world_model/include/world_model/interfaces/services/get_nearby_lanelets_service.hpp` | **New file.** Service handler (~40 lines, mirrors `GetLaneletAheadService` pattern) |
| `world_model/src/world_model_node.cpp` | Add one `interfaces_.push_back()` line to register the new service |

### prediction package (consumer)

| File | Change |
|------|--------|
| `prediction/include/prediction/trajectory_predictor.hpp` | Expand `PedestrianParams` struct (~12 new fields). Add `PedestrianLaneletEntry` cache struct. Add `PedestrianContext` enum. Declare new methods. |
| `prediction/src/trajectory_predictor.cpp` | Replace `generatePedestrianHypotheses` with context-aware dispatch. Add: `queryPedestrianNearbyLanelets`, `classifyPedestrianContext`, `computePedestrianCrosswalkScore`, `generateCrosswalkPedestrianHypotheses`, `generateParkingPedestrianHypotheses`, `generateRoadAdjacentPedestrianHypotheses`, `generateOpenAreaPedestrianHypotheses` |
| `prediction/include/prediction/prediction_node.hpp` | Add `nearby_lanelets_client_` service client, `pedestrian_lanelet_cache_mutex_` |
| `prediction/src/prediction_node.cpp` | Add `declare_parameter` calls for new pedestrian params. Create `nearby_lanelets_client_`. Wire `setPedestrianLaneletQueryFunction` callback. |
| `prediction/config/params.yaml` | Add new pedestrian parameter defaults |
| `prediction/DEVELOPING.md` | Update pedestrian prediction documentation |

### Not touched

No changes to: launch files, costmap, lanelet_markers, world_model_markers, behaviour, world_model_msgs, or any perception/action packages. Existing `GetLaneletAhead` service and all its consumers are unchanged.

## Production Safety Guarantees

1. **Graceful fallback chain** — service unavailable -> open-area fan. No nearby lanelets -> open-area fan. Context generator empty -> open-area fan. Always produces valid hypotheses.
2. **Existing services untouched** — `GetLaneletAhead` and all 3 other services are unchanged. Adding `GetNearbyLanelets` is purely additive.
3. **Probabilities always sum to 1.0** — each context path normalizes before returning. Intent classifier preserves them.
4. **Confidence smoothing unchanged** — existing EMA in prediction_node.cpp handles pedestrians without modification.
5. **Backward-compatible params** — all new params have defaults in params.yaml.
6. **Async queries non-blocking** — first cycle for a new pedestrian returns nullopt (falls back to open-area). Same proven pattern as vehicles.
7. **No changes to downstream consumers** — costmap, world model markers, behaviour tree all consume the same `WorldObjectArray` message.
8. **Thread safety** — new `pedestrian_lanelet_cache_` has its own mutex, separate from `vehicle_cache_mutex_`. No contention between vehicle and pedestrian query paths.
9. **Request cap** — pedestrian queries use a separate `nearby_lanelets_client_` with its own pending-request tracking, so they cannot starve vehicle lanelet queries.
10. **Connectivity fields empty for non-road lanelets** — `toLaneletMsg()` returns empty successors/left/right for crosswalks and parking. Pedestrian code only uses geometry and `lanelet_type`, so this is correct behavior.

## Reused Infrastructure

- `LaneletHandler::getLaneletsInRadius()` — existing spatial query, O(n) scan, returns all lanelet types
- `LaneletHandler::toLaneletMsg()` — existing serialization
- `constant_velocity_model_->generateTrajectory()` — all pedestrian hypotheses use CV model
- `buildHypothesis()` — standard hypothesis builder
- `computeStopProbability()` — sigmoid stop probability
- `stateFromDetection()` — state initialization from detection
- `extractYaw()` — heading from quaternion
- Confidence smoothing in `prediction_node.cpp` — already applies to all object types
- `InterfaceBase` pattern — for new service handler
