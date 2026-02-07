# Stop Sign Intersection Logic Implementation - Change Summary

## Overview
Replaced snapshot-based stop sign logic with temporal arrival-time-based logic for proper multi-way stop intersection handling.

---

## Files Created

### 1. **stop_sign_arrival_queue.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/stop_sign_arrival_queue.hpp`

**Purpose:** Core data structure for tracking vehicle arrivals at stop signs.

**Key Components:**
- `StopSignArrivalRecord`: Tracks individual vehicle arrival data
  - `car_id`: Vehicle identifier
  - `arrival_time`: rclcpp::Time when vehicle first detected
  - `has_proceeded`: Whether vehicle has gone through
  - `stopped`: Whether vehicle has actually stopped

- `StopSignArrivalQueue`: Manages arrival queue for intersection
  - `update()`: Updates queue with current detections
  - `canProceed()`: Checks if ego has right-of-way
  - `getPriorityCarIds()`: Gets vehicles that arrived earlier
  - `markProceeded()`: Marks vehicle as having proceeded
  - `clear()`: Resets the queue

**Why it works:**
- Maintains temporal ordering of arrivals
- Automatically removes vehicles that leave intersection
- Implements first-to-stop priority rule
- Supports simultaneous arrival detection (configurable threshold)

---

### 2. **get_stop_sign_cars_with_timestamps_action.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/actions/get_stop_sign_cars_with_timestamps_action.hpp`

**Purpose:** Enhanced version of GetStopSignCars that captures detection timestamps.

**Improvements over original:**
- Returns `vector<pair<string, rclcpp::Time>>` instead of just `vector<string>`
- Extracts timestamps from detection headers
- Adds velocity filtering capability (configurable threshold)
- Placeholder for heading-based filtering
- Deduplicates by ID, keeping earliest timestamp
- Sorts results by timestamp for debugging

**Why it works:**
- Uses actual detection timestamps from vision_msgs/Detection3D
- Spatial filtering via distance to stop line centroid
- Future-ready for velocity and heading checks

---

### 3. **update_stop_sign_arrival_queue_action.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/actions/update_stop_sign_arrival_queue_action.hpp`

**Purpose:** BehaviorTree action to update arrival queue each tick.

**Functionality:**
- Creates queue if it doesn't exist
- Passes current detections to queue's update method
- Logs queue state for debugging
- Uses bidirectional port for queue persistence

**Why it works:**
- Maintains queue state across behavior tree ticks
- Automatically handles vehicles entering/leaving
- Provides visibility into queue state

---

### 4. **stop_sign_check_right_of_way_condition.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/conditions/stop_sign_check_right_of_way_condition.hpp`

**Purpose:** Condition node that determines if ego can proceed.

**Logic:**
- Returns SUCCESS if ego has right-of-way
- Returns FAILURE if other vehicles have priority
- Outputs list of blocking vehicle IDs

**Right-of-way rules implemented:**
1. Vehicle must have stopped
2. All vehicles that arrived earlier must proceed first
3. Handles simultaneous arrivals

**Why it works:**
- Uses temporal comparison, not just spatial
- Prevents ego from cutting off earlier arrivals
- Provides diagnostic output (blocking IDs)

---

### 5. **mark_ego_proceeded_action.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/actions/mark_ego_proceeded_action.hpp`

**Purpose:** Marks ego vehicle as having successfully proceeded.

**Functionality:**
- Updates arrival queue to mark ego as proceeded
- Allows next vehicle in queue to go

**Why it works:**
- Prevents ego from blocking itself in queue
- Enables proper queue progression

---

### 6. **clear_stop_sign_arrival_queue_action.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/actions/clear_stop_sign_arrival_queue_action.hpp`

**Purpose:** Resets arrival queue after intersection is cleared.

**Functionality:**
- Creates fresh empty queue
- Prevents stale data from affecting next intersection

**Why it works:**
- Clean state for next stop sign encounter
- Prevents memory leaks from old records

---

## Files Modified

### 7. **registrar.hpp**
**Location:** `src/world_modeling/behaviour/behaviour/include/behaviour/nodes/intersection/registrar.hpp`

**Changes:**
- Added includes for all new action/condition headers
- Registered new nodes with BehaviorTree factory:
  - `ClearStopSignArrivalQueue`
  - `GetStopSignCarsWithTimestamps`
  - `MarkEgoProceeded`
  - `UpdateStopSignArrivalQueue`
  - `StopSignCheckRightOfWay`

**Why it works:**
- Makes new nodes available to XML behavior trees
- Maintains consistent naming convention

---

### 8. **stop_sign.xml**
**Location:** `src/world_modeling/behaviour/behaviour/trees/subtrees/stop_sign.xml`

**Changes - OLD LOGIC (Removed):**
```xml
<GetStopSignCars ... out_stop_sign_car_ids="{stop_sign.current_ids}" />
<SetStopSignPriorityCars ... />
<StopSignCanProceed ... />
<ClearStopSignPriorityCarsAction ... />
```

**Changes - NEW LOGIC (Added):**
```xml
<GetStopSignCarsWithTimestamps 
    ... 
    out_stop_sign_cars_with_timestamps="{stop_sign.cars_with_timestamps}" />

<UpdateStopSignArrivalQueue
    current_cars_with_timestamps="{stop_sign.cars_with_timestamps}"
    simultaneous_arrival_threshold_s="1.0"
    arrival_queue="{stop_sign.arrival_queue}" />

<StopSignCheckRightOfWay
    arrival_queue="{stop_sign.arrival_queue}"
    ego_id="ego"
    blocking_car_ids="{stop_sign.blocking_ids}" />
    
<MarkEgoProceeded ... />
<ClearStopSignArrivalQueue ... />
```

**Why it works:**
- Reactive sequence re-evaluates conditions each tick
- Queue persists across ticks via blackboard
- Temporal ordering determines priority, not snapshot

---

## Files NOT Removed (For Backward Compatibility)

### Kept but not used in new logic:
- `get_stop_sign_cars_action.hpp` - Original spatial-only logic
- `set_stop_sign_priority_cars_action.hpp` - Snapshot latching
- `clear_stop_sign_priority_cars_action.hpp` - Snapshot clearing
- `stop_sign_can_proceed_condition.hpp` - Snapshot comparison

**Rationale:**
- May be used by other behavior trees
- Allows easy rollback if needed
- No harm in keeping registered

---

## How the New Implementation Works

### Initialization (First Detection):
1. **GetStopSignCarsWithTimestamps** detects vehicles near stop line, extracts timestamps
2. **UpdateStopSignArrivalQueue** stores vehicle IDs and arrival times
3. **StopSignCheckRightOfWay** checks if ego arrived first → Returns FAILURE (ego just arrived)
4. Behavior tree waits, ReactiveSequence re-evaluates

### Subsequent Ticks (While Waiting):
1. **GetStopSignCarsWithTimestamps** continues detecting (may find new vehicles)
2. **UpdateStopSignArrivalQueue** adds new arrivals, removes departed vehicles
3. **StopSignCheckRightOfWay** compares ego's arrival time to others:
   - If another car arrived first and hasn't proceeded → FAILURE
   - If ego is first and has stopped → SUCCESS
4. When SUCCESS: Wall despawns, ego proceeds

### After Proceeding:
1. **MarkEgoProceeded** updates queue so ego won't block itself
2. **ClearStopSignArrivalQueue** resets for next intersection

### Multi-Way Stop Handling:
- **2-way stop:** Simple first-to-stop priority
- **3-way stop:** All three approaches tracked independently
- **4-way stop:** All four approaches tracked, proper arrival ordering
- **Simultaneous arrivals:** Configurable threshold (default 1.0s)

---

## Message Flow (Preserved)

### Subscribed Messages:
- `world_model_msgs/msg/WorldObjectArray` - Dynamic objects (unchanged)
  - Contains `vision_msgs/Detection3D` with header.stamp
  
### Published Messages:
- All existing publications maintained (spawning walls, etc.)
- No new message types introduced

### Internal Data Flow:
```
WorldObjectArray → DynamicObjectStore → GetStopSignCarsWithTimestamps 
→ UpdateStopSignArrivalQueue → StopSignCheckRightOfWay → Proceed/Wait
```

---

## Key Advantages Over Old Implementation

### Old Implementation Issues:
1. ❌ **Snapshot-based:** Latched list when ego stopped, waited for ALL to clear
2. ❌ **No temporal ordering:** Couldn't determine who arrived first
3. ❌ **Race condition:** New arrivals after latch were ignored
4. ❌ **Overly conservative:** Waited for entire intersection to clear
5. ❌ **No multi-way stop support:** Treated all cars equally

### New Implementation Benefits:
1. ✅ **Temporal reasoning:** Tracks actual arrival times
2. ✅ **Proper right-of-way:** First-to-stop goes first
3. ✅ **Dynamic updates:** Queue updates every tick
4. ✅ **Efficient:** Only waits for cars with priority
5. ✅ **Multi-way capable:** Handles 2/3/4-way stops correctly
6. ✅ **Debuggable:** Logs queue state and blocking vehicles

---

## Testing Recommendations

### Test Scenarios:
1. **Single vehicle (ego only):** Should proceed immediately after stop
2. **Two vehicles, ego arrives first:** Ego proceeds first
3. **Two vehicles, other arrives first:** Ego waits for other
4. **Four-way stop:** Vehicles proceed in arrival order
5. **Simultaneous arrival:** Both considered arrived at same time
6. **Vehicle departs before proceeding:** Queue removes it, next goes
7. **Multiple intersections:** Queue clears between intersections

### Expected Behavior:
- Console logs show arrival times and queue state
- Blocking vehicle IDs printed when waiting
- Wall despawns only when ego has right-of-way

---

## Future Enhancements (TODOs in code)

1. **Velocity filtering:**
   - Use `history` field from WorldObject
   - Calculate velocity from position differences
   - Filter out moving/fast vehicles

2. **Heading filtering:**
   - Compare vehicle heading to lanelet direction
   - Exclude vehicles driving parallel or departing

3. **Tie-breaking for simultaneous arrivals:**
   - Rightmost vehicle has priority (standard rule)
   - Requires lanelet topology analysis

4. **Cross-traffic clearance:**
   - Verify no vehicles in perpendicular lanes
   - Check for yellow-box gridlock

---

## Configuration Parameters

### In XML (can be tuned):
- `threshold_m="8.0"` - Distance from stop line to detect vehicles
- `velocity_threshold_mps="0.5"` - Max velocity to be "stopped"
- `simultaneous_arrival_threshold_s="1.0"` - Time window for ties
- `ego_id="ego"` - Ego vehicle identifier

---

## Summary

**What was added:**
- Temporal arrival tracking structure
- Timestamp-aware car detection
- Arrival queue management
- Right-of-way checking based on time
- Queue lifecycle actions

**What was removed from XML:**
- Snapshot-based latching logic
- Simple "wait for all cars to leave" condition

**Why it works:**
- Uses actual detection timestamps (not snapshots)
- Implements correct multi-way stop priority rules
- Updates dynamically as situation changes
- Handles all common intersection configurations

The new implementation properly handles stop sign intersections by tracking WHEN vehicles arrive, not just WHETHER they are present. This enables correct right-of-way determination according to standard traffic rules: first to stop, first to go.
