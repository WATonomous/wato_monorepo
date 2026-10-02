# Product Requirements Document: CARLA Interactive SIL Platform

> Current milestone: only trajectory injection and scenario/map switching.
> See [Controller SIL scope and implementation](CONTROLLER_SIL.md). The broader
> goals below remain a future backlog.


## Document Status

- Status: Draft
- Product area: Simulation and software-in-the-loop testing
- First release: Interactive CARLA tooling in Foxglove
- Primary users: Autonomy developers working on planning and motion control

## Product Vision

Build a simulation platform in which the WATonomous autonomy stack can drive a simulated vehicle through repeatable situations before code reaches the physical car.

The final platform should support:

- Interactive debugging from Foxglove
- Repeatable scenario definitions
- Motion-planning and control validation
- Perception and localization validation using simulated sensors
- Fault injection and environmental variation
- Automated SIL regression testing
- Machine-readable pass or fail results in CI
- Reproduction of failed tests from saved scenarios, seeds, and logs

The simulator is not the product by itself. The product is the complete workflow for creating a situation, running production autonomy software against it, observing what happened, and eventually deciding automatically whether the behavior was acceptable.

## What SIL Means

Software-in-the-loop, or SIL, means running production software against simulated inputs and a simulated vehicle or environment instead of the physical vehicle.

A CARLA SIL test is not normally a unit test.

- A unit test isolates a function, class, or node and usually replaces dependencies with mocks.
- A component SIL test runs one production subsystem, such as motion control, against a simulated vehicle.
- A subsystem SIL test runs several connected subsystems, such as planning and control.
- A full-stack SIL test runs most or all of the autonomy stack against simulated sensors, actors, vehicle dynamics, and maps.

SIL tests can run in CI, but their cost determines when they should run:

- Fast deterministic smoke scenarios may run on every push.
- Broader regression scenarios may run for pull requests.
- Expensive perception, randomized, or long-duration scenarios may run nightly or on demand.

The first release in this PRD is not automated SIL testing yet. It is the interactive scenario-authoring and debugging foundation needed before reliable automated tests can be created.

## Problem

The current CARLA integration can load a small set of scenarios, publish simulator truth, and accept vehicle commands. It does not provide a coherent developer workflow for manipulating the simulation while it runs.

Developers currently lack a simple way to:

- Switch between maps and scenarios from one interface
- Spawn a selected vehicle, pedestrian, or obstacle at a selected location
- Choose and preview a test trajectory
- Send the same trajectory to pure pursuit or MPC
- Start, stop, and reset a control experiment
- Turn an interactive experiment into a repeatable scenario

Without those capabilities, creating SIL regression tests is premature. The team first needs a usable way to author, inspect, and reproduce simulation behavior.

## Product Strategy

Development should progress through four product stages.

### V1: Interactive Scenario and Motion-Control Workbench

Provide a canonical Foxglove layout and ROS backend for scenario switching, map switching, runtime actor spawning, a catalog of predefined trajectories, and controller selection.

This is the release specified in detail by this PRD.

### V2: Reproducible Scenario Authoring

Allow an interactive session to be saved as a declarative scenario containing:

- Map and scenario identifiers
- Ego spawn
- Actor types, poses, and behaviors
- Trajectory definition
- Controller selection and parameters
- Random seeds
- Timed simulation events

The saved scenario must be replayable without Foxglove.

V2 may also add custom trajectory drawing after the preset workflow is stable.

### V3: Automated SIL Execution and Evaluation

Add a headless runner that:

- Starts or connects to CARLA
- Loads a saved scenario
- Waits for explicit readiness
- Runs the selected autonomy components
- Records MCAP and simulator truth
- Enforces a timeout
- Evaluates configured metrics
- Produces JSON and JUnit results

This is where preset interactive experiments become automated SIL regression tests.

### V4: Full-Stack Simulation

Add simulated production sensors, perception, Eidos localization, sensor degradation, environmental variation, and large regression suites.

CARLA ground truth must be hidden from the production autonomy path and used only by the evaluator in this mode.

## V1 Goal

A developer can start the CARLA stack and perform useful motion-control experiments entirely from one Foxglove layout without restarting the containers.

The developer must be able to:

1. Switch between scenarios, with the scenario selecting its required map.
2. Spawn and remove actors while simulation is running.
3. Select and preview predefined trajectories.
4. Send the selected trajectory to either pure pursuit or MPC.
5. Start, stop, and reset the experiment safely.
6. See the active state and actionable errors.

## V1 Non-Goals

- Automated pass or fail evaluation
- CI integration
- Unit-test expansion
- Perception validation
- Eidos or sensor-derived localization
- Camera and LiDAR fidelity
- Photorealistic rendering improvements
- Randomized scenario sweeps
- Distributed simulation execution
- A complete replacement for CARLA ScenarioRunner
- Click-drawn or free-form trajectory editing

These are later product stages. Pulling them into V1 would delay the first useful workflow and create an oversized implementation.

## Primary User Stories

### Scenario and Map Switching

As an autonomy developer, I want to select a scenario in Foxglove so that CARLA loads the correct map, ego spawn, environment, and initial actors as one operation.

Acceptance requirements:

- Foxglove lists or exposes every available scenario.
- The active scenario and map are visible.
- The scenario, not the user, selects the required map.
- Switching does not require rebuilding images or restarting containers.
- A failed switch leaves the simulation paused and reports the failed stage.
- The system never continues with a CARLA map and Lanelet2 map that do not match.

### Runtime Actor Spawning

As an autonomy developer, I want to select an actor and click a pose in Foxglove so that the actor appears in the running CARLA world.

V1 actor classes:

- Vehicle
- Pedestrian
- Cone
- Barrier

V1 actor behaviors:

- Stationary
- CARLA autopilot
- Constant velocity

Acceptance requirements:

- The user can select an actor preset and behavior.
- The user can click a 2D pose, including position and heading.
- The request uses the ROS `map` frame.
- The backend converts the pose to CARLA coordinates.
- The backend can snap a pose to the nearest road or sidewalk when requested.
- Spawn collisions and invalid placements return an actionable error.
- The user can delete an actor or clear every interactively injected actor.
- The actor manager deletes only actors it owns.
- The actor manager never calls `world.tick()`.

### Predefined Trajectories

As a motion-control developer, I want to select a known maneuver so that I can quickly exercise a controller without running the full planner.

Initial presets:

- Straight line
- Single lane change
- Double lane change
- Snake or slalom maneuver ending at a defined target
- Constant-radius left turn
- Constant-radius right turn
- Brake in curve
- Stop and dwell
- Oval or closed loop

Acceptance requirements:

- The fake-planner implementation from PR #487 is reused where practical.
- The user can configure target speed and maneuver scale.
- The trajectory is previewed before it is committed.
- The committed output uses `wato_trajectory_msgs/Trajectory`.
- Required behavior messages are published so the selected controller does not remain in standby.
- The same trajectory can be consumed by pure pursuit or MPC.

### Controller Selection

As a motion-control developer, I want to choose pure pursuit or MPC so that I can compare both controllers against the same trajectory and CARLA state.

Acceptance requirements:

- Only one controller may command the CARLA vehicle at a time.
- Selecting a controller deactivates the previous controller before activating the next one.
- The active controller is visible in Foxglove.
- Stop immediately disables control and commands a safe stop.
- Reset stops control before moving the ego vehicle.

## V1 User Experience

The canonical Foxglove layout should contain:

- A large 3D panel in top-down mode
- Scenario selection and load controls
- Active scenario, map, and simulation state
- Pause and resume controls
- Actor preset, behavior, and placement controls
- Delete actor and clear injected actors controls
- Trajectory preset list and basic parameter controls
- Preview, load, start, and stop trajectory controls
- Pure pursuit and MPC selection
- Reset ego control
- Ego speed and steering indicators
- Preview-path, active-trajectory, and actor markers
- Scenario, actor-manager, trajectory, and controller errors

The existing `config/foxglove_config/carla_sim.json` should be evolved into the canonical layout.

Foxglove's built-in 3D click-to-publish functionality should be used for actor placement:

- <https://docs.foxglove.dev/docs/visualization/panels/3d>

A custom Foxglove extension is optional. It should be created only if coordinating stock service and publish panels produces an unacceptable workflow.

## Functional Architecture

```text
Foxglove CARLA layout
    |
    +-> scenario controls
    +-> actor controls and clicked spawn pose
    +-> trajectory preset controls
    +-> controller controls
    |
    v
Interactive simulation backend
    +-> scenario server and map registry
    +-> actor manager
    +-> trajectory preset server
    +-> controller selector
    |
    v
CARLA server <-> Ackermann bridge <-> pure pursuit or MPC
```

The scenario server is the only CARLA client allowed to advance simulation time with `world.tick()`.

## Scenario and Map Model

A scenario must reference a map bundle instead of independently hardcoding CARLA and Lanelet2 paths.

Example scenario:

```yaml
id: stopped_vehicle_demo
map_id: town10hd
ego_spawn: main_road_start
weather: clear_noon
actors:
  - preset: stopped_vehicle
    spawn: main_road_obstacle
```

Example map bundle:

```yaml
id: town10hd

carla:
  built_in_map: Town10HD

lanelet2:
  path: /opt/watonomous/maps/osm/Town10HD.osm
  projector: local_cartesian

spawns:
  main_road_start: {x: 0.0, y: 0.0, yaw: 0.0}
  main_road_obstacle: {x: 20.0, y: 0.0, yaw: 0.0}
```

Custom maps use an `xodr_path` instead of `built_in_map`. The simulation container must mount the map directory so the scenario server can generate the OpenDRIVE world.

Scenario switching is one coordinated transaction:

1. Pause simulation.
2. Deactivate world-dependent lifecycle nodes.
3. Destroy actors owned by the outgoing scenario and actor manager.
4. Load or generate the CARLA map.
5. Restore synchronous settings and configure Traffic Manager.
6. Spawn the ego vehicle and scenario actors.
7. Reconfigure map-dependent consumers with the matching map bundle.
8. Reactivate lifecycle nodes.
9. Publish the new active state.
10. Resume simulation.

## Proposed ROS Interfaces

Names are provisional and must be reconciled with existing message conventions during implementation.

### Scenario Server

Retain:

- `/carla/scenario_server/get_available_scenarios`
- `/carla/scenario_server/switch_scenario`
- `/carla/scenario_server/pause`

Scenario status must include:

- Active scenario ID
- Active map ID
- Loading, ready, paused, or error state
- Last error
- Simulation frame

### Actor Manager

The actor-management implementation should be a new ROS package beside the existing bridge packages:

```text
src/simulation/carla_ros_bridge/
├── carla_actors/
│   ├── package.xml
│   ├── setup.py
│   ├── carla_actors/
│   │   ├── actor_manager_node.py
│   │   ├── actor_registry.py
│   │   └── spawn_utils.py
│   └── tests/
└── carla_msgs/
    └── srv/
        ├── SetSpawnPreset.srv
        ├── SpawnActor.srv
        ├── DeleteActor.srv
        ├── ClearInjectedActors.srv
        └── ListActors.srv
```

`carla_actors` is the ROS package. `actor_manager_node.py` is the executable node that connects to CARLA, tracks the actors it owns, and implements the service callbacks. The `.srv` files are service interface definitions and belong in the existing interface-only `carla_msgs` package.

The following are ROS service names exposed by the actor-manager node. They are not repository file paths:

Services:

- `/carla/actor_manager/set_spawn_preset`
- `/carla/actor_manager/spawn_actor`
- `/carla/actor_manager/delete_actor`
- `/carla/actor_manager/clear_injected_actors`
- `/carla/actor_manager/list_actors`

Foxglove click topic:

- `/carla/actor_manager/spawn_pose`

The spawn request should include blueprint or preset ID, actor class, pose, placement mode, behavior, initial speed, and optional lifetime.

### Trajectory Preset Server

Trajectory injection should be implemented as another ROS package beside the existing bridge packages:

```text
src/simulation/carla_ros_bridge/
├── carla_trajectories/
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── config/
│   │   └── trajectory_presets.yaml
│   ├── include/carla_trajectories/
│   ├── src/
│   │   └── trajectory_server_node.cpp
│   └── test/
└── carla_msgs/
    └── srv/
        ├── GetAvailableTrajectories.srv
        ├── SelectTrajectory.srv
        └── SetController.srv
```

`carla_trajectories` is the ROS package. Its trajectory-server node owns the selected preset, generates the `wato_trajectory_msgs/Trajectory`, publishes the controller's required behavior message, and exposes the runtime services. The `.srv` files define the request and response formats and remain in `carla_msgs`.

The preset catalog should be data-driven. Adding a new fixed trajectory should normally require a new YAML or JSON preset, not another ROS service or another node.

Services:

- `/carla/trajectories/get_available`
- `/carla/trajectories/select`
- `/carla/trajectories/start`
- `/carla/trajectories/stop`
- `/carla/trajectories/reset_ego`

Visualization topics:

- `/carla/trajectories/preview_path`
- `/carla/trajectories/active_trajectory`

Each preset should define or generate:

- A stable trajectory ID and display name
- A trajectory shape
- Default length and target speed
- Allowed user-adjustable parameters
- Spatial sample spacing
- Heading and curvature
- Velocity and stop profile

The initial implementation should reuse the trajectory-generation logic from PR #487 where practical, but expose it through the `carla_trajectories` package and the service contract above.

### Controller Selection

Service:

- `/carla/controller/select`

V1 should use lifecycle activation so exactly one controller is active. A command multiplexer may be added later if another command source must remain running.

## Non-Functional Requirements

- Scenario and map changes must be atomic from the user's perspective.
- Only one node may own CARLA simulation ticks.
- Only one node may command the ego vehicle at a time.
- Every operation must return a useful success or error response.
- Interactive actors must have explicit ownership and cleanup.
- No operation may require rebuilding the CARLA image.
- Scenario switching must not require restarting the containers.
- The same scenario and trajectory definitions must be usable later by a headless runner.
- UI-specific state must not be the only copy of scenario state.

## V1 Implementation Sequence

### 1. Runtime Correctness

- Fix simulation timestep compatibility.
- Synchronize and seed Traffic Manager.
- Ensure one tick owner.
- Ensure one localization and TF authority.
- Make the simulation profile launch simulation-specific action and world-model configurations.

### 2. Scenario and Map Registry

- Add map bundles.
- Make every scenario declare a map ID.
- Support built-in and custom OpenDRIVE maps.
- Make map-dependent reconfiguration part of scenario switching.

### 3. Predefined Trajectories and Controller Selection

- Rebase the useful fake-planner work from PR #487.
- Move the simulation-facing functionality into `carla_trajectories`.
- Define a data-driven trajectory preset catalog.
- Implement trajectory preview and commit.
- Add exclusive pure-pursuit and MPC selection.
- Add start, stop, and reset controls.

### 4. Runtime Actor Manager

- Implement spawn, list, delete, and clear operations.
- Implement road and sidewalk snapping.
- Support stationary, autopilot, and constant-velocity behaviors.
- Add actor markers and Foxglove controls.

### 5. Canonical Foxglove Layout

- Integrate all controls and status displays.
- Remove stale CARLA topics from the existing layout.
- Add concise operator instructions.
- Add a custom extension only if the stock panels are inadequate.

## V1 Definition of Done

Without restarting containers, a developer can:

1. Switch between at least two scenarios on different maps.
2. See the matching map and ego spawn after each switch.
3. Spawn and remove a vehicle and pedestrian through Foxglove.
4. Run at least five predefined trajectories.
5. Send the same preset trajectory to pure pursuit or MPC.
6. Start, stop, and reset the ego vehicle safely.
7. See actionable errors for failed map, spawn, trajectory, or controller operations.

The backend must also be structured so the same scenario and trajectory definitions can be executed later without Foxglove.

## How V1 Becomes Automated SIL

V1 produces the missing authoring primitives. V2 and V3 turn those primitives into tests.

```text
Interactive Foxglove experiment
    -> saved scenario and trajectory
    -> headless scenario runner
    -> recorded truth and ROS topics
    -> metric evaluator
    -> pass or fail result
    -> CI regression test
```

Example future test:

```yaml
id: mpc_slalom_10ms
scenario: empty_oval
controller: mpc
trajectory: steady_state_slalom
timeout_s: 30

requirements:
  collision_count: 0
  max_cross_track_error_m: 0.35
  max_lateral_acceleration_mps2: 4.0
  max_steering_rate_radps: 0.6
```

The test runner would execute the scenario, record actual CARLA vehicle motion, evaluate the requirements, and emit a machine-readable result.

The eventual CI policy should be tiered rather than running every scenario on every push:

- Every push: one or two fast control smoke scenarios
- Pull request: deterministic controller and planning regression set
- Nightly: broader maps, traffic, randomized actors, and perception scenarios
- On demand: long-duration, GPU-heavy, and fault-injection campaigns

## Risks

### Foxglove State Becomes Backend State

Risk: Actor or trajectory state exists only inside a panel and cannot be reproduced headlessly.

Mitigation: ROS nodes own all authoritative state. Foxglove only sends commands and displays state.

### Multiple CARLA Tick Owners

Risk: Actor or test tooling independently calls `world.tick()`, causing nondeterministic or stalled simulation.

Mitigation: The scenario server remains the only tick owner. Other nodes queue mutations for application at a controlled frame boundary.

### Multiple Ego Command Sources

Risk: Pure pursuit, MPC, teleoperation, or stale commands control the ego vehicle simultaneously.

Mitigation: Use lifecycle control and explicit command ownership.

### Map Misalignment

Risk: CARLA OpenDRIVE and Lanelet2 describe different geometry or coordinate origins.

Mitigation: Load both through one versioned map bundle and validate alignment before resuming simulation.

### Interactive Features Cannot Be Automated Later

Risk: V1 is implemented as Foxglove-specific scripts that cannot run headlessly.

Mitigation: Put all behavior behind ROS interfaces and declarative data. Treat Foxglove as one client of the backend.

## Open Product Decisions

1. Should V1 load both world modeling and motion control, or initially run control directly from CARLA truth odometry?
2. Which two maps are required for the V1 acceptance demonstration?
3. Which five trajectory presets are mandatory?
4. Should clicked actor poses default to exact placement or nearest valid lane placement?
5. Does V1 need scripted pedestrian crossing, or is constant velocity sufficient?
6. Should controller selection restart nodes or use lifecycle activation?
7. Which parameters should Foxglove expose for each preset: speed, length, amplitude, wavelength, turn radius, or stop distance?
8. Should trajectory presets be stored as sampled points or generated from mathematical parameters?
9. Should V1 support saving an interactive session, or should that remain strictly V2?
10. Which repository branch or PR should be the integration base for the fake planner work?
