# SimForge prototype for WATonomous EVE

Prepared 11 November 2026.

## Objective

Determine whether the locally available SimForge runtime can drive a useful WATonomous regression testing prototype. Start with CPU simulation and a ROS control loop. Then connect the actual WATonomous behavior, planning, and control modules to one traffic scenario.

The requested outcome is evidence that our software can observe a scenario, control the ego vehicle, change the outcome, and repeat the result. A rendered clip or a successful scripted scenario is not sufficient.

This handoff includes proposed source files, setup commands, implementation gates, and a reporting contract. The companion archive contains an `overlay` directory that can be copied into the monorepo. The complete source is also reproduced in the appendix so this Markdown file can be handed to an agent on its own.

## Current status

| Item | Status |
| --- | --- |
| Current WATonomous source inspection | Completed at commit `f6590d7c677548939c5c142620233ddb204662a9` |
| Public Python SDK inspection | Completed against `simforge-gym==0.2.0` |
| Published CLI surface inspection | Reference identifies `0.2.0-rc.9`; verify the actual installed CLI |
| Starter Python package and scripts | Created as a proposed overlay |
| Local checks | Seven checks passed against the published SDK |
| Python package build | Wheel built and installed in an isolated Python environment |
| Docker build and ROS execution | Not executed; Docker and ROS are unavailable in the preparation environment |
| Authenticated runtime and map download | Not executed |
| SimForge episode execution | Not executed |
| EVE stack inside SimForge | Not demonstrated |
| EVE dynamics calibration | Outside this prototype |

Do not describe the source as a verified complete integration. In particular, `contract.template.json` intentionally contains unset runtime state fields. The next agent must discover them from the actual runtime rather than guess array indices or coordinate conventions.

## 1. Establish the checkout and environment

Use an Ubuntu 24.04 x86_64 development machine or an appropriate WatCloud allocation. The initial test needs CPU simulation, Python 3.12, Docker Compose, and ROS 2 Jazzy through the supplied container. Rendering is a later stage.

Read the checkout's applicable `AGENTS.md` instructions. Inspect local changes, create a working branch, and compare the current source with the reference commit above. Preserve unrelated work.

From the checkout root:

```bash
git status --short
git switch -c prototype/simforge
cp -a /path/to/SimForge_Starter/overlay/. .
bash watod_scripts/simforge/sf build
```

Add `.simforge/` to the repository's ignore rules. It contains account state, installed assets, sockets, and generated results. Set a real package maintainer before preparing a PR; the starter package metadata uses an example address.

The starter uses a separate Compose project, `wato_simforge`, so the first smoke test does not require changes to `watod` module registration. Its default ROS domain is 183. Select a free domain on shared machines and use that same domain for every participating process.

The first container uses CycloneDDS. The existing repository defaults to Zenoh. Verify that the WATO images have CycloneDDS before switching them, or add Zenoh and its required configuration to the SimForge image instead. Matching the ROS domain alone is insufficient when the middleware differs.

WatCloud rootless networking needs explicit verification. The supplied Compose file uses host networking for a normal Linux Docker host. If your allocation uses the repository's shared network namespace arrangement, adapt this service to that namespace before connecting it to WATO containers. A Unix socket still connects the simulator and adapter inside their shared mounted directory.

Acceptance: the image builds and the seven package checks pass inside it:

```bash
bash watod_scripts/simforge/sf run python -m pytest /repo/src/simulation/sim_forge/test -q
```

## 2. Install and authenticate SimForge once

Open a container shell:

```bash
bash watod_scripts/simforge/sf run bash
```

Inside it, install the official CLI and authenticate:

```bash
curl -fsSL https://simforge.ai/install.sh -o /tmp/simforge_install.sh
sh /tmp/simforge_install.sh
simforge --version
simforge login --device
simforge runtime status --available
```

The account owner must complete the browser sign in and accept the applicable download agreement. The starter does not contain credentials or a downloaded runtime. Account state persists in the mounted `.simforge/home` directory.

Select an actual runtime version listed by `runtime status --available`, then install that exact version for CPU simulation:

```bash
read -r -p 'Runtime version from the available list: ' SIMFORGE_RUNTIME_VERSION
simforge runtime install --version "$SIMFORGE_RUNTIME_VERSION" --for 2d --accept-license
simforge maps pull richmond-field-station@v9 --for 2d --accept-license
simforge doctor
```

The stable SDK task set inspected for this handoff pins Richmond Field Station v9 with OpenDRIVE SHA256:

```text
531c1bd6f568c56a41fd868ffe1cec0e92bc7dcda9c794c67a8f80a450186b18
```

If that map version is unavailable, record the mismatch. Either obtain the pinned release or deliberately update the SDK task set and revalidate its sites. Do not silently substitute a different map release.

Capture CLI version, runtime status, map information, image ID, Python dependency versions, and the checked out commit with the run evidence. Save machine readable help from the actual installed binary. Some published HTML and Markdown pages describe different stages of development, so the installed runtime is the deciding source.

## 3. Materialize one official scenario

Inside the container:

```bash
python /repo/watod_scripts/simforge/prepare_workspace.py \
  --list --out /repo/.simforge/workspaces/unused

python /repo/watod_scripts/simforge/prepare_workspace.py \
  --scenario lead-hard-brake \
  --out /repo/.simforge/workspaces/lead_brake_authored
```

The helper uses the SDK's bundled template, exact map digest, and pinned site. If a base name matches several sites, it selects the first site by sorted task name and prints the exact selection. Use an explicit printed task name for later reproductions.

This template includes an authored ego braking response. At this stage that is intentional: we are checking installation and episode mechanics. It is not the WATO controller under test.

Start the server in terminal A:

```bash
bash watod_scripts/simforge/sf run simforge env serve \
  /repo/.simforge/workspaces/lead_brake_authored \
  --socket /run/simforge/env.sock \
  --no-sensors --max-sessions 1
```

Use a new socket path for independent runs. If a socket remains after a failed process, verify its owner has stopped before removing it.

Acceptance: the server announces readiness. A compilation failure, missing map, or unavailable runtime is an installation or compatibility blocker, not a failed driving evaluation.

## 4. Run the runtime smoke test

In terminal B:

```bash
bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_probe \
  --socket /run/simforge/env.sock --steps 5 \
  --out /repo/.simforge/results/probe_001

bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_smoke \
  --socket /run/simforge/env.sock --seed 7 --steps 100 \
  --out /repo/.simforge/results/smoke_001
```

The probe saves the real handshake, raw observations, and additional info. The smoke tool resets twice, uses the same seed and authored actions, and compares observations, relevant state, reward, events, and termination. Timing diagnostics are excluded from that comparison.

Acceptance: simulation time advances and both traces match. The report explicitly labels itself `runtime_smoke` and states that WATO driving was not evaluated.

If the published client cannot speak to the installed runtime, inspect the actual protocol and release pair. Update the adapter and dependency pin together, then rerun the checks. Do not use a newer documented recipe that is absent from the installed package: for example, the inspected stable 0.2.0 wheel does not provide the newer `closed_loop` modules shown in some documentation.

## 5. Discover and verify the state and control contracts

Copy the template to `.simforge/contract.json`. Fill its JSON pointers from the probe output. The pointer root is a snapshot containing `observation` and `info`.

Required ego values are planar position, heading, signed forward speed, and simulation time. The following facts must be measured or established from the installed runtime contract:

| Question | Required evidence |
| --- | --- |
| Which fields represent world position? | Raw snapshots plus known movement on the map |
| What are their units and axes? | A straight run and a turn in a known direction |
| What does heading zero mean? | A known forward direction and a left turn |
| Where is the ego origin? | Bounding box center versus axle or other documented reference |
| What does normalized steering mean? | Sign, road wheel interpretation, and limiting angle verified on that runtime |
| Are commands actually authoritative? | Different commands produce different ego trajectories |
| Are other ego behaviors still active? | Audit scenario actions and assistance settings in the active run |

`plane_matrix` maps the two selected raw position axes to ROS map x and y. It must preserve distances; it may include a reflection. `yaw_scale` and `yaw_offset_rad` first express heading in radians in those raw planar axes. The matrix then transforms its direction. `runtime_origin_to_base_m` is the vector from the raw vehicle origin to the ROS base in body coordinates with x forward and y left. Verify this offset against EVE's TF tree.

Do not assume the render rig frame, LiDAR output frame, and live state frame are identical. Do not copy the first two entries of `state_vector` into ROS coordinates simply because those entries exist.

Only after validating those values, set the top level `reviewed` and `control.reviewed` flags to true. `speed_kp` implements a simple uncalibrated adapter speed controller. It is plumbing for converting a target speed into pedals, not EVE's production longitudinal controller or a calibrated drivetrain.

The template leaves steering scale and sign unset. Keep that gate until the actual runtime's conversion is known.

## 6. Close a diagnostic ROS control loop

Start the diagnostic driver in terminal C:

```bash
bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_diagnostic_driver
```

Then start the bridge in terminal B:

```bash
bash watod_scripts/simforge/sf run ros2 run sim_forge simforge_ros_bridge \
  --socket /run/simforge/env.sock \
  --contract /repo/.simforge/contract.json \
  --controller-kind diagnostic --steps 100 \
  --out /repo/.simforge/results/ros_diagnostic_001
```

The bridge publishes `/clock`, `/ego/odom`, `/localization/pose`, and a direct `map` to `base_footprint` ground truth transform. It waits for `/simforge/command` with the current observation timestamp and then advances one simulator decision. The diagnostic driver copies the odometry timestamp into its command.

Check the control loop first with a stop command, then a slow straight command, then small steering commands in both directions. Restart each episode and compare traces. Check for positive forward motion, correct turn sign, and a stationary result under full braking.

The bridge deliberately aborts on missing commands, nonfinite state, unsupported reverse driving, or commands outside the reviewed envelope. Simulation remains paused while waiting for a command. This does not reproduce real actuator or computation delay.

Acceptance: odometry and transforms agree, commands change ego motion, and the resulting timestamps are correct. A diagnostic pass still does not demonstrate the WATO planner.

## 7. Provide complete actor truth and a matching Lanelet2 map

This is the major integration gate for behavior testing.

The inspected SDK exposes compact object observations and object identifiers. Those do not establish that full actor pose, dimensions, semantic class, and signal state are available in `info`. Inspect the live result. If it lacks that truth, investigate the runtime's persistent world truth interface or another documented live actor interface. The Python client includes world subscription support, but this handoff does not claim its payload schema has been verified.

Do not reconstruct accurate object boxes from a compact observation vector with guessed feature meanings. Do not replay authored actor poses as ground truth for a world whose traffic reacts to the ego.

The supplied `Mapper.actors` can consume an actual list of live truth records once configured. Its specification has `reviewed`, `list`, `fields`, and `class_map`. Field pointers are relative to each actor record. Required fields are `id`, `class`, `x`, `y`, `z`, `yaw`, `length`, `width`, and `height`. Traffic lights additionally need a supported `signal_state`. `z` is already in metres above the same vertical map origin. The mapper is planar; sloped or full 3D attitude testing needs a separate extension.

When that truth is verified, the bridge publishes `/simforge/tracked_detections_3d` as `vision_msgs/Detection3DArray`. It preserves actor IDs and publishes classes and signal labels compatible with the existing world model. For the first step it uses current poses; velocity handling and prediction quality still need validation.

Build or convert a Lanelet2 map for exactly the SimForge site being used. Align lane geometry, stop lines, signal associations, projection, and map origin. A map named `oval` from the existing CARLA setup is not a substitute for the Richmond scene.

Inspect the repository's map tools before writing a new converter. An OpenDRIVE conversion must preserve the topology and regulatory elements your world model uses. Overlay the lane boundaries, actor boxes, and ego pose in Foxglove, then check known landmarks numerically.

Create a map configuration overlay using the actual supported keys from `world_modeling_bringup/config/map_configs`. Fill the real map path and measured projection values. Avoid a plausible but unverified geographic origin.

Acceptance: an obstacle occupies the correct lane, a left side vehicle appears on the left, and an intersection's stop lines and signals coincide across simulation and world modeling. If a supported live truth interface or map path cannot be established, report the full stack prototype as blocked with that specific missing capability.

## 8. Connect the actual WATO stack

Relevant existing integration points:

| Existing file | Role |
| --- | --- |
| `src/simulation/simulation_bringup/config/carla_bridge.yaml` | Current simulation control and sensor configuration |
| `src/action/action_bringup/launch/action_sim.launch.yaml` | MPC and planning launch with the current CARLA command destination |
| `src/world_modeling/world_modeling_bringup/launch/world_modeling_sim.launch.yaml` | Ground truth localization mode, prediction, world model, behavior, and costmap |
| `src/interfacing/eve_description` | EVE mesh, TF, axle geometry, and sensor extrinsics |
| `modules/docker-compose.dev.yaml` | Existing editable action and world modeling containers |
| `watod_scripts/watod-setup-env.sh` and `watod` | Module and middleware configuration; inspect current names before editing |

Generate starting launch copies:

```bash
python watod_scripts/simforge/generate_launch.py \
  --repo . \
  --map-config "$PWD/maps/simforge/richmond_world_model.yaml" \
  --output "$PWD/.simforge/launch"
```

This generator changes the tracked detection source and Ackermann destination, selects ground truth localization, and sets simulation time on generated nodes. It does not implement the missing map, truth interface, obstacle cloud, or pipeline barrier.

The isolated SimForge container intentionally contains only the adapter. It cannot launch the action and world modeling packages by itself. Use the existing WATO development containers, build their workspaces, and launch each generated YAML in its corresponding container. Copy generated launches into the relevant mounted `src/.../launch` paths or add explicit mounts so each process can read its file. Map configuration paths must be valid inside those containers too.

For the manual prototype, configure `watod-config.local.sh` with development modules and a shared domain and middleware:

```bash
export ACTIVE_MODULES="world_modeling:dev action:dev infrastructure:dev"
export ROS_DOMAIN_ID=183
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Verify the selected middleware exists in those images, then use your normal `./watod up` and terminal workflow. Development services normally sleep until a launch is started manually. Do not activate the existing `simulation` module alongside this prototype because it starts the CARLA backend and its publishers.

Before launching the full graph, resolve these items:

1. Build and source the correct world modeling and action workspaces using their current development instructions.
2. Launch only one publisher for each localization transform and simulation clock. EIDOS remains disabled during this ground truth stage.
3. Publish the EVE description if required by consumers. The repository generates its URDF from Xacro during the build; the absence of the generated URDF from the source checkout is expected.
4. Satisfy the costmap's obstacle cloud dependency. Either provide a correctly transformed synthetic obstacle cloud from verified actor geometry, or introduce an explicit ground truth object occupancy input and test it. Do not quietly feed an empty cloud while claiming obstacle avoidance coverage.
5. Set an explicit route goal on the matching map and verify the behavior tree requests the intended maneuver.
6. Confirm the chosen controller consumes the current odometry before stamping a command for that tick.
7. Audit and remove ego scripted reactions or automatic assistance in a copied scenario workspace. Keep NPC behavior. Set `scenario_control_reviewed` only after a command authority experiment proves this is effective.

The starter's timestamp check prevents reuse of an earlier timestamp. It is not a complete synchronization barrier for an asynchronous ROS pipeline: a timer could emit a command using older planning inputs while already seeing the new clock. For CI, add a tick acknowledgement or equivalent input provenance through perception or truth ingestion, world modeling, behavior, planning, and control. Verify the final command actually depends on the current tick. Do not claim determinism for the whole stack until this is tested.

After these gates, start the bridge with `--controller-kind wato --publish-actors`. Stop the diagnostic driver first. The WATO mode intentionally refuses to run without a reviewed scenario or actor truth.

Acceptance: the actual behavior and planning graph changes an ego trajectory in response to a live actor. A repeat of the same scenario and stack produces the same pass or fail decision and comparable recorded trajectories under a documented tolerance.

## 9. Score the prototype with explicit coverage

The included scorer checks only collision events, a minimum duration, and nontrivial traveled distance. It rejects diagnostic runs, missing collision event data, incomplete bridge runs, and truncated episodes. A collision recorded in the final observation is included. It currently checks all reported collision events; restrict them to the ego explicitly once event actor identities are verified.

First establish the event kind names and semantics from actual runtime events. Determine whether events are incremental or repeated and whether they include actor identities. The starter checks collision presence; it does not attempt to count unique contacts.

Example after verifying the actual event kind:

```bash
read -r -p 'Verified collision event kind: ' COLLISION_EVENT_KIND
ros2 run sim_forge simforge_score /repo/.simforge/results/wato_001 \
  --collision-kind "$COLLISION_EVENT_KIND" \
  --minimum-duration 10 --minimum-travel 2
```

Exit 0 means the named prototype checks passed. Exit 1 means a driving assertion failed. Exit 2 means the evaluation could not establish its required evidence. Do not rename exit 2 into a successful skipped test.

Next add scenario specific assertions rather than relying on a generic reward. Lead braking needs gap and stopping position; a pedestrian scenario needs conflict region clearance; a lane change needs blind spot occupancy and lane crossing times. Route completion, lane compliance, comfort, and timing require separate implementation and ground truth definitions.

## 10. Automate fresh cases and record provenance

`run_suite.py` is a process orchestration skeleton. For each case it starts a fresh simulator and stack, runs the bridge, scores the trace, and terminates the process groups on success or error. Arguments are arrays and are not evaluated as shell source. It rejects an already existing socket.

`suite.template.json` deliberately starts with `reviewed: false`. Its combined launch expects a ROS environment containing both the action and world modeling packages, which the minimal adapter container does not provide. For the first automated suite, either assemble a suitable evaluation image from the existing WATO build outputs or replace the stack launcher with a supervised launcher for the existing module containers.

If containers are launched by an external command, add explicit Compose cleanup after every case: killing a Docker client process does not stop detached containers. Test cleanup by intentionally crashing the controller. Test that the next case has a fresh behavior tree, costmap, route, command queue, simulator state, and clock. The supplied `combined_launch.template.py` is useful only once a common stack environment exists.

Each result directory should contain:

1. Exact scenario package and digest, selected site, seed, and runtime dynamics identity.
2. WATO commit, local change status or patch, image IDs, SDK and dependency versions.
3. CLI/runtime/map versions and map digests.
4. The reviewed state and control contract.
5. Raw probe, simulator and stack logs, command and trajectory trace.
6. `run.json`, `evaluation.json`, and `junit.xml` with their coverage limits.
7. ROS bag of relevant observations, world objects, behavior, route, trajectory, command, TF, and clock when needed to diagnose failures.

Use an initial manually triggered workflow on an isolated prepared runner. The included CI example is a template, not an enabled workflow. Add pull request gating only after repeatability, lifecycle cleanup, actor truth, map alignment, and command provenance have passed.

## 11. Add sensors and EVE fidelity after the control prototype

A custom Kia visual mesh is not required to prove the initial control loop. The repo already has a Kia mesh reference and a 2.5667 m wheelbase definition. Reuse measured geometry and transforms, but verify physical dimensions before using them for clearance tests.

For rendering, install the 3D runtime, map pack, and assets. Use the GPU Compose override with a supported NVIDIA device and the required graphics capabilities. The official installation guide lists NVIDIA Vulkan, at least 8 GiB GPU memory, and driver 570 or newer for its supported Linux rendering path. Run `simforge doctor` on the chosen host rather than assuming any available GPU qualifies.

The existing CARLA configuration includes a LiDAR vertical field from +15 to −25 degrees. SimForge's sensor guide does not directly support that asymmetric field in its standard rig. It also lists no native IMU or GNSS rig sources. Newer CLI material mentions additional physical LiDAR options; verify their actual availability instead of treating the older guide or newer flags as proof of a complete sensor match.

Use the EVE TF data to construct a rig with carefully converted axes, origins, and intrinsics. A synthetic IMU or GNSS publisher can be added from ground truth with an explicit noise and timing model, but it remains an approximation. Until raw sensor and timing fidelity are established, label EIDOS and perception tests separately from the ground truth behavior suite.

Keep detailed dynamics validation separate. The published full vehicle demonstrations use synthetic parameters and describe restricted or research capabilities. A Kia mesh does not supply mass, inertia, tyre forces, steering lag, braking response, suspension, or drivetrain calibration. Compare a measured EVE steering step, braking run, and acceleration run against simulation before treating simulated distances as predictions for the real car.

## 12. Conclude with an evidence based verdict

The next agent must deliver a short report using one of these outcomes:

| Verdict | Meaning |
| --- | --- |
| Works for runtime smoke | A real local runtime resets, steps, and repeats |
| Works for ROS control prototype | ROS commands control the ego and produce verified state |
| Works for WATO behavior prototype | The actual stack reacts to live traffic with matched maps and reproducible checks |
| Blocked | A named missing interface, account entitlement, release, or environment prevents the next gate |
| Unsuitable for this requirement | A verified technical limitation conflicts with the required test, with supporting evidence |

Report the command used, environment, exact versions, passed gates, failed gates, and artifact paths. Separate simulator installation problems, adapter bugs, nondeterministic orchestration, and genuine driving failures. Include one command that another team member can use to repeat the demonstrated result.

Before considering a CARLA replacement, require equivalent coverage for the tests the team actually uses. A successful SimForge prototype can remain an additional backend even if detailed dynamics or sensor validation stays elsewhere.

## Agent task to paste

Implement and evaluate the SimForge prototype described in this document in the WATonomous monorepo. Read the applicable repository instructions, inspect the current checkout, and apply the proposed overlay on a working branch while preserving unrelated changes. Start with the actual installed CPU runtime and the pinned official task set. Capture the handshake and state schema, run a repeatability smoke test, and establish coordinate and control semantics experimentally. Fill the contract from evidence and prove a ROS control loop. Then attempt the actual world modeling, behavior, planning, and control graph with live actor truth, a matching Lanelet2 map, costmap inputs, exclusive ego command authority, and a valid synchronization strategy. Investigate the installed APIs rather than inventing missing state fields. Keep runtime smoke, diagnostic driving, and WATO behavior evaluation distinct. Do not claim perception, SLAM, or real vehicle dynamics coverage from ground truth tests. Resolve normal implementation issues autonomously. If a required capability cannot be established, document the exact blocker and retain the last working stage. Finish with reproducible commands, source changes, raw run evidence, test results, and one of the explicit verdicts above. Publishing or deployment is outside this task.

## Sources

These are primary sources checked during preparation. Recheck installed release behavior when implementing.

1. [WATonomous repository](https://github.com/WATonomous/wato_monorepo), reference commit recorded above.
2. [SimForge Gym guide](https://simforge.ai/docs/gym) and the [published SDK package](https://pypi.org/project/simforge-gym/0.2.0/). The inspected wheel is the basis for SDK calls in the starter.
3. [Installation](https://simforge.ai/docs/install) and [runtime authentication](https://simforge.ai/docs/sign-in).
4. [Machine readable CLI reference](https://simforge.ai/docs/cli.json).
5. [Sensor rigs](https://simforge.ai/docs/sensors) and [sensor output conventions](https://simforge.ai/docs/sensor-outputs).
6. [Maps](https://simforge.ai/docs/maps), [Worlds](https://simforge.ai/worlds), and [scenario authoring](https://simforge.ai/docs/scenarios).
7. [Cloud restrictions](https://simforge.ai/docs/cloud). Hosted evaluation of an arbitrary custom stack must not be assumed available.
8. [Vehicle dynamics demonstration](https://simforge.ai/news/vehicle-dynamics) and [ego dynamics progress note](https://simforge.ai/news/ego-vehicle-dynamics).

## Complete proposed source

Every following path is relative to the monorepo root. These are proposed additions, not files already committed to WATonomous. The archive contains the same files under `overlay/`.

### `docker/simforge.Dockerfile`

```dockerfile
FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates python3-venv python3-yaml python3-colcon-common-extensions libvulkan1 libegl1 libgl1 \
    ros-jazzy-ackermann-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs \
    ros-jazzy-rosgraph-msgs ros-jazzy-tf2-ros ros-jazzy-vision-msgs \
    ros-jazzy-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/simforge_venv
ENV PATH=/opt/simforge_venv/bin:$PATH
COPY src/simulation/sim_forge/requirements.txt /tmp/simforge_requirements.txt
RUN pip install --no-cache-dir -r /tmp/simforge_requirements.txt pytest==8.3.5
WORKDIR /opt/simforge_ws
COPY src/simulation/sim_forge src/sim_forge
RUN source /opt/ros/jazzy/setup.bash && python /usr/bin/colcon build --packages-select sim_forge
COPY watod_scripts/simforge/container_entrypoint.sh /usr/local/bin/simforge_entrypoint
ARG TASK_UID=1000
ARG TASK_GID=1000
RUN if ! getent group "${TASK_GID}" >/dev/null; then groupadd -g "${TASK_GID}" simforge; fi && \
    if ! getent passwd "${TASK_UID}" >/dev/null; then useradd -m -u "${TASK_UID}" -g "${TASK_GID}" simforge; fi && \
    mkdir -p /home/simforge && chown "${TASK_UID}:${TASK_GID}" /home/simforge && \
    chmod +x /usr/local/bin/simforge_entrypoint
ENV HOME=/home/simforge
ENV PATH=/home/simforge/.local/bin:/opt/simforge_venv/bin:$PATH
USER ${TASK_UID}:${TASK_GID}
WORKDIR /repo
ENTRYPOINT ["/usr/local/bin/simforge_entrypoint"]
CMD ["bash"]
```


### `docs/simforge_ci_example.yaml`

```yaml
# Copy to .github/workflows only after reviewing the local suite and runner.
name: SimForge prototype
on:
  workflow_dispatch:
jobs:
  prototype:
    runs-on: [self-hosted, linux, x64, simforge]
    steps:
      - uses: actions/checkout@v4
      - name: Build isolated adapter
        run: bash watod_scripts/simforge/sf build
      - name: Check adapter contracts
        run: bash watod_scripts/simforge/sf run python -m pytest /repo/src/simulation/sim_forge/test -q
      - name: Run reviewed suite
        run: bash watod_scripts/simforge/sf run python /repo/watod_scripts/simforge/run_suite.py /repo/.simforge/suite.json --out /repo/.simforge/results/ci_${{ github.run_id }}
      - uses: actions/upload-artifact@v4
        if: always()
        with:
          name: simforge_evidence
          path: .simforge/results
```


### `modules/docker-compose.simforge.gpu.yaml`

```yaml
services:
  simforge:
    environment:
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: graphics,compute,utility
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```


### `modules/docker-compose.simforge.yaml`

```yaml
services:
  simforge:
    build:
      context: ..
      dockerfile: docker/simforge.Dockerfile
      args:
        TASK_UID: ${SIMFORGE_UID:-1000}
        TASK_GID: ${SIMFORGE_GID:-1000}
    init: true
    network_mode: host
    environment:
      ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-183}
      RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
      SIMFORGE_RUNTIME_ROOT: /home/simforge/.local/share/simforge/runtime
      SIMFORGE_MAPS_CACHE_ROOT: /home/simforge/.local/share/simforge/maps
    volumes:
      - ..:/repo
      - ../.simforge/home:/home/simforge
      - ../.simforge/run:/run/simforge
```


### `src/simulation/sim_forge/config/contract.template.json`

```json
{
  "reviewed": false,
  "time": "/info/t_s",
  "ego": {"x": null, "y": null, "yaw": null, "speed": null},
  "plane_matrix": [[1, 0], [0, 1]],
  "map_offset_m": [0, 0],
  "yaw_scale": 1,
  "yaw_offset_rad": 0,
  "speed_scale": 1,
  "runtime_origin_to_base_m": [0, 0],
  "actors": null,
  "control": {
    "reviewed": false,
    "max_speed_mps": 3,
    "steer_limit_rad": null,
    "steer_sign": null,
    "speed_kp": 0.4
  },
  "scenario_control_reviewed": false
}
```


### `src/simulation/sim_forge/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sim_forge</name><version>0.1.0</version>
  <description>Proposed WATonomous SimForge adapter and diagnostic tools</description>
  <maintainer email="maintainers@example.com">WATonomous proposed package</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend><exec_depend>ackermann_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend><exec_depend>geometry_msgs</exec_depend>
  <exec_depend>rosgraph_msgs</exec_depend><exec_depend>tf2_ros</exec_depend>
  <exec_depend>vision_msgs</exec_depend>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```


### `src/simulation/sim_forge/requirements.txt`

```text
simforge-gym==0.2.0
```


### `src/simulation/sim_forge/resource/sim_forge`

```text

```


### `src/simulation/sim_forge/setup.cfg`

```ini
[develop]
script_dir=$base/lib/sim_forge
[install]
install_scripts=$base/lib/sim_forge
```


### `src/simulation/sim_forge/setup.py`

```python
from glob import glob
from setuptools import setup

setup(
    name="sim_forge", version="0.1.0", packages=["sim_forge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sim_forge"]),
        ("share/sim_forge", ["package.xml"]),
        ("share/sim_forge/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "simforge_probe = sim_forge.tools:probe",
        "simforge_smoke = sim_forge.tools:smoke",
        "simforge_ros_bridge = sim_forge.bridge:main",
        "simforge_diagnostic_driver = sim_forge.diagnostic:main",
        "simforge_score = sim_forge.score:main",
    ]},
)
```


### `src/simulation/sim_forge/sim_forge/__init__.py`

```python
"""Proposed adapter. No simulator or ROS runtime is imported at package import."""
```


### `src/simulation/sim_forge/sim_forge/bridge.py`

```python
"""One process owns simulation stepping. One invocation is one fresh episode."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

from .contract import Mapper, command_to_control, wrap
from .tools import connect, dump, jsonable


def main():
    # Imports are deferred so the contract and smoke tools work without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_ros import TransformBroadcaster
    from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

    p = argparse.ArgumentParser()
    p.add_argument("--socket", default="/run/simforge/env.sock")
    p.add_argument("--contract", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--command-timeout", type=float, default=10)
    p.add_argument("--controller-kind", choices=("diagnostic", "wato"), default="diagnostic")
    p.add_argument("--publish-actors", action="store_true")
    p.add_argument("--base-frame", default="base_footprint")
    a, ros_args = p.parse_known_args()
    if a.steps < 1 or a.command_timeout <= 0:
        p.error("steps and command timeout must be positive")
    config = json.loads(Path(a.contract).read_text())
    mapper = Mapper(config)
    if a.controller_kind == "wato" and config.get("scenario_control_reviewed") is not True:
        p.error("Audit ego scripted actions and engine assistance before a WATO run")
    if a.controller_kind == "wato" and not a.publish_actors:
        p.error("This initial WATO behavior evaluation requires complete actor truth")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = Node("simforge_bridge")
    # This node uses a monotonic wall deadline, not a timer driven by its own /clock.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    clock_pub = node.create_publisher(Clock, "/clock", qos)
    odom_pub = node.create_publisher(Odometry, "/ego/odom", qos)
    pose_pub = node.create_publisher(PoseStamped, "/localization/pose", qos)
    tracks_pub = node.create_publisher(Detection3DArray, "/simforge/tracked_detections_3d", qos)
    transforms = TransformBroadcaster(node)
    expected_ns = None
    command = None
    def callback(msg):
        nonlocal command
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if expected_ns is not None and stamp == expected_ns:
            command = msg
    subscription = node.create_subscription(AckermannDriveStamped, "/simforge/command", callback, qos)
    env = None
    status = {"kind": "ros_control_prototype", "controller_kind": a.controller_kind,
              "completed": False, "seed": a.seed,
              "actor_truth_enabled": a.publish_actors,
              "checks": ["command timestamps", "state mapping"],
              "limitations": ["adapter speed P controller is uncalibrated",
                              "no proof of full pipeline synchronization",
                              "not a vehicle dynamics validation"]}
    journal = (out / "trace.jsonl").open("w")
    previous = None
    try:
        env = connect(a.socket)
        dump(out / "hello.json", env.hello)
        dump(out / "contract.json", config)
        obs, info = env.reset(seed=a.seed)
        terminal, truncated = False, False
        for tick in range(a.steps+1):
            snapshot = jsonable({"observation": obs, "info": info})
            ego = mapper.ego(snapshot)
            actors = mapper.actors(snapshot) if a.publish_actors else []
            if previous is not None and ego.t <= previous.t:
                raise RuntimeError("Nonincreasing simulation time")
            yaw_rate = 0.0 if previous is None else wrap(ego.yaw-previous.yaw)/(ego.t-previous.t)
            row = {"tick": tick, "ego": asdict(ego), "events": info.get("events"),
                   "terminated": terminal, "truncated": truncated}
            expected_ns = round(ego.t*1_000_000_000)
            if expected_ns < 0:
                raise RuntimeError("ROS prototype expects nonnegative simulation time")
            sec, nanosec = divmod(expected_ns, 1_000_000_000)
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = "map", a.base_frame
            odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nanosec
            odom.pose.pose.position.x, odom.pose.pose.position.y = ego.x, ego.y
            odom.pose.pose.orientation.z = math.sin(ego.yaw/2)
            odom.pose.pose.orientation.w = math.cos(ego.yaw/2)
            odom.twist.twist.linear.x = ego.speed
            odom.twist.twist.angular.z = yaw_rate
            pose = PoseStamped()
            pose.header, pose.pose = odom.header, odom.pose.pose
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, a.base_frame
            tf.transform.translation.x, tf.transform.translation.y = ego.x, ego.y
            tf.transform.rotation = odom.pose.pose.orientation
            detections = Detection3DArray()
            detections.header = odom.header
            for actor in actors:
                if actor["id"] == str(env.ego):
                    continue
                det = Detection3D()
                det.header, det.id = odom.header, actor["id"]
                det.bbox.center.position.x = actor["x"]
                det.bbox.center.position.y = actor["y"]
                det.bbox.center.position.z = actor["z"]
                det.bbox.center.orientation.z = math.sin(actor["yaw"]/2)
                det.bbox.center.orientation.w = math.cos(actor["yaw"]/2)
                det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = actor["length"], actor["width"], actor["height"]
                labels = [actor["class"]]
                if actor["class"] == "traffic_light":
                    labels.append(actor["signal_state"])
                for label in labels:
                    result = ObjectHypothesisWithPose()
                    result.hypothesis.class_id, result.hypothesis.score = label, 1.0
                    result.pose.pose = det.bbox.center
                    det.results.append(result)
                detections.detections.append(det)
            command = None
            odom_pub.publish(odom)
            pose_pub.publish(pose)
            transforms.sendTransform(tf)
            if a.publish_actors:
                tracks_pub.publish(detections)
            clock = Clock()
            clock.clock = odom.header.stamp
            clock_pub.publish(clock)
            if terminal or truncated or tick == a.steps:
                journal.write(json.dumps(row, allow_nan=False)+"\n")
                journal.flush()
                status.update(completed=True, end="terminated" if terminal else "truncated" if truncated else "horizon")
                break
            deadline = time.monotonic()+a.command_timeout
            while command is None and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.05, max(0, deadline-time.monotonic())))
            if command is None:
                raise TimeoutError(f"No command stamped {expected_ns} for tick {tick}")
            control = command_to_control(command.drive.speed, command.drive.steering_angle, ego.speed, config["control"])
            row.update(command={"speed": command.drive.speed, "steering": command.drive.steering_angle}, control=control)
            journal.write(json.dumps(row, allow_nan=False)+"\n")
            journal.flush()
            previous = ego
            obs, _, terminal, truncated, info = env.step(control)
    except Exception as exc:
        status["error"] = str(exc)
        raise
    finally:
        journal.close()
        if env is not None:
            env.close()
        dump(out / "run.json", status)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
```


### `src/simulation/sim_forge/sim_forge/contract.py`

```python
"""Explicit field mapping. Never infer simulator coordinates or state array indices."""
from dataclasses import dataclass
import math


class ContractError(ValueError):
    pass


def at(document, pointer):
    if not isinstance(pointer, str) or not pointer.startswith("/"):
        raise ContractError(f"Configure a JSON pointer, got {pointer!r}")
    value = document
    try:
        for token in pointer[1:].split("/"):
            token = token.replace("~1", "/").replace("~0", "~")
            value = value[int(token)] if isinstance(value, (list, tuple)) else value[token]
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise ContractError(f"Runtime field missing: {pointer}") from exc
    return value


def finite(value):
    if isinstance(value, bool):
        raise ContractError("Boolean is not a numeric state")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"Expected numeric state, got {value!r}") from exc
    if not math.isfinite(number):
        raise ContractError("Nonfinite state")
    return number


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Ego:
    t: float
    x: float
    y: float
    yaw: float
    speed: float


class Mapper:
    def __init__(self, config):
        if config.get("reviewed") is not True:
            raise ContractError("Capture probe output and review contract.json before bridging")
        self.c = config
        self.m = config["plane_matrix"]
        if not isinstance(self.m, list) or len(self.m) != 2 or any(len(row) != 2 for row in self.m):
            raise ContractError("plane_matrix must be a 2 by 2 orthogonal matrix")
        self.m = [[finite(v) for v in row] for row in self.m]
        a, b = self.m
        if not (math.isclose(sum(v*v for v in a), 1, abs_tol=1e-6)
                and math.isclose(sum(v*v for v in b), 1, abs_tol=1e-6)
                and math.isclose(sum(x*y for x, y in zip(a, b)), 0, abs_tol=1e-6)):
            raise ContractError("plane_matrix must preserve distances")

    def position(self, x, y):
        a, b = self.m
        o = self.c["map_offset_m"]
        return a[0]*x+a[1]*y+finite(o[0]), b[0]*x+b[1]*y+finite(o[1])

    def heading(self, raw):
        h = finite(raw)*finite(self.c["yaw_scale"])+finite(self.c["yaw_offset_rad"])
        a, b = self.m
        return math.atan2(b[0]*math.cos(h)+b[1]*math.sin(h),
                          a[0]*math.cos(h)+a[1]*math.sin(h))

    def ego(self, snapshot):
        fields = self.c["ego"]
        x, y = self.position(finite(at(snapshot, fields["x"])), finite(at(snapshot, fields["y"])))
        yaw = self.heading(at(snapshot, fields["yaw"]))
        # Offset from the runtime reference to the ROS base in vehicle FLU coordinates.
        dx, dy = map(finite, self.c["runtime_origin_to_base_m"])
        x += math.cos(yaw)*dx-math.sin(yaw)*dy
        y += math.sin(yaw)*dx+math.cos(yaw)*dy
        speed = finite(at(snapshot, fields["speed"]))*finite(self.c["speed_scale"])
        return Ego(finite(at(snapshot, self.c["time"])), x, y, yaw, speed)

    def actors(self, snapshot):
        spec = self.c.get("actors")
        if not spec or spec.get("reviewed") is not True:
            raise ContractError("Full actor truth is unconfigured; do not fabricate boxes from compact observations")
        actors = at(snapshot, spec["list"])
        if not isinstance(actors, list):
            raise ContractError("Actor truth must be a list")
        output = []
        for actor in actors:
            row = {key: at(actor, pointer) for key, pointer in spec["fields"].items()}
            row["x"], row["y"] = self.position(finite(row["x"]), finite(row["y"]))
            row["yaw"] = self.heading(row["yaw"])
            for field in ("z", "length", "width", "height"):
                row[field] = finite(row[field])
            if min(row[k] for k in ("length", "width", "height")) <= 0:
                raise ContractError("Actor dimensions must be positive")
            raw_class = str(row["class"])
            if raw_class not in spec["class_map"]:
                raise ContractError(f"Unmapped actor class {raw_class}")
            row["class"] = spec["class_map"][raw_class]
            if row["class"] == "traffic_light":
                if row.get("signal_state") not in ("red", "yellow", "green"):
                    raise ContractError("Traffic light lacks a supported signal state")
            row["id"] = str(row["id"])
            output.append(row)
        return output


def command_to_control(speed, steering, ego_speed, config):
    if config.get("reviewed") is not True:
        raise ContractError("Review steering scale and sign with the actual runtime first")
    speed, steering, ego_speed = map(finite, (speed, steering, ego_speed))
    maximum = finite(config["max_speed_mps"])
    limit = finite(config["steer_limit_rad"])
    sign = finite(config["steer_sign"])
    kp = finite(config["speed_kp"])
    if maximum <= 0 or limit <= 0 or kp <= 0 or sign not in (-1, 1):
        raise ContractError("Invalid command conversion parameters")
    if speed < 0 or speed > maximum or abs(steering) > limit:
        raise ContractError("Command exceeds the reviewed forward driving envelope")
    effort = max(-1.0, min(1.0, kp*(speed-ego_speed)))
    throttle, brake = max(0.0, effort), max(0.0, -effort)
    if speed == 0:
        throttle, brake = 0.0, 1.0
    return [throttle, brake, sign*steering/limit]
```


### `src/simulation/sim_forge/sim_forge/diagnostic.py`

```python
"""An odometry stamped constant command for plumbing checks. Not the WATO planner."""
def main():
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = Node("simforge_diagnostic_driver")
    node.declare_parameter("speed_mps", 1.0)
    node.declare_parameter("steering_rad", 0.0)
    pub = node.create_publisher(AckermannDriveStamped, "/simforge/command", 10)
    def odometry(msg):
        cmd = AckermannDriveStamped()
        cmd.header = msg.header
        cmd.drive.speed = float(node.get_parameter("speed_mps").value)
        cmd.drive.steering_angle = float(node.get_parameter("steering_rad").value)
        pub.publish(cmd)
    node.create_subscription(Odometry, "/ego/odom", odometry, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```


### `src/simulation/sim_forge/sim_forge/score.py`

```python
"""Two explicit prototype checks. Never claim route or localization coverage."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .tools import dump


def assess(run, rows, *, minimum_duration, minimum_travel, collision_kinds):
    if not run.get("completed") or run.get("controller_kind") != "wato":
        raise ValueError("Only a completed WATO controlled episode may be scored as WATO driving")
    if not run.get("actor_truth_enabled"):
        raise ValueError("Missing actor truth")
    if run.get("end") == "truncated":
        raise ValueError("Truncated episode requires review and cannot pass these checks")
    if not collision_kinds or len(rows) < 2:
        raise ValueError("Configure reviewed collision event kinds and capture transitions")
    events = []
    for row in rows:
        if not isinstance(row.get("events"), list):
            raise ValueError("Collision event channel unavailable; cannot report zero collisions")
        for event in row["events"]:
            if not isinstance(event, dict) or not isinstance(event.get("kind"), str):
                raise ValueError("Invalid event contract")
        events.extend(row["events"])
    times = [float(row["ego"]["t"]) for row in rows]
    if not all(math.isfinite(t) for t in times) or not all(b > a for a, b in zip(times, times[1:])):
        raise ValueError("Invalid or nonincreasing simulation time")
    duration = times[-1]-times[0]
    travel = sum(math.hypot(b["ego"]["x"]-a["ego"]["x"], b["ego"]["y"]-a["ego"]["y"])
                 for a, b in zip(rows, rows[1:]))
    if not math.isfinite(travel):
        raise ValueError("Nonfinite trajectory")
    collision_seen = any(event["kind"] in collision_kinds for event in events)
    checks = {"no_collision_event": not collision_seen,
              "minimum_duration": duration >= minimum_duration,
              "minimum_travel": travel >= minimum_travel}
    return {"passed": all(checks.values()), "checks": checks,
            "duration_s": duration, "travel_m": travel,
            "coverage": "prototype collision events and nontrivial motion only",
            "unmeasured": ["route completion", "lane compliance", "stop line compliance",
                           "comfort", "real vehicle dynamics", "perception", "SLAM"]}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("episode")
    p.add_argument("--collision-kind", action="append", required=True)
    p.add_argument("--minimum-duration", type=float, required=True)
    p.add_argument("--minimum-travel", type=float, required=True)
    a = p.parse_args()
    if a.minimum_duration <= 0 or a.minimum_travel <= 0:
        p.error("Set positive duration and travel thresholds for this scenario")
    root = Path(a.episode)
    code = 2
    try:
        run = json.loads((root / "run.json").read_text())
        rows = [json.loads(line) for line in (root / "trace.jsonl").read_text().splitlines()]
        report = assess(run, rows, minimum_duration=a.minimum_duration,
                        minimum_travel=a.minimum_travel, collision_kinds=set(a.collision_kind))
        code = 0 if report["passed"] else 1
    except Exception as exc:
        report = {"passed": False, "error": str(exc), "status": "evaluation_error"}
    dump(root / "evaluation.json", report)
    suite = ET.Element("testsuite", name="simforge_prototype", tests="1",
                       failures=str(int(code == 1)), errors=str(int(code == 2)))
    case = ET.SubElement(suite, "testcase", name="collision_and_motion")
    if code:
        ET.SubElement(case, "error" if code == 2 else "failure").text = json.dumps(report)
    ET.ElementTree(suite).write(root / "junit.xml", encoding="utf-8", xml_declaration=True)
    print(json.dumps(report))
    raise SystemExit(code)
```


### `src/simulation/sim_forge/sim_forge/tools.py`

```python
"""Inspect the real SDK contract and test simulator repeatability, without ROS."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path


def jsonable(value):
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, bytes):
        return {"bytes": len(value), "sha256": hashlib.sha256(value).hexdigest()}
    return value


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + "\n")


def connect(path, *, timeout=20.0):
    from simforge_gym import SimForgeEnv, SocketTransport
    version = importlib.metadata.version("simforge-gym")
    if version != "0.2.0":
        raise RuntimeError(f"Review this adapter before using simforge-gym {version}")
    return SimForgeEnv(SocketTransport(path, timeout=timeout),
                       action_mode="control", observation="state", info_channel=True)


def args(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--socket", default="/run/simforge/env.sock")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--out", required=True)
    return parser


def probe():
    p = args("Capture the runtime's actual state contract. Authored control only.")
    a = p.parse_args()
    env = connect(a.socket)
    root = Path(a.out)
    try:
        dump(root / "hello.json", env.hello)
        obs, info = env.reset(seed=a.seed)
        dump(root / "reset.json", {"observation": obs, "info": info})
        for i in range(a.steps):
            obs, reward, terminated, truncated, info = env.step(None)
            dump(root / f"step_{i:04d}.json", {
                "observation": obs, "info": info, "reward": reward,
                "terminated": terminated, "truncated": truncated,
            })
            if terminated or truncated:
                break
    finally:
        env.close()
    print(f"Contract captured in {root}; this was not a WATO driving evaluation.")


def episode(env, seed, steps):
    obs, info = env.reset(seed=seed)
    rows = []
    def append(reward=None, terminated=False, truncated=False):
        # Deliberately exclude wall time and render timing from repeatability.
        rows.append(jsonable({"observation": obs, "info": {
            k: info[k] for k in ("t_s", "ego", "object_ids", "reward_terms", "events")
            if k in info}, "reward": reward,
            "terminated": terminated, "truncated": truncated}))
    append()
    for _ in range(steps):
        obs, reward, terminated, truncated, info = env.step(None)
        append(reward, terminated, truncated)
        if terminated or truncated:
            break
    return rows


def digest(rows):
    data = json.dumps(rows, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(data).hexdigest()


def check_repeatability(first, second):
    if len(first) < 2:
        raise ValueError("Episode returned no transitions")
    for rows in (first, second):
        times = [float(row["info"]["t_s"]) for row in rows]
        if not all(b > a for a, b in zip(times, times[1:])):
            raise ValueError("Simulation time did not advance monotonically")
    if digest(first) != digest(second):
        raise ValueError("Same seed and authored actions produced different traces")


def smoke():
    a = args("Check reset and same seed repeatability; not WATO driving quality").parse_args()
    if a.steps < 1:
        raise ValueError("steps must be positive")
    root = Path(a.out)
    env = connect(a.socket)
    report = {"kind": "runtime_smoke", "wato_driving_evaluated": False,
              "seed": a.seed, "gym_version": importlib.metadata.version("simforge-gym"),
              "passed": False}
    try:
        dump(root / "hello.json", env.hello)
        first = episode(env, a.seed, a.steps)
        second = episode(env, a.seed, a.steps)
        dump(root / "first.json", first)
        dump(root / "second.json", second)
        check_repeatability(first, second)
        report.update(passed=True, trace_sha256=digest(first), samples=len(first))
    except Exception as exc:
        report["error"] = str(exc)
        raise
    finally:
        env.close()
        dump(root / "summary.json", report)
    print(json.dumps(report))
```


### `src/simulation/sim_forge/test/test_contract_and_evaluation.py`

```python
import copy
import math
import pytest
from sim_forge.contract import ContractError, Mapper, command_to_control
from sim_forge.score import assess
from sim_forge.tools import check_repeatability


def contract():
    return {"reviewed": True, "time": "/info/t_s",
            "ego": {"x": "/info/ego/x", "y": "/info/ego/y",
                    "yaw": "/info/ego/yaw", "speed": "/info/ego/speed"},
            "plane_matrix": [[1, 0], [0, -1]], "map_offset_m": [10, 20],
            "yaw_scale": 1, "yaw_offset_rad": 0, "speed_scale": 1,
            "runtime_origin_to_base_m": [-1, 0]}


def test_reflection_and_reference_offset_together():
    raw = {"info": {"t_s": 1, "ego": {"x": 2, "y": 3, "yaw": math.pi/2, "speed": 4}}}
    ego = Mapper(contract()).ego(raw)
    assert (ego.x, ego.y) == pytest.approx((12, 18))
    assert ego.yaw == pytest.approx(-math.pi/2)
    assert ego.speed == 4


def test_missing_coordinates_and_unreviewed_profiles_do_not_default_to_zero():
    with pytest.raises(ContractError):
        Mapper({"reviewed": False})
    with pytest.raises(ContractError):
        Mapper(contract()).ego({"info": {"t_s": 1, "ego": {}}})
    with pytest.raises(ContractError):
        Mapper(contract()).actors({})


def test_reverse_and_nonfinite_commands_are_rejected():
    cfg = {"reviewed": True, "max_speed_mps": 3, "steer_limit_rad": 0.5,
           "steer_sign": -1, "speed_kp": 0.4}
    assert command_to_control(1, 0.25, 0, cfg) == pytest.approx([0.4, 0, -0.5])
    assert command_to_control(0, 0, 2, cfg) == [0, 1, 0]
    for speed in (-1, float("nan"), 4):
        with pytest.raises(ContractError):
            command_to_control(speed, 0, 0, cfg)


def trace():
    return [{"ego": {"t": 0, "x": 0, "y": 0}, "events": []},
            {"ego": {"t": 1, "x": 2, "y": 0}, "events": []}]


def evaluate(rows, kind="wato"):
    return assess({"completed": True, "controller_kind": kind, "actor_truth_enabled": True},
                  rows, minimum_duration=1, minimum_travel=1, collision_kinds={"collision"})


def test_collision_in_final_frame_fails_and_missing_channel_is_an_error():
    rows = trace()
    rows[-1]["events"] = [{"kind": "collision"}]
    assert evaluate(rows)["passed"] is False
    rows[-1]["events"] = None
    with pytest.raises(ValueError, match="unavailable"):
        evaluate(rows)


def test_parked_car_and_diagnostic_driver_cannot_pass_driving_checks():
    rows = trace()
    rows[-1]["ego"]["x"] = 0
    assert evaluate(rows)["passed"] is False
    with pytest.raises(ValueError, match="WATO"):
        evaluate(trace(), "diagnostic")


def test_reset_repeatability_detects_changes_and_frozen_time():
    first = [{"info": {"t_s": 0}}, {"info": {"t_s": 1}}]
    check_repeatability(first, copy.deepcopy(first))
    second = copy.deepcopy(first)
    second[-1]["info"]["t_s"] = 2
    with pytest.raises(ValueError, match="different traces"):
        check_repeatability(first, second)
    with pytest.raises(ValueError, match="monotonically"):
        check_repeatability([first[0], first[0]], [first[0], first[0]])
```


### `src/simulation/sim_forge/test/test_sdk_contract.py`

```python
"""Actual published SDK against a transport fixture. No simulator fidelity claim."""
import numpy as np
from simforge_gym import SimForgeEnv


class TransportFixture:
    def __init__(self):
        self.last_action = None
        self.closed = False
    def hello(self):
        return {"actionFields": ["target_speed_mps", "target_acceleration_mps2", "throttle", "brake", "steer"],
                "actionWidth": 5, "ego": "ego", "decisionHz": 10,
                "maxObjects": 1, "objectFeatures": 5, "stateVectorSize": 4}
    def response(self):
        state = np.array([1, 2, 3, 4], dtype="<f8").tobytes()
        objects = np.zeros((1, 5), dtype="<f4").tobytes()
        return {"observation": {
                    "state_vector": {"offset": 0, "length": 32, "dtype": "<f8", "shape": [4]},
                    "objects": {"offset": 32, "length": 20, "dtype": "<f4", "shape": [1, 5]}},
                "info": {"t_s": 1, "ego": {}, "object_ids": [], "reward_terms": {}, "events": []},
                "reward": 0, "terminated": False, "truncated": False}, state+objects
    def reset(self, seed=None, *, scenario=None):
        return self.response()
    def step(self, row):
        self.last_action = row
        return self.response()
    def close(self):
        self.closed = True


def test_published_sdk_applies_controls_by_named_slots_and_exposes_events():
    transport = TransportFixture()
    env = SimForgeEnv(transport, action_mode="control", observation="state", info_channel=True)
    obs, info = env.reset(seed=7)
    assert obs["state_vector"].tolist() == [1, 2, 3, 4]
    assert info["events"] == []
    env.step([0.2, 0.0, -0.5])
    assert np.isnan(transport.last_action[:2]).all()
    assert transport.last_action[2:].tolist() == [0.2, 0, -0.5]
    env.close()
    assert transport.closed
```


### `watod_scripts/simforge/combined_launch.template.py`

```python
"""After map and truth gates: include generated launch copies in one process."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    root = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(root, name)))
        for name in ("world_modeling_simforge.launch.yaml", "action_simforge.launch.yaml")
    ])
```


### `watod_scripts/simforge/container_entrypoint.sh`

```bash
#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/simforge_ws/install/setup.bash
exec "$@"
```


### `watod_scripts/simforge/generate_launch.py`

```python
"""Generate reviewed starting copies of existing launches. Does not prove stack readiness."""
import argparse
from pathlib import Path
import yaml

p = argparse.ArgumentParser()
p.add_argument("--repo", default=".")
p.add_argument("--map-config", required=True)
p.add_argument("--output", required=True)
a = p.parse_args()
root, out = Path(a.repo).resolve(), Path(a.output).resolve()
if out.exists() and any(out.iterdir()):
    raise FileExistsError("Choose an empty output directory")
out.mkdir(parents=True, exist_ok=True)
world = yaml.safe_load((root / "src/world_modeling/world_modeling_bringup/launch/world_modeling_sim.launch.yaml").read_text())
action = yaml.safe_load((root / "src/action/action_bringup/launch/action_sim.launch.yaml").read_text())
for entry in world["launch"]:
    arg = entry.get("arg")
    if arg and arg["name"] == "map_config_file":
        arg["default"] = str(Path(a.map_config).resolve())
    if arg and arg["name"] == "use_eidos":
        arg["default"] = "false"
for document in (world, action):
    for entry in document["launch"]:
        node = entry.get("node")
        if node:
            node.setdefault("param", []).append({"name": "use_sim_time", "value": True})
            for remap in node.get("remap", []):
                if remap.get("to") == "/carla/tracked_detections_3d":
                    remap["to"] = "/simforge/tracked_detections_3d"
                if remap.get("to") == "/carla/ackermann_control/command":
                    remap["to"] = "/simforge/command"
for filename, data in (("world_modeling_simforge.launch.yaml", world), ("action_simforge.launch.yaml", action)):
    (out / filename).write_text(yaml.safe_dump(data, sort_keys=False))
print(f"Generated starting launches in {out}; finish map, truth, costmap and synchronization gates in the handoff")
```


### `watod_scripts/simforge/prepare_workspace.py`

```python
"""Use the installed SDK's own template and map pin rather than inventing scenario JSON."""
import argparse
import json
from pathlib import Path
from simforge_gym import ScenarioSet, template_workspace

p = argparse.ArgumentParser()
p.add_argument("--task-set", default="richmond-field-station")
p.add_argument("--scenario", default="lead-hard-brake")
p.add_argument("--out", required=True)
p.add_argument("--list", action="store_true")
a = p.parse_args()
tasks = ScenarioSet.bundled(a.task_set)
if a.list:
    print(json.dumps({"map": tasks.map.__dict__, "scenarios": [s.name for s in tasks.scenarios]}, indent=2))
else:
    candidates = [s for s in tasks.scenarios if s.name == a.scenario or s.name.split("@")[0] == a.scenario]
    if not candidates:
        raise ValueError("No matching task. Run with --list first")
    scenario = sorted(candidates, key=lambda s: s.name)[0]
    if scenario.template is None or tasks.map is None:
        raise ValueError("Expected a template task with an exact map pin")
    path = Path(a.out)
    if path.exists() and any(path.iterdir()):
        raise FileExistsError("Choose a new empty workspace")
    path = template_workspace(scenario.template, None, path,
        map_id=tasks.map.name, xodr_sha256=tasks.map.xodr_sha256, site=scenario.site)
    print(json.dumps({"workspace": str(path), "map": tasks.map.__dict__,
                      "scenario": scenario.name, "control": "bundled authored ego; diagnostic only"}, indent=2))
```


### `watod_scripts/simforge/run_suite.py`

```python
"""Sequential cases with fresh server and stack process groups, including cleanup on failure."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop(process):
    if process is None:
        return
    # Signal the group even when its original parent exited.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def execute(spec, out):
    out.mkdir(parents=True, exist_ok=False)
    processes, logs = [], []
    try:
        for name in ("server", "stack"):
            argv = spec[name+"_argv"]
            if not isinstance(argv, list) or not argv or not all(isinstance(x, str) for x in argv):
                raise ValueError(f"{name}_argv must be an argument array")
            log = (out / f"{name}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            if name == "server":
                deadline = time.monotonic()+60
                while not Path(spec["socket"]).is_socket():
                    if processes[-1].poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError("Server did not create its socket; inspect server.log")
                    time.sleep(0.1)
        # Placeholders only replace entire arguments. No shell evaluation.
        for name in ("bridge", "score"):
            argv = [str(out) if item == "{out}" else item for item in spec[name+"_argv"]]
            log = (out / f"{name}.log").open("w")
            logs.append(log)
            proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(proc)
            code = proc.wait(timeout=spec.get("wall_timeout_s", 300))
            if code:
                return {"status": "failed" if name == "score" and code == 1 else "error", "stage": name, "exit_code": code}
        if any(process.poll() is not None for process in processes[:2]):
            return {"status": "error", "reason": "server or stack exited unexpectedly"}
        return {"status": "passed"}
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("suite")
    p.add_argument("--out", required=True)
    a = p.parse_args()
    suite = json.loads(Path(a.suite).read_text())
    if suite.get("reviewed") is not True or not suite.get("cases"):
        raise ValueError("Provide a reviewed nonempty suite after individual prototype gates pass")
    root = Path(a.out).resolve()
    root.mkdir(parents=True, exist_ok=False)
    results = []
    for i, case in enumerate(suite["cases"]):
        socket = Path(case["socket"])
        if socket.exists():
            raise FileExistsError(f"Refusing a stale or active socket: {socket}")
        try:
            result = execute(case, root / f"case_{i:03d}")
        except Exception as exc:
            result = {"status": "error", "reason": str(exc)}
        results.append({"name": case["name"], **result})
    (root / "suite.json").write_text(json.dumps(results, indent=2)+"\n")
    raise SystemExit(0 if all(row["status"] == "passed" for row in results) else 1)


if __name__ == "__main__":
    main()
```


### `watod_scripts/simforge/sf`

```bash
#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_dir"
mkdir -p .simforge/home .simforge/run .simforge/results .simforge/workspaces
export SIMFORGE_UID="$(id -u)"
export SIMFORGE_GID="$(id -g)"
compose=(docker compose -p wato_simforge -f modules/docker-compose.simforge.yaml)
action="${1:-help}"
if (($#)); then shift; fi
case "$action" in
  build) "${compose[@]}" build simforge ;;
  run) "${compose[@]}" run --rm simforge "$@" ;;
  *) printf '%s\n' 'Usage: bash watod_scripts/simforge/sf build' '       bash watod_scripts/simforge/sf run COMMAND [ARGS...]'; exit 2 ;;
esac
```


### `watod_scripts/simforge/suite.template.json`

```json
{
  "reviewed": false,
  "cases": [{
    "name": "reviewed_lead_brake_seed_7",
    "socket": "/run/simforge/case_000.sock",
    "wall_timeout_s": 300,
    "server_argv": ["simforge", "env", "serve", "/repo/.simforge/workspaces/lead_brake_wato", "--socket", "/run/simforge/case_000.sock", "--no-sensors", "--max-sessions", "1"],
    "stack_argv": ["ros2", "launch", "/repo/.simforge/launch/combined_simforge.launch.py"],
    "bridge_argv": ["ros2", "run", "sim_forge", "simforge_ros_bridge", "--socket", "/run/simforge/case_000.sock", "--contract", "/repo/.simforge/contract.json", "--out", "{out}", "--seed", "7", "--steps", "200", "--controller-kind", "wato", "--publish-actors"],
    "score_argv": ["ros2", "run", "sim_forge", "simforge_score", "{out}", "--collision-kind", "REPLACE_WITH_VERIFIED_EVENT_KIND", "--minimum-duration", "10", "--minimum-travel", "2"]
  }]
}
```
