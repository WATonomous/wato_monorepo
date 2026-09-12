# Running the Ring Road HD map in CARLA

This is the companion guide to running the UWaterloo Ring Road through the
WATO CARLA sim stack. Three artifacts work together:

| File | Purpose |
| --- | --- |
| `ringroad_carla.xodr` | OpenDRIVE road network loaded into CARLA. The road centerline is the **exact** coordinate polyline used to build `ringroad_utm.osm`. |
| `ringroad_sim.osm` | Copy of the lanelet map shifted so the ring center is `(0, 0)` — the CARLA world origin. Use this map, not the UTM one, in simulation. |
| `world_model_ring_road_sim.yaml` | `world_model` config pointing at `ringroad_sim.osm` with the `local_cartesian` projector (identity frame, == CARLA world). |

Regenerate if you ever change `build_lanelet_map.py` (both artifacts are rebuilt
deterministically by `build_carla_xodr.py`).

## Why the map has to be centered

CARLA always places an OpenDRIVE world's origin at `(0, 0)` of the Unreal
coordinate system. `ringroad_carla.xodr` is already centered on the ring-center
UTM point (`536942.221, 4813183.048` = map origin `43.47065, -80.54326`).
Because the road network is centered, the ego's CARLA odometry comes out in
ring-center-relative meters, so `ringroad_sim.osm` (same shift) guarantees the
lanelet map and the CARLA road surface coincide 1:1 — no second transform,
no `utm` projector needed.

## Requirements

- **A machine with a discrete NVIDIA GPU running Linux** (bare or WSL2), the
  NVIDIA driver, and `nvidia-container-toolkit` (WSL2: install the toolkit
  inside the WSL distro with `apt install nvidia-container-toolkit`).
  `carla_sim` runs `./CarlaUnreal.sh -RenderOffscreen -quality-level=Epic`.
- Docker with GPU support (`docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi` must succeed).
- WATO monorepo, `carla_sim` built (`image: carlasim/carla:0.10.0` is pulled
  by compose — no local build needed).
- No GPU / just a sanity check? `carla_sim_no_gpu` exists (`-nullrhi`, no
  rendering, no cameras) — enough to prove the world loads and `world_model`
  produces markers.

## 1. Add the map to the mount

The maps directory is bind-mounted into containers at `/opt/watonomous/maps`
(`maps/` on the host). Put the artifacts in place:

```bash
# on the host (path is usually ${MONO_DIR}/maps in the WATO dev environment)
mkdir -p maps/ring_road/lanelet
cp tools/maps/ring_road/ringroad_sim.osm   maps/ring_road/lanelet/ringroad_sim.osm
cp tools/maps/ring_road/ringroad_carla.xodr maps/ring_road/lanelet/ringroad_carla.xodr
```

## 2. Start CARLA and load the ring road world

Start the server (this can run on its own machine; the bridge connects to it):

```bash
docker compose --profile carla_gpu up carla_sim
```

Then load the OpenDRIVE world with the CARLA Python client (same wheel the
repo pins: `src/simulation/carla_sim/dist/carla-0.10.0-cp312-cp312-linux_x86_64.whl`,
or run from any container image that installs it):

```python
import carla

client = carla.Client("localhost", 2000)  # CARLA_PORT if you override it
client.set_timeout(30.0)

xml = open("/opt/watonomous/maps/ring_road/lanelet/ringroad_carla.xodr").read()
world = client.generate_opendrive_world(xml, {"max_road_length": 500})
world.wait_for_tick()

# sanity: map name flips to /OpenDRIVE and a waypoint exists on road 1
print(world.get_map().name)            # /OpenDRIVE
print(world.get_map().get_waypoint_xodr(1, -1, 5.0))  # first ring road waypoint

# put the ego on the road near ring-road seg0
bp = world.get_blueprint_library().filter("vehicle.*")[0]   # pick a model
spectator_transform = world.get_map().get_waypoint_xodr(1, -1, 8.0).transform
ego = world.spawn_actor(bp, spectator_transform)
world.get_spectator().set_transform(spectator_transform)
```

> The bridge's stock `light_traffic_scenario` calls `load_world("Town10HD")`,
> which overwrites our world. For a first ring-road run, load the XODR **after**
> the bridge connects, or run a scenario/the sim without the Town swap — do not
> let anything call `load_world()` before `generate_opendrive_world()`.

## 3. Run the WATO stack against the ring world

Use the sim config that matches the CARLA frame:

```bash
WATO_MAP_LAUNCH_PARAM=world_model_ring_road_sim.yaml ./watod ... # or however your launch wiring passes configs
# world_model now reads /opt/watonomous/maps/ring_road/lanelet/ringroad_sim.osm
```

Bring up the rest as usual (`carla_ros_bridge` / `simulation_bringup`,
`world_modeling_bringup`). CARLA-provided odometry + `carla_localization` TF
already live in the `map` frame == the centered lanelet frame, so markers should
sit on the CARLA asphalt with no extra offset.

## 4. Verify in Foxglove

- `map_viz_markers` (visualizations/markers) — the double ring of lanelets plus
  the University Ave link, direction arrows, traffic-light/stop markers: the
  whole ring should be visible when the ego is anywhere on the loop.
- `lane_context` lanelets `left_of`/`right_of` — should stay populated while
  driving the loop (routing is set up for all 32 forward lanelets).
- `route_ahead_markers` / `route_ahead` — follow the ring as the ego crosses
  segment joints.
- `carla_pygame_hud` (web port 5000) — top-down view of the ego on the ring.

Expected numbers for a smoke test:
- Ring length ≈ **2750 m**, splits into **32 roads** id 1..32, lane width **3.7 m**.
- Loop closure error **0.0 m** (road 32 connects to road 1).
- Spawn point `get_waypoint_xodr(1, -1, s)` sits ~3.7 m right (in lane) of the
  centerline of `ringroad_seg0_fwd`.

## Known limitations

- Traffic lights / stop lines exist only as **lanelet elements** (in
  `world_model` marker layers). The XODR intentionally carries geometry only;
  CARLA signal objects are out of scope — `world_model` is the source of truth
  for semantics.
- Speed limits in CARLA are not set (roads are `type="driving"` without
  `speed`); lanelet speed (40/50 km/h) governs planning via the routing graph.
- No sidewalks / z-fighting: the OpenDRIVE world is a flat drivable mesh. Use
  Town10HD + overlays if you need urban dressing around the loop.
