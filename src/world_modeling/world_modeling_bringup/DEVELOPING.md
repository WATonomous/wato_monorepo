# World modeling map configuration (Lanelet2 + OSM)

World modeling loads a general runtime configuration (`config_file`) followed
by a map-specific parameter overlay (`map_config_file`). The second file is the
source of the OSM path, projection, origin, and alignment. Select a preset with
`map:=<name>` or pass a custom `map_config_file` path.

| Preset | Projection | Alignment | Geographic origin |
| --- | --- | --- | --- |
| [oval](config/map_configs/oval.yaml) | UTM | fixed | 45.5017, -73.5673, 0.0 m |
| [town10hd](config/map_configs/town10hd.yaml) | Local Cartesian | fixed | 0.0, 0.0, 0.0 m |
| [test_track](config/map_configs/test_track.yaml) | UTM | TF from `utm` into `map` | Configured latitude/longitude select the UTM zone |

The oval OSM and generated XODR remain outside Git under `maps/`. Town10HD OSM
and the test-track OSM/Eidos prior map must also be supplied at their preset paths.
Town10HD retains the historical `(0, 0)` Local Cartesian convention; verify
the origin/projection if using an OSM from a different exporter.

## Run in CARLA

Start the CARLA bridge and load the matching CARLA world separately. Selecting
`map` here selects Lanelet geometry; it does not change the CARLA scenario.

```bash
# Oval is the simulation launch's default preset.
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  map:=oval use_eidos:=false

# With CARLA Town10HD and its matching OSM available:
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  map:=town10hd use_eidos:=false
```

Both presets use `alignment_mode: fixed`: projection output already matches
CARLA's `map` frame. CARLA supplies `map -> odom -> base_footprint`. The simulation
launch uses `/clock` and CARLA `/ego/odom` when Eidos is disabled.

## Run with Eidos

```bash
ros2 launch world_modeling_bringup world_modeling.launch.yaml map:=test_track
```

This launch defaults to the test-track preset. Use a matching Eidos prior map
via `eidos_map_path`; use `use_sim_time:=true` for bag playback. Eidos's GPS factor
publishes the geographic relationship and `eidos_transform` broadcasts it into
`/tf_static`, including for world models started after localization. The world
model waits for `utm -> map`, then converts **absolute** UTM points
with the full translation and rotation into the Eidos map frame. It does not
subtract the configured UTM origin as well. The result is OSM roads expressed
in Eidos's existing frame; Eidos's SLAM map is not moved.

Alignment is sampled once when loading. If localization changes that alignment,
restart the world-modeling launch (or perform a lifecycle cleanup/configure/activate)
to rebuild the aligned Lanelet map.

## Define a map

For a simulation map, choose an explicit geographic origin before exporting
XODR and keep it with the OSM/XODR pair. To centre a geographic track, choose
its projected centre and subtract it during XODR generation. The loader must
use the same projector and origin. Do not assume that every map's bounds
midpoint, first node, or XODR header is its true coordinate origin.

```yaml
/**/world_model:
  ros__parameters:
    osm_map_path: "/opt/watonomous/maps/osm/my_track.osm"
    projector_type: "utm"       # or "local_cartesian"
    alignment_mode: "fixed"    # projection output is already the map frame
    origin_lat: 45.5017
    origin_lon: -73.5673
    origin_alt: 0.0
```

```bash
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  use_eidos:=false map_config_file:=/path/to/my_track.yaml
```

For an independently established localization frame, set `alignment_mode: tf`
and `projected_frame` to the source frame. With UTM that frame must contain
absolute UTM coordinates in the same zone/hemisphere and elevation datum as
the OSM. With Local Cartesian it must contain ENU coordinates relative to the
configured origin; name it appropriately, for example `enu`.

All origin values are ROS doubles: use `0.0`, not `0`. Unknown projection or
alignment values fail configuration. A missing TF is retried; malformed or
missing OSM files report a map-load error. Fixed mode also allows an altitude
origin so elevated source maps can be centred in Z when exported consistently.

The current oval XODR is centred using the MGRS offset equivalent of its UTM
origin. To reproduce that coordinate convention with the repository converter:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh maps/osm/oval.osm --force \
  map=example_mgrs_offset map.mgrs_grid=18TXR \
  map.offset.x=11928.71216549247 map.offset.y=39683.63022774272 map.offset.z=0
```

See the [converter documentation](../../../watod_scripts/tools/osm_to_xodr/README.md)
for conversion options. Choosing a geographic reference without subtracting
the projected offset does not centre the exported coordinates.

## Build and validate

Inside the world-modeling dev container:

```bash
cd /ws
colcon build --symlink-install --packages-select world_model world_modeling_bringup
source install/setup.bash
colcon test --packages-select world_model --event-handlers console_direct+
colcon test-result --verbose
```

The projection tests cover oval coordinate agreement, configurable Local
Cartesian origins, full UTM-to-Eidos rotation/translation, reverse projection,
and invalid settings. Building/launching presets does not change colcon's package
dependency selection; `use_eidos` controls runtime processes.

For the Eidos workflow, also rebuild `eidos_transform` to pick up its durable
`/tf_static` alignment broadcaster:

```bash
colcon build --symlink-install --packages-select eidos_transform
```

For an installed world model run directly, supply both configuration files:

```bash
ros2 run world_model world_model_node --ros-args -r __ns:=/world_modeling \
  --params-file /path/to/world_model_sim.yaml --params-file /path/to/oval.yaml
```

Manage this lifecycle node separately when using `ros2 run`.

## Migration

Replace `wait_for_utm_transform: false` with `alignment_mode: fixed` and a true
wait with `alignment_mode: tf`. Replace `utm_frame` with `projected_frame`.
The old translation-only `utm_origin_x/y` loader arguments have been removed.
General runtime configs no longer contain map-specific values: load a map
overlay even when invoking the node outside the bringup launches.
