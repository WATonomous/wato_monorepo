# World Modeling Bringup

Launches the world-modeling stack with a named Lanelet2 map configuration.

## Usage

Run with CARLA ground-truth localization and select the matching map:

```bash
# Oval
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  use_eidos:=false map:=oval

# Town10HD
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  use_eidos:=false map:=town10hd

# Ring road
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  use_eidos:=false map:=ring_road
```

The `map` argument loads `config/map_configs/<map>.yaml`. Available presets are
`oval`, `ring_road`, `town10hd`, and `test_track`.

**Important:** Place the required OSM maps in the repository's root `maps/`
directory. Docker mounts this directory at `/opt/watonomous/maps/`. World
modeling cannot load a map if its OSM file is missing from this directory.

Pass a custom map configuration directly when needed:

```bash
ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml \
  use_eidos:=false map_config_file:=/path/to/map.yaml
```

Projection, alignment, and origin options are documented in the map preset
files under `config/map_configs/`.
