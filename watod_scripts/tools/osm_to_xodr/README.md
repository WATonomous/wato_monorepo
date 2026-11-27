# Lanelet2 to OpenDRIVE

Converts a Lanelet2 OSM map to CARLA OpenDRIVE with the pinned Tier IV
converter. Docker is the only host dependency.

## Usage

Create a local configuration from the tracked template:

```bash
cp watod_scripts/tools/osm_to_xodr/config.sh \
  watod_scripts/tools/osm_to_xodr/config.local.sh
```

Set the map projection and offset values in `config.local.sh`. This file is
ignored by Git.

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  INPUT.osm [OUTPUT.xodr] [--force] [map=CONFIG] [KEY=VALUE ...]
```

The output defaults to `maps/xodr/<input-name>.xodr`. Conversion settings are
loaded from `config.local.sh`; command-line Hydra overrides take precedence.

Convert the configured map:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh maps/osm/map.osm
```

Use `--force` to replace an existing output. Generated XODR, mapping, and
quality-control files under `maps/` are ignored by Git.

## Airfield

The Airfield source, converted XODR, wrapper metadata, and converter lane mapping
are tracked exceptions to the `maps/` ignore rule. The source is a synthetic
400 m by 70 m rectangle centered at latitude 45.5017, longitude -73.5673,
altitude 0 m. The matching ROS map config uses `utm` with `fixed` alignment.

To reproduce the conversion, create `config.local.sh` as above if absent, then
set these values in it (preserve any unrelated local configuration):

```bash
MAP_CONFIG="example_mgrs_offset"
MGRS_GRID="18TXR"
OFFSET_X="11928.712165492936"
OFFSET_Y="39683.630227741785"
OFFSET_Z="0"
FORCE=false
EXTRA_OPTIONS=()
```

Run from the repository root; explicit overrides record the intended settings
in the output metadata:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/osm/airfield.osm maps/xodr/airfield.xodr --force \
  map=example_mgrs_offset map.mgrs_grid=18TXR \
  map.offset.x=11928.712165492936 map.offset.y=39683.630227741785 \
  map.offset.z=0
```

The pinned converter subtracts the MGRS in-square offsets from its geometry.
They correspond to UTM zone 18N easting 611928.712165493 m and northing
5039683.630227742 m at the ROS origin. This produces map x = [-200, 200] m,
y = [-35, 35] m without a TF alignment transform. The converter's header
`geoReference` uses a quantized MGRS reverse lookup, and its header bounds retain
the pre-offset coordinates; use the geometry and recorded offsets when checking
alignment rather than inferring a new ROS origin from those header fields.

Conversion writes `airfield.xodr.metadata.json` with source/output SHA-256 hashes,
cache identity, pinned converter revision, and options, plus
`airfield.mapping.json` with lanelet-to-road/lane correspondence and hashes.
Keep both records from the same conversion as the XODR. Rerunning conversion
changes the embedded timestamp and output hashes; geometry should stay the same.

To update the tracked assets explicitly:

```bash
git add -f maps/osm/airfield.osm maps/xodr/airfield.xodr \
  maps/xodr/airfield.xodr.metadata.json maps/xodr/airfield.mapping.json
```

Do not add `config.local.sh`, conversion logs, or the converter's other diagnostic
outputs. See the [Airfield circle workflow](../../../src/simulation/simulation_generation/README.md#airfield-circle)
for offline generation, the configured spawn, and run commands.
