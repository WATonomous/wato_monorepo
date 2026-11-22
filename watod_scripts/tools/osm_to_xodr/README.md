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
