# Lanelet2 to OpenDRIVE

Converts a Lanelet2 OSM map to CARLA OpenDRIVE with the pinned Tier IV
converter. Docker is the only host dependency.

## Usage

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  INPUT.osm [OUTPUT.xodr] [--force] [map=CONFIG] [KEY=VALUE ...]
```

The output defaults to `maps/xodr/<input-name>.xodr`. Conversion settings are
loaded from `config.sh`; command-line Hydra overrides take precedence.

Convert the configured oval:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh maps/osm/oval.osm
```

Use `--force` to replace an existing output. Generated XODR, mapping, and
quality-control files under `maps/` are ignored by Git.
