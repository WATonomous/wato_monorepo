# Lanelet2 test-track preparation

Use this tool for flat, geographic Lanelet2 tracks with constant-width parallel
lanes, shared endpoint node IDs, and unique successors forming closed loops.
The supplied `oval.osm` is supported. Junctions, lane merges, regulatory
elements, changing lane counts, and projected metres stored as latitude/longitude
are rejected. Ordinary OSM highway exports use the separate `osm_to_xodr` tool.

On the cloud simulation host, from the repository root:

```bash
watod_scripts/watod-map.sh ~/Downloads/oval.osm maps/oval \
  --origin-lat 45.5017 --origin-lon -73.5673 \
  --map-id oval --spawn-lanelet 10000 --spawn-distance 5
```

Outputs are the canonical OSM copy, a generated XODR, and `map.yaml` containing
the common geographic origin and ego spawn. Maps are local assets ignored by Git.
Merge the metadata into `carla_scenarios/config/scenarios.yaml` for a new map.
Runtime `from_lanelet2: true` regenerates and caches OpenDRIVE under `maps/.cache`
when source geometry, origin, or converter code changes, so scenarios need only
the OSM and their YAML definition. Existing outputs require `--force`.

The shared converter runs without CARLA, ROS, NumPy, or a GPU. It projects WGS84
into local east/north metres using ECEF/ENU, builds lane groups and connected
OpenDRIVE roads, and uses cubic geometry with continuous tangents. CARLA handles
its own Y reflection when importing OpenDRIVE; the ROS bridge reflects it back.
The XODR georeference carries the geographic reference location; simulated GNSS
has not been validated against the local Cartesian projection.

For each new map, validate CARLA 0.10.0 loading, lane-center alignment, ego spawn,
and a drive around the complete loop. Conversion success alone is not that test.

To run directly with Python and PyYAML installed:

```bash
python3 watod_scripts/tools/lanelet_to_xodr/prepare_track.py \
  ~/Downloads/oval.osm maps/oval --origin-lat 45.5017 --origin-lon -73.5673
```

The runtime performs CARLA parser and centerline/heading checks before switching
worlds. To run the same offline check with the CARLA Python API installed:

```bash
python3 watod_scripts/tools/lanelet_to_xodr/check_map.py \
  maps/oval/oval.osm maps/oval/oval.xodr \
  --origin-lat 45.5017 --origin-lon -73.5673
```
