# OSM to OpenDRIVE

Convert an OpenStreetMap `.osm` XML export to `.xodr` with
`carla.Osm2Odr.convert`, following the
[CARLA conversion guide](https://carla.readthedocs.io/en/0.9.16/tuto_G_openstreetmap/).
Docker supplies the Python API. No CARLA server, ROS setup, or GPU is required.

The converter uses the published CARLA **0.9.16** Python API because the repo's
bundled **0.10.0** wheel does not expose `Osm2Odr`. This is a standalone file
conversion tool; the simulator's dependencies remain separate.
The image checks that the conversion API is available during its build.

## Usage

Run from the monorepo root:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh maps/my_map.osm
```

The script builds the image, reusing Docker's build cache on subsequent runs,
then writes `watod_scripts/tools/osm_to_xodr/output/my_map.xodr`.
Generated `.xodr` files are ignored by Git. The input file is mounted read-only,
and output files are written with your user and group IDs.

To select a different output path and generate traffic lights:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/my_map.osm maps/my_map.xodr --generate-traffic-lights
```

Existing output files require `--force` to overwrite. Paths may be absolute or
relative to your current directory. Use `.osm` XML, not a binary `.osm.pbf` file.

## Lanelet2 maps

A `.osm` extension does not guarantee an ordinary OpenStreetMap road network.
Lanelet2 stores lane boundaries and `type=lanelet` relations instead of `highway`
road centerlines. CARLA cannot convert those boundaries directly. The script
detects this input and explains the required mode before calling CARLA.

For the repository's four-lane, one-way oval:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/oval/lanelet/oval.osm --lanelet --center-map --force
```

This writes `watod_scripts/tools/osm_to_xodr/output/oval.xodr`. `--force` permits
rerunning the command. The source map is read-only and remains unchanged.

`--lanelet` groups adjacent lanes sharing boundaries into highway roads, derives
road centerlines from their outer boundaries, and preserves lane counts and shared
endpoints. It infers a uniform lane width unless `--default-lane-width` is supplied.
It also chooses a local geographic projection unless you supply `--proj-string`,
so a small oval retains its local scale.

This is an approximate adapter for simple one-way road layouts with equally
sampled boundaries and approximately uniform lane widths. Unsupported layouts
fail with an explanation. Lanelet2 regulatory elements, exact boundary markings,
and elevation are not preserved. CARLA may add junctions when splitting a closed
loop; inspect the result before using it for geometry-sensitive tests.
The oval currently triggers CARLA curve-smoothing warnings at a loop seam. Its
generated driving lanes parse and connect, but that does not verify the rendered
road surface in the simulator.

## Conversion settings

CARLA's settings retain their defaults unless you pass an option. The road type
filter is explicitly set to the types in CARLA's example: `motorway`,
`motorway_link`, `trunk`, `trunk_link`, `primary`, `primary_link`, `secondary`,
`secondary_link`, `tertiary`, `tertiary_link`, `unclassified`, and `residential`.

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/my_map.osm --center-map --default-lane-width 3.5 \
  --osm-way-types residential unclassified service
```

Run `osm_to_xodr.sh --help` for all options. Providing `--offset-x` or
`--offset-y` enables offsets. `--all-junctions-with-traffic-lights` also enables
traffic light generation. Use `--proj-string` to set the geographic projection.
See the [CARLA settings reference](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Osm2OdrSettings)
for their meaning.

For maps intended for Unreal, `--center-map` keeps coordinates near the origin.
When offsets are enabled, CARLA's default projection can produce large coordinates.
To specify a geographic origin directly, use zero offsets to disable automatic
coordinate normalization and supply a local projection:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh maps/my_map.osm \
  --offset-x 0 --offset-y 0 \
  --proj-string '+proj=tmerc +lat_0=43.471 +lon_0=-80.54 +datum=WGS84 +units=m +no_defs'
```

## Platform and output

Docker must be running. CARLA's wheel targets Linux x86-64, so the script builds
and runs with `--platform linux/amd64`. Apple Silicon and other ARM hosts need
Docker's amd64 emulation support. Windows users can run the script through WSL.
Files are read and written as UTF-8.

The output describes roads and junctions. Conversion does not recreate OSM
buildings or scenery and does not load the map into a running CARLA server.
An output with no roads is rejected; check the exported area and road filters.

## Verification

Run the integration tests against the real CARLA converter:

```bash
docker build --platform linux/amd64 -t watod/osm_to_xodr:carla-0.9.16 \
  watod_scripts/tools/osm_to_xodr
docker run --rm --platform linux/amd64 --network none \
  --volume "$PWD/watod_scripts/tools/osm_to_xodr:/tool:ro" \
  --entrypoint python watod/osm_to_xodr:carla-0.9.16 \
  -m unittest discover -s /tool/tests -v
```
