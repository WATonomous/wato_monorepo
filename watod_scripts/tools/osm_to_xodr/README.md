# OSM to OpenDRIVE

Convert an OpenStreetMap road-network `.osm` file to CARLA OpenDRIVE `.xodr`
using `carla.Osm2Odr.convert`. This is a temporary feasibility tool for #553.
It does not load the result into a CARLA server; #554 tracks direct conversion
inside the bridge and removal of this script.

## Why Docker?

The repository's CARLA 0.10.0 Python wheel does not expose `Osm2Odr`. The
converter is available in CARLA 0.9.16, whose wheel targets Linux x86-64.
The Dockerfile isolates that converter and its PROJ data without replacing the
repository's runtime client. Docker is useful here for a reproducible proof,
but it does not solve compatibility with the CARLA 0.10.0 server. The bridge
follow-up must resolve that boundary before using conversion at runtime.

## Convert

Run from the monorepo root after downloading an ordinary OSM road export:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh path/to/roads.osm
```

The wrapper builds the image, mounts the input read-only, and writes
`watod_scripts/tools/osm_to_xodr/output/roads.xodr`. Docker caches the image
for later runs. To choose an output path:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  path/to/roads.osm path/to/roads.xodr --center-map
```

Existing output files require `--force`. Generated `.xodr` files under the tool
are ignored by Git. Use XML `.osm`, not `.osm.pbf`.

**Input contract:** CARLA's converter reads OSM `way` elements tagged with a
supported `highway` type. A Lanelet2 file also uses the `.osm` extension but
stores lane boundaries and `type=lanelet` relations. Such a file is rejected
with a clear error because this PR does not include a Lanelet2 adapter.

Use `osm_to_xodr.sh --help` for road types, lane width, projection, offsets,
and traffic-light settings. The tested CARLA 0.9.16 wheel defaults to
`center_map=True` and `generate_traffic_lights=True`. Pass `--no-center-map`
with an explicit `--proj-string` when the CARLA map must share a known origin
with world modeling. Pass `--no-generate-traffic-lights` to disable signals;
`--all-junctions-with-traffic-lights` forces signal generation at junctions.
The presence of `<signal>` elements in XODR does not prove that traffic-light
actors render or control traffic correctly in a running CARLA world.

## Check in a CARLA server

With a compatible server running, load the generated XODR using CARLA's
`PythonAPI/util/config.py -x=/path/to/roads.xodr`, or call
`client.generate_opendrive_world(xodr_xml, parameters)` from a matching client.
For OSM-generated roads, use `wall_height=0.0` to avoid walls between opposing
roads and keep `enable_mesh_visibility=True`. Inspect road surfaces, lanes,
traffic-light actors and control states, and drive an ego vehicle through the
map. Conversion tests and `carla.Map` parsing alone cannot establish these
runtime results. CARLA's OpenDRIVE standalone mode builds roads and sidewalks;
it does not recreate all OSM scenery.

## Verify

Run the converter tests against the real CARLA API in Docker:

```bash
docker build --platform linux/amd64 -t watod/osm_to_xodr:carla-0.9.16 \
  watod_scripts/tools/osm_to_xodr
docker run --rm --platform linux/amd64 --network none \
  --volume "$PWD/watod_scripts/tools/osm_to_xodr:/tool:ro" \
  --entrypoint python watod/osm_to_xodr:carla-0.9.16 \
  -m unittest discover -s /tool/tests -v
```

On Apple Silicon, this uses amd64 emulation. The unit tests verify conversion
and input handling. They do not prove that a generated road renders correctly
or supports ego traversal in the repository's CARLA server. Perform that
server-side check before treating a map as usable.
