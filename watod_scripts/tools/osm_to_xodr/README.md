# Lanelet2 to OpenDRIVE

The command builds the `convert` target from
[Tier IV's upstream Dockerfile](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/Dockerfile)
and invokes its `convert` CLI. It pins revision
`4b707e067bfdb8db04223b87aa2da290c5cba607`; upstream owns the conversion code,
Docker dependencies, and tests. There is no local Python converter or test suite.
The host needs Docker only.

## Run

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  path/to/lanelet.osm [path/to/output.xodr] \
  map=CONFIG [upstream.key=value ...]
```

The output defaults to `maps/xodr/<input-name>.xodr` in this repository. The
wrapper selects `target=carla` and mounts the input read-only and the output
directory writable. Upstream also writes mapping and QC analysis files beside
the XODR. Existing XODR files require `--force`; upstream writes directly to the
output, so a failed run can leave partial output. Rootless Docker uses container
root, which maps to the daemon owner's host user; regular Docker uses the caller's
UID/GID. Map files under `maps/` are ignored by Git.

Arguments after the paths use upstream Hydra `key=value` syntax. The previous
`--origin-lat`, `--origin-lon`, and Osm2Odr options are removed. Select a map
configuration and supply its coordinate settings explicitly; the wrapper does
not infer a location or silently select upstream's example map. Input, output,
and CARLA target settings are managed by the wrapper.

## What the origin means

Lanelet2 OSM nodes contain latitude/longitude: they locate the road geometry
on Earth. They do not necessarily declare which geographic point should be the
simulation's `(0, 0)`. Our `maps/osm/oval.osm` has node coordinates and geographic
bounds, but no separate origin declaration.

Upstream projects those coordinates into metres using MGRS. Its map config
selects the geographic reference; its `map.offset` settings subtract a position
from the exported coordinates. Subtracting the projected centre places that
centre at `(0, 0)` without resizing the track. Merely supplying latitude/longitude
through `map=example_latlon` does not subtract this offset.

For example, the following uses the oval's bounds midpoint as a geographic
reference, but does **not** centre the exported map:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/osm/oval.osm \
  map=example_latlon \
  map.lat_lon.latitude=45.5017 map.lat_lon.longitude=-73.5673 \
  map.lat_lon.altitude=0
```

To put that midpoint at `(0, 0)`, use upstream's MGRS-plus-offset configuration.
The following offsets are that midpoint projected through upstream's MGRS
projector, in metres:

```bash
watod_scripts/tools/osm_to_xodr/osm_to_xodr.sh \
  maps/osm/oval.osm \
  map=example_mgrs_offset map.mgrs_grid=18TXR \
  map.offset.x=11928.71216549247 map.offset.y=39683.63022774272 \
  map.offset.z=0
```

These are explicit example choices for this oval. Use the agreed map reference
instead when aligning CARLA with world modeling. No custom geometry or projection
code runs in the wrapper. See
[upstream's origin and offset documentation](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/docs/usage.md#origin-specification)
for other configurations and preprocessing options.

## Verification

Use an end-to-end conversion smoke check and inspect the generated XODR in the
actual CARLA server. Upstream runs its own converter tests and post-conversion
QC/mapping analysis. Those checks do not establish rendered road surfaces,
vehicle traversal, traffic-signal behavior, or ROS bridge recovery. Those are
part of the issue #558 scenario test.
