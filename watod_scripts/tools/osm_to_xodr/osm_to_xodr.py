# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert OSM XML to OpenDRIVE using CARLA's standalone converter."""

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

DEFAULT_WAY_TYPES = [
    "motorway",
    "motorway_link",
    "trunk",
    "trunk_link",
    "primary",
    "primary_link",
    "secondary",
    "secondary_link",
    "tertiary",
    "tertiary_link",
    "unclassified",
    "residential",
]


def finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("must be a finite number")
    return number


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="input .osm XML file")
    parser.add_argument("output", type=Path, help="output .xodr file")
    parser.add_argument(
        "--force", action="store_true", help="overwrite existing output"
    )
    parser.add_argument("--osm-way-types", nargs="+", default=DEFAULT_WAY_TYPES)
    parser.add_argument("--offset-x", type=finite_float)
    parser.add_argument("--offset-y", type=finite_float)
    parser.add_argument(
        "--center-map", action=argparse.BooleanOptionalAction, default=None
    )
    parser.add_argument("--default-lane-width", type=finite_float)
    parser.add_argument("--elevation-layer-height", type=finite_float)
    parser.add_argument("--proj-string", help="PROJ projection string")
    parser.add_argument(
        "--generate-traffic-lights", action=argparse.BooleanOptionalAction, default=None
    )
    parser.add_argument(
        "--all-junctions-with-traffic-lights", action="store_true", default=None
    )
    parser.add_argument("--traffic-light-excluded-way-types", nargs="+")
    args = parser.parse_args()
    if args.default_lane_width is not None and args.default_lane_width <= 0:
        parser.error("--default-lane-width must be greater than zero")
    if args.all_junctions_with_traffic_lights and args.generate_traffic_lights is False:
        parser.error(
            "--all-junctions-with-traffic-lights conflicts with --no-generate-traffic-lights"
        )
    return args


def convert(args):
    if args.input.resolve() == args.output.resolve():
        raise ValueError("input and output must be different files")
    if args.output.suffix.lower() != ".xodr":
        raise ValueError("output must have a .xodr extension")
    if args.output.exists() and not args.force:
        raise FileExistsError(f"{args.output} already exists; use --force to overwrite")

    osm_data = args.input.read_text(encoding="utf-8-sig")
    osm_root = ET.fromstring(osm_data)
    if osm_root.tag != "osm":
        raise ValueError("input must be OpenStreetMap XML with an <osm> root")
    highways = [
        tag.get("v")
        for way in osm_root.findall("way")
        for tag in way.findall("tag")
        if tag.get("k") == "highway"
    ]
    if not any(highway in args.osm_way_types for highway in highways):
        if any(
            tag.get("k") == "type" and tag.get("v") == "lanelet"
            for relation in osm_root.findall("relation")
            for tag in relation.findall("tag")
        ):
            raise ValueError(
                "Lanelet2 map detected: CARLA Osm2Odr requires OSM highway ways. "
                "This tool does not adapt Lanelet2 maps."
            )
        raise ValueError("input contains no highway ways matching --osm-way-types")

    # Import after argument parsing so --help also works without CARLA installed.
    import carla

    if not hasattr(carla, "Osm2Odr") or not hasattr(carla, "Osm2OdrSettings"):
        raise RuntimeError(
            "this CARLA wheel lacks Osm2Odr; use the tool's Docker image"
        )
    settings = carla.Osm2OdrSettings()
    settings.set_osm_way_types(args.osm_way_types)
    for name in (
        "offset_x",
        "offset_y",
        "center_map",
        "default_lane_width",
        "elevation_layer_height",
        "proj_string",
        "generate_traffic_lights",
        "all_junctions_with_traffic_lights",
    ):
        value = getattr(args, name)
        if value is not None:
            setattr(settings, name, value)
    if args.offset_x is not None or args.offset_y is not None:
        settings.use_offsets = True
    if args.all_junctions_with_traffic_lights:
        settings.generate_traffic_lights = True
    if args.traffic_light_excluded_way_types is not None:
        settings.set_traffic_light_excluded_way_types(
            args.traffic_light_excluded_way_types
        )

    xodr_data = carla.Osm2Odr.convert(osm_data, settings)
    root = ET.fromstring(xodr_data)
    roads = root.findall("road")
    if root.tag != "OpenDRIVE" or not roads:
        raise ValueError("conversion produced no OpenDRIVE roads; check OSM road types")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w" if args.force else "x", encoding="utf-8") as output:
        output.write(xodr_data)
    print(f"Wrote {args.output} ({len(roads)} roads)")


def main():
    args = parse_args()
    try:
        convert(args)
    except (OSError, ValueError, RuntimeError, ImportError, ET.ParseError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
