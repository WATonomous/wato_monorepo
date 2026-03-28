#!/usr/bin/env python3
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
"""Convert a lanelet2 OSM map from local cartesian (local_x, local_y, ele) to
WGS84 (lat, lon, ele) using the UTM origin stored in an eidos map's
gps_factor/config.bin.

Usage:
    python3 lanelet_local_to_wgs84.py \
        --map-dir /path/to/eidos_map \
        --input   /path/to/local.osm \
        --output  /path/to/wgs84.osm

The eidos map directory must contain plugins/gps_factor/config.bin, which
stores the UTM offset, zone, hemisphere, and heading used during mapping.

The local coordinates are in the eidos "map" frame. The transform chain is:
    map_pos  = R_map_enu * utm_pos - offset
    utm_pos  = R_map_enu^T * (map_pos + offset)
    lat, lon = utm_to_latlon(utm_pos, zone, hemisphere)
"""

import argparse
import math
import struct
import sys
import xml.etree.ElementTree as ET


def utm_to_latlon(easting: float, northing: float, zone: int, is_north: bool):
    """Convert UTM coordinates to WGS84 lat/lon.

    Uses the Karney-style series expansion (accurate to ~1mm).
    """
    # WGS84 parameters
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    e_prime2 = e2 / (1 - e2)

    k0 = 0.9996
    x = easting - 500000.0
    y = northing if is_north else northing - 10000000.0

    M = y / k0
    mu = M / (a * (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256))

    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    phi1 = (
        mu
        + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
        + (21 * e1**2 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
        + (151 * e1**3 / 96) * math.sin(6 * mu)
        + (1097 * e1**4 / 512) * math.sin(8 * mu)
    )

    sin_phi1 = math.sin(phi1)
    cos_phi1 = math.cos(phi1)
    tan_phi1 = math.tan(phi1)

    N1 = a / math.sqrt(1 - e2 * sin_phi1**2)
    T1 = tan_phi1**2
    C1 = e_prime2 * cos_phi1**2
    R1 = a * (1 - e2) / (1 - e2 * sin_phi1**2) ** 1.5
    D = x / (N1 * k0)

    lat = phi1 - (N1 * tan_phi1 / R1) * (
        D**2 / 2
        - (5 + 3 * T1 + 10 * C1 - 4 * C1**2 - 9 * e_prime2) * D**4 / 24
        + (61 + 90 * T1 + 298 * C1 + 45 * T1**2 - 252 * e_prime2 - 3 * C1**2)
        * D**6
        / 720
    )

    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    lon = (
        lon0
        + (
            D
            - (1 + 2 * T1 + C1) * D**3 / 6
            + (5 - 2 * C1 + 28 * T1 - 3 * C1**2 + 8 * e_prime2 + 24 * T1**2)
            * D**5
            / 120
        )
        / cos_phi1
    )

    return math.degrees(lat), math.degrees(lon)


def load_gps_config(map_dir: str):
    """Read config.bin from eidos gps_factor plugin directory.

    Returns (offset_x, offset_y, offset_z, utm_zone, is_north, yaw).
    """
    import os

    config_path = os.path.join(map_dir, "plugins", "gps_factor", "config.bin")
    with open(config_path, "rb") as f:
        data = f.read()

    if len(data) != 5 * 8:
        raise ValueError(f"config.bin has {len(data)} bytes, expected 40")

    vals = struct.unpack("<5d", data)
    offset_x, offset_y, offset_z = vals[0], vals[1], vals[2]
    encoded_zone = int(vals[3])
    utm_zone = encoded_zone // 10
    is_north = (encoded_zone % 10) != 0
    yaw = vals[4]

    return offset_x, offset_y, offset_z, utm_zone, is_north, yaw


def local_to_utm(
    local_x: float,
    local_y: float,
    local_z: float,
    offset_x: float,
    offset_y: float,
    offset_z: float,
    yaw: float,
):
    """Convert local map coordinates to UTM.

    map_pos = R_map_enu * utm_pos - offset
    => utm_pos = R_map_enu^T * (map_pos + offset)

    R_map_enu = [[cos(-yaw), -sin(-yaw), 0],
                 [sin(-yaw),  cos(-yaw), 0],
                 [0,          0,         1]]

    R_map_enu^T = R_enu_map = [[cos(-yaw),  sin(-yaw), 0],
                               [-sin(-yaw), cos(-yaw), 0],
                               [0,          0,         1]]
    """
    cy = math.cos(-yaw)
    sy = math.sin(-yaw)

    # map_pos + offset
    mx = local_x + offset_x
    my = local_y + offset_y
    mz = local_z + offset_z

    # R_map_enu^T * (map_pos + offset)
    utm_e = cy * mx + sy * my
    utm_n = -sy * mx + cy * my
    utm_z = mz

    return utm_e, utm_n, utm_z


def convert_osm(
    input_path: str,
    output_path: str,
    offset_x: float,
    offset_y: float,
    offset_z: float,
    utm_zone: int,
    is_north: bool,
    yaw: float,
):
    """Parse OSM, convert local_x/local_y to lat/lon, write output."""
    tree = ET.parse(input_path)
    root = tree.getroot()

    converted = 0
    for node in root.iter("node"):
        tags = {t.get("k"): t for t in node.findall("tag")}

        if "local_x" not in tags or "local_y" not in tags:
            continue

        local_x = float(tags["local_x"].get("v"))
        local_y = float(tags["local_y"].get("v"))
        local_z = float(tags["ele"].get("v")) if "ele" in tags else 0.0

        utm_e, utm_n, utm_z = local_to_utm(
            local_x, local_y, local_z, offset_x, offset_y, offset_z, yaw
        )

        lat, lon = utm_to_latlon(utm_e, utm_n, utm_zone, is_north)

        node.set("lat", f"{lat:.10f}")
        node.set("lon", f"{lon:.10f}")

        if "ele" in tags:
            tags["ele"].set("v", f"{utm_z:.4f}")

        converted += 1

    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    return converted


def main():
    parser = argparse.ArgumentParser(
        description="Convert lanelet2 OSM from local cartesian to WGS84 "
        "using an eidos map's GPS config."
    )
    parser.add_argument(
        "--map-dir",
        required=True,
        help="Path to eidos map directory (contains plugins/gps_factor/config.bin)",
    )
    parser.add_argument(
        "--input", required=True, help="Input OSM file with local_x/local_y coordinates"
    )
    parser.add_argument(
        "--output", required=True, help="Output OSM file with lat/lon coordinates"
    )
    args = parser.parse_args()

    try:
        offset_x, offset_y, offset_z, utm_zone, is_north, yaw = load_gps_config(
            args.map_dir
        )
    except (FileNotFoundError, ValueError) as e:
        print(f"Error loading GPS config: {e}", file=sys.stderr)
        sys.exit(1)

    hemisphere = "N" if is_north else "S"
    print(f"UTM zone: {utm_zone}{hemisphere}")
    print(f"Heading (yaw): {math.degrees(yaw):.2f} deg")
    print(f"Offset: [{offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f}]")

    count = convert_osm(
        args.input, args.output, offset_x, offset_y, offset_z, utm_zone, is_north, yaw
    )
    print(f"Converted {count} nodes -> {args.output}")


if __name__ == "__main__":
    main()
