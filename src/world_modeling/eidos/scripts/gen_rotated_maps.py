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
"""Generate rotated variants of a lanelet2 OSM map.

Rotates local coordinates around the map origin (0,0) by a range of angles,
then converts to WGS84 using the eidos .map file's UTM offset (unchanged).

Usage:
    python3 gen_rotated_maps.py \\
        /path/to/wrestrc.map \\
        /path/to/test_track.osm \\
        /path/to/output_prefix
"""

import math
import sqlite3
import struct
import sys
import xml.etree.ElementTree as ET


def utm_to_latlon(easting, northing, zone, is_north):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    e_p2 = e2 / (1 - e2)
    k0 = 0.9996
    x = easting - 500000.0
    y = northing if is_north else northing - 10000000.0
    M = y / k0
    mu = M / (a * (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256))
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    p = (
        mu
        + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
        + (21 * e1**2 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
        + (151 * e1**3 / 96) * math.sin(6 * mu)
    )
    s, c, t = math.sin(p), math.cos(p), math.tan(p)
    N = a / math.sqrt(1 - e2 * s**2)
    T = t**2
    C = e_p2 * c**2
    R = a * (1 - e2) / (1 - e2 * s**2) ** 1.5
    D = x / (N * k0)
    lat = p - (N * t / R) * (
        D**2 / 2
        - (5 + 3 * T + 10 * C - 4 * C**2 - 9 * e_p2) * D**4 / 24
        + (61 + 90 * T + 298 * C + 45 * T**2 - 252 * e_p2 - 3 * C**2)
        * D**6
        / 720
    )
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    lon = (
        lon0
        + (
            D
            - (1 + 2 * T + C) * D**3 / 6
            + (5 - 2 * C + 28 * T - 3 * C**2 + 8 * e_p2 + 24 * T**2)
            * D**5
            / 120
        )
        / c
    )
    return math.degrees(lat), math.degrees(lon)


def convert_with_local_rotation(
    input_path,
    output_path,
    offset_x,
    offset_y,
    offset_z,
    utm_zone,
    is_north,
    base_yaw,
    extra_rotation_rad,
):
    """Rotate local coords around map origin (0,0), then convert to WGS84."""
    tree = ET.parse(input_path)
    root = tree.getroot()

    cr = math.cos(extra_rotation_rad)
    sr = math.sin(extra_rotation_rad)

    cy = math.cos(-base_yaw)
    sy = math.sin(-base_yaw)

    count = 0
    for node in root.iter("node"):
        tags = {t.get("k"): t for t in node.findall("tag")}
        if "local_x" not in tags or "local_y" not in tags:
            continue

        lx = float(tags["local_x"].get("v"))
        ly = float(tags["local_y"].get("v"))
        lz = float(tags["ele"].get("v")) if "ele" in tags else 0.0

        # Rotate local coords around map origin
        rx = cr * lx - sr * ly
        ry = sr * lx + cr * ly

        # Convert to UTM using the original (correct) yaw
        mx = rx + offset_x
        my = ry + offset_y
        mz = lz + offset_z
        utm_e = cy * mx + sy * my
        utm_n = -sy * mx + cy * my

        lat, lon = utm_to_latlon(utm_e, utm_n, utm_zone, is_north)
        node.set("lat", f"{lat:.10f}")
        node.set("lon", f"{lon:.10f}")
        if "ele" in tags:
            tags["ele"].set("v", f"{mz:.4f}")
        count += 1

    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    return count


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <map_file> <input.osm> <output_prefix>")
        sys.exit(1)

    map_path = sys.argv[1]
    input_osm = sys.argv[2]
    output_prefix = sys.argv[3]

    conn = sqlite3.connect(map_path)
    row = conn.execute(
        "SELECT data FROM global_data WHERE data_key = 'gps_factor/utm_to_map'"
    ).fetchone()
    conn.close()

    if row is None:
        print("No gps_factor/utm_to_map in map file", file=sys.stderr)
        sys.exit(1)

    vals = struct.unpack("<5d", row[0])
    offset_x, offset_y, offset_z, encoded_zone, base_yaw = vals
    utm_zone = int(encoded_zone) // 10
    is_north = (int(encoded_zone) % 10) != 0

    print(f"Base yaw: {math.degrees(base_yaw):.2f} deg")
    print("Rotating local coordinates around map origin (0,0)")

    for offset_deg in [x * 0.5 for x in range(-20, 21)]:
        extra_rad = math.radians(offset_deg)
        sign = "plus" if offset_deg >= 0 else "minus"
        deg_str = f"{abs(offset_deg):.1f}".replace(".", "p")
        name = f"{output_prefix}_{sign}{deg_str}deg.osm"
        count = convert_with_local_rotation(
            input_osm,
            name,
            offset_x,
            offset_y,
            offset_z,
            utm_zone,
            is_north,
            base_yaw,
            extra_rad,
        )
        print(f"  {offset_deg:+5.1f} deg -> {name} ({count} nodes)")


if __name__ == "__main__":
    main()
