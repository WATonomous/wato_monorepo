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
"""Build an OpenDRIVE (.xodr) road network for CARLA from the cached
ring road Lanelet2 map.

The generated world reuses the EXACT ring-loop centerline coordinates the
lanelet map was built from (UTM 17N, centered on the map origin), so the
CARLA road surface and the lanelet map align 1:1. Load it at runtime with:

    import carla
    xml = open("ringroad_carla.xodr").read()
    client.generate_opendrive_world(xml, {"max_road_length": 500})
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.join(HERE, "ringroad_utm.osm")
OUT = os.path.join(HERE, "ringroad_carla.xodr")

LANE_WIDTH = 3.7  # m, matches the lanelet width used by build_lanelet_map.py


def load_map(path):
    root = ET.parse(path).getroot()
    nodes, ways, lanelets = {}, {}, {}
    for e in root:
        if e.tag == "node":
            nodes[int(e.attrib["id"])] = (
                float(e.attrib["lat"]),
                float(e.attrib["lon"]),
            )
        elif e.tag == "way":
            ways[int(e.attrib["id"])] = [int(x.attrib["ref"]) for x in e.findall("nd")]
        elif e.tag == "relation":
            tags = {k.attrib["k"]: k.attrib["v"] for k in e.findall("tag")}
            members = [
                (x.attrib["role"], x.attrib["type"], int(x.attrib["ref"]))
                for x in e.findall("member")
            ]
            if tags.get("type") == "lanelet":
                lanelets[int(e.attrib["id"])] = (tags, members)
    return nodes, ways, lanelets


def centerline_points(nodes, ways, lanelets, rid):
    """The lanelet centerline. build_lanelet_map.py stores the loop centerline
    as the lanelet's RIGHT boundary (left = centerline offset by lane width),
    and the UV links as right = centerline too. Averaging left/right instead
    truncates at corners where the offset boundary carries extra points."""
    _, members = lanelets[rid]
    right = None
    for role, typ, ref in members:
        if role == "right" and typ == "way":
            right = ways[ref]
    if right is None:
        raise SystemExit(f"lanelet {rid} has no right (centerline) boundary")
    return [nodes[n] for n in right]


def write_centered_osm(src, dst, ox, oy):
    """Write a copy of the lanelet map shifted so ring-center == (0,0), making
    the map's local frame equal CARLA's world frame (XODR origin = (0,0))."""
    root = ET.parse(src).getroot()
    for e in root:
        if e.tag == "node":
            e.set("lon", f"{float(e.attrib['lon']) - ox:.6f}")
            e.set("lat", f"{float(e.attrib['lat']) - oy:.6f}")
    ET.ElementTree(root).write(dst, encoding="unicode", xml_declaration=True)
    with open(dst, "a") as f:
        f.write("\n")
    print(f"Wrote {dst}")


def main():
    nodes, ways, lanelets = load_map(MAP)

    fwd = {
        rid: tags
        for rid, (tags, _) in lanelets.items()
        if tags.get("name", "").startswith("ringroad_seg")
        and tags["name"].endswith("_fwd")
    }
    if not fwd:
        raise SystemExit("no forward ring lanelets found in the map")

    def successors(rid):
        return [
            rf
            for role, typ, rf in lanelets[rid][1]
            if role == "successor" and typ == "relation"
        ]

    start = sorted(fwd)[0]
    order = []
    seen = set()
    rid = start
    while rid not in seen:
        order.append(rid)
        seen.add(rid)
        nxt = [x for x in successors(rid) if x in fwd and x not in seen]
        if not nxt:
            break
        rid = nxt[0]
    if len(order) != len(fwd) or start not in successors(order[-1]):
        raise SystemExit(
            f"loop ordering failed: {len(order)}/{len(fwd)} lanelets, not closed"
        )

    # ordered ring polyline. Consecutive lanelets normally share their joint
    # node, but at the junction break the chain is gapped -- handle both.
    segments = [centerline_points(nodes, ways, lanelets, rid) for rid in order]
    starts = []
    pts = []
    for i, seg in enumerate(segments):
        if i == 0:
            pts = list(seg)
            starts.append(0)
            continue
        tail = pts[-1]
        if math.hypot(tail[0] - seg[0][0], tail[1] - seg[0][1]) < 1e-6:
            starts.append(len(pts) - 1)  # seg begins at the shared joint node
            pts += list(seg[1:])
        else:
            starts.append(len(pts))  # gapped: append the whole seg
            pts += list(seg)

    # center on map origin so CARLA world (0,0) == lanelet-map origin
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ox = (min(xs) + max(xs)) / 2.0
    oy = (min(ys) + max(ys)) / 2.0
    pts = [(x - ox, y - oy) for x, y in zip(xs, ys)]

    roads = [pts[s : s + len(segments[i])] for i, s in enumerate(starts)]
    if any(len(r) < 2 for r in roads):
        raise SystemExit("degenerate road after split")
    n = len(roads)

    road_xml = []
    for i, seg in enumerate(roads):
        pred = n if i == 0 else i
        succ = (i + 1) % n + 1
        link = (
            f"<link>"
            f'<predecessor elementType="road" elementId="{pred}"/>'
            f'<successor elementType="road" elementId="{succ}"/>'
            f"</link>"
        )
        planview = []
        s = 0.0
        for j in range(len(seg) - 1):
            x1, y1 = seg[j]
            x2, y2 = seg[j + 1]
            length = math.hypot(x2 - x1, y2 - y1)
            heading = math.atan2(y2 - y1, x2 - x1)
            planview.append(
                f'<geometry s="{s:.6f}" x="{x1:.6f}" y="{y1:.6f}" '
                f'hdg="{heading:.9f}" length="{length:.6f}"><line/></geometry>'
            )
            s += length
        road_xml.append(
            f'<road name="ringroad_seg{i}_fwd" id="{i + 1}" length="{s:.6f}" junction="-1">'
            f"{link}"
            f"<planView>{''.join(planview)}</planView>"
            f"<lanes>"
            f'<laneOffset s="0" a="0.0" b="0.0" c="0.0" d="0.0"/>'
            f'<laneSection s="0">'
            f'<center><lane id="0" type="none" level="false"/></center>'
            f"<right>"
            f'<lane id="-1" type="driving" level="false">'
            f'<width sOffset="0" a="{LANE_WIDTH:.2f}" b="0" c="0" d="0"/>'
            f'<height sOffset="0" inner="0" outer="0"/>'
            f"</lane></right>"
            f"</laneSection></lanes>"
            f"</road>"
        )

    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        "<OpenDRIVE>\n"
        '<header revMajor="1" revMinor="0" name="uwaterloo_ring_road" version="1.00" '
        'north="1" south="0" east="1" west="0">\n'
        "<geoReference>+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs</geoReference>\n"
        "</header>\n" + "\n".join(road_xml) + "\n</OpenDRIVE>\n"
    )

    with open(OUT, "w") as f:
        f.write(xml)

    write_centered_osm(MAP, os.path.join(HERE, "ringroad_sim.osm"), ox, oy)

    total = sum(
        math.hypot(seg[j][0] - seg[j - 1][0], seg[j][1] - seg[j - 1][1])
        for seg in roads
        for j in range(1, len(seg))
    )
    print(f"Wrote {OUT}")
    print(f"  {n} roads, {len(pts)} plan points, ring length {total:.0f} m")
    print(f"  world origin (0,0) = UTM easting {ox:.3f}, northing {oy:.3f}")
    print(
        f"  closure error: {math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]):.4f} m"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
