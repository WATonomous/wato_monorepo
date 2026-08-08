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
"""
Generate the 4-lane oval track as a single source of truth.

`maps/` is git-ignored, so this script is the tracked definition of the track. It emits
three artifacts guaranteed to share one geometry:

  1. maps/osm/oval_track_4_lane.osm   Lanelet2 map -> world_model (visualisation / routing)
  2. maps/osm/oval_track_4_lane.xodr  OpenDRIVE    -> CARLA (road mesh + ground pad)
  3. src/action/fake_planner/maneuvers/oval_track.json  -> the lane centreline to drive

The .xodr also carries a "ground pad" of wide shoulder lanes under and around the track,
so leaving the drawn lanes drops the car ~PAD_DROP onto flat ground instead of into the
void (CARLA only builds mesh where OpenDRIVE lanes are). It is deliberately not in the
.osm, and its lanes are non-driving so CARLA's waypoint API never snaps onto it.

Geometry: a "stadium" oval -- two parallel straights (length L, at +/-INNER_RADIUS) joined
by two tangent semicircular caps centred at (+/-L/2, 0), so curvature is continuous. All
lane boundaries are concentric offsets of that shape, giving exactly constant lane width.

Frames: CARLA is left-handed but negates Y when reading OpenDRIVE, and carla_localization
negates Y again when publishing ROS odom -- so ROS coordinates == OpenDRIVE coordinates.
The OSM is authored in the same frame via a local-cartesian projection about lat/lon (0, 0),
so world_model's "local_cartesian" projector reproduces these exact XY metres.

Usage:
    python3 tools/generate_oval_track.py            # writes all three artifacts
    python3 tools/generate_oval_track.py --check     # verify only, write nothing
"""

import argparse
import json
import math
import os
import re

# --- Track definition (the single source of truth) ---------------------------
LANE_WIDTH = 3.5  # metres, per lane
NUM_LANES = 4
INNER_RADIUS = (
    35.0  # metres, inner edge of the innermost lane == cap radius of the reference line
)
STRAIGHT_LENGTH = 84.0  # metres, length of each of the two straights
# Which lane the fake_planner maneuver follows, numbered outward from the inside (0 == innermost).
# Lane 1 leaves a full lane of road on the inside, so tracking error toward the track centre stays
# on the mesh. Keep OvalTrackScenario.START_Y in step with the start pose this implies.
DRIVE_LANE = 1
CRUISE_SPEED = 3.0  # m/s, constant so the controller laps continuously
SPEED_LIMIT_KMH = 50
NODE_SPACING = 2.0  # metres, OSM boundary discretisation

# Ground pad (OpenDRIVE only). Several moderate lanes triangulate better than one enormous one,
# and the slight drop avoids z-fighting with the track mesh.
PAD_MARGIN = 20.0  # metres of flat runoff beyond the track envelope on every side
PAD_LANE_WIDTH = 10.0  # metres, per pad shoulder lane
PAD_DROP = 0.03  # metres the pad sits below the road surface

# Derived
OUTER_RADIUS = INNER_RADIUS + NUM_LANES * LANE_WIDTH  # 49.0
CAP_CENTRE_X = STRAIGHT_LENGTH / 2.0  # 42.0
EXTENT_X = CAP_CENTRE_X + OUTER_RADIUS  # 91.0  -> 182 m wide
EXTENT_Y = OUTER_RADIUS  # 49.0  -> 98 m tall
PAD_HALF_LENGTH = EXTENT_X + PAD_MARGIN  # 111.0 (x extent of the pad)
PAD_LANES_PER_SIDE = int(math.ceil((EXTENT_Y + PAD_MARGIN) / PAD_LANE_WIDTH))  # 7
PAD_HALF_WIDTH = PAD_LANES_PER_SIDE * PAD_LANE_WIDTH  # 70.0 (y extent of the pad)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OSM_OUT = os.path.join(REPO, "maps", "osm", "oval_track_4_lane.osm")
XODR_OUT = os.path.join(REPO, "maps", "osm", "oval_track_4_lane.xodr")
MANEUVER_OUT = os.path.join(
    REPO, "src", "action", "fake_planner", "maneuvers", "oval_track.json"
)
# Not an output: verify() reads the scenario's ego spawn back out to prove the car is put down in
# the same lane the maneuver drives.
SCENARIO_PY = os.path.join(
    REPO,
    "src",
    "simulation",
    "carla_ros_bridge",
    "carla_scenarios",
    "carla_scenarios",
    "scenarios",
    "oval_track_scenario.py",
)

# WGS84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


# --- Local-cartesian projection about lat/lon (0, 0) -------------------------
# GeographicLib's LocalCartesian at origin (0,0,0) has ENU basis East=(0,1,0), North=(0,0,1),
# Up=(1,0,0) in ECEF, and origin ECEF = (a, 0, 0). So at height 0, east == Y_ecef and
# north == Z_ecef. Inverting for lat/lon:
#     Z = N(lat)(1-e^2) sin(lat) = north      (N depends on lat -> iterate)
#     Y = N(lat) cos(lat) sin(lon) = east
def xy_to_latlon(east, north):
    """Inverse of lanelet2's LocalCartesianProjector about (0, 0) at height 0."""
    lat = north / (WGS84_A * (1.0 - WGS84_E2))  # first guess (radians)
    for _ in range(8):
        n_rad = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
        lat = math.asin(max(-1.0, min(1.0, north / (n_rad * (1.0 - WGS84_E2)))))
    n_rad = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    lon = math.asin(max(-1.0, min(1.0, east / (n_rad * math.cos(lat)))))
    return math.degrees(lat), math.degrees(lon)


def latlon_to_xy(lat_deg, lon_deg):
    """Forward LocalCartesian about (0,0) at height 0 -- used to verify the inverse."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n_rad = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    east = n_rad * math.cos(lat) * math.sin(lon)
    north = n_rad * (1.0 - WGS84_E2) * math.sin(lat)
    return east, north


# --- Geometry ----------------------------------------------------------------
def stadium_segments(radius):
    """
    Return the 4 segments of a stadium ring at `radius`, travelled counter-clockwise
    starting at the west end of the bottom straight, each as a list of (x, y).

    Consecutive segments share their junction point exactly, which lets the OSM writer
    reuse a single node id at each junction.
    """

    def arc(cx, theta0, theta1):
        length = abs(theta1 - theta0) * radius
        n = max(2, int(math.ceil(length / NODE_SPACING)))
        return [
            (
                cx + radius * math.cos(theta0 + (theta1 - theta0) * i / n),
                radius * math.sin(theta0 + (theta1 - theta0) * i / n),
            )
            for i in range(n + 1)
        ]

    def line(x0, x1, y):
        n = max(2, int(math.ceil(abs(x1 - x0) / NODE_SPACING)))
        return [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1)]

    return [
        line(-CAP_CENTRE_X, CAP_CENTRE_X, -radius),  # bottom straight, heading +x
        arc(CAP_CENTRE_X, -math.pi / 2, math.pi / 2),  # east cap, CCW
        line(CAP_CENTRE_X, -CAP_CENTRE_X, radius),  # top straight, heading -x
        arc(-CAP_CENTRE_X, math.pi / 2, 3 * math.pi / 2),  # west cap, CCW
    ]


def dist_to_spine(x, y):
    """
    Distance from (x, y) to the track's spine: the segment (-L/2, 0)..(L/2, 0). A stadium
    ring of radius r is exactly the locus dist_to_spine == r, so this is the ground truth
    every generated point is checked against.
    """
    if abs(x) <= CAP_CENTRE_X:
        return abs(y)
    return math.hypot(abs(x) - CAP_CENTRE_X, y)


def scenario_start():
    """
    The CARLA scenario's ego spawn (START_X, START_Y), or None if unreadable. Scraped rather
    than imported: importing the scenario pulls in carla and the rest of the ROS package.
    """
    try:
        with open(SCENARIO_PY) as f:
            src = f.read()
    except OSError:
        return None
    out = []
    for name in ("START_X", "START_Y"):
        m = re.search(r"^\s*%s\s*=\s*(-?[\d.]+)" % name, src, re.M)
        if m is None:
            return None
        out.append(float(m.group(1)))
    return tuple(out)


def boundary_radius(k):
    return INNER_RADIUS + k * LANE_WIDTH


def lane_centre_radius(j):
    return INNER_RADIUS + (j + 0.5) * LANE_WIDTH


# --- Lanelet2 OSM ------------------------------------------------------------
def build_osm():
    nodes = []  # (id, lat, lon)
    ways = []  # (id, [node ids], tags dict)
    rels = []  # (id, left_way, right_way, tags dict)

    next_node = [1]
    next_way = [1000]
    next_rel = [10000]

    # ring_ways[k][s] -> way id of boundary k, segment s
    ring_ways = {}
    for k in range(NUM_LANES + 1):
        segs = stadium_segments(boundary_radius(k))
        seg_node_ids = []
        first_id = None
        for s, seg in enumerate(segs):
            ids = []
            for i, (x, y) in enumerate(seg):
                # Share the junction node with the previous segment, and close the ring by
                # reusing the first node at the end of the last segment.
                if i == 0 and seg_node_ids:
                    ids.append(seg_node_ids[-1][-1])
                    continue
                if s == len(segs) - 1 and i == len(seg) - 1:
                    ids.append(first_id)
                    continue
                lat, lon = xy_to_latlon(x, y)
                nid = next_node[0]
                next_node[0] += 1
                nodes.append((nid, lat, lon))
                if first_id is None:
                    first_id = nid
                ids.append(nid)
            seg_node_ids.append(ids)

        is_edge = k == 0 or k == NUM_LANES
        tags = (
            {"type": "line_thin", "subtype": "solid", "road_border": "yes"}
            if is_edge
            else {"type": "line_thin", "subtype": "dashed"}
        )
        ring_ways[k] = []
        for ids in seg_node_ids:
            wid = next_way[0]
            next_way[0] += 1
            ways.append((wid, ids, tags))
            ring_ways[k].append(wid)

    # One lanelet per (lane, segment). Driving CCW, "left" is toward the track
    # centre (smaller radius) and "right" is outward (larger radius).
    for j in range(NUM_LANES):
        for s in range(4):
            rid = next_rel[0]
            next_rel[0] += 1
            rels.append(
                (
                    rid,
                    ring_ways[j][s],  # left  = inner boundary
                    ring_ways[j + 1][s],  # right = outer boundary
                    {
                        "type": "lanelet",
                        "subtype": "road",
                        "location": "urban",
                        "one_way": "yes",
                        "participant:vehicle": "yes",
                        "speed_limit": str(SPEED_LIMIT_KMH),
                    },
                )
            )

    out = ['<?xml version="1.0" encoding="UTF-8"?>']
    out.append('<osm version="0.6" generator="tools/generate_oval_track.py">')
    min_lat, min_lon = xy_to_latlon(-EXTENT_X, -EXTENT_Y)
    max_lat, max_lon = xy_to_latlon(EXTENT_X, EXTENT_Y)
    out.append(
        '  <bounds minlat="%.12f" minlon="%.12f" maxlat="%.12f" maxlon="%.12f"/>'
        % (min_lat, min_lon, max_lat, max_lon)
    )
    for nid, lat, lon in nodes:
        out.append(
            '  <node id="%d" lat="%.12f" lon="%.12f" visible="true" version="1">'
            '<tag k="ele" v="0.0"/></node>' % (nid, lat, lon)
        )
    for wid, ids, tags in ways:
        out.append('  <way id="%d" visible="true" version="1">' % wid)
        for nid in ids:
            out.append('    <nd ref="%d"/>' % nid)
        for k, v in tags.items():
            out.append('    <tag k="%s" v="%s"/>' % (k, v))
        out.append("  </way>")
    for rid, left, right, tags in rels:
        out.append('  <relation id="%d" visible="true" version="1">' % rid)
        out.append('    <member type="way" ref="%d" role="left"/>' % left)
        out.append('    <member type="way" ref="%d" role="right"/>' % right)
        for k, v in tags.items():
            out.append('    <tag k="%s" v="%s"/>' % (k, v))
        out.append("  </relation>")
    out.append("</osm>")
    return "\n".join(out) + "\n", nodes, ways, rels


# --- OpenDRIVE ---------------------------------------------------------------
def build_xodr():
    """
    Two roads linked head-to-tail into a closed loop (a road may not link to itself):
      road 1 = bottom straight + east cap   (-42,-35) hdg 0  ->  (42, 35) hdg pi
      road 2 = top straight    + west cap   ( 42, 35) hdg pi ->  (-42,-35) hdg 2pi

    The reference line is the inner edge (radius INNER_RADIUS); the 4 driving lanes are on
    the right of it (ids -1..-4), i.e. outward, and carry traffic along +s.

    Plus road 3, the ground pad: an unlinked straight road along the spine whose shoulder
    lanes form a flat slab covering the track envelope + PAD_MARGIN.
    """
    cap_len = math.pi * INNER_RADIUS
    curv = 1.0 / INNER_RADIUS  # positive == counter-clockwise (left)
    road_len = STRAIGHT_LENGTH + cap_len

    def lanes_block():
        b = []
        b.append("        <center>")
        b.append('          <lane id="0" type="none" level="false">')
        b.append(
            '            <roadMark sOffset="0.0" type="solid" weight="standard" '
            'color="standard" width="0.15" laneChange="none"/>'
        )
        b.append("          </lane>")
        b.append("        </center>")
        b.append("        <right>")
        for i in range(1, NUM_LANES + 1):
            lane_id = -i
            outermost = i == NUM_LANES
            b.append('          <lane id="%d" type="driving" level="false">' % lane_id)
            b.append("            <link>")
            b.append('              <predecessor id="%d"/>' % lane_id)
            b.append('              <successor id="%d"/>' % lane_id)
            b.append("            </link>")
            b.append(
                '            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>'
                % LANE_WIDTH
            )
            b.append(
                '            <roadMark sOffset="0.0" type="%s" weight="standard" '
                'color="standard" width="0.15" laneChange="%s"/>'
                % (
                    "solid" if outermost else "broken",
                    "none" if outermost else "both",
                )
            )
            b.append("          </lane>")
        b.append("        </right>")
        return "\n".join(b)

    def road(rid, other, x0, y0, hdg, x1, y1, hdg1):
        r = []
        r.append(
            '  <road name="oval_%d" length="%.10f" id="%d" junction="-1">'
            % (rid, road_len, rid)
        )
        r.append("    <link>")
        r.append(
            '      <predecessor elementType="road" elementId="%d" contactPoint="end"/>'
            % other
        )
        r.append(
            '      <successor elementType="road" elementId="%d" contactPoint="start"/>'
            % other
        )
        r.append("    </link>")
        r.append("    <planView>")
        r.append(
            '      <geometry s="0.0" x="%.10f" y="%.10f" hdg="%.10f" length="%.10f">'
            "<line/></geometry>" % (x0, y0, hdg, STRAIGHT_LENGTH)
        )
        r.append(
            '      <geometry s="%.10f" x="%.10f" y="%.10f" hdg="%.10f" length="%.10f">'
            '<arc curvature="%.12f"/></geometry>'
            % (STRAIGHT_LENGTH, x1, y1, hdg1, cap_len, curv)
        )
        r.append("    </planView>")
        r.append("    <elevationProfile>")
        r.append('      <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>')
        r.append("    </elevationProfile>")
        r.append("    <lanes>")
        r.append('      <laneSection s="0.0">')
        r.append(lanes_block())
        r.append("      </laneSection>")
        r.append("    </lanes>")
        r.append("  </road>")
        return "\n".join(r)

    def pad_road(rid):
        def pad_lane(lane_id):
            return (
                '          <lane id="%d" type="shoulder" level="false">\n'
                '            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>\n'
                "          </lane>" % (lane_id, PAD_LANE_WIDTH)
            )

        r = []
        r.append(
            '  <road name="ground_pad" length="%.10f" id="%d" junction="-1">'
            % (2.0 * PAD_HALF_LENGTH, rid)
        )
        r.append("    <planView>")
        r.append(
            '      <geometry s="0.0" x="%.10f" y="0.0" hdg="0.0" length="%.10f">'
            "<line/></geometry>" % (-PAD_HALF_LENGTH, 2.0 * PAD_HALF_LENGTH)
        )
        r.append("    </planView>")
        r.append("    <elevationProfile>")
        r.append(
            '      <elevation s="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>' % -PAD_DROP
        )
        r.append("    </elevationProfile>")
        r.append("    <lanes>")
        r.append('      <laneSection s="0.0">')
        r.append("        <left>")
        for i in range(PAD_LANES_PER_SIDE, 0, -1):
            r.append(pad_lane(i))
        r.append("        </left>")
        r.append("        <center>")
        r.append('          <lane id="0" type="none" level="false"/>')
        r.append("        </center>")
        r.append("        <right>")
        for i in range(1, PAD_LANES_PER_SIDE + 1):
            r.append(pad_lane(-i))
        r.append("        </right>")
        r.append("      </laneSection>")
        r.append("    </lanes>")
        r.append("  </road>")
        return "\n".join(r)

    out = ['<?xml version="1.0" standalone="yes"?>']
    out.append("<OpenDRIVE>")
    out.append(
        '  <header revMajor="1" revMinor="4" name="oval_track_4_lane" version="1.00" '
        'date="generated" north="%.4f" south="%.4f" east="%.4f" west="%.4f" '
        'vendor="WATonomous">'
        % (PAD_HALF_WIDTH, -PAD_HALF_WIDTH, PAD_HALF_LENGTH, -PAD_HALF_LENGTH)
    )
    out.append(
        "    <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 "
        "+datum=WGS84 +units=m +no_defs]]></geoReference>"
    )
    out.append("  </header>")
    # road 1: bottom straight (west->east) then east cap
    out.append(
        road(1, 2, -CAP_CENTRE_X, -INNER_RADIUS, 0.0, CAP_CENTRE_X, -INNER_RADIUS, 0.0)
    )
    # road 2: top straight (east->west) then west cap
    out.append(
        road(
            2,
            1,
            CAP_CENTRE_X,
            INNER_RADIUS,
            math.pi,
            -CAP_CENTRE_X,
            INNER_RADIUS,
            math.pi,
        )
    )
    # road 3: the ground pad under and around everything
    out.append(pad_road(3))
    out.append("</OpenDRIVE>")
    return "\n".join(out) + "\n"


# --- fake_planner maneuver ---------------------------------------------------
def build_maneuver():
    r = lane_centre_radius(DRIVE_LANE)
    lap = 2 * STRAIGHT_LENGTH + 2 * math.pi * r
    return {
        "name": "oval_track",
        "description": (
            "Lane %d (radius %.2f m) of the 4-lane oval, in absolute track/odom coordinates. "
            "Generated by tools/generate_oval_track.py -- do not hand-edit; it is kept in "
            "agreement with maps/osm/oval_track_4_lane.osm and .xodr. Closed %.1f m lap at "
            "constant %.1f m/s." % (DRIVE_LANE, r, lap, CRUISE_SPEED)
        ),
        # For lane following the real trajectory is the lanelet centreline passed through
        # untouched, so the planner's waypoint spacing IS the map's node spacing.
        "sample_spacing_m": NODE_SPACING,
        "default_speed": CRUISE_SPEED,
        # Without this the core caps the maneuver with a braked stop pad and the car halts back
        # at the start line instead of lapping.
        "closed": True,
        "start": {"x": -CAP_CENTRE_X, "y": -r, "yaw": 0.0},
        "segments": [
            {"straight": {"length": STRAIGHT_LENGTH}},
            {"arc": {"radius": r, "angle": 180.0, "dir": "left"}},
            {"straight": {"length": STRAIGHT_LENGTH}},
            {"arc": {"radius": r, "angle": 180.0, "dir": "left"}},
        ],
    }


def expand_maneuver(doc):
    """
    Replay the segment expansion fake_planner_core.cpp performs (moving SE(2) cursor seeded
    from the optional `start` pose), so verify() can check the maneuver against the lane
    centreline before writing it.
    """
    pts = []
    st = doc.get("start", {})
    cx, cy, cth = st.get("x", 0.0), st.get("y", 0.0), st.get("yaw", 0.0)

    def append(u, v):
        pts.append(
            (
                cx + u * math.cos(cth) - v * math.sin(cth),
                cy + u * math.sin(cth) + v * math.cos(cth),
            )
        )

    for i, seg in enumerate(doc["segments"]):
        ((kind, p),) = seg.items()
        if i == 0:
            append(0.0, 0.0)
        if kind in ("straight", "dwell"):
            length = p["length"]
            n = max(1, int(math.ceil(length / 0.05)))
            for k in range(1, n + 1):
                append(length * k / n, 0.0)
            cx, cy = pts[-1]
        elif kind == "arc":
            radius = p["radius"]
            ang = math.radians(p["angle"])
            sign = 1.0 if p.get("dir", "left") == "left" else -1.0
            n = max(1, int(math.ceil(radius * ang / 0.05)))
            for k in range(1, n + 1):
                phi = ang * k / n
                append(radius * math.sin(phi), sign * radius * (1.0 - math.cos(phi)))
            cx, cy = pts[-1]
            cth += sign * ang
        else:
            raise ValueError("unsupported segment %r" % kind)
    return pts


# --- verification ------------------------------------------------------------
def verify(nodes, ways, rels):
    ok = True

    def check(label, cond, detail=""):
        nonlocal ok
        print("  %-46s %s %s" % (label, "PASS" if cond else "FAIL", detail))
        ok = ok and cond

    print("Verifying generated track:")

    # projection round-trip
    worst = 0.0
    for x, y in [(0, 0), (EXTENT_X, EXTENT_Y), (-EXTENT_X, -EXTENT_Y), (42, -35)]:
        lat, lon = xy_to_latlon(x, y)
        bx, by = latlon_to_xy(lat, lon)
        worst = max(worst, math.hypot(bx - x, by - y))
    check("projection round-trip error < 1 mm", worst < 1e-3, "(max %.2e m)" % worst)

    # node sharing / topology
    endpoints = {}
    for wid, ids, _ in ways:
        endpoints.setdefault(ids[0], []).append(wid)
        endpoints.setdefault(ids[-1], []).append(wid)
    shared = sum(1 for n, ws in endpoints.items() if len(ws) > 1)
    check(
        "every boundary junction shares one node",
        shared == (NUM_LANES + 1) * 4,
        "(%d junctions shared)" % shared,
    )

    # no duplicate positions
    pos = {}
    for nid, lat, lon in nodes:
        x, y = latlon_to_xy(lat, lon)
        pos.setdefault((round(x, 3), round(y, 3)), []).append(nid)
    dups = sum(1 for v in pos.values() if len(v) > 1)
    check("no duplicate node positions", dups == 0, "(%d dups)" % dups)

    # rings closed
    closed = 0
    all_way_ids = [ids for _, ids, _ in ways]
    for k in range(NUM_LANES + 1):
        kws = all_way_ids[k * 4 : k * 4 + 4]
        if kws[0][0] == kws[3][-1]:
            closed += 1
    check("all 5 boundary rings closed", closed == NUM_LANES + 1, "(%d/5)" % closed)

    # Every point sitting exactly on its stadium ring proves in one shot that the caps are
    # tangent to the straights (the locus is C1 by construction) and that the rings are true
    # concentric offsets, so lane width is constant everywhere.
    worst_r = 0.0
    for k in range(NUM_LANES + 1):
        for seg in stadium_segments(boundary_radius(k)):
            for x, y in seg:
                worst_r = max(worst_r, abs(dist_to_spine(x, y) - boundary_radius(k)))
    check(
        "every point exactly on its stadium ring",
        worst_r < 1e-9,
        "(max radial err %.2e m -> caps tangent, no kinks)" % worst_r,
    )
    check(
        "lane width constant == %.1f m" % LANE_WIDTH,
        all(
            abs((boundary_radius(k + 1) - boundary_radius(k)) - LANE_WIDTH) < 1e-12
            for k in range(NUM_LANES)
        ),
    )

    segs = stadium_segments(INNER_RADIUS)

    # straights symmetric
    top = segs[2]
    bot = segs[0]
    check(
        "straights symmetric & aligned",
        abs((bot[0][0] + bot[-1][0]) / 2 - (top[0][0] + top[-1][0]) / 2) < 1e-9
        and abs(abs(bot[-1][0] - bot[0][0]) - abs(top[-1][0] - top[0][0])) < 1e-9,
    )

    # envelope
    check(
        "envelope == %.0f x %.0f m" % (2 * EXTENT_X, 2 * EXTENT_Y),
        abs(EXTENT_X - 91.0) < 1e-9 and abs(EXTENT_Y - 49.0) < 1e-9,
    )

    # The pad must bury the whole envelope with runoff on every side, and its shoulder lanes
    # must never be mistaken for drivable road.
    xodr = build_xodr()
    check(
        "ground pad covers envelope + >= 15 m runoff",
        PAD_HALF_LENGTH >= EXTENT_X + 15.0 and PAD_HALF_WIDTH >= EXTENT_Y + 15.0,
        "(pad %.0f x %.0f m)" % (2 * PAD_HALF_LENGTH, 2 * PAD_HALF_WIDTH),
    )
    check(
        "ground pad lanes are non-driving shoulders",
        xodr.count('type="shoulder"') == 2 * PAD_LANES_PER_SIDE
        and 'name="ground_pad"' in xodr,
        "(%d shoulder lanes)" % (2 * PAD_LANES_PER_SIDE),
    )

    maneuver = build_maneuver()
    pts = expand_maneuver(maneuver)
    r_drive = lane_centre_radius(DRIVE_LANE)
    worst_m = max(abs(dist_to_spine(x, y) - r_drive) for x, y in pts)
    check(
        "maneuver lies on lane %d centreline" % DRIVE_LANE,
        worst_m < 1e-6,
        "(max err %.2e m)" % worst_m,
    )
    gap = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    check("maneuver closes the lap", gap < 1e-6, "(gap %.2e m)" % gap)
    # Closing geometry is not enough: without the flag the core appends a braked stop pad.
    check("maneuver flagged closed (laps, no stop pad)", maneuver.get("closed") is True)
    check(
        "maneuver starts on the track",
        abs(dist_to_spine(*pts[0]) - r_drive) < 1e-9,
        "(start %.2f, %.2f)" % pts[0],
    )
    # The maneuver is absolute, so a scenario spawn that drifts from its start pose puts the car
    # in a different lane from the path.
    spawn = scenario_start()
    check(
        "ego spawn matches the maneuver start",
        spawn is not None
        and abs(spawn[0] - (-CAP_CENTRE_X)) < 1e-9
        and abs(spawn[1] - (-r_drive)) < 1e-9,
        "(scenario %s vs maneuver %.2f, %.2f)"
        % (
            "%.2f, %.2f" % spawn if spawn else "unreadable (%s)" % SCENARIO_PY,
            -CAP_CENTRE_X,
            -r_drive,
        ),
    )

    print("  lanelets=%d ways=%d nodes=%d" % (len(rels), len(ways), len(nodes)))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="verify only; write nothing")
    args = ap.parse_args()

    osm, nodes, ways, rels = build_osm()
    xodr = build_xodr()
    maneuver = build_maneuver()

    print(
        "Oval track: %d lanes x %.1f m, inner radius %.1f m, straights %.1f m, "
        "envelope %.0f x %.0f m"
        % (
            NUM_LANES,
            LANE_WIDTH,
            INNER_RADIUS,
            STRAIGHT_LENGTH,
            2 * EXTENT_X,
            2 * EXTENT_Y,
        )
    )
    r = lane_centre_radius(DRIVE_LANE)
    print(
        "Drive lane %d: centreline radius %.2f m, lap %.1f m, start (%.2f, %.2f) hdg 0"
        % (DRIVE_LANE, r, 2 * STRAIGHT_LENGTH + 2 * math.pi * r, -CAP_CENTRE_X, -r)
    )
    ok = verify(nodes, ways, rels)
    if not ok:
        raise SystemExit("verification FAILED")

    if args.check:
        print("--check: no files written")
        return

    os.makedirs(os.path.dirname(OSM_OUT), exist_ok=True)
    with open(OSM_OUT, "w") as f:
        f.write(osm)
    with open(XODR_OUT, "w") as f:
        f.write(xodr)
    with open(MANEUVER_OUT, "w") as f:
        json.dump(maneuver, f, indent=2)
        f.write("\n")
    print("Wrote:\n  %s\n  %s\n  %s" % (OSM_OUT, XODR_OUT, MANEUVER_OUT))


if __name__ == "__main__":
    main()
