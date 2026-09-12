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
"""Build a Lanelet2 HD map (.osm) for the University of Waterloo Ring Road
from an OpenStreetMap export (ringroad_osm.osm).

Approach:
  * Read OSM, extract Ring Road loop (highway=unclassified) + regulatory nodes.
  * Project WGS84 -> UTM zone 17N (meters) to match Autoware/Lanelet2 local maps.
  * Road is bidirectional, one lane per direction (lanes=2). Each directional
    lanelet is one lane wide, spanning from the centerline to the road edge.
  * Compute the FULL-loop centerline and both outer boundary polylines ONCE,
    then slice them into lanelet segments at regulatory/junction nodes plus
    max-length splits. Shared boundary nodes then coincide exactly.
  * Add the University Avenue West junction: two one-way ring spurs, the two
    University Avenue West one-way arms and the connector between them, each a
    single directional lanelet, hooked into the loop topology with branching
    successor/predecessor relations plus traffic-light/stop regulatory elements.
  * Emit Lanelet2 .osm (with the required <annotation><meta> header):
    nodes (lat=x, lon=y), ways, lanelet relations (left/right/successor/
    predecessor/adjacent) and regulatory elements.
"""

import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

LANE_WIDTH = 3.7  # meters per lane
RING_HIGHWAY = "unclassified"
TARGET_SEG_LEN = 120.0  # m, max lanelet length before auto-splitting
MIN_SEG_LEN = 25.0


# ---------------------------------------------------------------------------
# UTM (WGS84) -----------------------------------------------------------------
# ---------------------------------------------------------------------------
def to_utm(lat, lon):
    latr, lonr = math.radians(lat), math.radians(lon)
    zone = math.floor((lon + 180.0) / 6.0) + 1
    lon0 = math.radians(zone * 6 - 183)
    a, f = 6378137.0, 1 / 298.257223563
    k0 = 0.9996
    e2 = f * (2 - f)
    n = a / math.sqrt(1 - e2 * math.sin(latr) ** 2)
    T = math.tan(latr) ** 2
    C = e2 / (1 - e2) * math.cos(latr) ** 2
    A = math.cos(latr) * (lonr - lon0)
    M = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * latr
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * latr)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * latr)
        - (35 * e2**3 / 3072) * math.sin(6 * latr)
    )
    E = 500000 + k0 * n * (
        A + (1 - T + C) * A**3 / 6 + (5 - 18 * T + T**2 + 72 * C - 58 * e2) * A**5 / 120
    )
    N = k0 * (
        M
        + n
        * math.tan(latr)
        * (
            A**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * e2) * A**6 / 720
        )
    )
    return E, N


def offset_polyline(xs, ys, offset_m):
    """Offset a closed polyline left (positive) / right (negative) by a
    perpendicular distance in meters, using the average neighbor tangent."""
    n = len(xs)
    out = []
    for i in range(n):
        ip, inx = (i - 1) % n, (i + 1) % n
        dx = xs[inx] - xs[ip]
        dy = ys[inx] - ys[ip]
        L = math.hypot(dx, dy)
        if L < 1e-9:
            out.append((xs[i], ys[i]))
            continue
        px, py = -dy / L, dx / L  # unit left normal
        out.append((xs[i] + offset_m * px, ys[i] + offset_m * py))
    return out


def offset_polyline_open(xs, ys, offset_m):
    """Offset an OPEN polyline by a perpendicular distance (+, left of travel)."""
    n = len(xs)
    out = []
    for i in range(n):
        if i == 0:
            dx, dy = xs[1] - xs[0], ys[1] - ys[0]
        elif i == n - 1:
            dx, dy = xs[-1] - xs[-2], ys[-1] - ys[-2]
        else:
            dx, dy = xs[i + 1] - xs[i - 1], ys[i + 1] - ys[i - 1]
        L = math.hypot(dx, dy)
        if L < 1e-9:
            out.append((xs[i], ys[i]))
            continue
        px, py = -dy / L, dx / L  # unit left normal
        out.append((xs[i] + offset_m * px, ys[i] + offset_m * py))
    return out


# ---------------------------------------------------------------------------
# OSM input parsing ------------------------------------------------------------
# ---------------------------------------------------------------------------
def parse_osm(path):
    tree = ET.parse(path)
    root = tree.getroot()
    nodes, ways, tags = {}, {}, {}
    for e in root:
        td = {k.attrib["k"]: k.attrib["v"] for k in e.findall("tag")}
        if e.tag == "node":
            nodes[e.attrib["id"]] = (float(e.attrib["lat"]), float(e.attrib["lon"]))
            if td:
                tags[e.attrib["id"]] = td
        elif e.tag == "way":
            ways[e.attrib["id"]] = [nd.attrib["ref"] for nd in e.findall("nd")]
            tags[e.attrib["id"]] = td
    return nodes, ways, tags


# ---------------------------------------------------------------------------
def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "ringroad_osm.osm"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "ringroad_utm.osm"

    nodes, ways, tags = parse_osm(path)
    ring_ids = [w for w in ways if tags.get(w, {}).get("highway") == RING_HIGHWAY]

    # ---- order loop ways & refs
    adj = defaultdict(list)
    for wid in ring_ids:
        r = ways[wid]
        adj[r[0]].append((wid, r[-1]))
        adj[r[-1]].append((wid, r[0]))
    order = [ring_ids[0]]
    cur_end = ways[ring_ids[0]][-1]
    while True:
        nxt = None
        for wid, other in adj.get(cur_end, []):
            if wid not in order:
                nxt = wid
                break
        if nxt is None:
            break
        order.append(nxt)
        r = ways[nxt]
        cur_end = r[-1] if r[-1] != cur_end else r[0]

    loop_refs = []
    for wid in order:
        r = ways[wid]
        if loop_refs and loop_refs[-1] == r[0]:
            loop_refs += r[1:]
        else:
            loop_refs += r
    if loop_refs[0] == loop_refs[-1]:
        loop_refs = loop_refs[:-1]

    # ---- project to UTM, de-dup
    raw = [to_utm(*nodes[nid]) for nid in loop_refs]
    pts = []
    for p in raw:
        if pts and math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) < 0.05:
            continue
        pts.append(p)
    if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 0.05:
        pts.pop()
    M = len(pts)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    edge_L = offset_polyline(xs, ys, +LANE_WIDTH)
    edge_R = offset_polyline(xs, ys, -LANE_WIDTH)

    # ---- regulatory split indices on the loop
    reg_map = {}
    ring_set = set(loop_refs)
    for i, nid in enumerate(loop_refs):
        tg = tags.get(nid, {})
        h = tg.get("highway")
        if h == "traffic_signals":
            reg_map[i] = "signal"
        elif h == "stop":
            reg_map[i] = "stop"
        elif h == "give_way":
            reg_map[i] = "give_way"
    # junction nodes shared with non-ring motor roads
    junction_nodes = set()
    for wid, w in ways.items():
        g = tags.get(wid, {}).get("highway")
        if g in (None, "unclassified", "cycleway"):
            continue
        for r in w:
            if r in ring_set:
                junction_nodes.add(r)
    for i, nid in enumerate(loop_refs):
        if nid in junction_nodes:
            reg_map[i] = "junction"

    # force the University Avenue West junction corners onto the loop as hard
    # split points (they are only shared with unclassified Ring Road spurs)
    for corner_nid in ("267812666", "533789218"):
        if corner_nid in ring_set:
            reg_map[loop_refs.index(corner_nid)] = "junction"

    # ---- arc-length parameterization
    cum = [0.0]
    per = [0.0]
    for i in range(1, M):
        dl = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        cum.append(cum[-1] + dl)
        per.append(dl)
    loop_len = cum[-1] + per[0]

    def arc_between(a, b):
        """uncumulated arc length between indices a,b (wrapping)."""
        d = cum[(b % M)] - cum[a % M]
        if d < 0:
            d += loop_len
        return d

    def index_for_arc(s):
        """nearest polyline index for absolute arc length s in [0, loop_len)."""
        s = s % loop_len
        lo, hi = 0, M - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if cum[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        return lo

    # ---- candidate split positions (arc length), from junctions/regulators
    split_arcs = set()
    hard_arcs = set()
    for i in reg_map:
        split_arcs.add(cum[i])
        if reg_map[i] in ("junction", "signal", "stop", "give_way"):
            hard_arcs.add(cum[i])

    # ---- add intermediate splits so segments stay under TARGET_SEG_LEN
    arcs = sorted(split_arcs)
    for a, b in zip(arcs, arcs[1:] + [arcs[0] + loop_len]):
        steps = max(0, int(math.ceil((b - a) / TARGET_SEG_LEN)) - 1)
        for k in range(1, steps + 1):
            split_arcs.add(a + (b - a) * k / (steps + 1))

    # ---- build segments from sorted arcs
    arcs = sorted(split_arcs)
    candidates = []  # (start_arc, end_arc)
    for a, b in zip(arcs, arcs[1:] + [arcs[0] + loop_len]):
        if b - a > 1e-6:
            candidates.append((a, b))

    # merge segments shorter than MIN_SEG_LEN into their successor, but never
    # merge across a hard split (junction / regulatory node)
    merged = []
    for a, b in candidates:
        if merged and b - merged[-1][1] < 1e-9:
            continue
        if (
            (b - a) < MIN_SEG_LEN
            and merged
            and a not in hard_arcs
            and b not in hard_arcs
            and merged[-1][0] not in hard_arcs
            and merged[-1][1] not in hard_arcs
        ):
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))

    segments = []
    for a, b in merged:
        ia, ib = index_for_arc(a), index_for_arc(b)
        segments.append((ia, ib))

    # =====================================================================
    # output builders
    # =====================================================================
    class Ids:
        def __init__(self):
            self.n = 1_000_000
            self.w = 2_000_000
            self.r = 3_000_000
            self.node_map = {}
            self.used = set()

        def node(self, x, y):
            k = (round(x, 3), round(y, 3))  # 1 mm snap: shared nodes coalesce
            if k not in self.node_map:
                while self.n in self.used:
                    self.n += 1
                self.node_map[k] = self.n
                self.used.add(self.n)
                self.n += 1
            return self.node_map[k]

        def way(self, nids):
            while self.w in self.used:
                self.w += 1
            self.used.add(self.w)
            wid = self.w
            self.w += 1
            return wid

        def rel(self):
            while self.r in self.used:
                self.r += 1
            self.used.add(self.r)
            rid = self.r
            self.r += 1
            return rid

    ids = Ids()
    o_nodes = {}  # nid -> (x,y)
    o_ways = []  # (wid, [nids], type, subtype)
    o_rels = []  # (rid, rtype, [role,ty,ref], tags)

    def add_way(pts, wtype, subtype):
        nids = [ids.node(p[0], p[1]) for p in pts]
        for k, p in zip(nids, pts):
            o_nodes.setdefault(k, p)
        wid = ids.way(nids)
        o_ways.append((wid, nids, wtype, subtype))
        return wid

    def add_rel(rtype, members, tags):
        rid = ids.rel()
        o_rels.append((rid, rtype, members, tags))
        return rid

    seg_fwd, seg_rev = [], []
    for si, (a, b) in enumerate(segments):
        n = (b - a) % M
        rng = [a + k for k in range(0, n + 1)]
        c_pts = [(xs[k % M], ys[k % M]) for k in rng]
        l_pts = [edge_L[k % M] for k in rng]
        r_pts = [edge_R[k % M] for k in rng]

        cw = add_way(c_pts, "line_thin", "solid")
        lw_f = add_way(l_pts, "line_thin", "dashed")  # forward left edge
        rw_r = add_way(r_pts, "line_thin", "dashed")  # reverse right edge

        # forward lanelet: left=outer+edge, right=centerline
        rid_f = add_rel(
            "lanelet",
            [("left", "way", lw_f), ("right", "way", cw)],
            {
                "subtype": "road",
                "location": "2",
                "speed_limit": "40",
                "participant:vehicle": "yes",
                "name": f"ringroad_seg{si}_fwd",
            },
        )
        # reverse lanelet: left=centerline, right=outer-edge
        rid_r = add_rel(
            "lanelet",
            [("left", "way", cw), ("right", "way", rw_r)],
            {
                "subtype": "road",
                "location": "2",
                "speed_limit": "40",
                "participant:vehicle": "yes",
                "name": f"ringroad_seg{si}_rev",
            },
        )
        seg_fwd.append(rid_f)
        seg_rev.append(rid_r)

    n_seg = len(seg_fwd)
    # successor/predecessor (same direction, wrap around)
    fwd_succ = {seg_fwd[i]: seg_fwd[(i + 1) % n_seg] for i in range(n_seg)}
    rev_succ = {seg_rev[i]: seg_rev[(i + 1) % n_seg] for i in range(n_seg)}
    fwd_pred = {seg_fwd[i]: seg_fwd[(i - 1) % n_seg] for i in range(n_seg)}
    rev_pred = {seg_rev[i]: seg_rev[(i - 1) % n_seg] for i in range(n_seg)}

    # rewrite lanelet relations: append successor/predecessor + adjacency
    adj_fwd = {seg_fwd[i]: seg_rev[i] for i in range(n_seg)}  # fwd.adjacentRight = rev
    adj_rev = {seg_rev[i]: seg_fwd[i] for i in range(n_seg)}  # rev.adjacentLeft  = fwd

    final_rels = []
    for rid, rtype, members, tags in o_rels:
        if rtype == "lanelet":
            if rid in fwd_succ:
                members = members + [
                    ("successor", "relation", fwd_succ[rid]),
                    ("predecessor", "relation", fwd_pred[rid]),
                    ("adjacentRight", "relation", adj_fwd[rid]),
                ]
            elif rid in rev_succ:
                members = members + [
                    ("successor", "relation", rev_succ[rid]),
                    ("predecessor", "relation", rev_pred[rid]),
                    ("adjacentLeft", "relation", adj_rev[rid]),
                ]
        final_rels.append((rid, rtype, members, tags))
    o_rels = final_rels

    # ---- regulatory elements: signal/stop lines at on-loop nodes
    def seg_contains(seg, i):
        a, b = seg
        n = (b - a) % M
        return 0 <= (i - a) % M <= n

    n_reg = 0
    for i in sorted(reg_map):
        kind = reg_map[i]
        if kind == "junction":
            continue
        # find segment containing index i
        si = next((k for k, seg in enumerate(segments) if seg_contains(seg, i)), None)
        if si is None or si >= n_seg:
            continue
        stop_pts = [edge_L[i], edge_R[i]]  # line across the road
        sw = add_way(stop_pts, "line_thin", "solid")
        subtype = {
            "signal": "traffic_light",
            "stop": "all_way_stop",
            "give_way": "right_of_way",
        }[kind]
        rid = add_rel(
            "regulatory_element",
            [
                ("refers", "relation", seg_fwd[si]),
                ("refers", "relation", seg_rev[si]),
                ("stop_line", "way", sw),
            ],
            {"subtype": subtype},
        )
        n_reg += 1

    # =====================================================================
    # University Avenue West junction links (one-way roads that meet the loop)
    # =====================================================================
    # Each link is a single directional lanelet (1-lane simplification) built
    # from its OSM way geometry, oriented in the direction of travel.
    def link_poly(ids_list, junc_at_end, cap):
        ded = []
        for nid in ids_list:
            p = to_utm(*nodes[nid])
            if ded and math.hypot(p[0] - ded[-1][0][0], p[1] - ded[-1][0][1]) < 0.05:
                continue
            ded.append((p, nid))
        if cap:
            # trim from the junction side: if the junction is at the end, walk
            # from the last element backwards; else walk from the first forwards.
            if junc_at_end:
                seq = ded[::-1]
                kept = [seq[0]]
                tot = 0.0
                for p, nid2 in seq[1:]:
                    tot += math.hypot(p[0] - kept[-1][0][0], p[1] - kept[-1][0][1])
                    if tot > cap:
                        break
                    kept.append((p, nid2))
                return kept[::-1]
            else:
                kept = [ded[0]]
                tot = 0.0
                for p, nid2 in ded[1:]:
                    tot += math.hypot(p[0] - kept[-1][0][0], p[1] - kept[-1][0][1])
                    if tot > cap:
                        break
                    kept.append((p, nid2))
                return kept
        return ded

    # traffic order for each link (junction node of the loop-side contact)
    links = {}
    # A: ring spur 267812666 -> 481695401 (exit from loop toward University)
    links["A"] = (
        "42685771",
        ["267812666", "11823207355", "1474002565", "481695401"],
        True,
        None,
        "40",
        "ringroad_uv_exit",
    )
    # D: ring spur 533789219 -> 533789218 (enter loop from University)
    links["D"] = (
        "42685772",
        ["533789219", "1474002569", "11242534967", "533789218"],
        True,
        None,
        "40",
        "ringroad_uv_enter",
    )
    # E: connector between the two University Ave points 533789219 -> 481695401
    links["E"] = (
        "182752070",
        ["533789219", "481695401"],
        True,
        None,
        "50",
        "uvway_connect",
    )
    # C: eastbound University Ave approach toward 533789219
    links["C"] = (
        "738334662",
        ["6913629045", "11823207356", "1668187723", "533789219"],
        True,
        260,
        "50",
        "uvway_east_in",
    )
    # B: westbound University Ave egress from 481695401
    links["B"] = (
        "41169447",
        [
            "481695401",
            "1863647984",
            "12603606727",
            "14050357146",
            "7805091929",
            "1503542132",
            "11970617687",
        ],
        False,
        260,
        "50",
        "uvway_west_out",
    )

    link_ids = {}
    link_pts_index = {}  # key -> dict osm_node_id -> kept polyline index
    extra_members = defaultdict(list)  # rid -> [(role,ty,ref)]

    def loop_index(nid):
        return loop_refs.index(nid)

    def seg_idx_ending_at(idx):
        for k, (ia, ib) in enumerate(segments):
            if (ib % M) == idx:
                return k
        return None

    def seg_idx_starting_at(idx):
        for k, (ia, ib) in enumerate(segments):
            if (ia % M) == idx:
                return k
        return None

    cornerA = loop_index("267812666")
    cornerD = loop_index("533789218")
    k_aend = seg_idx_ending_at(cornerA)
    k_astart = seg_idx_starting_at(cornerA)
    k_dstart = seg_idx_starting_at(cornerD)
    k_dend = seg_idx_ending_at(cornerD)
    if k_aend is None or k_astart is None or k_dstart is None or k_dend is None:
        raise SystemExit("junction corners not on segment boundaries!")

    for key, (way_id, id_list, junc_at_end, cap, speed, name) in links.items():
        poly = link_poly(id_list, junc_at_end, cap)
        if len(poly) < 2:
            raise SystemExit(f"link {key} too short")
        cx = [p[0] for p, _ in poly]
        cy = [p[1] for p, _ in poly]
        cw = add_way(list(zip(cx, cy)), "line_thin", "solid")
        lw = add_way(offset_polyline_open(cx, cy, +LANE_WIDTH), "line_thin", "solid")
        rid = add_rel(
            "lanelet",
            [("left", "way", lw), ("right", "way", cw)],
            {
                "subtype": "road",
                "location": "2",
                "speed_limit": speed,
                "participant:vehicle": "yes",
                "name": name,
            },
        )
        link_ids[key] = rid
        link_pts_index[key] = {nid: i for i, (p, nid) in enumerate(poly)}

    # ---- wire topology
    # branches from the ring loop to link A and from link D back onto the loop
    extra_members[seg_fwd[k_aend]].append(("successor", "relation", link_ids["A"]))
    extra_members[seg_rev[k_astart]].append(("successor", "relation", link_ids["A"]))
    extra_members[seg_fwd[k_dstart]].append(("predecessor", "relation", link_ids["D"]))
    extra_members[seg_rev[k_dend]].append(("predecessor", "relation", link_ids["D"]))

    def set_rel(id_, succ=None, pred=None):
        if succ:
            for r in succ:
                extra_members[id_].append(("successor", "relation", r))
        if pred:
            for r in pred:
                extra_members[id_].append(("predecessor", "relation", r))

    A, B, C, D, E = (link_ids[k] for k in "ABCDE")
    set_rel(A, succ=[B], pred=[seg_fwd[k_aend], seg_rev[k_astart]])
    set_rel(B, pred=[A, E])
    set_rel(E, succ=[B], pred=[C])
    set_rel(C, succ=[E, D])
    set_rel(D, succ=[seg_fwd[k_dstart], seg_rev[k_dend]], pred=[C])

    # ---- regulatory elements on links
    link_reg = {
        # key: (osm_node, subtype)
        "A": ("11823207355", "traffic_light"),
        "D": ("11242534967", "all_way_stop"),
        "C": ("11823207356", "traffic_light"),
    }
    for key, (osm_nid, subtype) in link_reg.items():
        idx = link_pts_index[key].get(osm_nid)
        if idx is None:
            continue
        poly = link_poly(*(links[key][1], links[key][2], links[key][3]))
        px, py = poly[idx][0]
        # perpendicular at idx
        if idx == 0:
            nx, ny = poly[1][0][0] - px, poly[1][0][1] - py
        elif idx == len(poly) - 1:
            nx, ny = px - poly[-2][0][0], py - poly[-2][0][1]
        else:
            nx, ny = (
                poly[idx + 1][0][0] - poly[idx - 1][0][0],
                poly[idx + 1][0][1] - poly[idx - 1][0][1],
            )
        L = math.hypot(nx, ny)
        ux, uy = -ny / L, nx / L
        sp = [(px, py), (px + ux * LANE_WIDTH, py + uy * LANE_WIDTH)]
        sw = add_way(sp, "line_thin", "solid")
        add_rel(
            "regulatory_element",
            [("refers", "relation", link_ids[key]), ("stop_line", "way", sw)],
            {"subtype": subtype},
        )
        n_reg += 1

    # apply all extra topology members (branching) to the stored relations
    rel_by_id = {r: k for k, (r, rt, m, tg) in enumerate(o_rels)}
    for rid, adds in extra_members.items():
        if rid not in rel_by_id:
            continue
        k = rel_by_id[rid]
        r_, rt, m, tg = o_rels[k]
        o_rels[k] = (r_, rt, m + adds, tg)

    # =====================================================================
    # serialize
    # =====================================================================
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<osm version="0.6" generator="uwaterloo-ring-road-lanelet2">',
        "  <annotation>",
        "    <meta>",
        "      <lanelet_version>1.0</lanelet_version>",
        "      <left_hand_traffic>no</left_hand_traffic>",
        "    </meta>",
        "  </annotation>",
    ]
    for nid, (x, y) in sorted(o_nodes.items()):
        lines.append(
            f'  <node id="{nid}" visible="true" version="1" '
            f'lat="{x:.3f}" lon="{y:.3f}" />'
        )
    for wid, nids, wtype, subtype in o_ways:
        nds = "".join(f'<nd ref="{i}" />' for i in nids)
        ts = f'<tag k="type" v="{wtype}" /><tag k="subtype" v="{subtype}" />'
        lines.append(f'  <way id="{wid}" version="1">{nds}{ts}</way>')
    for rid, rtype, members, tags in o_rels:
        ms = "".join(
            f'<member type="{ty}" ref="{rf}" role="{role}" />'
            for role, ty, rf in members
        )
        ts = "".join(f'<tag k="{k}" v="{v}" />' for k, v in tags.items())
        lines.append(
            f'  <relation id="{rid}" version="1">'
            f'<tag k="type" v="{rtype}" />{ts}{ms}</relation>'
        )
    lines.append("</osm>")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    seg_lens = [arc_between(a, b) for a, b in segments]
    print(f"Wrote {out_path}")
    print(
        f"  loop: {M} pts, {sum(seg_lens):.0f} m, {len(segments)} segments "
        f"(len range {min(seg_lens):.0f}-{max(seg_lens):.0f} m)"
    )
    print(
        f"  nodes={len(o_nodes)}  ways={len(o_ways)}  relations={len(o_rels)}  "
        f"reg_elements={n_reg}"
    )
    print(f"  junction links: { {k: link_ids[k] for k in 'ABCDE'} }")
    print(
        "  topology: A(B) links ring-exit, D(ring-entry): "
        f"A<-[{seg_fwd[k_aend]},{seg_rev[k_astart]}], "
        f"D->[{seg_fwd[k_dstart]},{seg_rev[k_dend]}]"
    )


if __name__ == "__main__":
    main()
