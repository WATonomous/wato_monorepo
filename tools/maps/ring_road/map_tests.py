#!/usr/bin/env python3
"""Offline test harness for the generated Lanelet2 ring-road map.

Mimics what WATO's world_model does with the .osm at load time:
  - lanelet::load (node lat=x/lon=y, lanelet relations, left/right boundaries)
  - German traffic rules (participant:vehicle allowed on subtype=road lanelets)
  - RoutingGraph::build from successor/predecessor/adjacent relations
  - findNearestLanelet / route queries / speed-limit reads

Runs structural, topological (routing) and geometric checks and prints PASS/FAIL.
Pure stdlib, no ROS/lanelet2 dependency required.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.join(HERE, "ringroad_utm.osm")
LANE_WIDTH = 3.7

# ---------------------------------------------------------------------------
# parse (the way lanelet2 io_osm -> map_ works)
# ---------------------------------------------------------------------------
def load(path):
    root = ET.parse(path).getroot()
    nodes, ways = {}, {}
    rels = {}  # id -> (tags, members[(role, type, ref)])
    for e in root:
        if e.tag == "node":
            nodes[int(e.attrib["id"])] = (float(e.attrib["lat"]), float(e.attrib["lon"]))
        elif e.tag == "way":
            ways[int(e.attrib["id"])] = [int(x.attrib["ref"]) for x in e.findall("nd")]
        elif e.tag == "relation":
            t = {k.attrib["k"]: k.attrib["v"] for k in e.findall("tag")}
            m = [(x.attrib["role"], x.attrib["type"], int(x.attrib["ref"]))
                 for x in e.findall("member")]
            rels[int(e.attrib["id"])] = (t, m)
    return nodes, ways, rels

def lanelet_geometry(nodes, ways, rels, rid):
    """left/right boundary point lists for a lanelet."""
    tags, mem = rels[rid]
    left = right = None
    for role, ty, rf in mem:
        if ty == "way" and role == "left":
            left = ways[rf]
        elif ty == "way" and role == "right":
            right = ways[rf]
    gl = [nodes[n] for n in left]
    gr = [nodes[n] for n in right]
    return gl, gr

def centerline(nodes, ways, rels, rid):
    gl, gr = lanelet_geometry(nodes, ways, rels, rid)
    n = min(len(gl), len(gr))
    mix = lambda a, b: ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return [mix(gl[i], gr[i]) for i in range(n)]

def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])

def point_seg_dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(vx, vy)
    if L < 1e-12:
        return dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (L * L)))
    return dist(p, (a[0] + t * vx, a[1] + t * vy))

def boundary_distance2d(p, poly):
    return min(point_seg_dist(p, poly[i], poly[i + 1]) for i in range(len(poly) - 1))

def lanelet_distance2d(p, gl, gr):
    return min(boundary_distance2d(p, gl), boundary_distance2d(p, gr))

# ---------------------------------------------------------------------------
def main():
    nodes, ways, rels = load(MAP)
    lanelets = {rid: (t, m) for rid, (t, m) in rels.items()
                if t.get("type") == "lanelet"}
    regs = {rid: (t, m) for rid, (t, m) in rels.items()
            if t.get("type") == "regulatory_element"}
    passed = failed = 0
    def check(ok, name, detail=""):
        nonlocal passed, failed
        print(("PASS" if ok else "FAIL"), name, detail)
        passed += ok; failed += (not ok)

    # ---- 1. structure ----
    check(bool(nodes) and bool(ways) and bool(lanelets),
          "map loads (nodes/ways/lanelets)",
          f"{len(nodes)}/{len(ways)}/{len(lanelets)}")
    check(any(e.tag == "annotation" for e in ET.parse(MAP).getroot()),
          "annotation meta header present")
    check(len(regs) >= 10, "regulatory elements present", f"{len(regs)}")

    # ---- 2. traffic rules / participating lanelets ----
    bad = 0
    for rid, (t, m) in lanelets.items():
        if t.get("subtype") != "road" or t.get("participant:vehicle") != "yes":
            bad += 1
    check(bad == 0, "all lanelets drivable (German vehicle rules)", f"bad={bad}")

    # ---- 3. routing graph ----
    succ = defaultdict(list); pred = defaultdict(list); adj = defaultdict(list)
    spd = {}
    for rid, (t, m) in lanelets.items():
        spd[rid] = float(t.get("speed_limit", "0"))
        for role, ty, rf in m:
            if role == "successor": succ[rid].append(rf)
            elif role == "predecessor": pred[rid].append(rf)
            elif role in ("adjacentLeft", "adjacentRight"): adj[rid].append(rf)

    asym = sum(1 for r, ss in succ.items() for x in ss if r not in pred[x])
    asym += sum(1 for r, pp in pred.items() for x in pp if r not in succ[x])
    check(asym == 0, "succ/pred symmetric", f"asym={asym}")

    name_of = {rid: t.get("name", str(rid)) for rid, (t, m) in lanelets.items()}
    rid_of = {v: k for k, v in name_of.items()}
    A = rid_of["ringroad_uv_exit"]; B = rid_of["uvway_west_out"]
    C = rid_of["uvway_east_in"];   D = rid_of["ringroad_uv_enter"]
    E = rid_of["uvway_connect"]

    def reachable(src, dst):
        """BFS following successor relations only (one-way routing like
        RoutingGraph::getRoute)."""
        seen = {src}; q = deque([src])
        while q:
            r = q.popleft()
            if r == dst:
                return True
            for x in succ[r]:
                if x not in seen:
                    seen.add(x); q.append(x)
        return False

    check(not reachable(B, A), "reverse route impossible w/ one-way egress (B->A)")

    # full lap: ring fwd lanelet (any) must reach every other ring lanelet via succ
    ring_fwd = [r for r in lanelets if name_of[r].endswith("_fwd")]
    ring_rev = [r for r in lanelets if name_of[r].endswith("_rev")]
    r0 = ring_fwd[0]
    ap = set()
    q = deque([r0]); seen = {r0}
    while q:
        r = q.popleft(); ap.add(r)
        for x in succ[r]:
            if x not in seen: seen.add(x); q.append(x)
    check(all(x in ap for x in ring_fwd), "loop forward is a closed ring",
          f"reachable {len(ap)}/{len(ring_fwd)} fwd lanelets")
    check(all(x not in ap for x in ring_rev), "loop directions are separated",
          "no fwd chain enters rev lanelets")

    r0r = ring_rev[0]
    ap2 = set(); q = deque([r0r]); seen = {r0r}
    while q:
        r = q.popleft(); ap2.add(r)
        for x in succ[r]:
            if x not in seen: seen.add(x); q.append(x)
    check(all(x in ap2 for x in ring_rev), "loop reverse is a closed ring",
          f"reachable {len(ap2)}/{len(ring_rev)} rev lanelets")

    # junction transitions
    check(reachable(r0, A) and reachable(A, B), "loop -> UV west egress drivable")
    check(reachable(C, E) and reachable(C, D),
          "UV east approach -> connector / ring-entry drivable")
    check(not reachable(A, r0), "ring exit is one-way (A->loop not reachable)")

    # ---- 4. geometry: nearest-lanelet behavior (findNearestLanelet) ----
    # sample points on the ring centerline should match their own lanelet
    g = centerline(nodes, ways, rels, r0)
    hit = g[len(g) // 2]
    best = min(lanelets,
               key=lambda r: lanelet_distance2d(hit, *lanelet_geometry(nodes, ways, rels, r)))
    check(best == r0, "nearest-lanelet finds correct lanelet at sample point",
          f"got {name_of[best]}")

    # lanelet width ≈ LANE_WIDTH on every lanelet (parallel boundaries)
    widths = []
    for rid in lanelets:
        gl, gr = lanelet_geometry(nodes, ways, rels, rid)
        # average distance from right boundary pts to left polyline
        w = sum(boundary_distance2d(p, gl) for p in gr) / max(1, len(gr))
        widths.append(w)
    wmin, wmax, wavg = min(widths), max(widths), sum(widths) / len(widths)
    check(wavg > 3.2 and wmax <= LANE_WIDTH * 1.02 and wmin > 1.5,
          "lanelet widths ~3.7 m (narrowest at sharp corner)",
          f"min={wmin:.2f} max={wmax:.2f} avg={wavg:.2f}")

    # speed limits parse
    check(all(v in (40.0, 50.0) for v in spd.values()), "speed limits 40/50 km/h parse",
          f"ring=40 UV=50 -> {sorted(set(spd.values()))}")

    # loop length: sum centerline lengths over all forward ring lanelets
    loop_len = 0.0
    for rc in sorted(ring_fwd):
        cl = centerline(nodes, ways, rels, rc)
        loop_len += sum(dist(cl[i], cl[i + 1]) for i in range(len(cl) - 1))
    check(2600 < loop_len < 2900, "ring loop ~2.75 km", f"{loop_len:.0f} m")

    print()
    print(f"RESULT: {passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)

if __name__ == "__main__":
    main()