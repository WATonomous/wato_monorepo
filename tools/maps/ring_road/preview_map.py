#!/usr/bin/env python3
"""Render a quick preview of the generated Lanelet2 map (ring + junction links +
regulatory stop lines) to a PNG. No deps beyond matplotlib."""
import os
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "ringroad_utm.osm")
OUT = os.path.join(HERE, "ringroad_utm_preview.png")

tree = ET.parse(ROOT); root = tree.getroot()
nodes, ways, rels = {}, {}, {}
for e in root:
    if e.tag == "node":
        nodes[int(e.attrib["id"])] = (float(e.attrib["lat"]), float(e.attrib["lon"]))
    elif e.tag == "way":
        ways[int(e.attrib["id"])] = [int(x.attrib["ref"]) for x in e.findall("nd")]
    elif e.tag == "relation":
        tags = {k.attrib["k"]: k.attrib["v"] for k in e.findall("tag")}
        mem = [(x.attrib["role"], int(x.attrib["ref"])) for x in e.findall("member")]
        rels[int(e.attrib["id"])] = (tags, mem)

fig, ax = plt.subplots(figsize=(12, 12))
ax.set_aspect("equal")

name_color = {"irrelevant": "red"}
for rid, (tags, mem) in rels.items():
    if tags.get("type") != "lanelet":
        continue
    name = tags.get("name", "")
    lw = [rf for role, rf in mem if role == "left"]
    rw = [rf for role, rf in mem if role == "right"]
    if not lw or not rw:
        continue
    # centerline for color logic
    if name.startswith("ringroad_seg") and name.endswith("_fwd"):
        col = "#3b82f6"   # cyan-blue: loop forward
    elif name.startswith("ringroad_seg") and name.endswith("_rev"):
        col = "#fb923c"   # orange: loop reverse
    else:
        col = "#a855f7"   # purple: junction links
    pts = [nodes[i] for i in ways[lw[0]]]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=col, lw=2.2, solid_capstyle="round", zorder=3)
    # thick road fill: draw centerline (approx) between boundaries
    out = [nodes[i] for i in ways[rw[0]]]
    ax.plot([p[0] for p in out], [p[1] for p in out], color=col, lw=2.2,
            solid_capstyle="round", zorder=3, alpha=0.55)
    # direction arrow for links
    if name.startswith(("ringroad_uv", "uvway")):
        sp = pts[len(pts) // 2]
        ep = pts[min(len(pts) - 1, len(pts) // 2 + 1)]
        dx = ep[0] - sp[0]; dy = ep[1] - sp[1]
        ax.add_patch(FancyArrow(sp[0], sp[1], dx, dy, width=5, head_width=12,
                                head_length=12, length_includes_head=True,
                                color="#6b21a8", zorder=5))

# stop lines
for rid, (tags, mem) in rels.items():
    if tags.get("type") != "regulatory_element":
        continue
    subtype = tags.get("subtype", "?")
    clr = {"traffic_light": "red", "all_way_stop": "#16a34a", "right_of_way": "#d97706"}.get(subtype, "black")
    for role, rf in mem:
        if role == "stop_line" and rf in ways:
            a, b = nodes[ways[rf][0]], nodes[ways[rf][-1]]
            ax.plot([a[0], b[0]], [a[1], b[1]], color=clr, lw=4, zorder=6)

ax.set_title("UWaterloo Ring Road + University Ave W junction (Lanelet2)")
ax.set_xticks([]); ax.set_yticks([])
fig.savefig(OUT, dpi=110, bbox_inches="tight")
print("saved", OUT)