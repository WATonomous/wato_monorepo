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
Score SLAM trajectories against a GNSS reference and an .osm lane map.

Every estimate is cropped to the time window shared by all inputs, the GNSS reference is
interpolated at the estimate's timestamps, and the estimate is aligned to it with the same
SE(3) Umeyama fit (no scale). Reported per method:
  ATE         RMS / mean / max position error vs GNSS (m)
  lane error  distance from each aligned pose to the nearest lane centerline (m)
The GNSS track is scored against the lane map too: that row is the floor of the lane
metric, since the car never drives exactly on the centerline.

Example:
  evaluate.py --ref gnss.tum --est "Eidos=eidos.tum" --est "RTAB-Map=rtabmap.tum" \\
      --osm test_track.osm --out results/bag1
"""

import argparse
import csv
import math
import os
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from evo.core.geometry import umeyama_alignment  # noqa: E402
from evo.tools import file_interface  # noqa: E402
from pyproj import Transformer  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

# Categorical slots in fixed order, light surface (dataviz reference palette)
SERIES_COLORS = [
    "#2a78d6",
    "#eb6834",
    "#1baf7a",
    "#eda100",
    "#e87ba4",
    "#008300",
    "#4a3aa7",
    "#e34948",
]
REF_COLOR = "#52514e"
LANE_COLOR = "#cfcdc4"
SURFACE = "#fcfcfb"

# OpenStreetMap highway=* values a car drives on (footway, cycleway, path, ... are excluded)
DRIVABLE = {
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "road",
}


def read_tum(path):
    traj = file_interface.read_tum_trajectory_file(path)
    return traj.timestamps, traj.positions_xyz


def read_epsg(path):
    with open(path) as f:
        first = f.readline()
    return first.split("EPSG:")[1].split()[0] if "EPSG:" in first else None


def resample(line, n):
    """Resample a polyline to n points evenly spaced by arc length."""
    seg = np.linalg.norm(np.diff(line, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(t, s, line[:, i]) for i in range(2)])


def densify(line, step):
    length = np.linalg.norm(np.diff(line, axis=0), axis=1).sum()
    return resample(line, max(2, math.ceil(length / step) + 1))


def load_centerlines(osm_path, epsg, step=0.1):
    root = ET.parse(osm_path).getroot()
    lonlat = {
        n.get("id"): (float(n.get("lon")), float(n.get("lat")))
        for n in root.iter("node")
        if n.get("lat")
    }
    if not lonlat:
        raise SystemExit(
            f"{osm_path}: no lat/lon on nodes; local-coordinate maps are not supported"
        )
    if epsg is None:
        lon = np.mean([v[0] for v in lonlat.values()])
        epsg = str(32600 + int((lon + 180.0) / 6.0) + 1)
    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    ways = {w.get("id"): w for w in root.iter("way")}

    def way_xy(way_id):
        pts = np.array(
            [
                lonlat[nd.get("ref")]
                for nd in ways[way_id].iter("nd")
                if nd.get("ref") in lonlat
            ]
        )
        e, n = to_utm.transform(pts[:, 0], pts[:, 1])
        return np.column_stack([e, n])

    lines = []
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "lanelet":
            continue
        bounds = {
            m.get("role"): m.get("ref")
            for m in rel.iter("member")
            if m.get("type") == "way"
        }
        if bounds.get("left") in ways and bounds.get("right") in ways:
            left, right = way_xy(bounds["left"]), way_xy(bounds["right"])
            n = max(
                2,
                math.ceil(
                    max(
                        np.linalg.norm(np.diff(b, axis=0), axis=1).sum()
                        for b in (left, right)
                    )
                    / step
                ),
            )
            lines.append((resample(left, n) + resample(right, n)) / 2.0)
    kind = "lanelet centerlines"
    if (
        not lines
    ):  # plain OpenStreetMap export: use drivable road centerlines, not footpaths
        for way_id, way in ways.items():
            highway = next(
                (t.get("v") for t in way.iter("tag") if t.get("k") == "highway"), None
            )
            if highway and highway.removesuffix("_link") in DRIVABLE:
                lines.append(densify(way_xy(way_id), step))
        kind = "OpenStreetMap drivable road centerlines"
    if not lines:
        raise SystemExit(f"{osm_path}: no lanelets or highway ways found")
    print(f"Lane map: {len(lines)} {kind}, EPSG:{epsg}")
    return lines, cKDTree(np.vstack(lines))


def interpolate_ref(ref_t, ref_xyz, t, max_gap):
    """Linearly interpolate reference positions at times t; mask out times inside GNSS dropouts."""
    idx = np.clip(np.searchsorted(ref_t, t), 1, len(ref_t) - 1)
    gap = ref_t[idx] - ref_t[idx - 1]
    ok = (t >= ref_t[0]) & (t <= ref_t[-1]) & (gap <= max_gap)
    xyz = np.column_stack([np.interp(t, ref_t, ref_xyz[:, i]) for i in range(3)])
    return xyz, ok


def stats(err):
    return {
        "rms": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "p95": float(np.percentile(err, 95)),
        "max": float(np.max(err)),
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--ref", required=True, help="GNSS reference TUM file (from to_tum.py gnss)"
    )
    parser.add_argument(
        "--est",
        action="append",
        required=True,
        metavar="NAME=FILE",
        help="Estimate TUM file",
    )
    parser.add_argument(
        "--osm", help="Lanelet2 or OpenStreetMap .osm file for the lane metric"
    )
    parser.add_argument(
        "--out", default="results", help="Output directory for results.csv and plots"
    )
    parser.add_argument(
        "--align-seconds",
        type=float,
        default=0.0,
        help="Align on the first N s only (0 = all)",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=0.2,
        help="Max GNSS gap to interpolate across (s)",
    )
    args = parser.parse_args()

    if len(args.est) > len(SERIES_COLORS):
        raise SystemExit(
            f"At most {len(SERIES_COLORS)} estimates per plot; split into several runs"
        )
    os.makedirs(args.out, exist_ok=True)
    ref_t, ref_xyz = read_tum(args.ref)
    ests = []
    for spec in args.est:
        name, path = spec.split("=", 1)
        ests.append((name, *read_tum(path)))

    # Compare every method over the same time window
    t0 = max([ref_t[0]] + [t[0] for _, t, _ in ests])
    t1 = min([ref_t[-1]] + [t[-1] for _, t, _ in ests])
    if t1 <= t0:
        raise SystemExit(
            "Inputs do not overlap in time; check that all files come from the same bag"
        )
    print(f"Common window: {t1 - t0:.1f} s")

    lane_tree, lines = None, []
    if args.osm:
        lines, lane_tree = load_centerlines(args.osm, read_epsg(args.ref))

    rows, plotted = [], []
    in_win = (ref_t >= t0) & (ref_t <= t1)
    if lane_tree is not None:
        d, _ = lane_tree.query(ref_xyz[in_win, :2])
        rows.append(
            {
                "method": "GNSS (lane floor)",
                "poses": int(in_win.sum()),
                **{f"lane_{k}": v for k, v in stats(d).items()},
            }
        )
    for name, t, xyz in ests:
        keep = (t >= t0) & (t <= t1)
        t, xyz = t[keep], xyz[keep]
        ref_at, ok = interpolate_ref(ref_t, ref_xyz, t, args.max_gap)
        if (~ok).any():
            print(
                f"{name}: skipped {int((~ok).sum())} of {len(t)} poses that fall in GNSS gaps > {args.max_gap} s"
            )
        t, xyz, ref_at = t[ok], xyz[ok], ref_at[ok]
        if len(t) < 3:
            raise SystemExit(f"{name}: fewer than 3 poses overlap the GNSS reference")
        fit = (
            (t - t[0] <= args.align_seconds)
            if args.align_seconds > 0
            else np.ones(len(t), bool)
        )
        r, trans, _ = umeyama_alignment(xyz[fit].T, ref_at[fit].T, False)
        aligned = (r @ xyz.T).T + trans
        ate = np.linalg.norm(aligned - ref_at, axis=1)
        row = {
            "method": name,
            "poses": len(t),
            **{f"ate_{k}": v for k, v in stats(ate).items()},
        }
        lane = None
        if lane_tree is not None:
            lane, _ = lane_tree.query(aligned[:, :2])
            row.update({f"lane_{k}": v for k, v in stats(lane).items()})
        rows.append(row)
        plotted.append((name, t, aligned, ate, lane))

    cols = [
        "method",
        "poses",
        "ate_rms",
        "ate_mean",
        "ate_max",
        "lane_rms",
        "lane_mean",
        "lane_p95",
        "lane_max",
    ]
    with open(os.path.join(args.out, "results.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    fmt = [
        str(r.get(c, ""))
        if c in ("method", "poses")
        else (f"{r[c]:.3f}" if c in r else "-")
        for r in rows
        for c in cols
    ]
    print("\n| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
    for i in range(len(rows)):
        print("| " + " | ".join(fmt[i * len(cols) : (i + 1) * len(cols)]) + " |")
    print("\n(all values in metres)")

    plot_trajectories(args, ref_t[in_win], ref_xyz[in_win], lines, plotted)
    plot_errors(args, t0, plotted, lane_tree is not None)


def break_gaps(t, *series, gap=1.0):
    """Insert NaN where consecutive samples are more than `gap` s apart so plots don't bridge dropouts."""
    cut = np.where(np.diff(t) > gap)[0] + 1
    return [np.insert(np.asarray(s, float), cut, np.nan, axis=0) for s in (t, *series)]


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color="#e4e3dd", linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def plot_trajectories(args, ref_t, ref_xyz, lines, plotted):
    origin = ref_xyz[0, :2]
    pad = 30.0
    xlim = (
        ref_xyz[:, 0].min() - origin[0] - pad,
        ref_xyz[:, 0].max() - origin[0] + pad,
    )
    ylim = (
        ref_xyz[:, 1].min() - origin[1] - pad,
        ref_xyz[:, 1].max() - origin[1] + pad,
    )
    height = min(10.0, max(4.0, 10.0 * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])))
    fig, ax = plt.subplots(figsize=(10, height), facecolor=SURFACE)
    for line in lines:
        ax.plot(
            line[:, 0] - origin[0],
            line[:, 1] - origin[1],
            color=LANE_COLOR,
            linewidth=1.0,
            zorder=1,
        )
    if lines:
        ax.plot([], [], color=LANE_COLOR, linewidth=1.0, label="Lane centerlines")
    _, rx, ry = break_gaps(ref_t, ref_xyz[:, 0] - origin[0], ref_xyz[:, 1] - origin[1])
    ax.plot(rx, ry, "--", color=REF_COLOR, linewidth=1.5, label="GNSS", zorder=2)
    for i, (name, t, aligned, _, _) in enumerate(plotted):
        _, x, y = break_gaps(t, aligned[:, 0] - origin[0], aligned[:, 1] - origin[1])
        ax.plot(x, y, color=SERIES_COLORS[i], linewidth=1.5, label=name, zorder=3)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Trajectories aligned to GNSS")
    style(ax)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "trajectories.png"), dpi=200)


def plot_errors(args, t0, plotted, has_lane):
    n = 2 if has_lane else 1
    fig, axes = plt.subplots(
        n, 1, figsize=(10, 3.2 * n), sharex=True, facecolor=SURFACE, squeeze=False
    )
    panels = [("Error vs GNSS (m)", 3)] + (
        [("Distance to lane centerline (m)", 4)] if has_lane else []
    )
    for ax, (label, field) in zip(axes[:, 0], panels):
        for i, entry in enumerate(plotted):
            t, err = break_gaps(entry[1] - t0, entry[field])
            ax.plot(t, err, color=SERIES_COLORS[i], linewidth=1.5, label=entry[0])
        ax.set_ylabel(label)
        ax.set_ylim(bottom=0)
        style(ax)
    axes[0, 0].legend(frameon=False, ncol=len(plotted))
    axes[-1, 0].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "errors.png"), dpi=200)


if __name__ == "__main__":
    main()
