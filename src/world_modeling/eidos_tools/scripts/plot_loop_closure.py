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
"""Render a before/after loop closure alignment figure from a dumped closure.

The eidos LoopClosureCloudVisualization plugin writes, per accepted closure:
  closure_<i>_to_<j>_source.csv      query keyframe cloud (body frame)
  closure_<i>_to_<j>_target.csv      matched keyframe cloud (body frame)
  closure_<i>_to_<j>_transforms.csv  pre-GICP and post-GICP relative poses

Both panels are drawn in the target keyframe's body frame: the target cloud is
fixed, and the source cloud is mapped through the pre-GICP estimate (left) and
the GICP result (right).
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SOURCE_COLOR = "#d1495b"
TARGET_COLOR = "#00798c"


def load_cloud(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[:, :3]


def load_transforms(path):
    poses = {}
    with open(path) as f:
        next(f)
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != 17:
                continue
            poses[parts[0]] = np.array([float(v) for v in parts[1:]]).reshape(4, 4)
    return poses


def apply(T, pts):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def axes_for(view):
    return (0, 1, "x [m]", "y [m]") if view == "xy" else (0, 2, "x [m]", "z [m]")


def panel(ax, source, target, title, view, point_size, alpha, limits, show_ylabel):
    ix, iy, xlabel, ylabel = axes_for(view)

    ax.scatter(
        target[:, ix], target[:, iy], s=point_size * 1.6, c=TARGET_COLOR,
        linewidths=0, alpha=alpha, label="matched keyframe", rasterized=True)
    ax.scatter(
        source[:, ix], source[:, iy], s=point_size, c=SOURCE_COLOR,
        linewidths=0, alpha=alpha, label="query keyframe", rasterized=True)

    ax.set_title(title, fontsize=8, pad=4)
    ax.set_xlabel(xlabel, fontsize=8, labelpad=2)
    if show_ylabel:
        ax.set_ylabel(ylabel, fontsize=8, labelpad=2)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7, length=2.5, width=0.6, pad=2)
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    (x0, x1), (y0, y1) = limits
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)


def shared_limits(clouds, view, max_range):
    """Identical limits on both panels so the two are directly comparable."""
    ix, iy = axes_for(view)[:2]
    if max_range is not None:
        return (-max_range, max_range), (-max_range, max_range)
    pts = np.vstack(clouds)
    cx, cy = pts[:, ix], pts[:, iy]
    pad = 0.04 * max(cx.ptp(), cy.ptp())
    return (cx.min() - pad, cx.max() + pad), (cy.min() - pad, cy.max() + pad)


def correction_of(poses):
    """Translation/rotation magnitude GICP applied on top of the graph estimate."""
    delta = poses["corrected"] @ np.linalg.inv(poses["initial"])
    shift = float(np.linalg.norm(delta[:3, 3]))
    angle = float(np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1))))
    return shift, angle


def rank_closures(target_dir):
    """All closures in a directory, largest GICP correction first."""
    ranked = []
    for tf_path in target_dir.glob("*_transforms.csv"):
        poses = load_transforms(tf_path)
        if "initial" not in poses or "corrected" not in poses:
            continue
        stem = tf_path.with_name(tf_path.name[: -len("_transforms.csv")])
        ranked.append((*correction_of(poses), stem))
    ranked.sort(key=lambda r: r[0], reverse=True)
    return ranked


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stem", help="Closure stem (dumps/closure_412_to_87), or a directory")
    ap.add_argument("-o", "--output", default=None, help="Output file (default: <stem>_alignment.pdf)")
    ap.add_argument("--view", choices=["xy", "xz"], default="xy", help="Projection plane (default: xy)")
    ap.add_argument("--point-size", type=float, default=0.6, help="Scatter point size")
    ap.add_argument("--alpha", type=float, default=0.65, help="Point opacity; <1 keeps overlap visible")
    ap.add_argument("--max-range", type=float, default=None, help="Axis limit in metres, symmetric about origin")
    ap.add_argument(
        "--min-z", type=float, default=None,
        help="Drop points below this height (sensor frame, e.g. -1.0) to remove ground rings, "
             "which otherwise dominate and hide the structural alignment")
    ap.add_argument(
        "--all", action="store_true",
        help="Directory mode: render every closure instead of only the largest correction")
    ap.add_argument(
        "--figsize", type=float, nargs=2, default=(5.5, 3.0), metavar=("W", "H"),
        help="Figure size in inches (default 5.5 3.0, sized so the equal-aspect panels "
             "sit flush; widen for a two-column spread)")
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args()

    target_dir = Path(args.stem)
    if target_dir.is_dir():
        ranked = rank_closures(target_dir)
        if not ranked:
            sys.exit(f"no closures found in {target_dir}")

        print(f"{len(ranked)} closures, by GICP correction:")
        for shift, angle, stem in ranked[:5]:
            print(f"  {shift:6.2f} m  {angle:5.1f}°   {stem.name}")
        if len(ranked) > 5:
            smallest = ranked[-1]
            print(f"  ... {len(ranked) - 5} more, smallest {smallest[0]:.2f} m  {smallest[1]:.1f}°")
        print()

        if args.all:
            for _, _, stem in ranked:
                render_one(stem, None, args)
        else:
            shift, angle, stem = ranked[0]
            print(f"rendering largest correction: {stem.name} ({shift:.2f} m, {angle:.1f}°)")
            render_one(stem, args.output, args)
        return

    stem = Path(args.stem)
    if not Path(f"{stem}_transforms.csv").exists():
        hint = "" if stem.parent.exists() else f" (no such directory: {stem.parent})"
        sys.exit(
            f"'{stem}' is neither a directory of closures nor a closure stem{hint}.\n"
            "Pass the dump directory (e.g. bags/closures) or a stem like "
            "bags/closures/closure_412_to_87.\n"
            "If the directory is empty, run the bag first — closures are dumped at runtime.")

    render_one(stem, args.output, args)


def render_one(stem, output, args):
    source_path = Path(f"{stem}_source.csv")
    target_path = Path(f"{stem}_target.csv")
    tf_path = Path(f"{stem}_transforms.csv")
    for p in (source_path, target_path, tf_path):
        if not p.exists():
            sys.exit(f"missing {p}")

    source = load_cloud(source_path)
    target = load_cloud(target_path)
    if args.min_z is not None:
        source = source[source[:, 2] >= args.min_z]
        target = target[target[:, 2] >= args.min_z]
        if len(source) == 0 or len(target) == 0:
            sys.exit(f"--min-z {args.min_z} removed every point; try a lower value")
    poses = load_transforms(tf_path)
    if "initial" not in poses or "corrected" not in poses:
        sys.exit(f"{tf_path} must contain 'initial' and 'corrected' rows")

    before = apply(poses["initial"], source)
    after = apply(poses["corrected"], source)

    limits = shared_limits([before, after, target], args.view, args.max_range)

    shift, angle = correction_of(poses)

    fig, axes = plt.subplots(1, 2, figsize=args.figsize, sharey=True)
    panel(axes[0], before, target, "(a) Graph estimate", args.view, args.point_size, args.alpha, limits, True)
    panel(axes[1], after, target, "(b) After GICP", args.view, args.point_size, args.alpha, limits, False)

    # Correction stated on the figure so it survives being read apart from the caption.
    axes[1].text(
        0.97, 0.04, f"correction\n{shift:.2f} m, {angle:.1f}°",
        transform=axes[1].transAxes, ha="right", va="bottom", fontsize=7,
        linespacing=1.3,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.7", linewidth=0.5))

    # Reserve margins explicitly: constrained_layout does not account for figure
    # legends in older matplotlib, which lets the legend collide with the x labels.
    fig.tight_layout(rect=(0, 0.06, 1, 1.0), w_pad=0.8)

    handles, labels = axes[0].get_legend_handles_labels()
    leg = fig.legend(
        handles, labels, loc="lower center", bbox_to_anchor=(0.5, -0.01),
        ncol=2, frameon=False, fontsize=7.5, handletextpad=0.4, columnspacing=1.6)
    for h in getattr(leg, "legend_handles", None) or leg.legendHandles:
        h.set_sizes([10])

    out = output or f"{stem}_alignment.pdf"
    fig.savefig(out, dpi=args.dpi, bbox_inches="tight", pad_inches=0.02)
    print(f"wrote {out}  ({len(source)} query pts, {len(target)} matched pts)")


if __name__ == "__main__":
    main()
