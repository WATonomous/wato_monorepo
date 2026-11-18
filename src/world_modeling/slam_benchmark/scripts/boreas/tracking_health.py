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
Check whether each SLAM run kept up with the car, as a Markdown table.

Eidos reports TRACKING even when its scan matching has stopped working, so this looks at
the output instead: how far each method's trajectory travels, compared with how far the
car drove (from the reference) over the part of the bag that was replayed. A method that
lost track stops adding poses, so its path falls well short of the distance driven.

For Eidos it also counts "GICP failed" lines in launch.log. Every failure is logged;
successful matches are only logged every 2 s, so successes can't be counted from the log.

Example:
  tracking_health.py results/boreas --ref results/boreas/reference/gnss.tum \\
      --bag /ws/bags/boreas-2020-11-26-13-58-lidar-imu --duration 120
"""

import argparse
import glob
import math
import os


def read_xy(path):
    """Return [(t, x, y)] from a TUM file."""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            v = line.split()
            rows.append((float(v[0]), float(v[1]), float(v[2])))
    rows.sort()
    return rows


def path_length(rows, t0=-math.inf, t1=math.inf):
    pts = [(x, y) for t, x, y in rows if t0 <= t <= t1]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def bag_window(bag):
    """(start, end) of the bag in seconds from its metadata.yaml, or None."""
    try:
        import yaml

        with open(os.path.join(bag, "metadata.yaml")) as f:
            info = yaml.safe_load(f)["rosbag2_bagfile_information"]
        start = info["starting_time"]["nanoseconds_since_epoch"] * 1e-9
        return start, start + info["duration"]["nanoseconds"] * 1e-9
    except (OSError, KeyError, TypeError, ImportError):
        return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("root", help="Dataset results folder holding test*/")
    parser.add_argument("--ref", required=True, help="Reference TUM file")
    parser.add_argument("--bag", help="Bag folder, for the replayed time window")
    parser.add_argument(
        "--duration", type=float, help="Seconds of the bag that were replayed"
    )
    parser.add_argument(
        "--min-coverage",
        type=float,
        default=0.9,
        help="Flag runs whose path is shorter than this fraction of the distance driven",
    )
    args = parser.parse_args()

    ref = read_xy(args.ref)
    window = (args.bag and bag_window(args.bag)) or (ref[0][0], ref[-1][0])
    t0, t1 = window
    if args.duration:
        t1 = min(t1, t0 + args.duration)
    driven = path_length(ref, t0, t1)

    print(
        f"\nTracking health over the first {t1 - t0:.0f} s of the bag "
        f"(the car drove {driven:.0f} m)\n"
    )
    print(
        "| test | method | GICP failures | poses | path (m) | of driven | last pose | |"
    )
    print("|---|---|---|---|---|---|---|---|")
    methods = [
        ("eidos_nogps", "Eidos (no GPS)", "poses.tum"),
        ("rtabmap", "RTAB-Map", "poses.txt"),
    ]
    for test in sorted(glob.glob(os.path.join(args.root, "test*"))):
        for folder, label, poses in methods:
            path = os.path.join(test, folder, poses)
            if not os.path.isfile(path):
                continue
            rows = read_xy(path)
            failures = "-"
            log = os.path.join(test, folder, "launch.log")
            if folder == "eidos_nogps" and os.path.isfile(log):
                with open(log, errors="replace") as f:
                    failures = str(sum("GICP failed" in line for line in f))
            length = path_length(rows)
            share = length / driven if driven > 0 else 0.0
            last = f"+{rows[-1][0] - t0:.0f} s" if rows else "-"
            flag = "LOST?" if share < args.min_coverage else ""
            print(
                f"| {os.path.basename(test)} | {label} | {failures} | {len(rows)} | "
                f"{length:.0f} | {100 * share:.0f}% | {last} | {flag} |"
            )
    print(
        f"\nLOST? = path shorter than {100 * args.min_coverage:.0f}% of the distance driven."
    )


if __name__ == "__main__":
    main()
