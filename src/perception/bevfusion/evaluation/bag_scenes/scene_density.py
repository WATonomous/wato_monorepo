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
"""Rank a bag's scene passes by how many objects BEVFusion detected there.

Needs a bag with recorded /perception/detections_3d_bev on the same timeline as the GPS bag.

Usage:
    scene_density.py <scenes.json> <bag_name_in_scenes.json> <detections_bag_dir>
"""

import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from track_stability import read_frames


def main():
    with open(sys.argv[1]) as f:
        events = json.load(f)[sys.argv[2]]["events"]
    frames = read_frames(sys.argv[3], "/perception/detections_3d_bev")
    t0 = frames[0][0]
    rows = []
    for ev in events:
        win = [d for t, d in frames if ev["start_s"] - 2 <= t - t0 <= ev["end_s"] + 2]
        if win:
            counts = [len(d) for d in win]
            near = [sum(math.hypot(x["x"], x["y"]) < 30 for x in d) for d in win]
            rows.append(
                (statistics.mean(counts), max(counts), statistics.mean(near), ev)
            )
    print(
        "| kind | window (s) | mean objects/frame | max | mean within 30 m | location |"
    )
    print("|---|---|---|---|---|---|")
    for mean, mx, near, ev in sorted(rows, key=lambda r: -r[0]):
        print(
            f"| {ev['kind']} | {ev['start_s']:.0f}-{ev['end_s']:.0f} | {mean:.1f} | {mx} | {near:.1f} | "
            f"{ev['name']} ({ev['lat']:.5f},{ev['lon']:.5f}) |"
        )


if __name__ == "__main__":
    main()
