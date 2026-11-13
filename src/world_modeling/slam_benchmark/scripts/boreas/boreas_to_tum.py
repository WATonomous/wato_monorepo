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
Convert Boreas Applanix ground truth into a TUM reference trajectory for evaluate.py.

Reads applanix/gps_post_process.csv from a Boreas sequence: the post-processed GNSS/INS
solution at 200 Hz. It is reported at the Applanix frame origin, which boreas_base shares
(zero translation in the bags' /tf_static), and stamped with the same UTC epochs as the bags.

Positions are the CSV's local east/north/up in metres (origin at the start of the sequence),
not UTM. UTM's grid scale shortens distances by about 0.02% in Toronto, and evaluate.py's
scale-free alignment would report that as error. With no EPSG code in the header, evaluate.py
cannot score an .osm lane map against this reference.

Orientation is written as identity, as for the GNSS reference: evaluate.py scores positions only.

Example:
  boreas_to_tum.py /ws/bags/boreas-2020-11-26-13-58 -o boreas_gt.tum
"""

import argparse
import csv
import os
import sys

# write_tum lives in the in-house bag tools, next door
sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "inhouse_bag")
)
from to_tum import write_tum  # noqa: E402


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "sequence",
        help="Boreas sequence directory, or its applanix/gps_post_process.csv",
    )
    parser.add_argument("-o", "--output", required=True)
    args = parser.parse_args()

    path = args.sequence
    if os.path.isdir(path):
        path = os.path.join(path, "applanix", "gps_post_process.csv")
    if not os.path.isfile(path):
        sys.exit(f"{path}: not found")

    with open(path) as f:
        rows = [
            (
                float(r["GPSTime"]),
                float(r["easting"]),
                float(r["northing"]),
                float(r["altitude"]),
                0.0,
                0.0,
                0.0,
                1.0,
            )
            for r in csv.DictReader(f)
        ]
    if not rows:
        sys.exit(f"{path}: no poses")
    write_tum(
        args.output, rows, f"Boreas Applanix ground truth from {path}, local ENU (m)"
    )


if __name__ == "__main__":
    main()
