#!/usr/bin/env python3
"""Check CARLA parser acceptance and waypoint alignment, without loading a world."""

import argparse
from pathlib import Path
import sys

parents = Path(__file__).resolve().parents
if len(parents) > 3:
    core = parents[3] / "src/simulation/carla_ros_bridge/carla_common"
    if core.is_dir():
        sys.path.insert(0, str(core))

from carla_common.lanelet_track import LaneletTrack, validate_carla_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("osm", type=Path)
    parser.add_argument("xodr", type=Path)
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--tolerance", type=float, default=0.25)
    args = parser.parse_args()
    track = LaneletTrack(args.osm, args.origin_lat, args.origin_lon)
    errors = validate_carla_map(args.xodr.read_text(), track, args.tolerance)
    print(f"CARLA accepted map: {len(errors)} samples, max centerline error {max(errors):.4f}m")


if __name__ == "__main__":
    main()
