#!/usr/bin/env python3
"""Prepare a simple Lanelet2 track, generated XODR, and projection metadata."""

import argparse
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

# Run from a checkout or from the standalone tool image.
parents = Path(__file__).resolve().parents
if len(parents) > 3:
    core = parents[3] / "src/simulation/carla_ros_bridge/carla_common"
    if core.is_dir():
        sys.path.insert(0, str(core))

from carla_common.lanelet_track import LaneletTrack, finite, opendrive
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--map-id", default="oval")
    parser.add_argument("--spawn-lanelet", type=int)
    parser.add_argument("--spawn-distance", type=float, default=5.0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    try:
        if not args.map_id.replace("_", "").replace("-", "").isalnum():
            raise ValueError("map-id must contain letters, digits, underscores or hyphens")
        track = LaneletTrack(args.input, args.origin_lat, args.origin_lon)
        generated = opendrive(track)
        lane_id = args.spawn_lanelet if args.spawn_lanelet is not None else min(track.lanes)
        if lane_id not in track.lanes:
            raise ValueError(f"unknown spawn lanelet: {lane_id}")
        distance = finite(args.spawn_distance)
        if not 0 <= distance <= track.lanes[lane_id].center.length:
            raise ValueError("spawn-distance must be within the selected lanelet")
        targets = [args.output_dir / f"{args.map_id}.osm", args.output_dir / f"{args.map_id}.xodr",
                   args.output_dir / "map.yaml"]
        for target in targets:
            if target.resolve() == args.input.resolve():
                continue
            if target.exists() and not args.force:
                raise ValueError(f"{target} already exists; use --force")
        metadata = {
            "id": args.map_id,
            "carla": {"from_lanelet2": True},
            "lanelet2": {"path": f"{args.map_id}/{args.map_id}.osm", "projector": "local_cartesian",
                         "origin": {"lat": args.origin_lat, "lon": args.origin_lon}},
            "ego_spawn": {"lanelet_id": lane_id, "distance_m": distance, "z": 0.4},
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        if targets[0].resolve() != args.input.resolve():
            shutil.copyfile(args.input, targets[0])
        targets[1].write_text(generated, encoding="utf-8")
        targets[2].write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
        print(f"Prepared {args.map_id}: {len(track.lanes)} lanelets at {args.output_dir}")
        print("Merge map.yaml into the scenario registry. Validate load/spawn/drive in CARLA 0.10.0.")
        return 0
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
