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
"""Rasterize a Lanelet2 map into BEV segmentation ground truth around a vehicle pose.

Output masks follow the same grid as BEVFusion's SegMap (see bevfusion_core.hpp):
  - classes: drivable_area, ped_crossing, walkway, stop_line, carpark_area, divider
  - 200 x 200 cells at 0.5 m, centered on the LiDAR (origin -50, -50)
  - row indexes x (forward), col indexes y (left), in the LiDAR frame
  - values: 0 = absent, 1 = present, IGNORE = class not available from the map

Poses are (x, y, yaw) of the LiDAR in map coordinates. Map coordinates come from the nodes'
local_x / local_y tags, or for maps with only lat/lon, from a UTM projection minus --origin (the same
projection world_model uses with projector_type 'utm', alignment_mode 'fixed').

Usage:
  # Sample poses on lanelet centerlines and write masks + preview images
  python lanelet_ground_truth.py --osm test_track.osm --sample-poses 5 --out gt_out

  # Explicit pose (meters, meters, degrees)
  python lanelet_ground_truth.py --osm test_track.osm --pose 120.0 85.0 90 --out gt_out

  # Lat/lon-only map: pass the origin from world_modeling_bringup/config/map_configs/<map>.yaml
  python lanelet_ground_truth.py --osm ring_road.osm --origin 43.470649301705 -80.54326355965 \\
      --sample-poses 5 --out gt_out
"""

import argparse
import math
import os
import random
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field

import cv2
import numpy as np

CLASS_NAMES = [
    "drivable_area",
    "ped_crossing",
    "walkway",
    "stop_line",
    "carpark_area",
    "divider",
]
DRIVABLE, PED_CROSSING, WALKWAY, STOP_LINE, CARPARK, DIVIDER = range(len(CLASS_NAMES))
# Classes our maps have no data for; marked IGNORE so training/eval skip them.
# PED_CROSSING is also ignored for maps without crosswalk lanelets (see rasterize).
UNLABELED_CLASSES = [WALKWAY, CARPARK]
CROSSWALK_SUBTYPE = "crosswalk"
IGNORE = 255

GRID_SIZE = 200
RESOLUTION = 0.5
ORIGIN = -50.0


@dataclass
class LaneletMap:
    nodes: dict = field(default_factory=dict)  # node id -> (x, y)
    ways: dict = field(
        default_factory=dict
    )  # way id -> {"nodes": [ids], "type": str, "subtype": str}
    lanelets: list = field(
        default_factory=list
    )  # [{"id": id, "left": way id, "right": way id, "subtype": str}]

    def road_lanelets(self):
        return [ll for ll in self.lanelets if ll["subtype"] != CROSSWALK_SUBTYPE]

    def crosswalk_lanelets(self):
        return [ll for ll in self.lanelets if ll["subtype"] == CROSSWALK_SUBTYPE]

    def way_points(self, way_id):
        return np.array(
            [self.nodes[n] for n in self.ways[way_id]["nodes"]], dtype=np.float64
        )

    def bounds(self, lanelet):
        """Left and right boundary points, with the right one flipped if it runs opposite to the left."""
        left, right = (
            self.way_points(lanelet["left"]),
            self.way_points(lanelet["right"]),
        )
        if np.linalg.norm(left[0] - right[0]) > np.linalg.norm(left[0] - right[-1]):
            right = right[::-1]
        return left, right


def utm_projector(origin_lat, origin_lon):
    """Return f(lat, lon) -> (x, y) meters: UTM easting/northing minus the origin's.

    Matches Lanelet2's UtmProjector with the origin offset on, as world_model uses for
    projector_type 'utm' with alignment_mode 'fixed'. The UTM zone is taken from the origin.
    """
    from pyproj import Transformer

    zone = int((origin_lon + 180.0) // 6) + 1
    epsg = (32600 if origin_lat >= 0 else 32700) + zone
    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    x0, y0 = to_utm.transform(origin_lon, origin_lat)

    def project(lat, lon):
        x, y = to_utm.transform(lon, lat)
        return x - x0, y - y0

    return project


def load_osm(path, origin=None):
    """Parse a Lanelet2 .osm file into map coordinates.

    Nodes with local_x / local_y tags use them directly. Otherwise origin=(lat, lon) is required
    and lat/lon are projected with utm_projector.
    """
    root = ET.parse(path).getroot()
    lmap = LaneletMap()
    project = utm_projector(*origin) if origin else None
    for node in root.iter("node"):
        tags = {t.get("k"): t.get("v") for t in node.iter("tag")}
        if "local_x" in tags and "local_y" in tags:
            xy = (float(tags["local_x"]), float(tags["local_y"]))
        elif project is not None:
            xy = project(float(node.get("lat")), float(node.get("lon")))
        else:
            raise ValueError(
                f"node {node.get('id')} has no local_x/local_y tags; pass the map origin "
                "(--origin LAT LON) to project lat/lon instead"
            )
        lmap.nodes[node.get("id")] = xy
    for way in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in way.iter("tag")}
        lmap.ways[way.get("id")] = {
            "nodes": [nd.get("ref") for nd in way.iter("nd")],
            "type": tags.get("type", ""),
            "subtype": tags.get("subtype", ""),
        }
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "lanelet":
            continue
        roles = {
            m.get("role"): m.get("ref")
            for m in rel.iter("member")
            if m.get("type") == "way"
        }
        if "left" in roles and "right" in roles:
            lmap.lanelets.append(
                {
                    "id": rel.get("id"),
                    "left": roles["left"],
                    "right": roles["right"],
                    "subtype": tags.get("subtype", ""),
                }
            )
    return lmap


def divider_way_ids(lmap):
    """Lane lines between lanes.

    Nearly every boundary in the map is tagged line_thin/solid, including road edges, so tags alone
    can't separate dividers from edges. A way bounding two or more lanelets sits between lanes; a way
    bounding only one lanelet is a road edge. Dashed lines are always dividers. Crosswalk lanelets
    don't count, since their edges cross the road rather than separating lanes.
    """
    usage = Counter()
    for ll in lmap.road_lanelets():
        usage[ll["left"]] += 1
        usage[ll["right"]] += 1
    return {
        w for w, n in usage.items() if n >= 2 or lmap.ways[w]["subtype"] == "dashed"
    }


def map_to_grid(points, pose):
    """Map-frame points (N, 2) -> grid (row, col) floats for a LiDAR pose (x, y, yaw)."""
    x0, y0, yaw = pose
    dx = points[:, 0] - x0
    dy = points[:, 1] - y0
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    rows = (forward - ORIGIN) / RESOLUTION
    cols = (left - ORIGIN) / RESOLUTION
    return np.stack([rows, cols], axis=1)


def _cv_points(points, pose):
    # OpenCV draws in (x=col, y=row) pixel order; grid cell (r, c) has its center at (c + 0.5, r + 0.5).
    rc = map_to_grid(points, pose) - 0.5
    return np.round(rc[:, ::-1]).astype(np.int32).reshape(-1, 1, 2)


def rasterize(lmap, pose, line_thickness_px=1):
    """Return uint8 masks (num_classes, GRID_SIZE, GRID_SIZE) around the pose."""
    masks = np.zeros((len(CLASS_NAMES), GRID_SIZE, GRID_SIZE), dtype=np.uint8)

    for ll in lmap.road_lanelets():
        left, right = lmap.bounds(ll)
        polygon = np.vstack([left, right[::-1]])
        cv2.fillPoly(masks[DRIVABLE], [_cv_points(polygon, pose)], 1)

    for ll in lmap.crosswalk_lanelets():
        left, right = lmap.bounds(ll)
        polygon = np.vstack([left, right[::-1]])
        cv2.fillPoly(masks[PED_CROSSING], [_cv_points(polygon, pose)], 1)

    for way_id in divider_way_ids(lmap):
        cv2.polylines(
            masks[DIVIDER],
            [_cv_points(lmap.way_points(way_id), pose)],
            False,
            1,
            line_thickness_px,
        )

    for way_id, way in lmap.ways.items():
        if way["type"] == "stop_line":
            cv2.polylines(
                masks[STOP_LINE],
                [_cv_points(lmap.way_points(way_id), pose)],
                False,
                1,
                line_thickness_px,
            )

    for cls in unlabeled_classes(lmap):
        masks[cls] = IGNORE
    return masks


def unlabeled_classes(lmap):
    """Classes the map has no data for."""
    if lmap.crosswalk_lanelets():
        return UNLABELED_CLASSES
    return [PED_CROSSING] + UNLABELED_CLASSES


def sample_lanelet_poses(lmap, n, seed=0):
    """Poses at random points on lanelet centerlines, heading along the lane."""
    rng = random.Random(seed)
    poses = []
    lanelets = lmap.road_lanelets()
    for ll in rng.sample(lanelets, min(n, len(lanelets))):
        left, right = lmap.bounds(ll)
        if len(left) < 2 or len(right) < 2:
            continue
        k = max(len(left), len(right), 21)
        # Approximate centerline by pairing boundary points by fractional index.
        t = np.linspace(0.0, 1.0, k)
        lx = np.interp(t, np.linspace(0, 1, len(left)), left[:, 0])
        ly = np.interp(t, np.linspace(0, 1, len(left)), left[:, 1])
        rx = np.interp(t, np.linspace(0, 1, len(right)), right[:, 0])
        ry = np.interp(t, np.linspace(0, 1, len(right)), right[:, 1])
        center = np.stack([(lx + rx) / 2, (ly + ry) / 2], axis=1)
        i = len(center) // 2
        j = min(i + 1, len(center) - 1)
        heading = math.atan2(
            center[j, 1] - center[i - 1, 1], center[j, 0] - center[i - 1, 0]
        )
        poses.append((float(center[i, 0]), float(center[i, 1]), heading))
    return poses


def preview(masks):
    """Color image with forward up and left on the left (grid rows/cols flipped for display)."""
    img = np.full((GRID_SIZE, GRID_SIZE, 3), 40, dtype=np.uint8)
    img[masks[DRIVABLE] == 1] = (90, 90, 90)
    img[masks[PED_CROSSING] == 1] = (200, 200, 200)
    img[masks[DIVIDER] == 1] = (0, 220, 255)
    img[masks[STOP_LINE] == 1] = (0, 0, 255)
    img = img[::-1, ::-1]
    img = cv2.resize(
        img, (GRID_SIZE * 3, GRID_SIZE * 3), interpolation=cv2.INTER_NEAREST
    )
    center = (GRID_SIZE * 3 // 2, GRID_SIZE * 3 // 2)
    cv2.arrowedLine(
        img, center, (center[0], center[1] - 30), (0, 255, 0), 2, tipLength=0.3
    )
    return img


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--osm", required=True, help="Lanelet2 .osm map")
    parser.add_argument("--out", default="gt_out", help="Output directory")
    parser.add_argument(
        "--pose",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW_DEG"),
        help="LiDAR pose in map frame",
    )
    parser.add_argument(
        "--sample-poses",
        type=int,
        default=0,
        help="Sample N poses on lanelet centerlines",
    )
    parser.add_argument(
        "--origin",
        nargs=2,
        type=float,
        metavar=("LAT", "LON"),
        help="Map origin for lat/lon-only maps (from the world_modeling map config)",
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    lmap = load_osm(args.osm, args.origin)
    print(
        f"Loaded {len(lmap.nodes)} nodes, {len(lmap.ways)} ways, {len(lmap.lanelets)} lanelets "
        f"({len(lmap.crosswalk_lanelets())} crosswalks)"
    )
    print(f"Divider ways: {len(divider_way_ids(lmap))}")

    poses = []
    if args.pose:
        x, y, yaw_deg = args.pose
        poses.append((x, y, math.radians(yaw_deg)))
    if args.sample_poses:
        poses += sample_lanelet_poses(lmap, args.sample_poses, args.seed)
    if not poses:
        parser.error("give --pose and/or --sample-poses")

    os.makedirs(args.out, exist_ok=True)
    for i, pose in enumerate(poses):
        masks = rasterize(lmap, pose)
        stem = os.path.join(args.out, f"gt_{i:03d}")
        np.save(stem + ".npy", masks)
        cv2.imwrite(stem + ".png", preview(masks))
        counts = ", ".join(
            f"{CLASS_NAMES[c]}={int((masks[c] == 1).sum())}"
            for c in (DRIVABLE, PED_CROSSING, DIVIDER, STOP_LINE)
        )
        print(
            f"{stem}: pose=({pose[0]:.1f}, {pose[1]:.1f}, {math.degrees(pose[2]):.0f} deg) {counts}"
        )


if __name__ == "__main__":
    main()
