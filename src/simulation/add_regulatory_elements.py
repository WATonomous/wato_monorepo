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
Extract traffic lights, stop lines, and speed limits from CARLA
and add them as regulatory elements to the Lanelet2 OSM file.
"""

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import carla
except ImportError:
    print("CARLA Python API not found. Please ensure CARLA is installed.")
    carla = None


# Earth radius for coordinate conversion (same as used in lanelet2)
EARTH_RADIUS = 6378137.0


@dataclass
class Node:
    """OSM Node representation."""

    id: int
    lat: float
    lon: float
    local_x: float
    local_y: float
    ele: float = 0.0


@dataclass
class Way:
    """OSM Way representation."""

    id: int
    node_refs: List[int]
    tags: Dict[str, str]


@dataclass
class Relation:
    """OSM Relation representation."""

    id: int
    members: List[Tuple[str, int, str]]  # (type, ref, role)
    tags: Dict[str, str]


@dataclass
class TrafficLightData:
    """Traffic light data from CARLA."""

    id: int
    location: Tuple[float, float, float]  # x, y, z
    trigger_volume_location: Tuple[float, float, float]
    trigger_volume_extent: Tuple[float, float, float]
    orientation: float  # yaw in degrees
    affected_waypoints: List[Tuple[float, float]]
    stop_waypoints: List[Tuple[float, float, float, float]]  # x, y, z, yaw


@dataclass
class SpeedLimitData:
    """Speed limit sign data from CARLA."""

    id: int
    location: Tuple[float, float, float]
    speed_limit: int  # km/h
    trigger_volume_location: Tuple[float, float, float]
    trigger_volume_extent: Tuple[float, float, float]


@dataclass
class StopSignData:
    """Stop sign data from CARLA."""

    id: int
    location: Tuple[float, float, float]
    trigger_volume_location: Tuple[float, float, float]
    trigger_volume_extent: Tuple[float, float, float]
    orientation: float


@dataclass
class YieldSignData:
    """Yield sign data from CARLA."""

    id: int
    location: Tuple[float, float, float]
    trigger_volume_location: Tuple[float, float, float]
    trigger_volume_extent: Tuple[float, float, float]
    orientation: float


@dataclass
class CrosswalkData:
    """Crosswalk data from CARLA."""

    id: int
    vertices: List[Tuple[float, float, float]]  # List of (x, y, z) vertices


@dataclass
class LaneMarkingData:
    """Lane marking data for a lanelet boundary."""

    way_id: int
    marking_type: str  # solid, broken, solid_solid, broken_broken, etc.
    color: str  # white, yellow, etc.


def local_to_latlon(
    x: float, y: float, origin_lat: float = 0.0, origin_lon: float = 0.0
) -> Tuple[float, float]:
    """Convert local x,y coordinates to lat/lon."""
    lat = origin_lat + (y / EARTH_RADIUS) * (180.0 / math.pi)
    lon = origin_lon + (x / (EARTH_RADIUS * math.cos(origin_lat * math.pi / 180.0))) * (
        180.0 / math.pi
    )
    return lat, lon


def latlon_to_local(
    lat: float, lon: float, origin_lat: float = 0.0, origin_lon: float = 0.0
) -> Tuple[float, float]:
    """Convert lat/lon to local x,y coordinates."""
    x = (
        (lon - origin_lon)
        * (math.pi / 180.0)
        * EARTH_RADIUS
        * math.cos(origin_lat * math.pi / 180.0)
    )
    y = (lat - origin_lat) * (math.pi / 180.0) * EARTH_RADIUS
    return x, y


class OSMParser:
    """Parse and modify Lanelet2 OSM files."""

    def __init__(self, osm_path: str):
        self.osm_path = osm_path
        self.tree = ET.parse(osm_path)
        self.root = self.tree.getroot()

        self.nodes: Dict[int, Node] = {}
        self.ways: Dict[int, Way] = {}
        self.relations: Dict[int, Relation] = {}

        self._parse()

    def _parse(self):
        """Parse the OSM file."""
        # Parse nodes
        for node_elem in self.root.findall("node"):
            node_id = int(node_elem.get("id"))
            lat = float(node_elem.get("lat"))
            lon = float(node_elem.get("lon"))

            local_x = 0.0
            local_y = 0.0
            ele = 0.0

            for tag in node_elem.findall("tag"):
                k = tag.get("k")
                v = tag.get("v")
                if k == "local_x":
                    local_x = float(v)
                elif k == "local_y":
                    local_y = float(v)
                elif k == "ele":
                    ele = float(v)

            self.nodes[node_id] = Node(node_id, lat, lon, local_x, local_y, ele)

        # Parse ways
        for way_elem in self.root.findall("way"):
            way_id = int(way_elem.get("id"))
            node_refs = [int(nd.get("ref")) for nd in way_elem.findall("nd")]
            tags = {tag.get("k"): tag.get("v") for tag in way_elem.findall("tag")}
            self.ways[way_id] = Way(way_id, node_refs, tags)

        # Parse relations
        for rel_elem in self.root.findall("relation"):
            rel_id = int(rel_elem.get("id"))
            members = []
            for member in rel_elem.findall("member"):
                members.append(
                    (member.get("type"), int(member.get("ref")), member.get("role"))
                )
            tags = {tag.get("k"): tag.get("v") for tag in rel_elem.findall("tag")}
            self.relations[rel_id] = Relation(rel_id, members, tags)

    def get_max_id(self) -> int:
        """Get the maximum ID used in the OSM file."""
        max_id = 0
        max_id = max(max_id, max(self.nodes.keys()) if self.nodes else 0)
        max_id = max(max_id, max(self.ways.keys()) if self.ways else 0)
        max_id = max(max_id, max(self.relations.keys()) if self.relations else 0)
        return max_id

    def get_lanelets(self) -> Dict[int, Relation]:
        """Get all lanelet relations."""
        return {
            rel_id: rel
            for rel_id, rel in self.relations.items()
            if rel.tags.get("type") == "lanelet"
        }

    def get_lanelet_centerline(self, lanelet_id: int) -> List[Tuple[float, float]]:
        """Get the approximate centerline of a lanelet as local (x, y) points."""
        lanelet = self.relations.get(lanelet_id)
        if not lanelet:
            return []

        left_way_id = None
        right_way_id = None

        for member_type, ref, role in lanelet.members:
            if role == "left":
                left_way_id = ref
            elif role == "right":
                right_way_id = ref

        if not left_way_id or not right_way_id:
            return []

        left_way = self.ways.get(left_way_id)
        right_way = self.ways.get(right_way_id)

        if not left_way or not right_way:
            return []

        # Get left and right boundary points
        left_points = []
        for node_id in left_way.node_refs:
            node = self.nodes.get(node_id)
            if node:
                left_points.append((node.local_x, node.local_y))

        right_points = []
        for node_id in right_way.node_refs:
            node = self.nodes.get(node_id)
            if node:
                right_points.append((node.local_x, node.local_y))

        # Compute centerline by averaging corresponding points
        # Use the shorter length and interpolate
        centerline = []
        num_points = min(len(left_points), len(right_points))
        for i in range(num_points):
            cx = (left_points[i][0] + right_points[i][0]) / 2
            cy = (left_points[i][1] + right_points[i][1]) / 2
            centerline.append((cx, cy))

        return centerline

    def get_lanelet_boundary_at_point(
        self, lanelet_id: int, x: float, y: float
    ) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Get the left and right boundary points of a lanelet at the position closest to (x, y).
        Returns ((left_x, left_y), (right_x, right_y)) or None if not found."""
        lanelet = self.relations.get(lanelet_id)
        if not lanelet:
            return None

        left_way_id = None
        right_way_id = None

        for member_type, ref, role in lanelet.members:
            if role == "left":
                left_way_id = ref
            elif role == "right":
                right_way_id = ref

        if not left_way_id or not right_way_id:
            return None

        left_way = self.ways.get(left_way_id)
        right_way = self.ways.get(right_way_id)

        if not left_way or not right_way:
            return None

        # Get left and right boundary points
        left_points = []
        for node_id in left_way.node_refs:
            node = self.nodes.get(node_id)
            if node:
                left_points.append((node.local_x, node.local_y))

        right_points = []
        for node_id in right_way.node_refs:
            node = self.nodes.get(node_id)
            if node:
                right_points.append((node.local_x, node.local_y))

        if not left_points or not right_points:
            return None

        # Find the centerline point closest to (x, y)
        num_points = min(len(left_points), len(right_points))
        best_idx = 0
        best_dist = float("inf")

        for i in range(num_points):
            cx = (left_points[i][0] + right_points[i][0]) / 2
            cy = (left_points[i][1] + right_points[i][1]) / 2
            dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        return (left_points[best_idx], right_points[best_idx])

    def find_lanelet_at_point(
        self, x: float, y: float, radius: float = 5.0
    ) -> Optional[int]:
        """Find the lanelet whose centerline is closest to (x, y) within radius."""
        best_lanelet = None
        best_dist = float("inf")

        for lanelet_id, lanelet in self.get_lanelets().items():
            centerline = self.get_lanelet_centerline(lanelet_id)
            if not centerline:
                continue

            for cx, cy in centerline:
                dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
                if dist < best_dist and dist < radius:
                    best_dist = dist
                    best_lanelet = lanelet_id

        return best_lanelet

    def find_affected_lanelets(
        self, x: float, y: float, radius: float = 20.0
    ) -> List[int]:
        """Find lanelets whose start is within radius of the given point."""
        affected = []

        for lanelet_id, lanelet in self.get_lanelets().items():
            centerline = self.get_lanelet_centerline(lanelet_id)
            if not centerline:
                continue

            # Check if the start of the lanelet is near the point
            start_x, start_y = centerline[0]
            dist = math.sqrt((start_x - x) ** 2 + (start_y - y) ** 2)

            if dist < radius:
                affected.append(lanelet_id)

        return affected

    def find_lanelets_crossing_line(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> List[int]:
        """Find lanelets that cross or pass near a line segment."""
        affected = []

        for lanelet_id, lanelet in self.get_lanelets().items():
            centerline = self.get_lanelet_centerline(lanelet_id)
            if len(centerline) < 2:
                continue

            # Check if any segment of centerline crosses or is near the line
            for i in range(len(centerline) - 1):
                px1, py1 = centerline[i]
                px2, py2 = centerline[i + 1]

                # Check distance from centerline midpoint to the stop line
                mid_x = (px1 + px2) / 2
                mid_y = (py1 + py2) / 2

                # Distance from midpoint to the line segment
                line_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if line_len < 0.001:
                    continue

                # Project point onto line
                t = max(
                    0,
                    min(
                        1,
                        ((mid_x - x1) * (x2 - x1) + (mid_y - y1) * (y2 - y1))
                        / (line_len**2),
                    ),
                )
                proj_x = x1 + t * (x2 - x1)
                proj_y = y1 + t * (y2 - y1)

                dist = math.sqrt((mid_x - proj_x) ** 2 + (mid_y - proj_y) ** 2)

                if dist < 5.0:  # Within 5 meters of the stop line
                    if lanelet_id not in affected:
                        affected.append(lanelet_id)
                    break

        return affected

    def find_lanelets_near_waypoints(
        self, waypoints: List[Tuple[float, float]], radius: float = 3.0
    ) -> List[int]:
        """Find lanelets whose centerline passes near any of the given waypoints."""
        affected = []

        for lanelet_id, lanelet in self.get_lanelets().items():
            centerline = self.get_lanelet_centerline(lanelet_id)
            if not centerline:
                continue

            # Check if any point on the centerline is near any waypoint
            for cx, cy in centerline:
                for wx, wy in waypoints:
                    dist = math.sqrt((cx - wx) ** 2 + (cy - wy) ** 2)
                    if dist < radius:
                        if lanelet_id not in affected:
                            affected.append(lanelet_id)
                        break
                else:
                    continue
                break

        return affected


class RegulatoryElementGenerator:
    """Generate regulatory elements from CARLA data."""

    def __init__(self, osm_parser: OSMParser):
        self.osm_parser = osm_parser
        self.next_id = osm_parser.get_max_id() + 1

        self.new_nodes: List[Node] = []
        self.new_ways: List[Way] = []
        self.new_relations: List[Relation] = []
        # Track which regulatory elements affect which lanelets
        self.lanelet_regulatory_elements: Dict[int, List[int]] = {}

    def _get_next_id(self) -> int:
        """Get the next available ID."""
        id_ = self.next_id
        self.next_id += 1
        return id_

    def _register_lanelet_regulatory_element(self, lanelet_id: int, reg_elem_id: int):
        """Register that a regulatory element affects a lanelet."""
        if lanelet_id not in self.lanelet_regulatory_elements:
            self.lanelet_regulatory_elements[lanelet_id] = []
        if reg_elem_id not in self.lanelet_regulatory_elements[lanelet_id]:
            self.lanelet_regulatory_elements[lanelet_id].append(reg_elem_id)

    def add_traffic_light(self, tl: TrafficLightData) -> int:
        """Add a traffic light regulatory element."""
        # Create a linestring representing the traffic light bulbs
        # Oriented perpendicular to the traffic light's facing direction
        yaw_rad = math.radians(tl.orientation)
        light_half_width = 1.0  # Approximate half-width of traffic light housing

        # Perpendicular direction (left-right across the light)
        perp_x = math.sin(yaw_rad)
        perp_y = -math.cos(yaw_rad)

        # Two endpoints of the traffic light linestring
        x1 = tl.location[0] + perp_x * light_half_width
        y1 = tl.location[1] + perp_y * light_half_width
        x2 = tl.location[0] - perp_x * light_half_width
        y2 = tl.location[1] - perp_y * light_half_width

        lat1, lon1 = local_to_latlon(x1, y1)
        lat2, lon2 = local_to_latlon(x2, y2)

        light_node1_id = self._get_next_id()
        light_node1 = Node(
            id=light_node1_id,
            lat=lat1,
            lon=lon1,
            local_x=x1,
            local_y=y1,
            ele=tl.location[2],
        )
        self.new_nodes.append(light_node1)

        light_node2_id = self._get_next_id()
        light_node2 = Node(
            id=light_node2_id,
            lat=lat2,
            lon=lon2,
            local_x=x2,
            local_y=y2,
            ele=tl.location[2],
        )
        self.new_nodes.append(light_node2)

        # Create way for light bulbs (LineString with 2+ nodes required by Lanelet2)
        light_way_id = self._get_next_id()
        light_way = Way(
            id=light_way_id,
            node_refs=[light_node1_id, light_node2_id],
            tags={"type": "traffic_light"},
        )
        self.new_ways.append(light_way)

        # Create stop line based on stop waypoints if available, otherwise use trigger volume
        # Note: Lanelet2 only allows ONE stop line per traffic light regulatory element
        stop_line_way_id = None
        all_affected_lanelets = []

        if tl.stop_waypoints:
            # Use CARLA's stop waypoints to find lanelet boundaries and create stop line
            # that spans the full width of all affected lanelets
            all_boundary_points = []  # Collect all left and right boundary points
            avg_z = sum(wp[2] for wp in tl.stop_waypoints) / len(tl.stop_waypoints)

            for wp_x, wp_y, wp_z, wp_yaw in tl.stop_waypoints:
                # Find the lanelet at this waypoint
                lanelet_id = self.osm_parser.find_lanelet_at_point(wp_x, wp_y)
                if lanelet_id:
                    all_affected_lanelets.append(lanelet_id)
                    # Get the boundary points at this position
                    boundaries = self.osm_parser.get_lanelet_boundary_at_point(
                        lanelet_id, wp_x, wp_y
                    )
                    if boundaries:
                        left_pt, right_pt = boundaries
                        all_boundary_points.append(left_pt)
                        all_boundary_points.append(right_pt)

            # If we found boundary points, create stop line spanning them
            if len(all_boundary_points) >= 2:
                # Sort points to create a proper line (sort by x, then y)
                all_boundary_points.sort(key=lambda p: (p[0], p[1]))

                # Use first and last points (extremes) for the stop line
                sl_x1, sl_y1 = all_boundary_points[0]
                sl_x2, sl_y2 = all_boundary_points[-1]

                lat1, lon1 = local_to_latlon(sl_x1, sl_y1)
                lat2, lon2 = local_to_latlon(sl_x2, sl_y2)

                sl_node1_id = self._get_next_id()
                sl_node1 = Node(
                    id=sl_node1_id,
                    lat=lat1,
                    lon=lon1,
                    local_x=sl_x1,
                    local_y=sl_y1,
                    ele=avg_z,
                )
                self.new_nodes.append(sl_node1)

                sl_node2_id = self._get_next_id()
                sl_node2 = Node(
                    id=sl_node2_id,
                    lat=lat2,
                    lon=lon2,
                    local_x=sl_x2,
                    local_y=sl_y2,
                    ele=avg_z,
                )
                self.new_nodes.append(sl_node2)

                stop_line_way_id = self._get_next_id()
                stop_line_way = Way(
                    id=stop_line_way_id,
                    node_refs=[sl_node1_id, sl_node2_id],
                    tags={"type": "stop_line"},
                )
                self.new_ways.append(stop_line_way)
        else:
            # Fall back to trigger volume
            yaw_rad = math.radians(tl.orientation)

            # Stop line center is at trigger volume location
            sl_x = tl.trigger_volume_location[0]
            sl_y = tl.trigger_volume_location[1]
            sl_z = tl.trigger_volume_location[2]

            # Stop line extent (perpendicular to facing direction)
            extent = max(tl.trigger_volume_extent[0], tl.trigger_volume_extent[1])

            # Perpendicular direction
            perp_x = math.sin(yaw_rad)
            perp_y = -math.cos(yaw_rad)

            # Stop line endpoints
            sl_x1 = sl_x + perp_x * extent
            sl_y1 = sl_y + perp_y * extent
            sl_x2 = sl_x - perp_x * extent
            sl_y2 = sl_y - perp_y * extent

            # Create stop line nodes
            lat1, lon1 = local_to_latlon(sl_x1, sl_y1)
            lat2, lon2 = local_to_latlon(sl_x2, sl_y2)

            sl_node1_id = self._get_next_id()
            sl_node1 = Node(
                id=sl_node1_id,
                lat=lat1,
                lon=lon1,
                local_x=sl_x1,
                local_y=sl_y1,
                ele=sl_z,
            )
            self.new_nodes.append(sl_node1)

            sl_node2_id = self._get_next_id()
            sl_node2 = Node(
                id=sl_node2_id,
                lat=lat2,
                lon=lon2,
                local_x=sl_x2,
                local_y=sl_y2,
                ele=sl_z,
            )
            self.new_nodes.append(sl_node2)

            # Create stop line way
            stop_line_way_id = self._get_next_id()
            stop_line_way = Way(
                id=stop_line_way_id,
                node_refs=[sl_node1_id, sl_node2_id],
                tags={"type": "stop_line"},
            )
            self.new_ways.append(stop_line_way)

            # Find affected lanelets
            all_affected_lanelets = self.osm_parser.find_lanelets_crossing_line(
                sl_x1, sl_y1, sl_x2, sl_y2
            )

        # Also use affected waypoints from CARLA if available
        if tl.affected_waypoints:
            waypoint_lanelets = self.osm_parser.find_lanelets_near_waypoints(
                tl.affected_waypoints, radius=5.0
            )
            all_affected_lanelets.extend(waypoint_lanelets)

        # Remove duplicate lanelet IDs
        all_affected_lanelets = list(set(all_affected_lanelets))

        # Create regulatory element relation
        reg_elem_id = self._get_next_id()
        members = []

        # Add the stop line as ref_line (only one allowed per traffic light)
        if stop_line_way_id is not None:
            members.append(("way", stop_line_way_id, "ref_line"))

        # Add the traffic light
        members.append(("way", light_way_id, "refers"))

        # Add affected lanelets as members
        for lanelet_id in all_affected_lanelets:
            members.append(("relation", lanelet_id, "yield"))

        reg_elem = Relation(
            id=reg_elem_id,
            members=members,
            tags={"type": "regulatory_element", "subtype": "traffic_light"},
        )
        self.new_relations.append(reg_elem)

        # Register this regulatory element with affected lanelets
        for lanelet_id in all_affected_lanelets:
            self._register_lanelet_regulatory_element(lanelet_id, reg_elem_id)

        return reg_elem_id

    def add_speed_limit(self, sl: SpeedLimitData) -> int:
        """Add a speed limit regulatory element."""
        # Create a linestring representing the sign face (small horizontal extent)
        sign_half_width = 0.3  # Approximate half-width of sign face

        x1 = sl.location[0] - sign_half_width
        x2 = sl.location[0] + sign_half_width
        y = sl.location[1]

        lat1, lon1 = local_to_latlon(x1, y)
        lat2, lon2 = local_to_latlon(x2, y)

        sign_node1_id = self._get_next_id()
        sign_node1 = Node(
            id=sign_node1_id,
            lat=lat1,
            lon=lon1,
            local_x=x1,
            local_y=y,
            ele=sl.location[2],
        )
        self.new_nodes.append(sign_node1)

        sign_node2_id = self._get_next_id()
        sign_node2 = Node(
            id=sign_node2_id,
            lat=lat2,
            lon=lon2,
            local_x=x2,
            local_y=y,
            ele=sl.location[2],
        )
        self.new_nodes.append(sign_node2)

        # Create way for speed limit sign (LineString with 2+ nodes required by Lanelet2)
        sign_way_id = self._get_next_id()
        sign_way = Way(
            id=sign_way_id,
            node_refs=[sign_node1_id, sign_node2_id],
            tags={"type": "traffic_sign", "subtype": "speed_limit"},
        )
        self.new_ways.append(sign_way)

        # Find affected lanelets (near the sign's trigger volume)
        affected_lanelets = self.osm_parser.find_affected_lanelets(
            sl.trigger_volume_location[0],
            sl.trigger_volume_location[1],
            radius=max(sl.trigger_volume_extent[0], sl.trigger_volume_extent[1]) + 10.0,
        )

        # Create regulatory element relation
        reg_elem_id = self._get_next_id()
        members = [
            ("way", sign_way_id, "refers"),
        ]

        # Add affected lanelets as members
        for lanelet_id in affected_lanelets:
            members.append(("relation", lanelet_id, "lanelet"))

        reg_elem = Relation(
            id=reg_elem_id,
            members=members,
            tags={
                "type": "regulatory_element",
                "subtype": "speed_limit",
                "sign_type": f"de274-{sl.speed_limit}",  # German sign format
                "speed_limit": str(sl.speed_limit),
            },
        )
        self.new_relations.append(reg_elem)

        # Register this regulatory element with affected lanelets
        for lanelet_id in affected_lanelets:
            self._register_lanelet_regulatory_element(lanelet_id, reg_elem_id)

        return reg_elem_id

    def add_stop_sign(self, ss: StopSignData) -> int:
        """Add a stop sign regulatory element with stop line."""
        # Create a linestring representing the sign face, perpendicular to orientation
        yaw_rad = math.radians(ss.orientation)
        sign_half_width = 0.3  # Approximate half-width of sign face

        perp_x = math.sin(yaw_rad)
        perp_y = -math.cos(yaw_rad)

        x1 = ss.location[0] + perp_x * sign_half_width
        y1 = ss.location[1] + perp_y * sign_half_width
        x2 = ss.location[0] - perp_x * sign_half_width
        y2 = ss.location[1] - perp_y * sign_half_width

        lat1, lon1 = local_to_latlon(x1, y1)
        lat2, lon2 = local_to_latlon(x2, y2)

        sign_node1_id = self._get_next_id()
        sign_node1 = Node(
            id=sign_node1_id,
            lat=lat1,
            lon=lon1,
            local_x=x1,
            local_y=y1,
            ele=ss.location[2],
        )
        self.new_nodes.append(sign_node1)

        sign_node2_id = self._get_next_id()
        sign_node2 = Node(
            id=sign_node2_id,
            lat=lat2,
            lon=lon2,
            local_x=x2,
            local_y=y2,
            ele=ss.location[2],
        )
        self.new_nodes.append(sign_node2)

        # Create way for stop sign (LineString with 2+ nodes required by Lanelet2)
        sign_way_id = self._get_next_id()
        sign_way = Way(
            id=sign_way_id,
            node_refs=[sign_node1_id, sign_node2_id],
            tags={"type": "traffic_sign", "subtype": "stop"},
        )
        self.new_ways.append(sign_way)

        # Create stop line based on trigger volume
        yaw_rad = math.radians(ss.orientation)

        # Stop line center is at trigger volume location
        sl_x = ss.trigger_volume_location[0]
        sl_y = ss.trigger_volume_location[1]
        sl_z = ss.trigger_volume_location[2]

        # Stop line extent (perpendicular to facing direction)
        extent = max(ss.trigger_volume_extent[0], ss.trigger_volume_extent[1])

        # Perpendicular direction
        perp_x = math.sin(yaw_rad)
        perp_y = -math.cos(yaw_rad)

        # Stop line endpoints
        sl_x1 = sl_x + perp_x * extent
        sl_y1 = sl_y + perp_y * extent
        sl_x2 = sl_x - perp_x * extent
        sl_y2 = sl_y - perp_y * extent

        # Create stop line nodes
        lat1, lon1 = local_to_latlon(sl_x1, sl_y1)
        lat2, lon2 = local_to_latlon(sl_x2, sl_y2)

        sl_node1_id = self._get_next_id()
        sl_node1 = Node(
            id=sl_node1_id, lat=lat1, lon=lon1, local_x=sl_x1, local_y=sl_y1, ele=sl_z
        )
        self.new_nodes.append(sl_node1)

        sl_node2_id = self._get_next_id()
        sl_node2 = Node(
            id=sl_node2_id, lat=lat2, lon=lon2, local_x=sl_x2, local_y=sl_y2, ele=sl_z
        )
        self.new_nodes.append(sl_node2)

        # Create stop line way
        stop_line_way_id = self._get_next_id()
        stop_line_way = Way(
            id=stop_line_way_id,
            node_refs=[sl_node1_id, sl_node2_id],
            tags={"type": "stop_line"},
        )
        self.new_ways.append(stop_line_way)

        # Find affected lanelets
        affected_lanelets = self.osm_parser.find_lanelets_crossing_line(
            sl_x1, sl_y1, sl_x2, sl_y2
        )

        # Create regulatory element relation
        reg_elem_id = self._get_next_id()
        members = [
            ("way", stop_line_way_id, "ref_line"),
            ("way", sign_way_id, "refers"),
        ]

        # Add affected lanelets as members
        for lanelet_id in affected_lanelets:
            members.append(("relation", lanelet_id, "lanelet"))

        reg_elem = Relation(
            id=reg_elem_id,
            members=members,
            tags={
                "type": "regulatory_element",
                "subtype": "traffic_sign",
                "sign_type": "stop",
            },
        )
        self.new_relations.append(reg_elem)

        # Register this regulatory element with affected lanelets
        for lanelet_id in affected_lanelets:
            self._register_lanelet_regulatory_element(lanelet_id, reg_elem_id)

        return reg_elem_id

    def add_yield_sign(self, ys: YieldSignData) -> int:
        """Add a yield sign regulatory element."""
        # Create a linestring representing the sign face, perpendicular to orientation
        yaw_rad = math.radians(ys.orientation)
        sign_half_width = 0.3  # Approximate half-width of sign face

        perp_x = math.sin(yaw_rad)
        perp_y = -math.cos(yaw_rad)

        x1 = ys.location[0] + perp_x * sign_half_width
        y1 = ys.location[1] + perp_y * sign_half_width
        x2 = ys.location[0] - perp_x * sign_half_width
        y2 = ys.location[1] - perp_y * sign_half_width

        lat1, lon1 = local_to_latlon(x1, y1)
        lat2, lon2 = local_to_latlon(x2, y2)

        sign_node1_id = self._get_next_id()
        sign_node1 = Node(
            id=sign_node1_id,
            lat=lat1,
            lon=lon1,
            local_x=x1,
            local_y=y1,
            ele=ys.location[2],
        )
        self.new_nodes.append(sign_node1)

        sign_node2_id = self._get_next_id()
        sign_node2 = Node(
            id=sign_node2_id,
            lat=lat2,
            lon=lon2,
            local_x=x2,
            local_y=y2,
            ele=ys.location[2],
        )
        self.new_nodes.append(sign_node2)

        # Create way for yield sign (LineString with 2+ nodes required by Lanelet2)
        sign_way_id = self._get_next_id()
        sign_way = Way(
            id=sign_way_id,
            node_refs=[sign_node1_id, sign_node2_id],
            tags={"type": "traffic_sign", "subtype": "yield"},
        )
        self.new_ways.append(sign_way)

        # Find affected lanelets (near the sign's trigger volume)
        affected_lanelets = self.osm_parser.find_affected_lanelets(
            ys.trigger_volume_location[0],
            ys.trigger_volume_location[1],
            radius=max(ys.trigger_volume_extent[0], ys.trigger_volume_extent[1]) + 10.0,
        )

        # Create regulatory element relation
        reg_elem_id = self._get_next_id()
        members = [
            ("way", sign_way_id, "refers"),
        ]

        # Add affected lanelets as members
        for lanelet_id in affected_lanelets:
            members.append(("relation", lanelet_id, "lanelet"))

        reg_elem = Relation(
            id=reg_elem_id,
            members=members,
            tags={
                "type": "regulatory_element",
                "subtype": "traffic_sign",
                "sign_type": "yield",
            },
        )
        self.new_relations.append(reg_elem)

        # Register this regulatory element with affected lanelets
        for lanelet_id in affected_lanelets:
            self._register_lanelet_regulatory_element(lanelet_id, reg_elem_id)

        return reg_elem_id

    def add_crosswalk(self, cw: CrosswalkData) -> int:
        """Add a crosswalk regulatory element."""
        # Create nodes for crosswalk polygon vertices
        node_ids = []
        for vx, vy, vz in cw.vertices:
            lat, lon = local_to_latlon(vx, vy)
            node_id = self._get_next_id()
            node = Node(id=node_id, lat=lat, lon=lon, local_x=vx, local_y=vy, ele=vz)
            self.new_nodes.append(node)
            node_ids.append(node_id)

        # Create way for crosswalk polygon (closed)
        crosswalk_way_id = self._get_next_id()
        crosswalk_way = Way(
            id=crosswalk_way_id,
            node_refs=node_ids,
            tags={"type": "crosswalk", "area": "yes"},
        )
        self.new_ways.append(crosswalk_way)

        # Find affected lanelets by checking which lanelets cross the crosswalk
        # Use the centroid of the crosswalk
        centroid_x = sum(v[0] for v in cw.vertices) / len(cw.vertices)
        centroid_y = sum(v[1] for v in cw.vertices) / len(cw.vertices)

        # Calculate approximate extent
        max_dist = max(
            math.sqrt((v[0] - centroid_x) ** 2 + (v[1] - centroid_y) ** 2)
            for v in cw.vertices
        )

        affected_lanelets = self.osm_parser.find_affected_lanelets(
            centroid_x, centroid_y, radius=max_dist + 5.0
        )

        # Create regulatory element relation
        reg_elem_id = self._get_next_id()
        members = [
            ("way", crosswalk_way_id, "ref_line"),
        ]

        # Add affected lanelets as members
        for lanelet_id in affected_lanelets:
            members.append(("relation", lanelet_id, "yield"))

        reg_elem = Relation(
            id=reg_elem_id,
            members=members,
            tags={"type": "regulatory_element", "subtype": "crosswalk"},
        )
        self.new_relations.append(reg_elem)

        # Register this regulatory element with affected lanelets
        for lanelet_id in affected_lanelets:
            self._register_lanelet_regulatory_element(lanelet_id, reg_elem_id)

        return reg_elem_id

    def update_lane_markings(self, lane_markings: List[LaneMarkingData]):
        """Update existing ways with lane marking information."""
        updated_count = 0
        for lm in lane_markings:
            if lm.way_id in self.osm_parser.ways:
                way = self.osm_parser.ways[lm.way_id]
                # Add lane marking tags
                way.tags["type"] = "line_thin"
                way.tags["subtype"] = lm.marking_type
                if lm.color:
                    way.tags["color"] = lm.color
                updated_count += 1
        return updated_count


class CARLADataExtractor:
    """Extract traffic regulatory data from CARLA."""

    def __init__(self, host: str = "localhost", port: int = 2000):
        if carla is None:
            raise RuntimeError("CARLA Python API not available")

        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

    def get_traffic_lights(self) -> List[TrafficLightData]:
        """Extract all traffic lights from CARLA."""
        traffic_lights = []

        actors = self.world.get_actors()
        tl_actors = [a for a in actors if "traffic_light" in a.type_id]

        for tl in tl_actors:
            location = tl.get_location()
            transform = tl.get_transform()
            trigger = tl.trigger_volume

            # Get affected waypoints (negate Y for coordinate system conversion)
            affected_wps = []
            try:
                affected_lanes = tl.get_affected_lane_waypoints()
                for wp in affected_lanes:
                    affected_wps.append(
                        (wp.transform.location.x, -wp.transform.location.y)
                    )
            except Exception:
                pass

            # Get stop waypoints (where vehicles should stop)
            # Negate Y coordinate to match OSM map coordinate system
            stop_wps = []
            try:
                stop_waypoints = tl.get_stop_waypoints()
                for wp in stop_waypoints:
                    stop_wps.append(
                        (
                            wp.transform.location.x,
                            -wp.transform.location.y,  # Negate Y
                            wp.transform.location.z,
                            -wp.transform.rotation.yaw,  # Negate yaw as well
                        )
                    )
            except Exception:
                pass

            # Transform trigger volume to world coordinates
            trigger_world = transform.transform(trigger.location)

            tl_data = TrafficLightData(
                id=tl.id,
                location=(location.x, -location.y, location.z),  # Negate Y
                trigger_volume_location=(
                    trigger_world.x,
                    -trigger_world.y,  # Negate Y
                    trigger_world.z,
                ),
                trigger_volume_extent=(
                    trigger.extent.x,
                    trigger.extent.y,
                    trigger.extent.z,
                ),
                orientation=-transform.rotation.yaw,  # Negate yaw
                affected_waypoints=affected_wps,
                stop_waypoints=stop_wps,
            )
            traffic_lights.append(tl_data)

        return traffic_lights

    def get_speed_limits(self) -> List[SpeedLimitData]:
        """Extract all speed limit signs from CARLA."""
        speed_limits = []

        actors = self.world.get_actors()
        sl_actors = [a for a in actors if "speed_limit" in a.type_id]

        for sl in sl_actors:
            location = sl.get_location()
            transform = sl.get_transform()
            trigger = sl.trigger_volume

            # Extract speed limit value from type_id (e.g., "traffic.speed_limit.30")
            try:
                speed_value = int(sl.type_id.split(".")[-1])
            except ValueError:
                speed_value = 50  # Default

            # Transform trigger volume to world coordinates
            trigger_world = transform.transform(trigger.location)

            sl_data = SpeedLimitData(
                id=sl.id,
                location=(location.x, -location.y, location.z),  # Negate Y
                speed_limit=speed_value,
                trigger_volume_location=(
                    trigger_world.x,
                    -trigger_world.y,  # Negate Y
                    trigger_world.z,
                ),
                trigger_volume_extent=(
                    trigger.extent.x,
                    trigger.extent.y,
                    trigger.extent.z,
                ),
            )
            speed_limits.append(sl_data)

        return speed_limits

    def get_stop_signs(self) -> List[StopSignData]:
        """Extract stop sign data from CARLA."""
        stop_signs = []

        actors = self.world.get_actors()
        ss_actors = [
            a
            for a in actors
            if "stop" in a.type_id.lower() and "traffic" in a.type_id.lower()
        ]

        for ss in ss_actors:
            location = ss.get_location()
            transform = ss.get_transform()
            trigger = ss.trigger_volume

            # Transform trigger volume to world coordinates
            trigger_world = transform.transform(trigger.location)

            ss_data = StopSignData(
                id=ss.id,
                location=(location.x, -location.y, location.z),  # Negate Y
                trigger_volume_location=(
                    trigger_world.x,
                    -trigger_world.y,  # Negate Y
                    trigger_world.z,
                ),
                trigger_volume_extent=(
                    trigger.extent.x,
                    trigger.extent.y,
                    trigger.extent.z,
                ),
                orientation=-transform.rotation.yaw,  # Negate yaw
            )
            stop_signs.append(ss_data)

        return stop_signs

    def get_yield_signs(self) -> List[YieldSignData]:
        """Extract yield sign data from CARLA landmarks."""
        yield_signs = []

        try:
            # Get all landmarks from the map
            landmarks = self.carla_map.get_all_landmarks()

            for landmark in landmarks:
                # Check if it's a yield sign (type 205 in Vienna Convention, or name contains 'yield')
                if "yield" in landmark.name.lower() or landmark.type == "205":
                    transform = landmark.transform
                    location = transform.location

                    # Create yield sign data (negate Y coordinates for coordinate system)
                    ys_data = YieldSignData(
                        id=landmark.id,
                        location=(location.x, -location.y, location.z),  # Negate Y
                        trigger_volume_location=(
                            location.x,
                            -location.y,
                            location.z,
                        ),  # Negate Y
                        trigger_volume_extent=(5.0, 5.0, 2.0),  # Default extent
                        orientation=-transform.rotation.yaw,  # Negate yaw
                    )
                    yield_signs.append(ys_data)
        except Exception as e:
            print(f"    Warning: Could not extract yield signs from landmarks: {e}")

        return yield_signs

    def get_crosswalks(self) -> List[CrosswalkData]:
        """Extract crosswalk data from CARLA map."""
        crosswalks = []

        try:
            # get_crosswalks returns a list of locations forming crosswalk polygons
            # Each crosswalk is a set of vertices (5 per crosswalk - closed polygon)
            crosswalk_vertices = self.carla_map.get_crosswalks()

            if crosswalk_vertices:
                # Group vertices into crosswalks (each crosswalk has 5 vertices - closed polygon)
                vertices_per_crosswalk = 5
                num_crosswalks = len(crosswalk_vertices) // vertices_per_crosswalk

                for i in range(num_crosswalks):
                    start_idx = i * vertices_per_crosswalk
                    end_idx = start_idx + vertices_per_crosswalk

                    vertices = []
                    for j in range(start_idx, end_idx):
                        loc = crosswalk_vertices[j]
                        vertices.append((loc.x, loc.y, loc.z))

                    cw_data = CrosswalkData(
                        id=i + 1,  # Generate ID
                        vertices=vertices,
                    )
                    crosswalks.append(cw_data)
        except Exception as e:
            print(f"    Warning: Could not extract crosswalks: {e}")

        return crosswalks

    def get_lane_markings(self, osm_parser) -> List[LaneMarkingData]:
        """Extract lane marking types for lanelet boundaries."""
        lane_markings = []

        try:
            lanelets = osm_parser.get_lanelets()

            for lanelet_id, lanelet in lanelets.items():
                # Get left and right way IDs
                left_way_id = None
                right_way_id = None

                for member_type, ref, role in lanelet.members:
                    if role == "left":
                        left_way_id = ref
                    elif role == "right":
                        right_way_id = ref

                if not left_way_id or not right_way_id:
                    continue

                # Get centerline to sample waypoints
                centerline = osm_parser.get_lanelet_centerline(lanelet_id)
                if not centerline:
                    continue

                # Sample a point from the middle of the lanelet
                mid_idx = len(centerline) // 2
                cx, cy = centerline[mid_idx]

                # Get CARLA waypoint at this location
                # Negate Y to convert from OSM coordinates back to CARLA coordinates
                carla_loc = carla.Location(x=cx, y=-cy, z=0.5)
                waypoint = self.carla_map.get_waypoint(carla_loc)

                if waypoint:
                    # The OSM map preserves CARLA's left/right labeling even after Y negation
                    # So OSM left boundary = CARLA left, OSM right boundary = CARLA right

                    # Get left lane marking -> apply to OSM left boundary
                    left_marking = waypoint.left_lane_marking
                    if left_marking:
                        marking_type = self._convert_marking_type(left_marking.type)
                        color = self._convert_marking_color(left_marking.color)
                        if marking_type:
                            lane_markings.append(
                                LaneMarkingData(
                                    way_id=left_way_id,
                                    marking_type=marking_type,
                                    color=color,
                                )
                            )

                    # Get right lane marking -> apply to OSM right boundary
                    right_marking = waypoint.right_lane_marking
                    if right_marking:
                        marking_type = self._convert_marking_type(right_marking.type)
                        color = self._convert_marking_color(right_marking.color)
                        if marking_type:
                            lane_markings.append(
                                LaneMarkingData(
                                    way_id=right_way_id,
                                    marking_type=marking_type,
                                    color=color,
                                )
                            )

        except Exception as e:
            print(f"    Warning: Could not extract lane markings: {e}")

        return lane_markings

    def _convert_marking_type(self, carla_type) -> Optional[str]:
        """Convert CARLA lane marking type to Lanelet2 format."""
        type_map = {
            carla.LaneMarkingType.Solid: "solid",
            carla.LaneMarkingType.Broken: "dashed",
            carla.LaneMarkingType.SolidSolid: "solid_solid",
            carla.LaneMarkingType.SolidBroken: "solid_dashed",
            carla.LaneMarkingType.BrokenSolid: "dashed_solid",
            carla.LaneMarkingType.BrokenBroken: "dashed",
            carla.LaneMarkingType.BottsDots: "dashed",
            carla.LaneMarkingType.Grass: None,
            carla.LaneMarkingType.Curb: "curb",
            carla.LaneMarkingType.NONE: None,
        }
        return type_map.get(carla_type, None)

    def _convert_marking_color(self, carla_color) -> str:
        """Convert CARLA lane marking color to Lanelet2 format."""
        color_map = {
            carla.LaneMarkingColor.White: "white",
            carla.LaneMarkingColor.Yellow: "yellow",
            carla.LaneMarkingColor.Blue: "blue",
            carla.LaneMarkingColor.Green: "green",
            carla.LaneMarkingColor.Red: "red",
            carla.LaneMarkingColor.Other: "white",
        }
        return color_map.get(carla_color, "white")


def write_osm_file(
    osm_parser: OSMParser, generator: RegulatoryElementGenerator, output_path: str
):
    """Write the modified OSM file."""
    # Create new root
    root = ET.Element("osm", version="0.6")
    root.text = "\n  "

    # Add existing nodes
    all_nodes = list(osm_parser.nodes.values()) + generator.new_nodes
    all_nodes.sort(key=lambda n: n.id)

    for node in all_nodes:
        node_elem = ET.SubElement(
            root,
            "node",
            {
                "id": str(node.id),
                "lat": str(node.lat),
                "lon": str(node.lon),
                "version": "1",
                "visible": "True",
            },
        )
        node_elem.text = "\n    "
        node_elem.tail = "\n  "

        ET.SubElement(
            node_elem, "tag", {"k": "local_y", "v": str(node.local_y)}
        ).tail = "\n    "
        ET.SubElement(
            node_elem, "tag", {"k": "local_x", "v": str(node.local_x)}
        ).tail = "\n    "
        ele_tag = ET.SubElement(node_elem, "tag", {"k": "ele", "v": str(node.ele)})
        ele_tag.tail = "\n  "

    # Add existing ways
    all_ways = list(osm_parser.ways.values()) + generator.new_ways
    all_ways.sort(key=lambda w: w.id)

    for way in all_ways:
        way_elem = ET.SubElement(
            root, "way", {"id": str(way.id), "version": "1", "visible": "true"}
        )
        way_elem.text = "\n    "
        way_elem.tail = "\n  "

        for node_ref in way.node_refs:
            nd_elem = ET.SubElement(way_elem, "nd", {"ref": str(node_ref)})
            nd_elem.tail = "\n    "

        for k, v in way.tags.items():
            tag_elem = ET.SubElement(way_elem, "tag", {"k": k, "v": v})
            tag_elem.tail = "\n    "

    # Add existing relations
    all_relations = list(osm_parser.relations.values()) + generator.new_relations
    all_relations.sort(key=lambda r: r.id)

    for rel in all_relations:
        rel_elem = ET.SubElement(
            root, "relation", {"id": str(rel.id), "version": "1", "visible": "true"}
        )
        rel_elem.text = "\n    "
        rel_elem.tail = "\n  "

        for member_type, ref, role in rel.members:
            member_elem = ET.SubElement(
                rel_elem, "member", {"ref": str(ref), "role": role, "type": member_type}
            )
            member_elem.tail = "\n    "

        # If this is a lanelet, add references to its regulatory elements
        if (
            rel.tags.get("type") == "lanelet"
            and rel.id in generator.lanelet_regulatory_elements
        ):
            for reg_elem_id in generator.lanelet_regulatory_elements[rel.id]:
                member_elem = ET.SubElement(
                    rel_elem,
                    "member",
                    {
                        "ref": str(reg_elem_id),
                        "role": "regulatory_element",
                        "type": "relation",
                    },
                )
                member_elem.tail = "\n    "

        for k, v in rel.tags.items():
            tag_elem = ET.SubElement(rel_elem, "tag", {"k": k, "v": v})
            tag_elem.tail = "\n    "

    # Write the file
    tree = ET.ElementTree(root)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
        f.write("<!--generated with add_regulatory_elements.py-->\n")
        tree.write(f, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(
        description="Add regulatory elements from CARLA to Lanelet2 OSM file"
    )
    parser.add_argument(
        "--input", "-i", default="Town10HD.osm", help="Input OSM file path"
    )
    parser.add_argument(
        "--output",
        "-o",
        default="Town10HD_with_regulatory.osm",
        help="Output OSM file path",
    )
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")
    parser.add_argument(
        "--no-carla",
        action="store_true",
        help="Skip CARLA connection (for testing OSM parsing only)",
    )

    args = parser.parse_args()

    print(f"Parsing OSM file: {args.input}")
    osm_parser = OSMParser(args.input)
    print(f"  Found {len(osm_parser.nodes)} nodes")
    print(f"  Found {len(osm_parser.ways)} ways")
    print(f"  Found {len(osm_parser.relations)} relations")
    print(f"  Found {len(osm_parser.get_lanelets())} lanelets")
    print(f"  Max ID: {osm_parser.get_max_id()}")

    generator = RegulatoryElementGenerator(osm_parser)

    if not args.no_carla:
        try:
            print(f"\nConnecting to CARLA at {args.host}:{args.port}...")
            extractor = CARLADataExtractor(args.host, args.port)

            print("Extracting traffic lights...")
            traffic_lights = extractor.get_traffic_lights()
            print(f"  Found {len(traffic_lights)} traffic lights")

            for tl in traffic_lights:
                reg_id = generator.add_traffic_light(tl)
                print(f"    Added traffic light {tl.id} as regulatory element {reg_id}")

            print("Extracting speed limits...")
            speed_limits = extractor.get_speed_limits()
            print(f"  Found {len(speed_limits)} speed limit signs")

            for sl in speed_limits:
                reg_id = generator.add_speed_limit(sl)
                print(
                    f"    Added speed limit {sl.speed_limit} km/h as regulatory element {reg_id}"
                )

            print("Extracting stop signs...")
            stop_signs = extractor.get_stop_signs()
            print(f"  Found {len(stop_signs)} stop signs")

            for ss in stop_signs:
                reg_id = generator.add_stop_sign(ss)
                print(f"    Added stop sign {ss.id} as regulatory element {reg_id}")

            print("Extracting yield signs...")
            yield_signs = extractor.get_yield_signs()
            print(f"  Found {len(yield_signs)} yield signs")

            for ys in yield_signs:
                reg_id = generator.add_yield_sign(ys)
                print(f"    Added yield sign {ys.id} as regulatory element {reg_id}")

            # Note: Crosswalks are skipped as they're not a standard Lanelet2 regulatory element type
            # print("Extracting crosswalks...")
            # crosswalks = extractor.get_crosswalks()
            # print(f"  Found {len(crosswalks)} crosswalks")
            # for cw in crosswalks:
            #     reg_id = generator.add_crosswalk(cw)
            #     print(f"    Added crosswalk {cw.id} as regulatory element {reg_id}")

            print("Extracting lane markings...")
            lane_markings = extractor.get_lane_markings(osm_parser)
            print(f"  Found {len(lane_markings)} lane marking segments")

            updated = generator.update_lane_markings(lane_markings)
            print(f"    Updated {updated} lanelet boundaries with marking types")

        except Exception as e:
            print(f"Error connecting to CARLA: {e}")
            print("To run without CARLA, use --no-carla flag")
            return 1
    else:
        print("\nSkipping CARLA connection (--no-carla flag)")

    print(f"\nWriting output to: {args.output}")
    print(f"  Adding {len(generator.new_nodes)} new nodes")
    print(f"  Adding {len(generator.new_ways)} new ways")
    print(f"  Adding {len(generator.new_relations)} new regulatory elements")

    write_osm_file(osm_parser, generator, args.output)
    print("Done!")

    return 0


if __name__ == "__main__":
    exit(main())
