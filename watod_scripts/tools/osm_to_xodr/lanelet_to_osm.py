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

"""Adapt simple, uniformly sampled, one-way Lanelet2 roads for CARLA Osm2Odr."""

import math
import xml.etree.ElementTree as ET


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def distance(a, b):
    """Local distance in metres, sufficient for comparing nearby lane boundaries."""
    lat = math.radians((a[0] + b[0]) / 2)
    return 6371000 * math.hypot(
        math.radians(a[0] - b[0]), math.radians(a[1] - b[1]) * math.cos(lat)
    )


def local_projection(root):
    points = [(float(n.get("lat")), float(n.get("lon"))) for n in root.findall("node")]
    lat = (min(p[0] for p in points) + max(p[0] for p in points)) / 2
    lon = (min(p[1] for p in points) + max(p[1] for p in points)) / 2
    return f"+proj=tmerc +lat_0={lat} +lon_0={lon} +datum=WGS84 +units=m +no_defs"


def lanelet_roads(root, lane_width=None):
    """Return ordinary highway ways, inferred lane width, and source lanelet count.

    Shared boundaries define adjacent lanes. Shared pairs of boundary endpoints
    define road connections. Only equally sampled, approximately uniform-width,
    one-way lanelets are supported; unsupported layouts fail before conversion.
    """
    nodes = {}
    for node in root.findall("node"):
        try:
            point = float(node.get("lat")), float(node.get("lon"))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "Lanelet2 nodes require WGS84 lat/lon coordinates"
            ) from exc
        if not all(math.isfinite(x) for x in point) or not (
            -90 <= point[0] <= 90 and -180 <= point[1] <= 180
        ):
            raise ValueError(f"invalid coordinates on node {node.get('id')}")
        nodes[node.get("id")] = point
    ways = {
        way.get("id"): tuple(nd.get("ref") for nd in way.findall("nd"))
        for way in root.findall("way")
    }
    lanelets = []
    widths = []
    for relation in root.findall("relation"):
        attributes = tags(relation)
        if attributes.get("type") != "lanelet":
            continue
        relation_id = relation.get("id")
        if (
            attributes.get("subtype", "road") != "road"
            or attributes.get("participant:vehicle") == "no"
        ):
            raise ValueError(
                f"lanelet {relation_id}: only vehicle road lanelets supported"
            )
        if attributes.get("one_way", "yes") != "yes":
            raise ValueError(f"lanelet {relation_id}: two-way lanelets are unsupported")
        bounds = {}
        for role in ("left", "right"):
            members = [
                member
                for member in relation.findall("member")
                if member.get("role") == role and member.get("type") == "way"
            ]
            if len(members) != 1 or members[0].get("ref") not in ways:
                raise ValueError(f"lanelet {relation_id}: requires one {role} boundary")
            refs = ways[members[0].get("ref")]
            if len(refs) < 2 or any(ref not in nodes for ref in refs):
                raise ValueError(
                    f"lanelet {relation_id}: invalid {role} boundary nodes"
                )
            bounds[role] = refs
        left, right = bounds["left"], bounds["right"]
        if len(left) != len(right):
            raise ValueError(
                f"lanelet {relation_id}: boundaries must have matching sample counts"
            )
        # Boundary ways can be stored in opposite directions in the XML.
        aligned = distance(nodes[left[0]], nodes[right[0]]) + distance(
            nodes[left[-1]], nodes[right[-1]]
        )
        reversed_distance = distance(nodes[left[0]], nodes[right[-1]]) + distance(
            nodes[left[-1]], nodes[right[0]]
        )
        if reversed_distance < aligned:
            right = right[::-1]
        # Infer travel direction from the geometric left/right roles.
        a, b, c = nodes[left[0]], nodes[left[1]], nodes[right[0]]
        cross = (b[1] - a[1]) * (a[0] - c[0]) - (b[0] - a[0]) * (a[1] - c[1])
        if cross < 0:
            left, right = left[::-1], right[::-1]
        widths.extend(distance(nodes[a], nodes[b]) for a, b in zip(left, right))
        lanelets.append((left, right, attributes))
    if not lanelets:
        raise ValueError("--lanelet requires Lanelet2 relations with type=lanelet")
    inferred_width = sum(widths) / len(widths)
    if inferred_width < 0.5 or max(widths) - min(widths) > max(
        0.1, inferred_width * 0.05
    ):
        raise ValueError("Lanelet2 mode requires approximately uniform lane widths")
    lane_width = inferred_width if lane_width is None else lane_width

    # Follow shared boundaries from the rightmost to the leftmost lane.
    by_right = {}
    by_left = {}
    for index, (left, right, _) in enumerate(lanelets):
        if left in by_left or right in by_right:
            raise ValueError("Lanelet2 mode does not support branching lane boundaries")
        by_right[right] = index
        by_left[left] = index
    groups = []
    visited = set()
    for index, (_, right, _) in enumerate(lanelets):
        if right in by_left:
            continue
        group = []
        while index is not None:
            if index in visited:
                raise ValueError("cyclic or overlapping Lanelet2 adjacency")
            visited.add(index)
            group.append(index)
            index = by_right.get(lanelets[index][0])
        groups.append(group)
    if len(visited) != len(lanelets):
        raise ValueError("Lanelet2 boundaries could not be grouped into road sections")

    output = ET.Element("osm", version="0.6", generator="watod osm_to_xodr")
    center_nodes = {}
    road_elements = []
    for group in groups:
        right = lanelets[group[0]][1]
        left = lanelets[group[-1]][0]
        if len(left) != len(right):
            raise ValueError("outer road boundaries must have matching sample counts")
        speeds = {lanelets[index][2].get("speed_limit") for index in group}
        if len(speeds) > 1:
            raise ValueError("adjacent Lanelet2 lanes have incompatible speed limits")
        road = ET.Element("way", id=str(len(road_elements) + 1))
        for left_ref, right_ref in zip(left, right):
            key = left_ref, right_ref
            if key not in center_nodes:
                node_id = str(len(center_nodes) + 1)
                center_nodes[key] = node_id
                a, b = nodes[left_ref], nodes[right_ref]
                ET.SubElement(
                    output,
                    "node",
                    id=node_id,
                    lat=str((a[0] + b[0]) / 2),
                    lon=str((a[1] + b[1]) / 2),
                )
            ET.SubElement(road, "nd", ref=center_nodes[key])
        attributes = {
            "highway": "unclassified",
            "oneway": "yes",
            "lanes": str(len(group)),
            "width": str(lane_width * len(group)),
        }
        speed = next(iter(speeds))
        if speed:
            attributes["maxspeed"] = speed
        for key, value in attributes.items():
            ET.SubElement(road, "tag", k=key, v=value)
        road_elements.append(road)
    output.extend(road_elements)
    return output, lane_width, len(lanelets)
