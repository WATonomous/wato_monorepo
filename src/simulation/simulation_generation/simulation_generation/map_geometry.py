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
"""Project Lanelet2 driving surfaces and validate anchored generation offline."""

import copy
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from pyproj import Transformer
from pyproj.exceptions import ProjError
from shapely.geometry import LineString, Point, Polygon, box, mapping
from shapely.ops import unary_union
from shapely.validation import explain_validity

from simulation_runtime.assets import finite_number

STOP_PAD_M = 10.0
SAMPLING_ALLOWANCE_M = 0.1


def projector(parameters):
    """Match world_model's fixed-origin UTM or LocalCartesian projection."""
    if parameters.get("alignment_mode") != "fixed":
        raise ValueError(
            "Map boundary validation requires alignment_mode: fixed; an offline "
            "generator cannot resolve a live projected_frame -> map TF"
        )
    lat, lon, alt = (
        finite_number(parameters.get(key), key)
        for key in ("origin_lat", "origin_lon", "origin_alt")
    )
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError("Invalid geographic map origin")
    if parameters.get("projector_type") == "utm":
        if not -80 <= lat <= 84:
            raise ValueError("UTM map origin must be between 80 S and 84 N")
        zone = min(60, int((lon + 180) / 6) + 1)
        # GeographicLib's standard UTM zone includes Norway/Svalbard exceptions.
        if 56 <= lat < 64 and 3 <= lon < 12:
            zone = 32
        elif 72 <= lat < 84 and 0 <= lon < 42:
            zone = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
        transform = Transformer.from_crs(
            "EPSG:4326", (32600 if lat >= 0 else 32700) + zone, always_xy=True
        )
        ox, oy = transform.transform(lon, lat, errcheck=True)

        def forward(latitude, longitude, altitude):
            x, y = transform.transform(longitude, latitude, errcheck=True)
            return x - ox, y - oy, altitude - alt

        return forward
    if parameters.get("projector_type") != "local_cartesian":
        raise ValueError("Boundary projection must be utm or local_cartesian")
    transform = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    origin = transform.transform(lon, lat, alt, errcheck=True)
    phi, lam = math.radians(lat), math.radians(lon)

    def forward(latitude, longitude, altitude):
        point = transform.transform(longitude, latitude, altitude, errcheck=True)
        dx, dy, dz = (a - b for a, b in zip(point, origin))
        return (
            -math.sin(lam) * dx + math.cos(lam) * dy,
            -math.sin(phi) * math.cos(lam) * dx
            - math.sin(phi) * math.sin(lam) * dy
            + math.cos(phi) * dz,
            math.cos(phi) * math.cos(lam) * dx
            + math.cos(phi) * math.sin(lam) * dy
            + math.sin(phi) * dz,
        )

    return forward


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def driving_lane(attributes):
    return (
        attributes.get("type") == "lanelet"
        and attributes.get("subtype", "road") in {"road", "highway", "play_street"}
        and attributes.get("participant:vehicle", "yes") != "no"
        and attributes.get("participant:vehicle:car", "yes") != "no"
    )


class MapGeometry:
    def __init__(self, osm_path, parameters, spawn, *, clearance_m=3.0):
        self.clearance = finite_number(
            clearance_m, "boundary clearance", minimum=0, strict=True
        )
        self.source_sha256 = hashlib.sha256(Path(osm_path).read_bytes()).hexdigest()
        self.projection = {
            key: parameters.get(key)
            for key in (
                "projector_type",
                "alignment_mode",
                "origin_lat",
                "origin_lon",
                "origin_alt",
            )
        }
        forward = projector(parameters)
        try:
            document = ET.parse(osm_path).getroot()
        except ET.ParseError as error:
            raise ValueError(f"Invalid Lanelet2 OSM: {error}") from error
        if document.tag != "osm":
            raise ValueError("Expected an OSM document")
        nodes = {}
        for node in document.findall("node"):
            try:
                lat, lon = float(node.attrib["lat"]), float(node.attrib["lon"])
                altitude = float(tags(node).get("ele", 0))
                if not all(math.isfinite(v) for v in (lat, lon, altitude)) or not (
                    -90 <= lat <= 90 and -180 <= lon <= 180
                ):
                    raise ValueError("invalid geographic coordinate")
                nodes[node.attrib["id"]] = forward(lat, lon, altitude)
            except (KeyError, ValueError, ProjError) as error:
                raise ValueError(
                    f"Invalid OSM node {node.get('id')}: {error}"
                ) from error
        ways = {}
        for way in document.findall("way"):
            try:
                ways[way.attrib["id"]] = [
                    nodes[nd.attrib["ref"]] for nd in way.findall("nd")
                ]
            except KeyError as error:
                raise ValueError(
                    f"Missing node reference in OSM way {way.get('id')}"
                ) from error
        polygons = []
        self.lanes = {}
        for relation in document.findall("relation"):
            if not driving_lane(tags(relation)):
                continue
            identifier = relation.get("id")
            try:
                members = {
                    m.get("role"): ways[m.attrib["ref"]]
                    for m in relation.findall("member")
                    if m.get("role") in {"left", "right"} and m.get("type") == "way"
                }
                left, right = members["left"], members["right"]
                if min(len(left), len(right)) < 2:
                    raise ValueError("lane bounds need at least two points each")
                if math.dist(left[0], right[-1]) + math.dist(
                    left[-1], right[0]
                ) < math.dist(left[0], right[0]) + math.dist(left[-1], right[-1]):
                    right = list(reversed(right))
                polygon = Polygon([p[:2] for p in left + list(reversed(right))])
                if not polygon.is_valid or polygon.area <= 0:
                    raise ValueError(explain_validity(polygon))
                a, b = LineString(left), LineString(right)
                # Interpolate bounds independently: they need not share node counts.
                count = max(2, math.ceil(max(a.length, b.length) / 0.5))
                centers = []
                for index in range(count + 1):
                    p, q = (
                        a.interpolate(index / count, normalized=True),
                        b.interpolate(index / count, normalized=True),
                    )
                    centers.append(((p.x + q.x) / 2, (p.y + q.y) / 2, (p.z + q.z) / 2))
                line = LineString(centers)
                if line.length <= 0:
                    raise ValueError("lane centerline has no length")
                self.lanes[identifier] = {"line": line}
                polygons.append(polygon)
            except (KeyError, ValueError) as error:
                raise ValueError(
                    f"Invalid driving lanelet {identifier}: {error}"
                ) from error
        if not polygons:
            raise ValueError("OSM contains no usable vehicle-driving lanelet polygons")
        self.surface = unary_union(polygons)
        if self.surface.is_empty or not self.surface.is_valid:
            raise ValueError("Driving surface union is invalid")
        self.spawn_lane = None
        if "pose" in spawn:
            self.spawn = copy.deepcopy(spawn["pose"])
        else:
            # Freeze a deterministic source-map spawn so model and validator use
            # the same origin; CARLA's later straight-lane search is not involved.
            identifier = max(
                sorted(self.lanes),
                key=lambda key: self.surface.boundary.distance(
                    self.lanes[key]["line"].interpolate(0.5, normalized=True)
                ),
            )
            line = self.lanes[identifier]["line"]
            self.spawn_lane = identifier
            self.spawn_s = line.length / 2
            self.spawn = self.lane_pose(identifier, self.spawn_s, 0.5)
        point = Point(self.spawn["x"], self.spawn["y"])
        x0, y0, x1, y1 = self.surface.bounds
        if (
            self.clearance >= min(x1 - x0, y1 - y0) / 2
            or not self.surface.covers(point)
            or not self.surface.covers(
                point.buffer(self.clearance + SAMPLING_ALLOWANCE_M)
            )
        ):
            raise ValueError(
                "Ego spawn lacks the configured boundary clearance; adjust the playground spawn or --boundary-clearance-m"
            )

    def lane_pose(self, identifier, distance, z_offset):
        visited = 0
        while True:
            lane = self.lanes[identifier]
            line = lane["line"]
            if distance <= line.length:
                point = line.interpolate(distance)
                before = line.interpolate(max(0, distance - 0.1))
                after = line.interpolate(min(line.length, distance + 0.1))
                return {
                    "frame": "map",
                    "x": point.x,
                    "y": point.y,
                    "z": point.z + z_offset,
                    "yaw_deg": math.degrees(
                        math.atan2(after.y - before.y, after.x - before.x)
                    ),
                }
            distance -= line.length
            end = Point(line.coords[-1])
            successors = [
                key
                for key, value in self.lanes.items()
                if key != identifier
                and end.distance(Point(value["line"].coords[0])) < 0.01
                and abs(end.z - value["line"].coords[0][2]) < 0.1
            ]
            if len(successors) != 1 or visited > len(self.lanes):
                raise ValueError(
                    "Lane-relative actor leaves an unambiguous OSM lane; supply a map-frame x/y pose instead"
                )
            identifier = successors[0]
            visited += 1

    def resolve_actors(self, actors):
        result = copy.deepcopy(actors)
        for actor in result:
            pose = actor["pose"]
            if pose["frame"] == "lane":
                if self.spawn_lane is None:
                    raise ValueError(
                        "This playground uses an explicit ego spawn; actor poses must use the map frame because no ego lane waypoint is selected"
                    )
                actor["pose"] = self.lane_pose(
                    self.spawn_lane,
                    self.spawn_s + pose["distance_m"],
                    pose.get("z_offset_m", 0.4),
                )
            pose = actor["pose"]
            point = Point(pose["x"], pose["y"])
            if not self.surface.covers(point) or not self.surface.covers(
                point.buffer(self.clearance + SAMPLING_ALLOWANCE_M)
            ):
                raise ValueError(
                    f"Actor {actor['id']} at map ({pose['x']:.2f}, {pose['y']:.2f}) leaves the driving surface or its {self.clearance:g} m boundary clearance"
                )
        return result

    def anchored_path(self, points):
        # Match fake_planner_core.cpp's uniform 0.5 m resampling, then extend
        # the last resampled heading with its zero-speed reference pad.
        cumulative = [0.0]
        for a, b in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + math.dist(a, b))
        sampled = []
        index = 0
        distance = 0.0
        while distance < cumulative[-1]:
            while index + 1 < len(cumulative) and cumulative[index + 1] < distance:
                index += 1
            span = cumulative[index + 1] - cumulative[index]
            fraction = (distance - cumulative[index]) / span if span > 1e-9 else 0.0
            sampled.append(
                tuple(
                    a + fraction * (b - a)
                    for a, b in zip(points[index], points[index + 1])
                )
            )
            distance += 0.5
        if not sampled or math.dist(sampled[-1], points[-1]) > 1e-6:
            sampled.append(points[-1])
        dx, dy = sampled[-1][0] - sampled[-2][0], sampled[-1][1] - sampled[-2][1]
        norm = math.hypot(dx, dy)
        sampled.append(
            (
                sampled[-1][0] + STOP_PAD_M * dx / norm,
                sampled[-1][1] + STOP_PAD_M * dy / norm,
            )
        )
        yaw = math.radians(self.spawn.get("yaw_deg", 0))
        c, s = math.cos(yaw), math.sin(yaw)
        return [
            (self.spawn["x"] + x * c - y * s, self.spawn["y"] + x * s + y * c)
            for x, y in sampled
        ]

    def validate_path(self, points):
        line = LineString(self.anchored_path(points))
        # A buffer checks the entire polyline, including segments crossing holes
        # between samples. Extra allowance covers sampling/resampling differences.
        if not self.surface.covers(line.buffer(self.clearance + SAMPLING_ALLOWANCE_M)):
            outside = line.difference(
                self.surface.buffer(-self.clearance - SAMPLING_ALLOWANCE_M)
            )
            point = (
                outside.representative_point()
                if not outside.is_empty
                else line.interpolate(0.5, normalized=True)
            )
            raise ValueError(
                f"Trajectory including the {STOP_PAD_M:g} m stop pad leaves the driving surface or its {self.clearance:g} m boundary clearance near map ({point.x:.2f}, {point.y:.2f}); reduce the turn/offset/length to fit the supplied map polygons"
            )
        return {
            "checked": True,
            "frame": "map",
            "boundary_clearance_m": self.clearance,
            "sampling_allowance_m": SAMPLING_ALLOWANCE_M,
            "stop_pad_m": STOP_PAD_M,
            "bounds_m": dict(zip(("min_x", "min_y", "max_x", "max_y"), line.bounds)),
        }

    def context(self):
        # A 300 m authored path plus its stop pad fits this local context window.
        x, y = self.spawn["x"], self.spawn["y"]
        nearby = self.surface.intersection(box(x - 320, y - 320, x + 320, y + 320))
        approximate = nearby.simplify(0.05, preserve_topology=True)
        return {
            "version": 1,
            "frame": "map",
            "units": "metres; yaw_deg is degrees counterclockwise from +x",
            "axes": "ROS map: x east, y north; never flip y/yaw for CARLA",
            "source_osm_sha256": self.source_sha256,
            "projection": self.projection,
            "bounds_m": dict(
                zip(("min_x", "min_y", "max_x", "max_y"), self.surface.bounds)
            ),
            "driving_lanelets": len(self.lanes),
            "ego_spawn": self.spawn,
            "lane_relative_actors_supported": self.spawn_lane is not None,
            "boundary_clearance_m": self.clearance,
            "stop_pad_m": STOP_PAD_M,
            "drivable_polygons": mapping(approximate),
            "polygon_note": "Within 320 m of ego; simplified at 0.05 m for context only. Host validates against full polygons, preserving holes and disconnected areas.",
        }
