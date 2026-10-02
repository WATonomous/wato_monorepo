"""Geometry for flat, directed Lanelet2 test tracks, independent of ROS/CARLA.

Coordinates are WGS84 projected into local east/north metres. This deliberately
supports simple lane chains, not junctions, regulatory rules, or route planning.
"""

from bisect import bisect_right
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def finite(value, name="value"):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _ecef(lat, lon, altitude=0.0):
    lat, lon = math.radians(lat), math.radians(lon)
    e2 = 6.6943799901413165e-3
    n = 6378137.0 / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    return (
        (n + altitude) * math.cos(lat) * math.cos(lon),
        (n + altitude) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - e2) + altitude) * math.sin(lat),
    )


def local_xy(lat, lon, origin_lat, origin_lon, altitude=0.0):
    """WGS84 ECEF -> local ENU, matching LocalCartesianProjector at height zero."""
    for value, limit, name in ((lat, 90, "latitude"), (lon, 180, "longitude"),
                               (origin_lat, 90, "origin latitude"),
                               (origin_lon, 180, "origin longitude")):
        if abs(finite(value, name)) > limit:
            raise ValueError(f"{name} is outside geographic degrees; declare/normalize the source CRS")
    origin = _ecef(origin_lat, origin_lon)
    xyz = _ecef(lat, lon, altitude)
    dx, dy, dz = (a - b for a, b in zip(xyz, origin))
    lat0, lon0 = math.radians(origin_lat), math.radians(origin_lon)
    return (
        -math.sin(lon0) * dx + math.cos(lon0) * dy,
        -math.sin(lat0) * math.cos(lon0) * dx
        - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz,
    )


class Polyline:
    def __init__(self, points):
        self.points = []
        for point in points:
            point = (finite(point[0]), finite(point[1]))
            if not self.points or math.dist(point, self.points[-1]) > 1e-7:
                self.points.append(point)
        if len(self.points) < 2:
            raise ValueError("a path needs at least two distinct points")
        self.distances = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.distances.append(self.distances[-1] + math.dist(a, b))
        self.length = self.distances[-1]

    def at(self, distance, closed=False):
        if closed:
            distance %= self.length
        distance = min(max(distance, 0.0), self.length)
        i = min(bisect_right(self.distances, distance) - 1, len(self.points) - 2)
        a, b = self.points[i], self.points[i + 1]
        t = (distance - self.distances[i]) / (self.distances[i + 1] - self.distances[i])
        return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                math.atan2(b[1] - a[1], b[0] - a[0]))

    def project(self, x, y, start=0.0, end=None):
        end = self.length if end is None else min(end, self.length)
        best = (math.inf, start, 0.0)
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            lo, hi = self.distances[i], self.distances[i + 1]
            if hi < start or lo > end:
                continue
            dx, dy = b[0] - a[0], b[1] - a[1]
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)
            t = min(max(t, max(0.0, (start - lo) / (hi - lo))),
                    min(1.0, (end - lo) / (hi - lo)))
            distance = math.hypot(x - a[0] - t * dx, y - a[1] - t * dy)
            candidate = (distance, lo + t * (hi - lo), math.atan2(dy, dx))
            if candidate[0] < best[0]:
                best = candidate
        return best


@dataclass
class Lane:
    id: int
    left_id: int
    right_id: int
    left_nodes: tuple
    right_nodes: tuple
    left: Polyline
    right: Polyline
    center: Polyline
    successor: int | None = None


class LaneletTrack:
    def __init__(self, path, origin_lat, origin_lon):
        self.path = Path(path)
        self.origin_lat = finite(origin_lat)
        self.origin_lon = finite(origin_lon)
        root = ET.parse(self.path).getroot()
        if root.tag != "osm":
            raise ValueError("expected Lanelet2 OSM XML")
        self.nodes = {}
        heights = []
        for node in root.findall("node"):
            if int(node.get("id")) in self.nodes:
                raise ValueError(f"duplicate node ID {node.get('id')}")
            tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
            altitude = finite(tags.get("ele", 0.0), "elevation")
            heights.append(altitude)
            self.nodes[int(node.get("id"))] = local_xy(
                finite(node.get("lat")), finite(node.get("lon")),
                self.origin_lat, self.origin_lon, altitude)
        self.flat = bool(heights) and max(heights) - min(heights) <= 0.01
        ways = {int(w.get("id")): tuple(int(n.get("ref")) for n in w.findall("nd"))
                for w in root.findall("way")}
        if len(ways) != len(root.findall("way")):
            raise ValueError("duplicate way IDs")
        self.lanes = {}
        for relation in root.findall("relation"):
            tags = {t.get("k"): t.get("v") for t in relation.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            if tags.get("subtype", "road") != "road" or tags.get("one_way", "yes") != "yes":
                raise ValueError("test tracks require one-way road lanelets")
            if relation.findall("member[@role='regulatory_element']"):
                raise ValueError("regulatory elements are outside the test-track converter scope")
            bounds = {m.get("role"): int(m.get("ref")) for m in relation.findall("member")
                      if m.get("type") == "way"}
            if "left" not in bounds or "right" not in bounds:
                raise ValueError("lanelet is missing left/right boundaries")
            left_nodes, right_nodes = ways[bounds["left"]], ways[bounds["right"]]
            left = Polyline([self.nodes[n] for n in left_nodes])
            right = Polyline([self.nodes[n] for n in right_nodes])
            # Lanelet2 boundaries must have the same travel direction.
            direct = math.dist(left.points[0], right.points[0]) + math.dist(left.points[-1], right.points[-1])
            reversed_distance = math.dist(left.points[0], right.points[-1]) + math.dist(left.points[-1], right.points[0])
            if direct > reversed_distance:
                raise ValueError("opposing boundary directions must be normalized before import")
            count = max(len(left.points), len(right.points))
            centers = []
            for i in range(count):
                fraction = i / (count - 1)
                l, r = left.at(left.length * fraction), right.at(right.length * fraction)
                centers.append(((l[0] + r[0]) / 2, (l[1] + r[1]) / 2))
            lane = Lane(int(relation.get("id")), bounds["left"], bounds["right"],
                        left_nodes, right_nodes, left, right, Polyline(centers))
            for i in range(21):
                fraction = i / 20
                l, r = left.at(left.length * fraction), right.at(right.length * fraction)
                _, _, heading = lane.center.at(lane.center.length * fraction)
                signed_width = -math.sin(heading) * (l[0]-r[0]) + math.cos(heading) * (l[1]-r[1])
                if signed_width <= 1.0:
                    raise ValueError(f"lanelet {lane.id}: left/right boundaries are crossed or on the wrong side")
            if lane.id in self.lanes:
                raise ValueError(f"duplicate lanelet ID {lane.id}")
            self.lanes[lane.id] = lane
        if not self.lanes:
            raise ValueError("no road lanelets found; ordinary OSM needs the osm_to_xodr tool")
        starts = {}
        for lane in self.lanes.values():
            starts.setdefault((lane.left_nodes[0], lane.right_nodes[0]), []).append(lane.id)
        for lane in self.lanes.values():
            successors = starts.get((lane.left_nodes[-1], lane.right_nodes[-1]), [])
            if len(successors) > 1:
                raise ValueError(f"ambiguous successors for lanelet {lane.id}: {successors}")
            lane.successor = successors[0] if successors else None

    def chain(self, lane_id):
        visited, points = [], []
        while lane_id is not None and lane_id not in visited:
            visited.append(lane_id)
            lane = self.lanes[lane_id]
            points.extend(lane.center.points)
            lane_id = lane.successor
        if lane_id is not None and lane_id != visited[0]:
            raise ValueError("lane chain merges into an unrelated cycle")
        return Polyline(points), lane_id is not None

    def nearest(self, x, y, yaw, max_distance=5.0):
        candidates = []
        for lane in self.lanes.values():
            distance, along, heading = lane.center.project(x, y)
            alignment = math.cos(heading - yaw)
            if distance <= max_distance and alignment > 0.5:
                candidates.append((distance + (1.0 - alignment), lane.id, along))
        if not candidates:
            raise ValueError("ego is not near a lanelet with a compatible travel direction")
        _, lane_id, along = min(candidates)
        return lane_id, along


def opendrive(track):
    """Convert parallel, constant-width closed lane chains to OpenDRIVE 1.4.

Each longitudinal lanelet group becomes one road with all its driving lanes.
    Cubic plan-view geometry approximates the source boundaries; a CARLA map
load and waypoint-alignment check remain required before using a new map.
"""
    if not track.flat:
        raise ValueError("only flat tracks are supported")
    if any(lane.successor is None for lane in track.lanes.values()):
        raise ValueError("converter requires closed tracks with shared endpoint node IDs")
    by_left = {lane.left_id: lane for lane in track.lanes.values()}
    if len(by_left) != len(track.lanes):
        raise ValueError("shared/branching left boundaries are unsupported")
    right_ids = {lane.right_id for lane in track.lanes.values()}
    groups = {}
    consumed = set()
    for lane in sorted(track.lanes.values(), key=lambda l: l.id):
        if lane.left_id in right_ids:
            continue
        group, current = [], lane
        while current:
            if current.id in consumed:
                raise ValueError("overlapping lane groups")
            consumed.add(current.id)
            group.append(current)
            current = by_left.get(current.right_id)
        groups[lane.id] = group
    if consumed != set(track.lanes):
        raise ValueError("lanes must form parallel groups with an outer left boundary")
    # Flat OpenDRIVE without junctions must not silently accept crossing roads.
    segments = [(a, b) for group in groups.values()
                for a, b in zip(group[0].left.points, group[0].left.points[1:])]
    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]
    for i, (a, b) in enumerate(segments):
        u = (b[0]-a[0], b[1]-a[1])
        for c, d in segments[i+1:]:
            v, offset = (d[0]-c[0], d[1]-c[1]), (c[0]-a[0], c[1]-a[1])
            denominator = cross(u, v)
            if abs(denominator) > 1e-9:
                t, s = cross(offset, v)/denominator, cross(offset, u)/denominator
                if 1e-7 < t < 1-1e-7 and 1e-7 < s < 1-1e-7:
                    raise ValueError("crossing roads require junctions and are unsupported")
    root = ET.Element("OpenDRIVE")
    ET.SubElement(root, "header", revMajor="1", revMinor="4", name=track.path.stem,
                  version="1.00", north="0", south="0", east="0", west="0", vendor="WATonomous")
    ET.SubElement(root.find("header"), "geoReference").text = (
        f"+proj=tmerc +lat_0={track.origin_lat} +lon_0={track.origin_lon} "
        "+k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")
    # CARLA imports OpenDRIVE right-handed XY and performs its own Y reflection.
    # Geographic origin is authoritative in the bundle (no simulated GNSS claim).
    for road_id, group in groups.items():
        reference = group[0].left
        widths = []
        for lane in group:
            samples = [math.dist(lane.left.at(lane.left.length * i / 20)[:2],
                                 lane.right.at(lane.right.length * i / 20)[:2]) for i in range(21)]
            width = sum(samples) / len(samples)
            if width < 1.0 or max(abs(w - width) for w in samples) > 0.15:
                raise ValueError(f"lanelet {lane.id} is not a constant-width lane")
            widths.append(width)
        successor = group[0].successor
        predecessor = [rid for rid, g in groups.items() if g[0].successor == road_id]
        if successor not in groups or len(predecessor) != 1:
            raise ValueError("road groups must form unambiguous closed cycles")
        following = groups[successor]
        if len(following) != len(group) or any(a.successor != b.id for a, b in zip(group, following)):
            raise ValueError("lane counts/connectivity change at road boundaries")
        road = ET.SubElement(root, "road", id=str(road_id), name=f"lanelet_{road_id}", junction="-1")
        link = ET.SubElement(road, "link")
        ET.SubElement(link, "predecessor", elementType="road", elementId=str(predecessor[0]), contactPoint="end")
        ET.SubElement(link, "successor", elementType="road", elementId=str(successor), contactPoint="start")
        plan = ET.SubElement(road, "planView")
        # Continuous tangents across sample and road boundaries prevent lane
        # offsets from jumping at the corners of a polygonal oval.
        previous_point = groups[predecessor[0]][0].left.points[-2]
        next_point = following[0].left.points[1]
        padded = [previous_point] + reference.points + [next_point]
        headings = []
        for a, p, b in zip(padded, padded[1:], padded[2:]):
            before, after = math.dist(a, p), math.dist(p, b)
            headings.append(math.atan2((p[1]-a[1])/before + (b[1]-p[1])/after,
                                       (p[0]-a[0])/before + (b[0]-p[0])/after))
        road_length = 0.0
        for i, (a, b) in enumerate(zip(reference.points, reference.points[1:])):
            heading = headings[i]
            chord = math.dist(a, b)
            dx, dy = b[0]-a[0], b[1]-a[1]
            u, v = math.cos(heading)*dx + math.sin(heading)*dy, -math.sin(heading)*dx + math.cos(heading)*dy
            du, dv = chord*math.cos(headings[i+1]-heading), chord*math.sin(headings[i+1]-heading)
            cu, du3 = 3*u-2*chord-du, -2*u+chord+du
            cv, dv3 = 3*v-dv, -2*v+dv
            def derivative(t):
                return math.hypot(chord+2*cu*t+3*du3*t*t, 2*cv*t+3*dv3*t*t)
            # Simpson integration of cubic arc length (normalised parameter).
            intervals = 64
            length = (derivative(0)+derivative(1) + sum(
                (4 if j % 2 else 2)*derivative(j/intervals) for j in range(1, intervals))) / (3*intervals)
            geometry = ET.SubElement(plan, "geometry", s=f"{road_length:.9f}",
                                     x=f"{a[0]:.9f}", y=f"{a[1]:.9f}",
                                     hdg=f"{heading:.12f}", length=f"{length:.9f}")
            ET.SubElement(geometry, "paramPoly3", aU="0", bU=f"{chord:.12f}",
                          cU=f"{cu:.12f}", dU=f"{du3:.12f}", aV="0", bV="0",
                          cV=f"{cv:.12f}", dV=f"{dv3:.12f}", pRange="normalized")
            road_length += length
        road.set("length", f"{road_length:.9f}")
        elevation = ET.SubElement(road, "elevationProfile")
        ET.SubElement(elevation, "elevation", s="0", a="0", b="0", c="0", d="0")
        lanes = ET.SubElement(road, "lanes")
        section = ET.SubElement(lanes, "laneSection", s="0")
        center = ET.SubElement(section, "center")
        center_lane = ET.SubElement(center, "lane", id="0", type="none", level="false")
        ET.SubElement(center_lane, "roadMark", sOffset="0", type="solid", weight="standard", color="white", width="0.15")
        right = ET.SubElement(section, "right")
        for i, width in enumerate(widths, 1):
            lane = ET.SubElement(right, "lane", id=str(-i), type="driving", level="false")
            lane_link = ET.SubElement(lane, "link")
            ET.SubElement(lane_link, "predecessor", id=str(-i))
            ET.SubElement(lane_link, "successor", id=str(-i))
            ET.SubElement(lane, "width", sOffset="0", a=f"{width:.9f}", b="0", c="0", d="0")
            ET.SubElement(lane, "roadMark", sOffset="0", type="solid" if i == len(widths) else "broken",
                          weight="standard", color="white", width="0.15", laneChange="both")
    ET.indent(root)
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def validate_carla_map(xodr, track, tolerance=0.25):
    """Parse with CARLA and check source centerlines against actual driving lanes.

    CARLA's spatial lookup approximates longitudinal position. Refine s locally
    through its exact OpenDRIVE lookup before measuring geometry alignment.
    No running CARLA server is needed.
    """
    import carla
    parsed = carla.Map("track_validation", xodr)
    lengths = {int(r.get("id")): float(r.get("length"))
               for r in ET.fromstring(xodr).findall("road")}
    errors = []
    for lane in track.lanes.values():
        for i in range(math.ceil(lane.center.length / 2.0) + 1):
            x, y, yaw = lane.center.at(min(i * 2.0, lane.center.length))
            waypoint = parsed.get_waypoint(carla.Location(x=x, y=-y, z=0), project_to_road=True)
            if waypoint is None:
                raise ValueError(f"no CARLA waypoint near lanelet {lane.id}")
            def at(s):
                return parsed.get_waypoint_xodr(waypoint.road_id, waypoint.lane_id, s)
            def error(candidate):
                if candidate is None:
                    return math.inf
                p = candidate.transform.location
                return math.hypot(p.x-x, p.y+y)
            lo, hi = max(0.0, waypoint.s-1.0), min(lengths[waypoint.road_id]-1e-7, waypoint.s+1.0)
            for _ in range(24):
                a, b = lo+(hi-lo)/3, hi-(hi-lo)/3
                if error(at(a)) < error(at(b)):
                    hi = b
                else:
                    lo = a
            best = min([waypoint, at(lo), at(hi), at((lo+hi)/2)], key=error)
            distance = error(best)
            alignment = math.cos(-math.radians(best.transform.rotation.yaw)-yaw)
            if distance > tolerance or alignment < math.cos(math.radians(30)):
                raise ValueError(f"lanelet {lane.id}: CARLA centerline error={distance:.3f}m, "
                                 f"heading alignment={alignment:.3f}")
            errors.append(distance)
    return errors
