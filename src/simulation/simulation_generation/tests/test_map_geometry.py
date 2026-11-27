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
"""Boundary containment: projection, holes, stop pad, actors and map anchoring."""

import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
for directory in (
    "src/simulation/simulation_generation",
    "src/simulation/simulation_runtime",
    "src/simulation/carla_ros_bridge/carla_scenarios",
):
    sys.path.insert(0, str(ROOT / directory))

from pyproj import Transformer  # noqa: E402
from simulation_generation.map_geometry import MapGeometry, projector  # noqa: E402
from simulation_generation.trajectory import sample_geometry  # noqa: E402


PARAMETERS = {
    "projector_type": "utm",
    "alignment_mode": "fixed",
    "origin_lat": 45.5017,
    "origin_lon": -73.5673,
    "origin_alt": 0.0,
}


class MapGeometryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.osm = Path(temporary.name) / "surface.osm"

    def map(
        self, rectangles, *, spawn=(-20.0, 0.0, 0.0), clearance=3.0, subtype="road"
    ):
        forward = Transformer.from_crs(4326, 32618, always_xy=True)
        inverse = Transformer.from_crs(32618, 4326, always_xy=True)
        ox, oy = forward.transform(-73.5673, 45.5017)
        document = ET.Element("osm")
        for lane, (x0, y0, x1, y1) in enumerate(rectangles, 1):
            first = lane * 4
            for offset, (x, y) in enumerate(((x0, y1), (x1, y1), (x0, y0), (x1, y0))):
                lon, lat = inverse.transform(ox + x, oy + y)
                ET.SubElement(
                    document, "node", id=str(first + offset), lat=str(lat), lon=str(lon)
                )
            for index, refs in enumerate(((first, first + 1), (first + 2, first + 3))):
                way = ET.SubElement(document, "way", id=str(lane * 2 + index))
                for ref in refs:
                    ET.SubElement(way, "nd", ref=str(ref))
            relation = ET.SubElement(document, "relation", id=str(lane))
            for index, role in enumerate(("left", "right")):
                ET.SubElement(
                    relation, "member", type="way", ref=str(lane * 2 + index), role=role
                )
            ET.SubElement(relation, "tag", k="type", v="lanelet")
            ET.SubElement(relation, "tag", k="subtype", v=subtype)
        ET.ElementTree(document).write(self.osm)
        return MapGeometry(
            self.osm,
            PARAMETERS,
            {
                "pose": {
                    "frame": "map",
                    "x": spawn[0],
                    "y": spawn[1],
                    "yaw_deg": spawn[2],
                    "z": 0.5,
                }
            },
            clearance_m=clearance,
        )

    def test_union_allows_crossing_internal_lane_boundaries(self):
        geometry = self.map([(-60, -20, 60, 0), (-60, 0, 60, 20)])
        result = geometry.validate_path([(0, 0), (20, 5)])
        self.assertTrue(result["checked"])
        self.assertAlmostEqual(geometry.surface.area, 4800, places=4)

    def test_hole_is_rejected_even_when_path_endpoints_and_bounding_box_fit(self):
        geometry = self.map(
            [(-60, 5, 60, 20), (-60, -20, 60, -5), (-60, -5, -5, 5), (5, -5, 60, 5)]
        )
        with self.assertRaisesRegex(ValueError, "driving surface"):
            geometry.validate_path([(0, 0), (40, 0)])

    def test_path_cannot_jump_between_disconnected_surfaces(self):
        geometry = self.map([(-60, -20, -5, 20), (5, -20, 60, 20)])
        with self.assertRaisesRegex(ValueError, "driving surface"):
            geometry.validate_path([(0, 0), (40, 0)])

    def test_stop_pad_is_checked_past_the_authored_end(self):
        geometry = self.map([(-60, -20, 30, 20)])
        with self.assertRaisesRegex(ValueError, "10 m stop pad"):
            geometry.validate_path([(0, 0), (45, 0)])

    def test_reference_center_inside_map_can_still_lack_clearance(self):
        geometry = self.map([(-60, -10, 60, 10)])
        with self.assertRaisesRegex(ValueError, "3 m boundary clearance"):
            geometry.validate_path([(0, 0), (20, 8)])

    def test_yaw_rotates_path_and_stop_pad_in_ros_map_frame(self):
        geometry = self.map([(-30, -30, 30, 30)], spawn=(0, 0, 90))
        result = geometry.validate_path([(0, 0), (5, 0)])
        self.assertAlmostEqual(result["bounds_m"]["max_y"], 15)
        self.assertAlmostEqual(result["bounds_m"]["max_x"], 0)

    def test_actor_coordinates_and_clearance_are_enforced(self):
        geometry = self.map([(-60, -20, 60, 20)])
        actor = {
            "id": "parked",
            "blueprint": "vehicle.mini.cooper",
            "pose": {"frame": "map", "x": 15.0, "y": 10.0},
        }
        self.assertEqual(geometry.resolve_actors([actor]), [actor])
        for x, y in ((61, 0), (0, 18)):
            actor["pose"].update(x=x, y=y)
            with self.assertRaisesRegex(ValueError, "Actor parked"):
                geometry.resolve_actors([actor])

    def test_bad_projection_or_geometry_fails_closed(self):
        for parameters in (
            {**PARAMETERS, "alignment_mode": "tf"},
            {**PARAMETERS, "projector_type": "unknown"},
            {**PARAMETERS, "origin_lat": math.nan},
        ):
            with self.assertRaises(ValueError):
                projector(parameters)
        with self.assertRaisesRegex(ValueError, "no usable vehicle-driving"):
            self.map([(-60, -20, 60, 20)], subtype="walkway")
        self.osm.write_text("<osm/>")
        with self.assertRaisesRegex(ValueError, "no usable vehicle-driving"):
            MapGeometry(self.osm, PARAMETERS, {})

    def test_utm_and_local_cartesian_origins_have_zero_offsets(self):
        for kind in ("utm", "local_cartesian"):
            forward = projector(
                {**PARAMETERS, "projector_type": kind, "origin_alt": 12.0}
            )
            self.assertLess(math.dist(forward(45.5017, -73.5673, 12), (0, 0, 0)), 1e-8)
            x, y, _ = forward(45.5018, -73.5672, 12)
            self.assertGreater(x, 7)
            self.assertGreater(y, 10)

    def test_airfield_circle_and_exported_xy_context_agree(self):
        import yaml

        parameters = yaml.safe_load(
            (
                ROOT
                / "src/world_modeling/world_modeling_bringup/config/map_configs/airfield.yaml"
            ).read_text()
        )["/**/world_model"]["ros__parameters"]
        spawn = yaml.safe_load(
            (
                ROOT
                / "src/simulation/carla_ros_bridge/carla_scenarios/config/playgrounds/airfield.yaml"
            ).read_text()
        )["spawn"]
        geometry = MapGeometry(ROOT / "maps/osm/airfield.osm", parameters, spawn)
        candidate = json.loads(
            (
                ROOT
                / "src/simulation/simulation_generation/examples/airfield_circle.json"
            ).read_text()
        )
        points, _ = sample_geometry(candidate)
        checked = geometry.validate_path(points)
        self.assertAlmostEqual(checked["bounds_m"]["max_x"], 20.0, places=4)
        context = geometry.context()
        self.assertEqual(context["driving_lanelets"], 160)
        self.assertAlmostEqual(context["bounds_m"]["min_x"], -200, places=4)
        self.assertAlmostEqual(context["bounds_m"]["max_y"], 35, places=4)
        self.assertEqual(context["ego_spawn"]["y"], -20)
        self.assertLess(len(json.dumps(context)), 10000)


if __name__ == "__main__":
    unittest.main()
