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

import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lanelet_to_osm import lanelet_roads, tags  # noqa: E402


def lanelet_fixture():
    """Four connected road sections, each with four adjacent one-way lanes."""
    root = ET.Element("osm", version="0.6")
    for boundary in range(5):
        radius = 54 - boundary * 3.5
        for sample in range(32):
            angle = 2 * math.pi * sample / 32
            ET.SubElement(
                root,
                "node",
                id=str(1 + boundary * 32 + sample),
                lat=str(45 + radius * math.sin(angle) / 111195),
                lon=str(
                    -73 + radius * math.cos(angle) / (111195 * math.cos(math.pi / 4))
                ),
            )
    for section in range(4):
        for boundary in range(5):
            way = ET.SubElement(root, "way", id=str(1000 + section * 5 + boundary))
            for sample in range(section * 8, section * 8 + 9):
                ET.SubElement(way, "nd", ref=str(1 + boundary * 32 + sample % 32))
        for lane in range(4):
            relation = ET.SubElement(
                root, "relation", id=str(10000 + section * 4 + lane)
            )
            for role, boundary in (("left", lane + 1), ("right", lane)):
                ET.SubElement(
                    relation,
                    "member",
                    type="way",
                    ref=str(1000 + section * 5 + boundary),
                    role=role,
                )
            for key, value in (
                ("type", "lanelet"),
                ("subtype", "road"),
                ("one_way", "yes"),
            ):
                ET.SubElement(relation, "tag", k=key, v=value)
    return root


class LaneletAdapterTest(unittest.TestCase):
    def test_adjacent_lanes_form_connected_four_lane_loop(self):
        output, width, count = lanelet_roads(lanelet_fixture())
        self.assertEqual(count, 16)
        self.assertAlmostEqual(width, 3.5, places=3)
        roads = output.findall("way")
        self.assertEqual(len(roads), 4)
        self.assertTrue(all(tags(road)["lanes"] == "4" for road in roads))
        for index, road in enumerate(roads):
            self.assertEqual(
                road.findall("nd")[-1].get("ref"),
                roads[(index + 1) % len(roads)].find("nd").get("ref"),
            )

    def test_reversed_boundary_storage_preserves_connections(self):
        root = lanelet_fixture()
        for way in root.findall("way")[::2]:
            way[:] = list(way)[::-1]
        output, _, _ = lanelet_roads(root)
        expected, _, _ = lanelet_roads(lanelet_fixture())
        self.assertEqual(ET.tostring(output), ET.tostring(expected))

    def test_unequal_samples_fail_clearly(self):
        root = lanelet_fixture()
        way = root.find("way")
        way.remove(way.find("nd"))
        with self.assertRaisesRegex(ValueError, "sample counts"):
            lanelet_roads(root)

    def test_missing_boundary_fails_clearly(self):
        root = lanelet_fixture()
        root.remove(root.find("way"))
        with self.assertRaisesRegex(ValueError, "boundary"):
            lanelet_roads(root)

    def test_two_way_lanelets_fail_clearly(self):
        root = lanelet_fixture()
        root.find("relation/tag[@k='one_way']").set("v", "no")
        with self.assertRaisesRegex(ValueError, "two-way"):
            lanelet_roads(root)


if __name__ == "__main__":
    unittest.main()
