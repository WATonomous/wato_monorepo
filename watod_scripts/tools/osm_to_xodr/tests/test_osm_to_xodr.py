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

"""Integration checks run inside the converter's Docker image."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

CONVERTER = Path(__file__).resolve().parents[1] / "osm_to_xodr.py"
OSM = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="43.4700" lon="-80.5400"/>
  <node id="2" lat="43.4710" lon="-80.5400"/>
  <node id="3" lat="43.4720" lon="-80.5400"/>
  <node id="4" lat="43.4710" lon="-80.5410"/>
  <node id="5" lat="43.4710" lon="-80.5390"/>
  <way id="10">
    <nd ref="1"/><nd ref="2"/><nd ref="3"/>
    <tag k="highway" v="residential"/>
    <tag k="name" v="Test Road"/>
  </way>
  <way id="20">
    <nd ref="4"/><nd ref="2"/><nd ref="5"/>
    <tag k="highway" v="residential"/>
  </way>
</osm>
"""


class ConversionTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        self.input = self.directory / "input map.osm"
        self.output = self.directory / "nested output" / "map.xodr"
        self.input.write_text(OSM, encoding="utf-8")

    def run_converter(self, *options):
        return subprocess.run(
            [
                sys.executable,
                str(CONVERTER),
                str(self.input),
                str(self.output),
                *options,
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_default_conversion(self):
        result = self.run_converter()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("proj.db", result.stderr)
        root = ET.parse(self.output).getroot()
        self.assertEqual(root.tag, "OpenDRIVE")
        self.assertTrue(root.findall("road"))
        import carla

        road_map = carla.Map("test_osm", self.output.read_text(encoding="utf-8"))
        self.assertTrue(road_map.generate_waypoints(2.0))
        self.assertEqual(self.input.read_text(encoding="utf-8"), OSM)

    def test_settings_and_traffic_lights(self):
        result = self.run_converter(
            "--center-map",
            "--default-lane-width",
            "3.5",
            "--all-junctions-with-traffic-lights",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        root = ET.parse(self.output).getroot()
        widths = root.findall(".//lane[@type='driving']/width")
        self.assertTrue(widths)
        self.assertTrue(all(float(width.attrib["a"]) == 3.5 for width in widths))
        self.assertTrue(root.findall(".//signal"))

    def test_osm_traffic_lights_can_be_disabled(self):
        self.input.write_text(
            OSM.replace(
                '<node id="2" lat="43.4710" lon="-80.5400"/>',
                '<node id="2" lat="43.4710" lon="-80.5400">'
                '<tag k="highway" v="traffic_signals"/>'
                "</node>",
            ),
            encoding="utf-8",
        )
        enabled = self.run_converter("--generate-traffic-lights")
        self.assertEqual(enabled.returncode, 0, enabled.stderr)
        self.assertTrue(ET.parse(self.output).getroot().findall(".//signal"))
        self.output.unlink()
        disabled = self.run_converter("--no-generate-traffic-lights")
        self.assertEqual(disabled.returncode, 0, disabled.stderr)
        self.assertFalse(ET.parse(self.output).getroot().findall(".//signal"))

    def test_local_projection(self):
        projection = (
            "+proj=tmerc +lat_0=43.471 +lon_0=-80.54 +k=1 "
            "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
        )
        result = self.run_converter("--no-center-map", "--proj-string", projection)
        self.assertEqual(result.returncode, 0, result.stderr)
        root = ET.parse(self.output).getroot()
        self.assertEqual(root.findtext("header/geoReference").strip(), projection)
        for geometry in root.findall(".//planView/geometry"):
            self.assertLess(abs(float(geometry.attrib["x"])), 200)
            self.assertLess(abs(float(geometry.attrib["y"])), 200)

    def test_conflicting_traffic_light_options_fail(self):
        result = self.run_converter(
            "--no-generate-traffic-lights", "--all-junctions-with-traffic-lights"
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("conflicts", result.stderr)
        self.assertFalse(self.output.exists())

    def test_existing_output_requires_force(self):
        self.output.parent.mkdir()
        self.output.write_text("existing output", encoding="utf-8")
        result = self.run_converter()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--force", result.stderr)
        self.assertEqual(self.output.read_text(), "existing output")
        result = self.run_converter("--force")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(ET.parse(self.output).getroot().tag, "OpenDRIVE")

    def test_invalid_xml_creates_no_output(self):
        self.input.write_text("<osm>", encoding="utf-8")
        result = self.run_converter()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())

    def test_missing_input_creates_no_output(self):
        self.input.unlink()
        result = self.run_converter()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())

    def test_filtered_roads_create_no_output(self):
        result = self.run_converter("--osm-way-types", "motorway")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())

    def test_invalid_lane_width_creates_no_output(self):
        result = self.run_converter("--default-lane-width", "0")
        self.assertEqual(result.returncode, 2)
        self.assertFalse(self.output.exists())

    def test_lanelet_input_is_rejected_clearly(self):
        self.input.write_text(
            '<osm version="0.6"><relation id="1">'
            '<tag k="type" v="lanelet"/>'
            "</relation></osm>",
            encoding="utf-8",
        )
        result = self.run_converter()
        self.assertEqual(result.returncode, 1)
        self.assertIn("Lanelet2 map detected", result.stderr)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
