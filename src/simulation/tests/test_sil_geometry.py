"""Portable regressions for the map converter and injected trajectory contract.

Run: python3 -m unittest discover -s src/simulation/tests -v
Requires PyYAML, no ROS or running CARLA server.
"""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

BRIDGE = Path(__file__).resolve().parents[1] / 'carla_ros_bridge'
for package in ('carla_common', 'carla_scenarios', 'carla_fake_planner'):
    sys.path.insert(0, str(BRIDGE / package))

from carla_common.lanelet_track import LaneletTrack, local_xy, opendrive
from carla_scenarios.map_registry import Registry
from carla_fake_planner.trajectories import Run, parameters


def track_xml():
    """Two closed CCW lanes, eight linked roads, explicit shared boundaries."""
    root = ET.Element('osm', version='0.6')
    sectors, samples = 8, 16
    for boundary in range(3):
        radius = 35 + 3.5 * boundary
        for i in range(sectors * samples):
            angle = 2 * math.pi * i / (sectors * samples)
            x, y = radius * math.cos(angle), radius * math.sin(angle)
            ET.SubElement(root, 'node', id=str(boundary*1000+i+1),
                          lon=str(math.degrees(x/6378137)), lat=str(math.degrees(y/6335439.327)))
        for sector in range(sectors):
            way = ET.SubElement(root, 'way', id=str(10000+boundary*100+sector))
            for j in range(samples+1):
                index = (sector*samples+j) % (sectors*samples)
                ET.SubElement(way, 'nd', ref=str(boundary*1000+index+1))
    for lane in range(2):
        for sector in range(sectors):
            relation = ET.SubElement(root, 'relation', id=str(20000+lane*100+sector))
            for role, boundary in (('left', lane), ('right', lane+1)):
                ET.SubElement(relation, 'member', type='way', role=role,
                              ref=str(10000+boundary*100+sector))
            ET.SubElement(relation, 'tag', k='type', v='lanelet')
            ET.SubElement(relation, 'tag', k='subtype', v='road')
    return root


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'track.osm'
        self.root = track_xml()
        self.save()
        self.track = LaneletTrack(self.path, 0, 0)
        self.presets = yaml.safe_load((BRIDGE/'carla_fake_planner/config/trajectories.yaml').read_text())['presets']

    def save(self):
        ET.ElementTree(self.root).write(self.path)

    def run_path(self, name, overrides=None, start=5):
        pose = self.track.lanes[20000].center.at(start)
        return Run(self.track, pose, self.presets[name], overrides or {})

    def test_projection_has_shared_metric_origin(self):
        self.assertEqual(local_xy(45, -73, 45, -73), (0, 0))
        east = local_xy(45, -72.999, 45, -73)
        self.assertAlmostEqual(east[0], 78.8468, places=3)
        self.assertLess(abs(east[1]), .001)
        with self.assertRaises(ValueError):
            local_xy(1000, 0, 0, 0)

    def test_closed_lane_links_and_heading_selection(self):
        path, closed = self.track.chain(20000)
        self.assertTrue(closed)
        self.assertAlmostEqual(path.length, 2*math.pi*36.75, delta=.2)
        pose = self.track.lanes[20000].center.at(5)
        self.assertEqual(self.track.nearest(*pose)[0], 20000)
        with self.assertRaises(ValueError):
            self.track.nearest(pose[0], pose[1], pose[2]+math.pi)

    def test_converter_links_all_lanes_and_keeps_seams_continuous(self):
        roads = ET.fromstring(opendrive(self.track)).findall('road')
        by_id = {r.get('id'): r for r in roads}
        self.assertEqual(len(roads), 8)
        for road in roads:
            self.assertEqual(len(road.findall('lanes/laneSection/right/lane')), 2)
            next_road = by_id[road.find('link/successor').get('elementId')]
            last, first = road.findall('planView/geometry')[-1], next_road.find('planView/geometry')
            poly = last.find('paramPoly3')
            u = sum(float(poly.get(k)) for k in ('aU','bU','cU','dU'))
            v = sum(float(poly.get(k)) for k in ('aV','bV','cV','dV'))
            heading = float(last.get('hdg'))
            endpoint = (float(last.get('x'))+u*math.cos(heading)-v*math.sin(heading),
                        float(last.get('y'))+u*math.sin(heading)+v*math.cos(heading))
            self.assertLess(math.dist(endpoint, (float(first.get('x')),float(first.get('y')))), 1e-7)
            du = float(poly.get('bU'))+2*float(poly.get('cU'))+3*float(poly.get('dU'))
            dv = float(poly.get('bV'))+2*float(poly.get('cV'))+3*float(poly.get('dV'))
            self.assertAlmostEqual(math.cos(heading+math.atan2(dv,du)-float(first.get('hdg'))),1,places=10)

    def test_reject_duplicate_nodes_and_reversed_sides(self):
        self.root.append(ET.fromstring(ET.tostring(self.root.find('node'))))
        self.save()
        with self.assertRaisesRegex(ValueError, 'duplicate node'):
            LaneletTrack(self.path, 0, 0)
        self.root = track_xml()
        members = self.root.find('relation').findall('member')
        members[0].set('role','right'); members[1].set('role','left')
        self.save()
        with self.assertRaisesRegex(ValueError, 'wrong side'):
            LaneletTrack(self.path, 0, 0)

    def test_reject_open_track_conversion(self):
        self.track.lanes[20007].successor = None
        with self.assertRaisesRegex(ValueError, 'closed tracks'):
            opendrive(self.track)

    def test_forward_window_rolls_across_multiple_laps(self):
        run = self.run_path('follow_lane')
        for distance in range(0, math.ceil(3*run.path.length), 5):
            run.advance(run.path.at(run.start+distance, True))
            self.assertAlmostEqual(run.progress, distance, delta=.01)
            first = run.samples()[0]
            self.assertLess(math.dist(first[:2], run.path.at(run.start+distance, True)[:2]), .01)
            self.assertGreater(first[3], 0)
        before = run.progress
        run.advance(run.path.at(run.start+before-1, True))
        self.assertGreaterEqual(run.progress, before)

    def test_snake_lateral_geometry_and_interpolated_speed(self):
        run = self.run_path('snake', {'length_m':100, 'wavelength_m':20})
        center = run.path.at(run.start+5, True)
        self.assertAlmostEqual(math.dist(run.position(5), center[:2]), .8, places=5)
        self.assertAlmostEqual(run.speed(15), 17.5/3.6)
        self.assertEqual(run.samples(full=True)[-1][3], 0)

    def test_brake_ramp_step_and_speed_units(self):
        run = self.run_path('hard_brake', {'speed_kph':50, 'brake_distance_m':2})
        self.assertAlmostEqual(run.speed(39), 50/3.6)
        self.assertAlmostEqual(run.speed(41), 25/3.6)
        self.assertEqual(run.speed(42), 0)
        step = self.run_path('hard_brake', {'brake_distance_m':0})
        self.assertAlmostEqual(step.speed(39.99), 30/3.6)
        self.assertEqual(step.speed(40), 0)
        run.progress = run.length
        self.assertEqual(run.samples(), [])

    def test_parameter_errors_do_not_mutate_preset(self):
        for override in ({'speed_kph':float('nan')}, {'speed_kph':51}, {'sample_spacing_m':.001},
                         {'bad_key':1}, {'speed_profile':[{'distance_m':0,'speed_kph':10},
                                                         {'distance_m':0,'speed_kph':20}]}):
            with self.assertRaises(ValueError):
                parameters(self.presets['follow_lane'], override)
        self.assertEqual(self.presets['follow_lane']['parameters']['speed_kph'],20)

    def test_missing_map_leaves_registry_bundle_unchanged(self):
        registry_path = Path(self.tmp.name)/'scenarios.yaml'
        registry_path.write_text(yaml.safe_dump({'version':1,'maps':{'missing':{
            'carla':{'from_lanelet2':True},'lanelet2':{'path':'missing.osm','origin':{'lat':0,'lon':0}}}},
            'scenarios':{'test':{'module':'test.scenario','map':'missing'}}}))
        registry = Registry(registry_path, self.tmp.name)
        _, _, bundle = registry.scenario('test.scenario')
        with self.assertRaisesRegex(ValueError,'missing Lanelet2'):
            bundle.prepare()
        self.assertIsNone(bundle.xodr_path)


if __name__ == '__main__':
    unittest.main()
