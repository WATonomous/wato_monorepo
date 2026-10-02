"""ROS service regressions with mocked CARLA actors, not a dynamics/SIL test.

Run in a sourced ROS workspace containing carla_msgs, carla_fake_planner and
carla_scenarios. Native ROS messages and lifecycle services are exercised.
"""
import math
import subprocess
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock

try:
    import rclpy
except ImportError:
    raise unittest.SkipTest('requires a sourced ROS workspace')

# A server is deliberately unnecessary for these protocol tests.
sys.modules['carla'] = types.ModuleType('carla')
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.parameter import Parameter
from carla_msgs.msg import ScenarioStatus
from carla_msgs.srv import SelectTrajectory
from lanelet_msgs.srv import GetLaneletAhead
from wato_trajectory_msgs.msg import Trajectory
from carla_fake_planner.node import CarlaFakePlanner
from carla_scenarios.map_registry import MapBundle
from carla_scenarios.world_model_reload import WorldModelReload, call
from std_srvs.srv import Trigger
from test_sil_geometry import track_xml


class StubWorldModel(LifecycleNode):
    def __init__(self):
        super().__init__('world_model', namespace='/world_modeling')
        for name, value in {'osm_map_path':'wrong.osm', 'projector_type':'local_cartesian',
                            'origin_lat':0.0, 'origin_lon':0.0}.items():
            self.declare_parameter(name, value)
        self.transitions = []
        self.create_service(GetLaneletAhead, '/world_modeling/get_lanelet_ahead', self.query)

    def query(self, request, response):
        response.success = True
        return response

    def on_configure(self, state):
        self.transitions.append('configure')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.transitions.append('activate')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.transitions.append('deactivate')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.transitions.append('cleanup')
        return TransitionCallbackReturn.SUCCESS


class NestedScenario(LifecycleNode):
    def __init__(self):
        super().__init__('test_scenario', namespace='/carla')
        self.finish = self.create_client(Trigger, '/carla/finish_scenario_switch',
                                         callback_group=ReentrantCallbackGroup())
        self.activated = False

    def on_activate(self, state):
        if call(self.finish, Trigger.Request(), timeout=8).success:
            self.activated = True
            return TransitionCallbackReturn.SUCCESS
        return TransitionCallbackReturn.FAILURE


class StubBridge(LifecycleNode):
    def __init__(self):
        super().__init__('test_bridge', namespace='/carla')
        self.reject_activation = False

    def on_activate(self, state):
        return (TransitionCallbackReturn.FAILURE if self.reject_activation
                else TransitionCallbackReturn.SUCCESS)


class RosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.osm = Path(self.tmp.name)/'track.osm'
        import xml.etree.ElementTree as ET
        ET.ElementTree(track_xml()).write(self.osm)
        self.nodes = []
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.thread = None
        self.addCleanup(self.cleanup)

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def spin_background(self):
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def cleanup(self):
        self.executor.shutdown(timeout_sec=5)
        if self.thread:
            self.thread.join(timeout=5)
        for node in reversed(self.nodes):
            node.destroy_node()

    def wait(self, condition, timeout=3):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline:
            if self.thread:
                time.sleep(.02)
            else:
                self.executor.spin_once(timeout_sec=.02)
        self.assertTrue(condition())

    def planner(self):
        planner = self.add(CarlaFakePlanner())
        status = ScenarioStatus(state='running', generation=1, map_id='test',
                                osm_map_path=str(self.osm), origin_lat=0., origin_lon=0.,
                                trajectories_enabled=True)
        planner.scenario_callback(status)
        pose = planner.track.lanes[20000].center.at(5)
        actor = Mock()
        actor.get_velocity.return_value = types.SimpleNamespace(x=0., y=0., z=0.)
        planner.ego = Mock(return_value=(actor, pose))
        return planner

    def select(self, planner, name, overrides=''):
        request = SelectTrajectory.Request(trajectory_id=name, parameters_yaml=overrides)
        return planner.select(request, SelectTrajectory.Response())

    def test_native_trajectory_message_selection_replacement_and_invalid_request(self):
        planner = self.planner()
        received = []
        observer = self.add(Node('observer'))
        observer.create_subscription(Trajectory, '/action/trajectory_planning/trajectory', received.append, 10)
        self.wait(lambda: planner.trajectory_pub.get_subscription_count() > 0)
        self.assertTrue(self.select(planner, 'hard_brake', 'speed_kph: 50').success)
        self.wait(lambda: bool(received))
        self.assertEqual(received[-1].header.frame_id, 'map')
        self.assertAlmostEqual(received[-1].points[0].max_speed, 50/3.6)
        self.assertEqual(received[-1].points[-1].max_speed, 0)
        previous = planner.run
        self.assertFalse(self.select(planner, 'snake', 'amplitude_m: .nan').success)
        self.assertIs(planner.run, previous)
        actor, _ = planner.ego.return_value
        pose = planner.track.lanes[20000].center.at(15)
        planner.ego.return_value = actor, pose
        self.assertTrue(self.select(planner, 'snake').success)
        self.assertLess(math.dist(planner.run.position(0), pose[:2]), .01)

    def test_pause_and_map_generation_clear_controller_input(self):
        planner = self.planner()
        self.assertTrue(self.select(planner, 'follow_lane').success)
        run = planner.run
        planner.scenario.state = 'paused'
        planner.publish()
        self.assertIs(planner.run, run)
        self.assertFalse(planner.was_publishing)
        planner.scenario_callback(ScenarioStatus(state='loading', generation=2, map_id='other'))
        self.assertIsNone(planner.run)
        self.assertIsNone(planner.track)

    def test_world_model_reload_and_restart_parameters(self):
        world = self.add(StubWorldModel())
        world.trigger_configure(); world.trigger_activate()
        coordinator = self.add(Node('reload_test'))
        reload = WorldModelReload(coordinator, '/world_modeling/world_model', ReentrantCallbackGroup())
        bundle = MapBundle('test', {'lanelet2':{'path':str(self.osm),'origin':{'lat':45.5,'lon':-73.5}}},
                           Path(self.tmp.name))
        self.spin_background()
        self.wait(lambda: reload.state.service_is_ready())
        reload.prepare(); reload.apply(bundle, (0., 0., 0.), 1)
        self.assertEqual(world.get_parameter('osm_map_path').value, str(self.osm))
        self.assertEqual(world.get_parameter('origin_lat').value,45.5)
        self.assertEqual(world.transitions, ['configure','activate','deactivate','cleanup','configure','activate'])
        self.assertFalse(reload.needs_reload(bundle, 1))
        # A fast restart may not be observed as a missing discovery service.
        world.set_parameters([Parameter('osm_map_path', value='restart-default.osm')])
        self.assertTrue(reload.needs_reload(bundle, 1))

    def test_cpp_nested_startup_repeated_switch_and_transition_failure(self):
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('carla_lifecycle'))/'lib/carla_lifecycle/lifecycle_manager'
        if not executable.is_file():
            self.skipTest('requires built carla_lifecycle executable')
        scenario = self.add(NestedScenario())
        bridge = self.add(StubBridge())
        coordinator = self.add(Node('cpp_observer'))
        prepare = coordinator.create_client(Trigger, '/carla/prepare_for_scenario_switch')
        finish = coordinator.create_client(Trigger, '/carla/finish_scenario_switch')
        self.spin_background()
        log = open(Path(self.tmp.name)/'manager.log','w+')
        self.addCleanup(log.close)
        process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/carla',
            '-p', 'scenario_server_name:=/carla/test_scenario',
            '-p', 'node_names:=[/carla/test_bridge,/carla/absent_hud]',
            '-p', 'optional_node_names:=[/carla/absent_hud]',
            '-p', 'startup_retry_interval:=0.1', '-p', 'service_timeout:=1.0'],
            stdout=log, stderr=subprocess.STDOUT)
        def stop():
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        self.addCleanup(stop)
        try:
            self.wait(lambda: scenario.activated, timeout=10)
            for _ in range(2):
                self.assertTrue(call(prepare, Trigger.Request(), timeout=8).success)
                self.assertTrue(call(finish, Trigger.Request(), timeout=8).success)
                self.assertIsNone(process.poll())
            self.assertTrue(call(prepare, Trigger.Request(), timeout=8).success)
            bridge.reject_activation = True
            self.assertFalse(call(finish, Trigger.Request(), timeout=8).success)
        except Exception:
            log.flush(); log.seek(0)
            print(log.read())
            raise


if __name__ == '__main__':
    unittest.main()
