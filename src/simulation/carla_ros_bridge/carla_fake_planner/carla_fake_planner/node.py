"""Services select standard WATO trajectories; action owns the controller."""

import json
import math
from pathlib import Path

import carla
import rclpy
from ament_index_python.packages import get_package_share_directory
from behaviour_msgs.msg import ExecuteBehaviour
from carla_msgs.msg import ScenarioStatus
from carla_msgs.srv import GetAvailableTrajectories, SelectTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wato_trajectory_msgs.msg import Trajectory, TrajectoryPoint
import yaml

from carla_common import connect_carla, find_ego_vehicle
from carla_common.lanelet_track import LaneletTrack
from carla_fake_planner.trajectories import Run, parameters


class CarlaFakePlanner(Node):
    def __init__(self):
        super().__init__("carla_fake_planner")
        default_catalog = Path(get_package_share_directory("carla_fake_planner")) / "config/trajectories.yaml"
        self.declare_parameter("presets_file", str(default_catalog))
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_timeout", 10.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("trajectory_topic", "/action/trajectory_planning/trajectory")
        self.declare_parameter("behaviour_topic", "/behaviour/execute_behaviour")
        self.catalog = yaml.safe_load(Path(self.get_parameter("presets_file").value).read_text())
        if self.catalog.get("version") != 1:
            raise ValueError("trajectory catalog version must be 1")
        for preset in self.catalog["presets"].values():
            parameters(preset, {})
        self.client = None
        self.track = None
        self.scenario = None
        self.map_key = None
        self.run = None
        self.selected = ""
        self.state = "idle"
        self.error = ""
        self.was_publishing = False
        self.trajectory_pub = self.create_publisher(
            Trajectory, self.get_parameter("trajectory_topic").value, 10)
        self.behaviour_pub = self.create_publisher(
            ExecuteBehaviour, self.get_parameter("behaviour_topic").value, 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(PathMessage, "~/path", retained)
        self.status_pub = self.create_publisher(String, "~/status", retained)
        self.create_subscription(ScenarioStatus, "/carla/scenario_server/scenario_status",
                                 self.scenario_callback, retained)
        self.create_service(GetAvailableTrajectories, "~/get_available", self.available)
        self.create_service(SelectTrajectory, "~/select", self.select)
        self.create_service(Trigger, "~/stop", self.stop)
        rate = float(self.get_parameter("publish_rate").value)
        if not 1 <= rate <= 100:
            raise ValueError("publish_rate must be 1..100")
        self.create_timer(1.0 / rate, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def scenario_callback(self, message):
        key = (message.generation, message.map_id)
        changed = key != self.map_key
        if changed or message.state in ("loading", "error", "idle"):
            self.cancel("idle")
        self.scenario = message
        if changed:
            self.map_key = key
            self.track = None
            self.client = None
            if message.trajectories_enabled and message.state in ("starting", "running", "paused"):
                try:
                    self.track = LaneletTrack(message.osm_map_path, message.origin_lat, message.origin_lon)
                    self.error = ""
                except Exception as error:
                    self.error = str(error)
                    self.state = "error"
        # A loading status may precede the new map metadata with the same generation.
        if self.track is None and message.trajectories_enabled and message.state == "running":
            try:
                self.track = LaneletTrack(message.osm_map_path, message.origin_lat, message.origin_lon)
                self.error = ""
            except Exception as error:
                self.error = str(error)
                self.state = "error"

    def ego(self):
        if self.client is None:
            self.client = connect_carla(self.get_parameter("carla_host").value,
                                        self.get_parameter("carla_port").value,
                                        self.get_parameter("carla_timeout").value)
        ego = find_ego_vehicle(self.client.get_world(), "ego_vehicle")
        if ego is None:
            raise ValueError("ego vehicle is not ready")
        transform = ego.get_transform()
        pose = (transform.location.x, -transform.location.y, -math.radians(transform.rotation.yaw))
        return ego, pose

    def available(self, request, response):
        response.trajectory_ids = list(self.catalog["presets"])
        response.descriptions = [preset["description"] for preset in self.catalog["presets"].values()]
        response.presets_yaml = yaml.safe_dump(self.catalog, sort_keys=False)
        return response

    def select(self, request, response):
        try:
            if self.scenario is None or self.scenario.state != "running" or not self.scenario.trajectories_enabled:
                raise ValueError("load the fake_planner scenario and wait for running status first")
            if self.track is None:
                raise ValueError(self.error or "Lanelet2 map is not ready")
            if request.trajectory_id not in self.catalog["presets"]:
                raise ValueError(f"unknown trajectory: {request.trajectory_id}")
            overrides = yaml.safe_load(request.parameters_yaml) if request.parameters_yaml else {}
            _, pose = self.ego()
            # Generate/validate first. A bad selection preserves the current run.
            run = Run(self.track, pose, self.catalog["presets"][request.trajectory_id], overrides)
            self.run = run
            self.selected = request.trajectory_id
            self.error = ""
            self.state = "publishing"
            self.publish_path(run.samples(full=True))
            self.publish()
            response.success = True
            response.message = f"Publishing {self.selected} from the current pose"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def empty(self):
        message = Trajectory()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_pub.publish(message)
        behaviour = ExecuteBehaviour()
        behaviour.behaviour = "standby"
        self.behaviour_pub.publish(behaviour)

    def cancel(self, state="stopped"):
        if self.run is not None or self.was_publishing:
            self.empty()
        self.run = None
        self.selected = ""
        self.state = state
        self.was_publishing = False

    def stop(self, request, response):
        self.cancel()
        # Also clear any controller input at a switch boundary before CARLA ticks stop.
        self.empty()
        response.success = True
        response.message = "Trajectory cleared; controller standby speed must be zero"
        return response

    def publish_path(self, samples):
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw, _ in samples:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            message.poses.append(pose)
        self.path_pub.publish(message)

    def publish(self):
        ready = self.scenario is not None and self.scenario.state == "running"
        if self.run is not None and ready:
            try:
                ego, pose = self.ego()
                self.run.advance(pose)
                velocity = ego.get_velocity()
                actual_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                if self.run.progress >= self.run.length:
                    self.cancel("completed")
                elif (math.isfinite(self.run.length) and self.run.progress > 1.0
                        and self.run.speed(self.run.progress + 0.5) <= 0.001 and actual_speed < 0.1):
                    self.cancel("completed")
                else:
                    message = Trajectory()
                    message.header.frame_id = "map"
                    message.header.stamp = self.get_clock().now().to_msg()
                    for x, y, yaw, speed in self.run.samples():
                        point = TrajectoryPoint()
                        point.pose.position.x, point.pose.position.y = x, y
                        point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                        point.max_speed = speed
                        message.points.append(point)
                    self.trajectory_pub.publish(message)
                    behaviour = ExecuteBehaviour()
                    behaviour.behaviour = "drive"
                    self.behaviour_pub.publish(behaviour)
                    self.was_publishing = True
            except Exception as error:
                self.cancel("error")
                self.error = str(error)
                self.get_logger().error(self.error)
        elif self.run is not None and self.was_publishing:
            # Pause preserves the selected geometry while stopping controller input.
            self.empty()
            self.was_publishing = False
        status = {"state": self.state, "trajectory": self.selected,
                  "progress_m": self.run.progress if self.run else 0.0,
                  "scenario_state": self.scenario.state if self.scenario else "unknown", "error": self.error}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = CarlaFakePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
