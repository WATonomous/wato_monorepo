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
"""Reviewed YAML experiments, shared by the host CLI and Foxglove services.

The runner never ticks CARLA or starts a second controller. The scenario server
owns physics; the host starts the declared stack between ``open`` and ``ready``.
A wall-clock executor keeps service replies and status alive while /clock stops.
"""

import copy
import json
import math
import os
import re
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from carla_msgs.srv import SwitchScenario
from geometry_msgs.msg import PointStamped, PoseStamped
from lanelet_msgs.msg import RouteAhead
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from wato_trajectory_msgs.msg import Trajectory


OUTCOME_CODES = {"pass": 0, "assertion_failure": 1, "data_error": 2, "interrupted": 3}
STACK_NODES = {
    "controller_mpc": [
        "/action/fake_planner/fake_planner_node",
        "/action/mpc_controller_node",
    ],
    "autonomy": [
        "/world_modeling/prediction_node",
        "/world_modeling/world_model",
        "/world_modeling/costmap_node",
        "/action/lattice_planning/lattice_planning_node",
        "/action/trajectory_planning/trajectory_planner_node",
        "/action/mpc_controller_node",
    ],
}


def utc_now():
    """Use UTC for artifacts and monotonic time for execution deadlines."""
    return datetime.now(timezone.utc).isoformat()


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def load_definition(directory, category, name):
    """Resolve only installed, named definitions; reject malformed manifests early."""
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise ValueError(f"Invalid {category} name {name!r}; use watod sim list.")
    path = directory / category / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"Missing {category} definition: {path}; use watod sim list.")
    with path.open(encoding="utf-8") as stream:
        definition = yaml.safe_load(stream)
    if not isinstance(definition, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    if definition.get("version") != 1 or definition.get("id") != name:
        raise ValueError(f"Expected version: 1 and id: {name} in {path}")
    if category == "scenarios":
        stack = definition.get("stack")
        completion = definition.get("completion")
        if (stack, completion) not in {
            ("controller_mpc", "maneuver_finished"),
            ("autonomy", "stopped_behind_actor"),
        }:
            raise ValueError(f"Unsupported stack/completion in {path}")
        if (
            stack == "controller_mpc"
            and definition.get("maneuver") != "steady_state_slalom"
        ):
            raise ValueError(
                "V1 controller preset requires shipped steady_state_slalom"
            )
        events = definition.get("events")
        if not isinstance(events, list) or not events:
            raise ValueError(f"events must be a nonempty ordered YAML sequence: {path}")
        previous = -1.0
        for event in events:
            if not isinstance(event, dict) or set(event) != {"when", "do"}:
                raise ValueError(f"Each event needs exactly when/do: {path}")
            trigger = event["when"]
            if trigger == "play":
                at = 0.0
            elif isinstance(trigger, dict) and set(trigger) == {"elapsed_s"}:
                at = trigger["elapsed_s"]
            else:
                raise ValueError(f"Unsupported event trigger: {trigger}")
            if (
                isinstance(at, bool)
                or not isinstance(at, (int, float))
                or not math.isfinite(at)
                or at < 0.0
                or at < previous
            ):
                raise ValueError(
                    "Event timestamps must be finite, nonnegative and ordered"
                )
            previous = at
            if event["do"] not in {"start_maneuver", "begin_evaluation"}:
                raise ValueError(f"Unsupported reviewed action: {event['do']}")
            if event["do"] == "start_maneuver" and stack != "controller_mpc":
                raise ValueError("start_maneuver requires controller_mpc")
        for section in ("deadlines", "thresholds"):
            values = definition.get(section)
            if not isinstance(values, dict) or not values:
                raise ValueError(f"Missing {section}: {path}")
            for key, value in values.items():
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{section}.{key} must be numeric")
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{section}.{key} must be finite and positive")
        required_deadlines = {
            "readiness_s",
            "execution_s",
            "stalled_clock_s",
            "first_output_s",
            "data_freshness_s",
        }
        required_thresholds = {"min_progress_m", "min_samples"}
        required_thresholds |= (
            {"lateral_tracking_rmse_m", "max_lateral_error_m"}
            if stack == "controller_mpc"
            else {
                "min_clearance_m",
                "max_clearance_m",
                "stopped_speed_mps",
                "stopped_duration_s",
            }
        )
        if not required_deadlines <= definition["deadlines"].keys():
            raise ValueError(f"Missing deadline fields in {path}")
        if not required_thresholds <= definition["thresholds"].keys():
            raise ValueError(f"Missing metric thresholds in {path}")
        if previous > definition["deadlines"]["execution_s"]:
            raise ValueError("An event falls after the execution deadline")
        if (
            stack == "autonomy"
            and definition["thresholds"]["min_clearance_m"]
            >= definition["thresholds"]["max_clearance_m"]
        ):
            raise ValueError("min_clearance_m must be below max_clearance_m")
        load_definition(directory, "playgrounds", definition.get("playground", ""))
    return definition


class InvalidTransition(ValueError):
    """A rejected user command must leave the current experiment intact."""


class SimulationRunner(Node):
    """One experiment and one fresh result directory per attempt."""

    def __init__(self):
        super().__init__("simulation_runner")
        self.declare_parameter(
            "results_directory",
            os.environ.get(
                "WATOD_SIM_RESULTS_DIRECTORY", "/opt/watonomous/sim_results"
            ),
        )
        self.declare_parameter(
            "session_id", os.environ.get("WATOD_SIM_SESSION_ID", "manual")
        )
        self.declare_parameter(
            "revision", os.environ.get("WATOD_SIM_REVISION", "unknown")
        )
        self.declare_parameter("effective_images", "{}")
        self.definitions = (
            Path(get_package_share_directory("carla_scenarios")) / "config"
        )
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.io_group = ReentrantCallbackGroup()
        self.latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.state = "IDLE"
        self.server_active = False
        self.base_state_future = None
        self.base_checked_wall = 0.0
        self.kind = ""
        self.experiment = ""
        self.definition = {}
        self.world = {}
        self.run_id = ""
        self.result_directory = None
        self.outcome = None
        self.reasons = []
        self.events = []
        self.next_event = 0
        self.sim_time = None
        self.last_clock_wall = time.monotonic()
        self.started_sim = None
        self.elapsed_s = 0.0
        self.started_utc = ""
        self.pause_started_wall = None
        self.paused_wall_s = 0.0
        self.started_wall = None
        self.accepting_samples = False
        self.samples = {}
        self.reference = []
        self.actual_path = RosPath()
        self.actual_path.header.frame_id = "map"
        self.error_squared = 0.0
        self.error_max = 0.0
        self.error_count = 0
        self.sample_count = 0
        self.progress_m = 0.0
        self.closest_clearance = None
        self.final_clearance = None
        self.stop_started_sim = None
        self.stopped_duration_s = 0.0
        self.planner_status = ""
        self.collision = False
        self.trajectory_seen = False
        self.maneuver_finished_observed = False
        self.command_while_paused = False
        self.last_evaluated_stamp = None
        self.status_pub = self.create_publisher(String, "/sim/status", self.latched_qos)
        self.path_pub = self.create_publisher(
            RosPath, "/sim/actual_path", self.latched_qos
        )
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.speed_pub = self.create_publisher(Float64, "/sim/speed_mps", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clients = {}
        self.create_subscription(
            Clock, "/clock", self._clock, 10, callback_group=self.io_group
        )
        for key, topic, message_type in (
            ("odom", "/ego/odom", Odometry),
            ("detections", "/carla/detections_3d", Detection3DArray),
            ("costmap", "/world_modeling/costmap", OccupancyGrid),
            ("route", "/world_modeling/lanelet/route_ahead", RouteAhead),
            ("trajectory", "/action/trajectory_planning/trajectory", Trajectory),
        ):
            self.create_subscription(
                message_type,
                topic,
                lambda msg, key=key: self._sample(key, msg),
                qos_profile_sensor_data,
                callback_group=self.io_group,
            )
        self.create_subscription(
            RosPath,
            "/sim/reference_path",
            self._reference,
            self.latched_qos,
            callback_group=self.io_group,
        )
        self.create_subscription(
            String,
            "/action/fake_planner/status",
            self._planner_status,
            self.latched_qos,
            callback_group=self.io_group,
        )
        self.create_subscription(
            Bool,
            "/carla/experiment_collision",
            self._collision,
            self.latched_qos,
            callback_group=self.io_group,
        )
        self.create_service(
            SwitchScenario, "/sim/open", self._open, callback_group=self.control_group
        )
        for command in (
            "ready",
            "play",
            "pause",
            "resume",
            "stop",
            "reset",
            "close",
            "status",
        ):
            self.create_service(
                Trigger,
                f"/sim/{command}",
                lambda request, response, command=command: self._control(
                    command, response
                ),
                callback_group=self.control_group,
            )
        # The node intentionally uses wall time, including this heartbeat timer.
        self.create_timer(0.1, self._update, callback_group=self.control_group)
        self.get_logger().info("Simulation runner IDLE; use watod sim open")

    def _clock(self, message):
        current = seconds(message.clock)
        if self.sim_time is None or current != self.sim_time:
            self.last_clock_wall = time.monotonic()
        self.sim_time = current

    def _sample(self, key, message):
        if not self.accepting_samples:
            return
        stamp = seconds(message.header.stamp)
        # Reject retained pre-reset samples and observations from another clock epoch.
        if self.sim_time is None or abs(stamp - self.sim_time) > 1.0:
            return
        if key == "odom":
            position = message.pose.pose.position
            rotation = message.pose.pose.orientation
            velocity = message.twist.twist.linear
            if not all(
                math.isfinite(value)
                for value in (
                    position.x,
                    position.y,
                    position.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    velocity.x,
                    velocity.y,
                    velocity.z,
                )
            ):
                return
            self.speed_pub.publish(Float64(data=math.hypot(velocity.x, velocity.y)))
        self.samples[key] = (message, time.monotonic(), stamp)
        if key == "trajectory" and message.points:
            if (
                self.state == "RUNNING"
                and self.started_sim is not None
                and stamp >= self.started_sim
            ):
                self.trajectory_seen = True
            elif self.state == "PAUSED" and self.pause_started_wall is not None:
                # Allow packets already in flight at the acknowledged boundary.
                if time.monotonic() - self.pause_started_wall > 0.25:
                    self.command_while_paused = True

    def _reference(self, message):
        if not self.accepting_samples:
            return
        if message.header.frame_id == "map" and len(message.poses) >= 2:
            reference = [
                (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
            ]
            if all(math.isfinite(value) for point in reference for value in point):
                self.reference = reference

    def _planner_status(self, message):
        if not self.accepting_samples:
            return
        self.planner_status = message.data
        if message.data == "finished" and self.state == "RUNNING":
            self.maneuver_finished_observed = True

    def _collision(self, message):
        if not self.accepting_samples:
            return
        self.samples["collision"] = (message, time.monotonic(), self.sim_time)
        self.collision = self.collision or message.data

    def _client(self, service_type, endpoint):
        if endpoint not in self.clients:
            self.clients[endpoint] = self.create_client(
                service_type, endpoint, callback_group=self.io_group
            )
        return self.clients[endpoint]

    def _call(self, service_type, endpoint, request=None, timeout=15.0):
        """Wait on a separate executor group, bounded by monotonic wall time."""
        start = time.monotonic()
        client = self._client(service_type, endpoint)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f"Missing ROS endpoint {endpoint} after {time.monotonic() - start:.1f}s; fix the stack and run watod sim reset."
            )
        future = client.call_async(request or service_type.Request())
        while not future.done() and time.monotonic() - start < timeout:
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            raise RuntimeError(
                f"ROS endpoint {endpoint} timed out after {time.monotonic() - start:.1f}s; fix the stack and run watod sim reset."
            )
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS endpoint {endpoint} returned no result")
        if hasattr(result, "success") and not result.success:
            raise RuntimeError(
                f"{endpoint}: {getattr(result, 'message', 'operation rejected')}"
            )
        return result

    def _pause_world(self, paused):
        request = SetBool.Request()
        request.data = paused
        self._call(SetBool, "/carla/pause", request)

    def _hold(self):
        self._call(Trigger, "/carla/scenario_server/hold")

    def _world_endpoints_ready(self):
        return all(
            self._client(service_type, endpoint).service_is_ready()
            for service_type, endpoint in (
                (SwitchScenario, "/carla/scenario_server/prepare_experiment"),
                (SetBool, "/carla/pause"),
                (Trigger, "/carla/scenario_server/hold"),
                (Trigger, "/carla/scenario_server/release"),
                (Trigger, "/carla/scenario_server/close_experiment"),
            )
        )

    def _refresh_base_state(self):
        """Poll lifecycle state without blocking status or startup diagnostics."""
        if self.base_state_future is not None and self.base_state_future.done():
            try:
                response = self.base_state_future.result()
                self.server_active = (
                    response.current_state.id == State.PRIMARY_STATE_ACTIVE
                )
            except Exception:
                self.server_active = False
            self.base_state_future = None
        now = time.monotonic()
        if self.base_state_future is not None:
            if now - self.base_checked_wall > 3.0:
                self.base_state_future.cancel()
                self.base_state_future = None
                self.server_active = False
            return
        if now - self.base_checked_wall < 1.0:
            return
        self.base_checked_wall = now
        client = self._client(GetState, "/carla/scenario_server/get_state")
        if client.service_is_ready():
            self.base_state_future = client.call_async(GetState.Request())
        else:
            self.server_active = False

    def _snapshot(self):
        return {
            "state": self.state,
            "session_ready": self.server_active and self._world_endpoints_ready(),
            "experiment": self.experiment,
            "kind": self.kind,
            "stack": self.definition.get("stack", ""),
            "run_id": self.run_id,
            "session_id": self.get_parameter("session_id").value,
            "elapsed_s": self.elapsed_s,
            "outcome": self.outcome,
            "exit_code": OUTCOME_CODES.get(self.outcome),
            "result_directory": str(self.result_directory)
            if self.result_directory
            else None,
            "reasons": self.reasons,
            "test_timeout_s": self.definition.get("deadlines", {}).get(
                "execution_s", 90.0
            )
            * 5
            + 180.0,
            "wall_heartbeat_utc": utc_now(),
        }

    def _reply(self, response):
        response.success = True
        response.message = json.dumps(self._snapshot(), allow_nan=False)
        return response

    def _require(self, command, states):
        if self.state not in states:
            next_command = {
                "IDLE": "watod sim open scenario NAME",
                "PREPARING": "watod sim close (or finish stack preparation)",
                "READY": "watod sim play",
                "RUNNING": "watod sim pause or watod sim close",
                "PAUSED": "watod sim resume or watod sim reset",
                "FINISHED": "watod sim reset or watod sim close",
            }[self.state]
            if command == "open":
                raise InvalidTransition(
                    f"State {self.state}. Run watod sim close first."
                )
            raise InvalidTransition(
                f"Cannot {command} in state {self.state}; next: {next_command}."
            )

    def _open(self, request, response):
        opened_attempt = False
        try:
            self._require("open", {"IDLE"})
            if not self.server_active or not self._world_endpoints_ready():
                raise ValueError("Run watod sim setup first.")
            parts = request.scenario_name.split("/")
            if len(parts) != 2 or parts[0] not in {"scenario", "playground"}:
                raise ValueError(
                    "Use scenario/NAME or playground/NAME; run watod sim list."
                )
            kind, name = parts
            definition = load_definition(
                self.definitions,
                "scenarios" if kind == "scenario" else "playgrounds",
                name,
            )
            # Validate everything local before creating an attempt or mutating the world.
            self.kind, self.experiment, self.definition = kind, name, definition
            self._new_attempt()
            opened_attempt = True
            self._prepare_world()
            return self._reply(response)
        except Exception as error:
            if opened_attempt and self.state == "PREPARING":
                self._finish("data_error", [str(error)])
            response.success = False
            response.message = str(error)
            return response

    def _new_attempt(self):
        self.accepting_samples = False
        run_id = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
            + "_"
            + uuid.uuid4().hex[:12]
        )
        result_directory = Path(self.get_parameter("results_directory").value) / run_id
        result_directory.mkdir(parents=True, exist_ok=False)
        self.run_id, self.result_directory = run_id, result_directory
        self.state = "PREPARING"
        self.started_utc = utc_now()
        self.outcome, self.reasons = None, []
        self.events, self.next_event = [], 0
        self.started_sim, self.elapsed_s = None, 0.0
        self.started_wall, self.pause_started_wall, self.paused_wall_s = None, None, 0.0
        self.error_squared = self.error_max = 0.0
        self.error_count = self.sample_count = 0
        self.progress_m = 0.0
        self.closest_clearance = self.final_clearance = None
        self.stop_started_sim, self.stopped_duration_s = None, 0.0
        self.trajectory_seen = self.command_while_paused = self.collision = False
        self.maneuver_finished_observed = False
        self.planner_status, self.reference = "", []
        self.last_evaluated_stamp = None
        self.samples.clear()
        self.world = {}
        self.tf_buffer.clear()
        self._write_configuration()
        self.actual_path = RosPath()
        self.actual_path.header.frame_id = "map"
        self.path_pub.publish(self.actual_path)

    def _prepare_world(self):
        name = (
            self.definition["playground"]
            if self.kind == "scenario"
            else self.experiment
        )
        request = SwitchScenario.Request()
        request.scenario_name = name
        result = self._call(
            SwitchScenario,
            "/carla/scenario_server/prepare_experiment",
            request,
            timeout=120.0,
        )
        self.world = json.loads(result.message)
        if self.definition.get("stack") == "autonomy" and not self.world.get(
            "map", {}
        ).get("provenance"):
            raise ValueError(
                "Missing converted-map provenance; reconvert the oval map, then run watod sim reset."
            )
        self.samples.clear()
        self.collision = False
        self.accepting_samples = True
        self._hold()
        self._pause_world(False)
        self._write_configuration()

    def _write_configuration(self):
        configuration = {
            "run_id": self.run_id,
            "session_id": self.get_parameter("session_id").value,
            "git_revision": self.get_parameter("revision").value,
            "effective_images": json.loads(
                self.get_parameter("effective_images").value
            ),
            "definition": self.definition,
            "world": self.world,
            "environment": {
                key: os.environ.get(key, "unknown")
                for key in ("WATOD_SIM_IMAGES", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION")
            },
            "threshold_status": "provisional; target-host baseline pending",
        }
        with (self.result_directory / "configuration.yaml").open(
            "w", encoding="utf-8"
        ) as stream:
            yaml.safe_dump(configuration, stream, sort_keys=False)

    def _control(self, command, response):
        try:
            if command == "status":
                return self._reply(response)
            getattr(self, f"_do_{command}")()
            return self._reply(response)
        except Exception as error:
            if isinstance(error, InvalidTransition):
                pass
            elif command in {"ready", "reset"} and self.state == "PREPARING":
                self._finish("data_error", [str(error)])
            elif (
                command in {"play", "pause", "resume"}
                and self.state in {"READY", "RUNNING", "PAUSED"}
                and not isinstance(error, InvalidTransition)
            ):
                self._finish("data_error", [str(error)])
            response.success = False
            response.message = str(error)
            return response

    def _node_state(self, name):
        return self._call(GetState, f"{name}/get_state", timeout=3.0).current_state.id

    def _transition(self, name, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        self._call(ChangeState, f"{name}/change_state", request)

    def _do_ready(self):
        self._require("ready", {"PREPARING"})
        timeout = self.definition.get("deadlines", {}).get("readiness_s", 60.0)
        start = time.monotonic()
        pending = "waiting for /clock and /ego/odom"
        goal_sent = False
        while time.monotonic() - start < timeout:
            pending = self._readiness()
            if (
                self.definition.get("stack") == "autonomy"
                and self._fresh("odom")
                and not goal_sent
            ):
                if (
                    self.goal_pub.get_subscription_count() > 0
                    and self._all_nodes_active()
                ):
                    self._publish_goal()
                    goal_sent = True
            if pending is None:
                self._pause_world(True)
                # No scenario time has elapsed yet; setup ticks are excluded.
                self.state = "READY"
                self.elapsed_s = 0.0
                self.events.append(
                    {
                        "event": "ready",
                        "utc": utc_now(),
                        "elapsed_s": 0.0,
                        "setup_wall_s": time.monotonic() - start,
                    }
                )
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"Readiness timed out after {time.monotonic() - start:.1f}s: {pending}. Fix the named input and run watod sim reset."
        )

    def _all_nodes_active(self):
        try:
            return all(
                self._node_state(node) == State.PRIMARY_STATE_ACTIVE
                for node in STACK_NODES.get(self.definition.get("stack"), [])
            )
        except RuntimeError:
            return False

    def _fresh(self, key):
        sample = self.samples.get(key)
        if sample is None or self.sim_time is None:
            return False
        maximum = self.definition.get("deadlines", {}).get("data_freshness_s", 2.0)
        return 0.0 <= self.sim_time - sample[2] <= maximum

    def _readiness(self):
        if self.sim_time is None or time.monotonic() - self.last_clock_wall > 5.0:
            return "/clock is missing or stalled"
        if not self._fresh("odom"):
            return "fresh /ego/odom is missing"
        try:
            odometry = self.samples["odom"][0]
            x, y, _ = self._ego_pose(odometry)
            velocity = odometry.twist.twist.linear
            if math.hypot(velocity.x, velocity.y) > 0.1:
                return "ego must be stationary during preparation"
            initial = self.world.get("ego", {})
            if math.hypot(x - initial.get("x", x), y - initial.get("y", y)) > 0.5:
                return "ego moved away from its initial spawn during preparation"
        except (TransformException, ValueError) as error:
            return f"map-frame ego transform: {error}"
        # Freeze only after all ego-dependent bridge nodes have completed their
        # configure-time sensor ticks. Localization becoming active is not enough.
        for node in (
            "carla_localization",
            "ackermann_control",
            "lidar_publisher",
            "bbox_publisher",
        ):
            try:
                if self._node_state(f"/carla/{node}") != State.PRIMARY_STATE_ACTIVE:
                    return f"bridge lifecycle node /carla/{node} must be active"
            except RuntimeError as error:
                return str(error)
        if self.kind == "playground":
            return None
        for node in STACK_NODES[self.definition["stack"]]:
            try:
                if self._node_state(node) != State.PRIMARY_STATE_ACTIVE:
                    return f"lifecycle node {node} must be active"
            except RuntimeError as error:
                return str(error)
        if not self._fresh("collision"):
            return "/carla/experiment_collision observation missing"
        # Both presets must have exactly one trajectory publisher and one command owner.
        if self.count_publishers("/action/trajectory_planning/trajectory") != 1:
            return (
                "expected exactly one /action/trajectory_planning/trajectory publisher"
            )
        if self.count_publishers("/carla/ackermann_control/command") != 1:
            return "expected exactly one /carla/ackermann_control/command publisher"
        if self.definition["stack"] == "controller_mpc":
            if self.planner_status != "ready" or not self.reference:
                return "/action/fake_planner/status ready and /sim/reference_path preview required"
            return None
        if not self._fresh("detections") or not self._parked_observed():
            return "parked actor missing in /carla/detections_3d"
        if not self._fresh("costmap") or not self._parked_in_costmap():
            return "parked actor not occupied in planner input /world_modeling/costmap"
        if not self._fresh("route") or not self.samples["route"][0].has_active_route:
            return "route acceptance missing on /world_modeling/lanelet/route_ahead"
        return None

    def _publish_goal(self):
        goal = self.world.get("goal")
        if not goal:
            raise ValueError(
                "World preparation did not provide a goal beyond the parked car"
            )
        message = PointStamped()
        message.header.frame_id = "map"
        message.header.stamp = Time(seconds=self.sim_time).to_msg()
        message.point.x = float(goal["x"])
        message.point.y = float(goal["y"])
        message.point.z = float(goal.get("z", 0.0))
        self.goal_pub.publish(message)
        self.events.append(
            {
                "event": "goal_published",
                "utc": utc_now(),
                "elapsed_s": 0.0,
                "goal": goal,
            }
        )

    def _do_play(self):
        self._require("play", {"READY"})
        if self.kind != "scenario":
            # Playgrounds remain available for independently launched modules.
            self._call(Trigger, "/carla/scenario_server/release")
            self._pause_world(False)
            self.state = "RUNNING"
            return
        self.started_sim = self.sim_time
        self.started_wall = time.monotonic()
        self.last_clock_wall = self.started_wall
        self.trajectory_seen = False
        self.events.append({"event": "play", "utc": utc_now(), "elapsed_s": 0.0})
        self._fire_events(0.0)
        self._call(Trigger, "/carla/scenario_server/release")
        self._pause_world(False)
        self.state = "RUNNING"

    def _do_pause(self):
        self._require("pause", {"RUNNING"})
        self._pause_world(True)
        self.state = "PAUSED"
        self.pause_started_wall = time.monotonic()
        self.command_while_paused = False
        self.events.append(
            {"event": "pause", "utc": utc_now(), "elapsed_s": self.elapsed_s}
        )

    def _do_resume(self):
        self._require("resume", {"PAUSED"})
        if self.command_while_paused:
            raise RuntimeError(
                "Trajectory commands were published while physics was paused; run watod sim reset."
            )
        self.paused_wall_s += time.monotonic() - self.pause_started_wall
        self._pause_world(False)
        self.last_clock_wall = time.monotonic()
        self.state = "RUNNING"
        self.events.append(
            {"event": "resume", "utc": utc_now(), "elapsed_s": self.elapsed_s}
        )

    def _do_stop(self):
        self._require("stop", {"READY", "RUNNING", "PAUSED", "PREPARING"})
        self._finish("interrupted", ["Stopped by operator"])

    def _do_close(self):
        if self.state == "IDLE":
            return
        if self.outcome is None:
            self._finish("interrupted", ["Experiment closed by operator"])
        self._call(Trigger, "/carla/scenario_server/close_experiment")
        self.state = "IDLE"
        self.kind = self.experiment = ""
        self.definition = {}
        self.accepting_samples = False
        self.samples.clear()

    def _do_reset(self):
        self._require("reset", {"READY", "RUNNING", "PAUSED", "FINISHED"})
        if self.outcome is None:
            self._finish("interrupted", ["Reset by operator"])
        # Deactivate and clean before reloading actors: no queued command may reach
        # a new ego, and controller/behavior history must not cross attempt IDs.
        nodes = STACK_NODES.get(self.definition.get("stack"), [])
        for node in reversed(nodes):
            state = self._node_state(node)
            if state == State.PRIMARY_STATE_ACTIVE:
                self._transition(node, Transition.TRANSITION_DEACTIVATE)
                state = State.PRIMARY_STATE_INACTIVE
            if state == State.PRIMARY_STATE_INACTIVE:
                self._transition(node, Transition.TRANSITION_CLEANUP)
        if self.definition.get("stack") == "autonomy":
            self._call(Trigger, "/world_modeling/reset_bt")
        self._new_attempt()
        self._prepare_world()
        for node in nodes:
            if self._node_state(node) == State.PRIMARY_STATE_UNCONFIGURED:
                self._transition(node, Transition.TRANSITION_CONFIGURE)
            self._transition(node, Transition.TRANSITION_ACTIVATE)
        if self.definition.get("stack") == "controller_mpc":
            self._call(Trigger, "/action/fake_planner/reset")
        self._do_ready()

    def _fire_events(self, elapsed):
        sequence = self.definition.get("events", [])
        while self.next_event < len(sequence):
            event = sequence[self.next_event]
            trigger = event["when"]
            at = 0.0 if trigger == "play" else trigger["elapsed_s"]
            if elapsed < at:
                return
            # Consume before delivery. A failed action terminates the attempt;
            # it is never retried and cannot accidentally run twice.
            index = self.next_event
            self.next_event += 1
            record = {
                "index": index,
                "when": trigger,
                "do": event["do"],
                "elapsed_s": elapsed,
                "utc": utc_now(),
            }
            self.events.append(record)
            if event["do"] == "start_maneuver":
                self._call(Trigger, "/action/fake_planner/start_trajectory")
            record["delivered"] = True

    def _ego_pose(self, message):
        position = message.pose.pose.position
        heading = yaw(message.pose.pose.orientation)
        frame = message.header.frame_id
        if frame == "map":
            return position.x, position.y, heading
        transform = self.tf_buffer.lookup_transform("map", frame, Time())
        rotation = yaw(transform.transform.rotation)
        translation = transform.transform.translation
        return (
            translation.x
            + math.cos(rotation) * position.x
            - math.sin(rotation) * position.y,
            translation.y
            + math.sin(rotation) * position.x
            + math.cos(rotation) * position.y,
            heading + rotation,
        )

    @staticmethod
    def _footprint_center(actor, x=None, y=None, heading=None):
        x = actor["x"] if x is None else x
        y = actor["y"] if y is None else y
        heading = actor["yaw"] if heading is None else heading
        offset_x = actor.get("center_offset_x_m", 0.0)
        offset_y = actor.get("center_offset_y_m", 0.0)
        return (
            x + math.cos(heading) * offset_x - math.sin(heading) * offset_y,
            y + math.sin(heading) * offset_x + math.cos(heading) * offset_y,
        )

    def _parked_observed(self):
        parked = self.world.get("parked")
        if not parked:
            return False
        center_x, center_y = self._footprint_center(parked)
        message = self.samples["detections"][0]
        if message.header.frame_id != "map":
            return False
        return any(
            math.hypot(
                item.bbox.center.position.x - center_x,
                item.bbox.center.position.y - center_y,
            )
            < max(2.0, parked["length_m"] * 0.5)
            for item in message.detections
        )

    def _parked_in_costmap(self):
        """Require the actual planner grid to contain the ground-truth obstacle."""
        grid = self.samples["costmap"][0]
        parked = self.world.get("parked")
        if (
            not parked
            or not math.isfinite(grid.info.resolution)
            or grid.info.resolution <= 0
        ):
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                grid.header.frame_id, "map", Time()
            )
        except TransformException:
            return False
        rotation = yaw(transform.transform.rotation)
        translation = transform.transform.translation
        parked_x, parked_y = self._footprint_center(parked)
        x = (
            translation.x
            + math.cos(rotation) * parked_x
            - math.sin(rotation) * parked_y
        )
        y = (
            translation.y
            + math.sin(rotation) * parked_x
            + math.cos(rotation) * parked_y
        )
        origin = grid.info.origin
        angle = yaw(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        col = math.floor(
            (math.cos(angle) * dx + math.sin(angle) * dy) / grid.info.resolution
        )
        row = math.floor(
            (-math.sin(angle) * dx + math.cos(angle) * dy) / grid.info.resolution
        )
        if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
            return False
        index = row * grid.info.width + col
        return index < len(grid.data) and grid.data[index] >= 50

    def _lateral_error(self, x, y):
        closest = math.inf
        for (ax, ay), (bx, by) in zip(self.reference, self.reference[1:]):
            dx, dy = bx - ax, by - ay
            norm = dx * dx + dy * dy
            fraction = (
                max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / norm))
                if norm
                else 0.0
            )
            closest = min(
                closest, math.hypot(x - ax - fraction * dx, y - ay - fraction * dy)
            )
        return closest

    def _evaluate_sample(self):
        message, _, stamp = self.samples["odom"]
        if self.last_evaluated_stamp is not None and stamp <= self.last_evaluated_stamp:
            return
        if (
            self.last_evaluated_stamp is not None
            and stamp - self.last_evaluated_stamp > 0.5
        ):
            # A gap in odometry cannot establish a continuous stopped interval.
            self.stop_started_sim = None
            self.stopped_duration_s = 0.0
        self.last_evaluated_stamp = stamp
        x, y, heading = self._ego_pose(message)
        velocity = message.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if not all(math.isfinite(value) for value in (x, y, heading, speed, stamp)):
            raise ValueError("Non-finite odometry or velocity sample")
        lane = self.world["lane"]
        ego = self.world["ego"]
        self.progress_m = (x - ego["x"]) * lane["dx"] + (y - ego["y"]) * lane["dy"]
        self.sample_count += 1
        pose = PoseStamped()
        pose.header = copy.deepcopy(message.header)
        pose.header.frame_id = "map"
        pose.pose = copy.deepcopy(message.pose.pose)
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = (
            math.sin(heading / 2),
            math.cos(heading / 2),
        )
        pose.pose.orientation.x = pose.pose.orientation.y = 0.0
        self.actual_path.header.stamp = message.header.stamp
        self.actual_path.poses.append(pose)
        self.path_pub.publish(self.actual_path)
        if self.definition["stack"] == "controller_mpc":
            error = self._lateral_error(x, y)
            if not math.isfinite(error):
                raise ValueError("Reference path is missing or invalid")
            self.error_squared += error * error
            self.error_max = max(self.error_max, error)
            self.error_count += 1
            return
        parked = self.world["parked"]
        lane_yaw = math.atan2(lane["dy"], lane["dx"])
        ego_half = 0.5 * (
            abs(math.cos(heading - lane_yaw)) * ego["length_m"]
            + abs(math.sin(heading - lane_yaw)) * ego["width_m"]
        )
        parked_half = 0.5 * (
            abs(math.cos(parked["yaw"] - lane_yaw)) * parked["length_m"]
            + abs(math.sin(parked["yaw"] - lane_yaw)) * parked["width_m"]
        )
        ego_center_x, ego_center_y = self._footprint_center(ego, x, y, heading)
        parked_center_x, parked_center_y = self._footprint_center(parked)
        clearance = (
            (parked_center_x - ego_center_x) * lane["dx"]
            + (parked_center_y - ego_center_y) * lane["dy"]
            - ego_half
            - parked_half
        )
        self.final_clearance = clearance
        self.closest_clearance = (
            min(self.closest_clearance, clearance)
            if self.closest_clearance is not None
            else clearance
        )
        limits = self.definition["thresholds"]
        lateral_offset = abs(
            -(x - lane["x"]) * lane["dy"] + (y - lane["y"]) * lane["dx"]
        )
        stopped = (
            lateral_offset <= limits.get("max_lateral_offset_m", 1.5)
            and limits["min_clearance_m"] <= clearance <= limits["max_clearance_m"]
            and speed <= limits["stopped_speed_mps"]
            and self.progress_m >= limits["min_progress_m"]
        )
        if stopped:
            if self.stop_started_sim is None:
                self.stop_started_sim = stamp
            self.stopped_duration_s = stamp - self.stop_started_sim
        else:
            self.stop_started_sim = None
            self.stopped_duration_s = 0.0

    def _update(self):
        self._refresh_base_state()
        self.status_pub.publish(
            String(data=json.dumps(self._snapshot(), allow_nan=False))
        )
        if self.state != "RUNNING" or self.kind != "scenario":
            return
        try:
            deadlines = self.definition["deadlines"]
            wall_elapsed = time.monotonic() - self.started_wall - self.paused_wall_s
            if time.monotonic() - self.last_clock_wall > deadlines["stalled_clock_s"]:
                self._finish(
                    "data_error",
                    [f"/clock stalled for {deadlines['stalled_clock_s']} wall seconds"],
                )
                return
            if self.sim_time < self.started_sim:
                self._finish(
                    "data_error", ["Simulation clock moved backwards during an attempt"]
                )
                return
            self.elapsed_s = self.sim_time - self.started_sim
            self._fire_events(self.elapsed_s)
            if self.collision:
                self._finish("assertion_failure", ["Ego collision observed"])
                return
            if (
                not self.trajectory_seen
                and self.elapsed_s > deadlines["first_output_s"]
            ):
                self._finish("data_error", ["No trajectory output after play"])
                return
            required = ["odom", "collision"]
            if (
                self.elapsed_s > deadlines["first_output_s"]
                and self.planner_status != "finished"
            ):
                required.append("trajectory")
            if self.definition["stack"] == "autonomy":
                required += ["detections", "costmap", "route"]
            missing = [key for key in required if not self._fresh(key)]
            if missing:
                self._finish(
                    "data_error",
                    [f"Missing or stale required observations: {', '.join(missing)}"],
                )
                return
            if self.definition["stack"] == "autonomy":
                if not self._parked_observed() or not self._parked_in_costmap():
                    self._finish(
                        "data_error",
                        ["Parked actor absent from detections or planner costmap"],
                    )
                    return
                if not self.samples["route"][0].has_active_route:
                    self._finish("data_error", ["Autonomy route no longer active"])
                    return
            self._evaluate_sample()
            limits = self.definition["thresholds"]
            if (
                self.definition["stack"] == "controller_mpc"
                and self.planner_status == "finished"
            ):
                reasons = []
                if (
                    self.error_count < limits["min_samples"]
                    or self.progress_m < limits["min_progress_m"]
                ):
                    reasons.append(
                        "Maneuver completion lacks sufficient driven distance or error samples"
                    )
                rmse = (
                    math.sqrt(self.error_squared / self.error_count)
                    if self.error_count
                    else None
                )
                if rmse is None or rmse > limits["lateral_tracking_rmse_m"]:
                    reasons.append(
                        "Lateral tracking RMSE exceeds threshold or is missing"
                    )
                if self.error_max > limits["max_lateral_error_m"]:
                    reasons.append("Maximum lateral error exceeds threshold")
                self._finish("assertion_failure" if reasons else "pass", reasons)
            elif self.definition["stack"] == "autonomy":
                if self.final_clearance is not None and self.final_clearance < 0.0:
                    self._finish(
                        "assertion_failure",
                        ["Ego crossed the parked actor's rear bumper"],
                    )
                elif (
                    self.stopped_duration_s >= limits["stopped_duration_s"]
                    and self.sample_count >= limits["min_samples"]
                ):
                    self._finish("pass", [])
            if self.state == "RUNNING" and self.elapsed_s >= deadlines["execution_s"]:
                self._finish(
                    "assertion_failure", ["Scenario execution deadline exceeded"]
                )
            elif (
                self.state == "RUNNING"
                and wall_elapsed
                > deadlines["execution_s"] * 5 + deadlines["stalled_clock_s"]
            ):
                self._finish(
                    "data_error",
                    ["Execution wall deadline exceeded (excluding deliberate pauses)"],
                )
        except Exception as error:
            self._finish("data_error", [str(error)])

    def _metrics(self):
        return {
            "lateral_tracking_rmse_m": {
                "value": math.sqrt(self.error_squared / self.error_count)
                if self.error_count
                else None,
                "unit": "m",
            },
            "max_lateral_error_m": {
                "value": self.error_max if self.error_count else None,
                "unit": "m",
            },
            "completed": {
                "value": self.maneuver_finished_observed
                if self.definition.get("stack") == "controller_mpc"
                else self.outcome == "pass",
                "unit": "bool",
            },
            "closest_clearance_m": {"value": self.closest_clearance, "unit": "m"},
            "final_clearance_m": {"value": self.final_clearance, "unit": "m"},
            "distance_progressed_m": {"value": self.progress_m, "unit": "m"},
            "stopped_duration_s": {"value": self.stopped_duration_s, "unit": "s"},
            "sample_count": {"value": self.sample_count, "unit": "samples"},
        }

    def _finish(self, outcome, reasons):
        """Always save an outcome, including setup errors and operator interruption."""
        if self.outcome is not None:
            return
        self.outcome, self.reasons, self.state = outcome, list(reasons), "FINISHED"
        try:
            self._pause_world(True)
            self._hold()
            if self.definition.get("stack") == "controller_mpc":
                self._call(Trigger, "/action/fake_planner/stop_trajectory", timeout=3.0)
        except Exception as error:
            self.reasons.append(f"Finalization: {error}")
            if self.outcome == "pass":
                self.outcome = "data_error"
        self.events.append(
            {
                "event": "finished",
                "utc": utc_now(),
                "elapsed_s": self.elapsed_s,
                "outcome": self.outcome,
            }
        )
        if self.result_directory:
            result = {
                **self._snapshot(),
                "started_utc": self.started_utc,
                "finished_utc": utc_now(),
                "metrics": self._metrics(),
            }
            (self.result_directory / "events.json").write_text(
                json.dumps(self.events, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            (self.result_directory / "result.json").write_text(
                json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
        self.get_logger().info(
            f"Attempt {self.run_id}: {self.outcome}; results {self.result_directory}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulationRunner()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if node.state != "IDLE" and node.outcome is None:
            node._finish("interrupted", ["Runner shutdown"])
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
