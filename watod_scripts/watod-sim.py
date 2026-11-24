#!/usr/bin/env python3
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
"""Host-side ownership and Compose adapter for the ROS simulation runner.

The runner owns experiment semantics. This helper only preflights installed
images, starts application runtimes and forwards the operator's ROS requests.
All mutable files live under this workspace's ignored .cache/watod-sim folder.
"""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
import shlex
import shutil
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

# A fresh, short-lived ROS client avoids relying on a host ROS installation or
# parsing ros2 CLI repr output. Arguments are passed separately, never via eval.
ROS_CLIENT = """
import json, sys, time
import rclpy
from std_srvs.srv import Trigger
from carla_msgs.srv import SwitchScenario
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

rclpy.init()
node = rclpy.create_node("watod_sim_client_" + str(__import__("os").getpid()))
endpoint, kind, value, timeout = sys.argv[1:]
timeout = float(timeout)
service_type = (
    SwitchScenario
    if kind == "open"
    else SetParameters
    if kind == "runtime"
    else Trigger
)
client = node.create_client(service_type, endpoint)
started = time.monotonic()
try:
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("unavailable service " + endpoint)
    request = service_type.Request()
    if kind == "open":
        request.scenario_name = value
    elif kind == "runtime":
        request.parameters = [
            Parameter(
                name="effective_images",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=value
                ),
            )
        ]
    future = client.call_async(request)
    remaining = max(0.0, timeout - (time.monotonic() - started))
    rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
    if not future.done() or future.result() is None:
        raise RuntimeError("timed out calling " + endpoint)
    result = future.result()
    success = (
        all(item.successful for item in result.results)
        if kind == "runtime"
        else result.success
    )
    message = (
        ("{}" if success else str(result.results))
        if kind == "runtime"
        else result.message
    )
    print("WATOD_RESPONSE=" + json.dumps({"success": success, "message": message}))
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


MAP_PREFLIGHT = """
import os, sys
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory("world_modeling_bringup"))
configuration = share / "config" / "map_configs" / (sys.argv[1] + ".yaml")
parameters = yaml.safe_load(configuration.read_text())["/**/world_model"]["ros__parameters"]
osm = Path(parameters["osm_map_path"])
if not osm.is_file() or not os.access(osm, os.R_OK):
    raise RuntimeError("Missing readable OSM map from " + str(configuration) + ": " + str(osm))
"""


class SessionError(RuntimeError):
    """An actionable, user-facing session or dependency error."""


class Session:
    def __init__(self):
        self.root = Path(os.environ["MONO_DIR"]).resolve()
        self.directory = self.root / ".cache/watod-sim"
        self.metadata = self.directory / "session.json"
        self.results = self.directory / "results"
        digest = hashlib.sha256(str(self.root).encode()).hexdigest()[:10]
        self.project = (
            os.environ.get("COMPOSE_PROJECT_NAME", "watod") + "_sim_" + digest
        )
        self.environment = dict(os.environ)
        self.environment["COMPOSE_PROJECT_NAME"] = self.project
        self.environment["WATOD_SIM_RESULTS_DIRECTORY"] = str(self.results)
        # Send /clock time broadcasts to Foxglove, including world-reset jumps.
        self.environment["WATOD_SIM_USE_SIM_TIME"] = "true"
        self.state = None
        self.compose = [
            "docker",
            "compose",
            "--project-name",
            self.project,
            "--env-file",
            str(self.root / "modules/.env"),
            "-f",
            str(self.root / "modules/docker-compose.yaml"),
        ]
        if self.environment.get("WATCLOUD_MODE") == "true":
            self.compose += [
                "-f",
                str(self.root / "modules/docker-compose.watcloud.yaml"),
            ]
        self.compose += ["-f", str(self.root / "modules/docker-compose.dep.yaml")]
        # config must include profiled services, while every create/start below
        # names its exact service and disables automatic dependency startup.
        self.compose += ["--profile", "*"]
        self.car_service = (
            "carla_sim"
            if self.environment.get("CARLA_RENDER_MODE") == "gpu"
            else "carla_sim_no_gpu"
        )

    def run(self, arguments, *, timeout=60, check=True):
        try:
            result = subprocess.run(
                arguments,
                cwd=self.root,
                env=self.environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise SessionError(f"Could not execute {arguments[0]}: {error}") from error
        if check and result.returncode:
            detail = (result.stderr or result.stdout or "").strip()
            command = " ".join(arguments[:5])
            if arguments[: len(self.compose)] == self.compose:
                # Include the actual Compose operation rather than truncating
                # the diagnostic before the subcommand and its flags.
                command = "docker compose " + " ".join(arguments[len(self.compose) :])
            raise SessionError(
                f"Command failed ({result.returncode}): {command}\n{detail}"
            )
        return result

    def dc(self, *arguments, **kwargs):
        return self.run(self.compose + list(arguments), **kwargs)

    def save(self):
        # Replace atomically inside the workspace so an interrupted write cannot
        # turn a valid session into an unparseable ownership record.
        pending = self.metadata.with_suffix(".pending")
        pending.write_text(json.dumps(self.state, indent=2) + "\n")
        pending.replace(self.metadata)

    def inspect(self, container):
        result = self.run(["docker", "inspect", container], check=False)
        if result.returncode:
            return None
        return json.loads(result.stdout)[0]

    def owned(self, container):
        info = self.inspect(container)
        if info is None:
            return None
        labels = info["Config"].get("Labels") or {}
        if labels.get("com.docker.compose.project") != self.project:
            raise SessionError(
                f"Ownership mismatch for {container}; refusing to modify it."
            )
        return info

    def recover_for_stop(self):
        """Recover cleanup ownership from Docker labels, never a reusable session."""
        result = self.run(
            [
                "docker",
                "ps",
                "--all",
                "--quiet",
                "--filter",
                f"label=com.docker.compose.project={self.project}",
            ]
        )
        containers = {}
        images = {}
        session_environment = {}
        services = (
            "network_namespace",
            "carla_sim",
            "carla_sim_no_gpu",
            "infrastructure_bringup",
            "sim_session",
            "sim_action",
            "sim_world",
        )
        required_files = {
            self.root / "modules/docker-compose.yaml",
            self.root / "modules/docker-compose.dep.yaml",
        }
        allowed_files = required_files | {
            self.root / "modules/docker-compose.watcloud.yaml"
        }
        for container in result.stdout.split():
            info = self.owned(container)
            if info is None:
                continue
            labels = info["Config"].get("Labels") or {}
            service = labels.get("com.docker.compose.service")
            working_directory = labels.get("com.docker.compose.project.working_dir")
            config_files = {
                Path(name).resolve()
                for name in labels.get(
                    "com.docker.compose.project.config_files", ""
                ).split(",")
                if name
            }
            if (
                service not in services
                or service in containers
                or not working_directory
                or Path(working_directory).resolve() != self.root / "modules"
                or not required_files <= config_files <= allowed_files
                or labels.get("com.docker.compose.oneoff") != "False"
            ):
                raise SessionError(
                    f"Cannot verify orphan ownership for {container}; "
                    "inspect its Compose labels before removing it."
                )
            if service == "sim_session":
                if not any(
                    mount.get("Type") == "bind"
                    and mount.get("Destination") == "/opt/watonomous/sim_results"
                    and Path(mount["Source"]).resolve() == self.results
                    for mount in info.get("Mounts", [])
                ):
                    raise SessionError(
                        f"Results mount does not belong to this workspace: {container}"
                    )
                session_environment = dict(
                    item.split("=", 1)
                    for item in info["Config"].get("Env", [])
                    if "=" in item
                )
            containers[service] = info["Id"]
            images[service] = info["Config"]["Image"]
        if not containers:
            return False
        self.state = {
            "workspace": str(self.root),
            "project": self.project,
            "session_id": session_environment.get("WATOD_SIM_SESSION_ID")
            or str(uuid.uuid4()),
            "revision": session_environment.get("WATOD_SIM_REVISION", "unknown"),
            "containers": {
                service: containers[service]
                for service in services
                if service in containers
            },
            "images": images,
            "recovered_for_stop": True,
        }
        # Persist verified IDs before cleanup so an interrupted stop can be retried.
        self.save()
        print(
            f"Recovered {len(containers)} verified containers for cleanup "
            f"in {self.project}.",
            file=sys.stderr,
        )
        return True

    def namespace_users(self, info):
        """Keep a shared namespace while any other container still references it."""
        targets = {info["Id"], info["Name"].lstrip("/")}
        users = []
        result = self.run(["docker", "ps", "--all", "--quiet"])
        for container in result.stdout.split():
            other = self.inspect(container)
            if other and other["HostConfig"]["NetworkMode"] in {
                "container:" + target for target in targets
            }:
                users.append(other["Name"].lstrip("/"))
        return users

    def load(self, *, require_running=True):
        if not self.metadata.exists():
            raise SessionError(
                "No recorded simulation session. Run watod sim setup; "
                "if leftover containers block setup, run watod sim stop first."
            )
        try:
            self.state = json.loads(self.metadata.read_text())
        except (ValueError, OSError) as error:
            raise SessionError(f"Cannot read {self.metadata}: {error}") from error
        if (
            self.state.get("workspace") != str(self.root)
            or self.state.get("project") != self.project
        ):
            raise SessionError(
                "Session configuration changed. Restore the original config and run watod sim stop first."
            )
        if require_running and self.state.get("recovered_for_stop"):
            raise SessionError(
                "Session ownership was recovered for cleanup only. "
                "Run watod sim stop, then watod sim setup."
            )
        self.environment["WATOD_SIM_SESSION_ID"] = self.state["session_id"]
        self.environment["WATOD_SIM_REVISION"] = self.state["revision"]
        for name, container in self.state["containers"].items():
            info = self.owned(container)
            if (
                require_running
                and name not in ("sim_action", "sim_world")
                and (info is None or not info["State"]["Running"])
            ):
                raise SessionError(
                    f"Session component {name} is not running. Run watod sim stop, then watod sim setup."
                )
        if require_running:
            for key in (
                "CARLA_PORT",
                "FOXGLOVE_BRIDGE_PORT",
                "ROS_DOMAIN_ID",
                "RMW_IMPLEMENTATION",
                "CARLA_RENDER_MODE",
                "WATCLOUD_MODE",
                "TAG",
            ):
                if self.state["configuration"].get(key) != self.environment.get(key):
                    raise SessionError(
                        f"Session setting {key} changed. Run watod sim stop with the original configuration first."
                    )

    def start(self, service, *, command=None):
        if command:
            variable = (
                "WATOD_SIM_ACTION_COMMAND"
                if service == "sim_action"
                else "WATOD_SIM_WORLD_COMMAND"
            )
            self.environment[variable] = command
        # `create` does not support --no-deps. `up --no-start` accepts that
        # flag while still letting us record ownership before starting anything.
        self.dc(
            "up",
            "--no-start",
            "--no-build",
            "--pull",
            "never",
            "--no-deps",
            service,
        )
        container = self.dc("ps", "--all", "--quiet", service).stdout.strip()
        if not container or "\n" in container:
            raise SessionError(
                f"Expected exactly one container for {service}; got {container!r}."
            )
        self.state["containers"][service] = container
        self.save()
        self.dc("start", service)

    def remove(self, services):
        for service in services:
            container = self.state["containers"].get(service)
            if container is None:
                continue
            info = self.owned(container)
            if info:
                users = (
                    self.namespace_users(info) if service == "network_namespace" else []
                )
                if users:
                    print(
                        f"Preserving shared network namespace {container[:12]}; "
                        f"referenced by {', '.join(users)}.",
                        file=sys.stderr,
                    )
                else:
                    try:
                        stopped = self.run(
                            ["docker", "stop", "--time", "15", container],
                            timeout=25,
                            check=False,
                        )
                        force = stopped.returncode != 0
                    except SessionError:
                        force = True
                    if force:
                        # Expired SLURM cgroups can leave Docker's running state stale.
                        self.owned(container)
                    self.run(
                        ["docker", "rm", *(["--force"] if force else []), container],
                    )
            del self.state["containers"][service]
            self.save()

    def rpc(self, action, value="", *, timeout=90, check=True):
        container = self.state["containers"].get("sim_session")
        if not container:
            raise SessionError("Run watod sim setup first.")
        endpoint = (
            "/simulation_runner/set_parameters"
            if action == "runtime"
            else "/sim/" + action
        )
        started = time.monotonic()
        result = self.run(
            [
                "docker",
                "exec",
                container,
                "/bin/bash",
                "-c",
                'source /opt/watonomous/setup.bash && exec python3 -c "$@"',
                "watod-sim-client",
                ROS_CLIENT,
                endpoint,
                action if action in ("open", "runtime") else "trigger",
                value,
                str(timeout),
            ],
            timeout=timeout + 10,
            check=False,
        )
        responses = [
            line.removeprefix("WATOD_RESPONSE=")
            for line in result.stdout.splitlines()
            if line.startswith("WATOD_RESPONSE=")
        ]
        if result.returncode or not responses:
            raise SessionError(
                f"{endpoint} unavailable or failed after {time.monotonic() - started:.1f}s. "
                f"Inspect docker logs {container}; run watod sim close after fixing the dependency.\n"
                f"{result.stderr.strip()}"
            )
        response = json.loads(responses[-1])
        if not response["success"] and check:
            raise SessionError(response["message"])
        try:
            snapshot = json.loads(response["message"])
        except ValueError:
            snapshot = {"message": response["message"]}
        if not response["success"]:
            snapshot["success"] = False
        return snapshot

    def check_images(self, services):
        config = json.loads(self.dc("config", "--format", "json").stdout)
        for service in services:
            image = config["services"][service]["image"]
            if self.run(["docker", "image", "inspect", image], check=False).returncode:
                raise SessionError(
                    f"Missing prepared image {image} for {service}. Prepare the module image, then rerun watod sim setup."
                )
        return config

    def save_startup_diagnostics(self, error):
        """Keep bounded evidence before failed-startup cleanup removes containers."""
        directory = self.directory / "startup-failures" / self.state["session_id"]
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "failure.json").write_text(
            json.dumps(
                {"error": f"{type(error).__name__}: {error}", "session": self.state},
                indent=2,
            )
            + "\n"
        )
        shutil.copyfile(self.directory / "compose.json", directory / "compose.json")
        for service, container in self.state["containers"].items():
            for suffix, command in (
                ("log", ["logs", "--timestamps", "--tail", "500"]),
                (
                    "state.log",
                    ["inspect", "--format", "{{.Image}} {{json .State}}"],
                ),
            ):
                try:
                    result = self.run(
                        ["docker", *command, container], timeout=5, check=False
                    )
                    output = result.stdout + result.stderr
                except SessionError as capture_error:
                    output = str(capture_error) + "\n"
                (directory / f"{service}.{suffix}").write_text(output)
        print(f"Startup diagnostics saved in {directory}", file=sys.stderr)

    def setup(self):
        if self.metadata.exists():
            self.load()
            snapshot = self.rpc("status", timeout=15)
            print(
                f"Reusing session {self.state['session_id']} ({snapshot.get('state')})."
            )
            self.print_address()
            return
        if not shutil.which("docker"):
            raise SessionError(
                "Docker is required. Install Docker Compose, then run watod sim setup."
            )
        if self.dc("ps", "--all", "--quiet").stdout.strip():
            raise SessionError(
                f"Unrecorded containers already use Compose project {self.project}. "
                "Run watod sim stop to clean verified leftovers, "
                "then retry watod sim setup."
            )
        base = [self.car_service, "infrastructure_bringup", "sim_session"]
        if self.environment.get("WATCLOUD_MODE") == "true":
            base.insert(0, "network_namespace")
        config = self.check_images(base)
        # A dedicated Compose project isolates stop from unrelated watod modules.
        # Refuse occupied ports rather than attaching to somebody else's server.
        held_ports = []
        try:
            for variable, offset in (
                ("CARLA_PORT", 0),
                ("CARLA_PORT", 1),
                ("CARLA_PORT", 2),
                ("FOXGLOVE_BRIDGE_PORT", 0),
            ):
                port = int(self.environment[variable]) + offset
                handle = socket.socket()
                held_ports.append(handle)
                try:
                    handle.bind(("0.0.0.0", port))
                except OSError as error:
                    raise SessionError(
                        f"Port {port} ({variable}+{offset}) is already in use. Release the conflicting service or configure another port before watod sim setup."
                    ) from error
        finally:
            for handle in held_ports:
                handle.close()
        self.results.mkdir(parents=True, exist_ok=True)
        revision = self.run(["git", "rev-parse", "HEAD"]).stdout.strip()
        self.state = {
            "workspace": str(self.root),
            "project": self.project,
            "session_id": str(uuid.uuid4()),
            "revision": revision,
            "containers": {},
            "images": {name: config["services"][name]["image"] for name in base},
            "configuration": {
                key: self.environment.get(key)
                for key in (
                    "CARLA_PORT",
                    "FOXGLOVE_BRIDGE_PORT",
                    "ROS_DOMAIN_ID",
                    "RMW_IMPLEMENTATION",
                    "CARLA_RENDER_MODE",
                    "WATCLOUD_MODE",
                    "TAG",
                )
            },
        }
        self.environment["WATOD_SIM_SESSION_ID"] = self.state["session_id"]
        self.environment["WATOD_SIM_REVISION"] = revision
        self.save()
        (self.directory / "compose.json").write_text(
            json.dumps(config, indent=2) + "\n"
        )
        started = time.monotonic()
        try:
            for service in base:
                self.start(service)
            setup_deadline = started + float(
                self.environment.get("WATOD_SIM_SETUP_TIMEOUT_S", "180")
            )
            while True:
                snapshot = self.rpc(
                    "status", timeout=max(1, setup_deadline - time.monotonic())
                )
                if snapshot.get("session_ready"):
                    break
                if time.monotonic() >= setup_deadline:
                    raise SessionError(
                        "CARLA world services did not become ready before the setup deadline. Inspect sim_session logs, then rerun watod sim setup."
                    )
                time.sleep(0.5)
            port = int(self.environment["FOXGLOVE_BRIDGE_PORT"])
            deadline = time.monotonic() + 30
            while True:
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=1):
                        break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise SessionError(
                            f"Foxglove port {port} did not become ready after 30s. Run watod sim setup after fixing infrastructure."
                        )
                    time.sleep(0.5)
        except BaseException as error:
            try:
                self.save_startup_diagnostics(error)
            except Exception as capture_error:
                # Diagnostic failures must not replace the startup error or
                # prevent cleanup of the containers recorded by this session.
                print(
                    f"Could not save startup diagnostics: {capture_error}",
                    file=sys.stderr,
                )
            self.remove(reversed(list(self.state["containers"])))
            self.metadata.unlink(missing_ok=True)
            raise
        self.state["setup_wall_s"] = time.monotonic() - started
        self.save()
        print(f"Session ready in {self.state['setup_wall_s']:.1f}s.")
        self.print_address()

    def print_address(self):
        host = self.environment.get("WATOD_SIM_FOXGLOVE_HOST", "localhost")
        print(f"Foxglove: ws://{host}:{self.environment['FOXGLOVE_BRIDGE_PORT']}")
        print(f"Results: {self.results}")

    def preflight(self, kind, name):
        directory = self.root / "src/simulation/carla_ros_bridge/carla_scenarios/config"
        path = (
            directory
            / ("scenarios" if kind == "scenario" else "playgrounds")
            / (name + ".yaml")
        )
        if not path.is_file():
            raise SessionError(
                f"Unknown {kind} {name!r}; expected {path}. Run watod sim list."
            )
        # The authoritative loader validates YAML and resolves installed assets
        # before /sim/open touches the current world. Preflight the stack image
        # packages and maps too, using idle containers that this session owns.
        if kind == "playground":
            return
        container = self.state["containers"]["sim_session"]
        script = """
import json, sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from carla_scenarios.definitions import load_definition
from simulation_runtime.sim_presets import PRESETS, launch_command

base = Path(get_package_share_directory("carla_scenarios")) / "config"
definition = load_definition(base, "scenarios", sys.argv[1])
groups = PRESETS[definition["stack"]]
for role in groups:
    launch_command(role, {"preset": definition["stack"], "attempt_id": "preflight", "inputs": definition["inputs"]})
print(json.dumps({"definition": definition, "groups": groups}))
"""
        result = self.run(
            [
                "docker",
                "exec",
                container,
                "bash",
                "-c",
                'source /opt/watonomous/setup.bash && exec python3 -c "$@"',
                "watod-sim-preflight",
                script,
                name,
            ]
        )
        preparation = json.loads(result.stdout)
        definition, groups = preparation["definition"], preparation["groups"]
        stack = definition["stack"]
        self.check_images(["sim_" + role for role in groups])
        try:
            for role, group in groups.items():
                service = "sim_" + role
                self.start(service, command="sleep infinity")
                packages = [
                    "simulation_runtime",
                    "wato_lifecycle_manager",
                    *group["packages"],
                ]
                checks = " && ".join(
                    "ros2 pkg prefix " + shlex.quote(package) for package in packages
                )
                checks += ' && test -x "$(ros2 pkg prefix simulation_runtime)/lib/simulation_runtime/application_runtime"'
                if stack == "controller_pure_pursuit":
                    checks += ' && test -r "$(ros2 pkg prefix --share fake_planner)/maneuvers/steady_state_slalom.json"'
                container = self.state["containers"][service]
                result = self.run(
                    [
                        "docker",
                        "exec",
                        container,
                        "bash",
                        "-c",
                        "source /opt/watonomous/setup.bash && " + checks,
                    ],
                    check=False,
                )
                if result.returncode:
                    raise SessionError(
                        f"Missing launch package or maneuver in {service}; expected packages {packages}.\n{result.stdout}{result.stderr}\nPrepare the image/assets, then retry watod sim open."
                    )
                if service == "sim_world":
                    self.run(
                        [
                            "docker",
                            "exec",
                            container,
                            "bash",
                            "-c",
                            'source /opt/watonomous/setup.bash && exec python3 -c "$@"',
                            "watod-sim-map-preflight",
                            MAP_PREFLIGHT,
                            definition["inputs"]["map"],
                        ]
                    )
        except BaseException:
            self.remove(["sim_action", "sim_world"])
            raise
        return preparation

    def open(self, kind, name):
        snapshot = self.rpc("status", timeout=15)
        if snapshot.get("state") != "IDLE":
            raise SessionError(
                f"State is {snapshot.get('state')}. Run watod sim close first."
            )
        started = time.monotonic()
        preparation = self.preflight(kind, name)
        try:
            effective_images = {}
            for service, container in self.state["containers"].items():
                info = self.owned(container)
                if info:
                    effective_images[service] = {
                        "tag": info["Config"]["Image"],
                        "id": info["Image"],
                    }
            self.rpc("runtime", json.dumps(effective_images), timeout=15)
            snapshot = self.rpc("open", f"{kind}/{name}", timeout=150)
            self.remove(["sim_action", "sim_world"])
            if kind == "scenario":
                if snapshot.get("stack") != preparation["definition"]["stack"]:
                    raise SessionError(
                        "Installed definition changed during open; close and retry."
                    )
                for role in preparation["groups"]:
                    self.start(
                        "sim_" + role,
                        command="exec ros2 run simulation_runtime application_runtime",
                    )
            snapshot = self.rpc(
                "ready",
                timeout=float(self.environment.get("WATOD_SIM_READY_TIMEOUT_S", "120")),
            )
        except BaseException:
            with contextlib.suppress(SessionError):
                self.rpc("close", timeout=30)
            self.remove(["sim_action", "sim_world"])
            raise
        snapshot["open_wall_s"] = time.monotonic() - started
        return snapshot

    def close(self):
        snapshot = self.rpc("close")
        self.remove(["sim_action", "sim_world"])
        return snapshot

    def test(self, name):
        snapshot = self.open("scenario", name)
        code = 2
        try:
            snapshot = self.rpc("play")
            remaining = float(snapshot.get("test_timeout_s", 600))
            previous = time.monotonic()
            while snapshot.get("state") != "FINISHED":
                time.sleep(0.5)
                now = time.monotonic()
                if snapshot.get("state") != "PAUSED":
                    remaining -= now - previous
                previous = now
                if remaining <= 0:
                    snapshot = self.rpc("stop")
                    raise SessionError(
                        "Bounded test wait expired; interrupted result saved. Inspect watod sim status and session logs."
                    )
                snapshot = self.rpc("status", timeout=15)
            code = int(snapshot.get("exit_code", 2))
            print(json.dumps(snapshot, indent=2))
        finally:
            self.close()
        return code

    def stop(self):
        if self.metadata.exists():
            self.load(require_running=False)
        elif not self.recover_for_stop():
            print(f"No simulation session containers found for {self.project}.")
            return
        container = self.state["containers"].get("sim_session")
        info = self.owned(container) if container else None
        if info and info["State"]["Running"]:
            # A partial/dead runner must never prevent cleanup of owned processes.
            with contextlib.suppress(SessionError):
                self.rpc("close", timeout=30)
        self.remove(reversed(list(self.state["containers"])))
        self.metadata.unlink(missing_ok=True)
        print(f"Session stopped. Results and caches preserved in {self.directory}.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="List definitions without starting Docker")
    sub.add_parser("setup", help="Start or reuse the base session; never builds images")
    opening = sub.add_parser("open")
    opening.add_argument("kind", choices=("playground", "scenario"))
    opening.add_argument("name")
    testing = sub.add_parser("test")
    testing.add_argument("name")
    for command in ("play", "pause", "resume", "status", "reset", "close", "stop"):
        sub.add_parser(command)
    arguments = parser.parse_args(
        ["--help"] if sys.argv[1:] in ([], ["help"]) else None
    )
    session = Session()
    if arguments.command == "list":
        base = session.root / "src/simulation/carla_ros_bridge/carla_scenarios/config"
        for label, folder in (
            ("Playgrounds", "playgrounds"),
            ("Scenarios", "scenarios"),
        ):
            print(label + ":")
            for definition in sorted((base / folder).glob("*.yaml")):
                print("  " + definition.stem)
        return 0
    session.directory.mkdir(parents=True, exist_ok=True)
    # Serialize transitions across terminals without retaining a background host
    # daemon. flock is released automatically even when the CLI is interrupted.
    with (session.directory / "cli.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise SessionError(
                "Another watod sim command is in progress; wait for it to finish."
            ) from error
        if arguments.command == "setup":
            session.setup()
        elif arguments.command == "stop":
            session.stop()
        else:
            session.load()
            if arguments.command == "open":
                snapshot = session.open(arguments.kind, arguments.name)
            elif arguments.command == "test":
                return session.test(arguments.name)
            elif arguments.command == "close":
                snapshot = session.close()
            else:
                snapshot = session.rpc(arguments.command)
            print(json.dumps(snapshot, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SessionError as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print(
            "Interrupted. Run watod sim status, then watod sim close to release the experiment.",
            file=sys.stderr,
        )
        sys.exit(3)
