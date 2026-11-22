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
images, starts their declared stack and forwards the operator's ROS requests.
All mutable files live under this workspace's ignored .cache/watod-sim folder.
"""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import uuid


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

PRESETS = {
    "controller_pure_pursuit": {
        "sim_action": "ros2 launch fake_planner fake_planner_sim.launch.yaml "
        "frame_id:=map start_on_activate:=false",
    },
    "autonomy": {
        "sim_action": "ros2 launch action_bringup action_sim.launch.yaml "
        "stop_distance:=2.0 max_speed:=3.0",
        "sim_world": "ros2 launch world_modeling_bringup world_modeling_sim.launch.yaml "
        "use_eidos:=false map:=oval enable_overtaking:=false",
    },
}


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

    def run(self, arguments, *, timeout=60, capture=True, check=True):
        try:
            result = subprocess.run(
                arguments,
                cwd=self.root,
                env=self.environment,
                text=True,
                stdout=subprocess.PIPE if capture else None,
                stderr=subprocess.PIPE if capture else None,
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

    def load(self, *, require_running=True):
        if not self.metadata.exists():
            raise SessionError("Run watod sim setup first.")
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
            "up", "--no-start", "--no-build", "--pull", "never", "--no-deps", service
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
            if self.owned(container):
                self.run(["docker", "stop", "--time", "15", container], timeout=25)
                self.run(["docker", "rm", container])
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
                f"Unrecorded containers already use Compose project {self.project}; inspect their ownership before setup."
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
        except BaseException:
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
import yaml
from ament_index_python.packages import get_package_share_directory

base = Path(get_package_share_directory("carla_scenarios")) / "config"
definition = yaml.safe_load((base / "scenarios" / (sys.argv[1] + ".yaml")).read_text())
print(json.dumps({"stack": definition["stack"]}))
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
        stack = json.loads(result.stdout)["stack"]
        if stack not in PRESETS:
            raise SessionError(f"Unsupported stack preset {stack!r} in {path}.")
        self.check_images(list(PRESETS[stack]))
        try:
            for service in PRESETS[stack]:
                self.start(service, command="sleep infinity")
                packages = (
                    ["fake_planner", "ackermann_pure_pursuit"]
                    if stack == "controller_pure_pursuit"
                    else (
                        [
                            "action_bringup",
                            "lattice_planning",
                            "trajectory_planner",
                            "ackermann_pure_pursuit",
                        ]
                        if service == "sim_action"
                        else [
                            "world_modeling_bringup",
                            "world_model",
                            "costmap",
                            "behaviour",
                            "prediction",
                        ]
                    )
                )
                checks = " && ".join(
                    "ros2 pkg prefix " + package for package in packages
                )
                if service == "sim_world":
                    checks += " && test -r /opt/watonomous/maps/osm/oval.osm"
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
                        f"Missing launch package, maneuver or map in {service}; expected packages {packages}, map /opt/watonomous/maps/osm/oval.osm for autonomy.\n{result.stdout}{result.stderr}\nPrepare the image/assets, then retry watod sim open."
                    )
        except BaseException:
            self.remove(["sim_action", "sim_world"])
            raise
        return stack

    def open(self, kind, name):
        snapshot = self.rpc("status", timeout=15)
        if snapshot.get("state") != "IDLE":
            raise SessionError(
                f"State is {snapshot.get('state')}. Run watod sim close first."
            )
        started = time.monotonic()
        stack = self.preflight(kind, name)
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
                if snapshot.get("stack") != stack:
                    raise SessionError(
                        "Installed definition changed during open; close and retry."
                    )
                for service, command in PRESETS[stack].items():
                    self.start(service, command=command)
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
        self.load(require_running=False)
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
        ["--help"] if sys.argv[1:] == ["help"] or not sys.argv[1:] else None
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
