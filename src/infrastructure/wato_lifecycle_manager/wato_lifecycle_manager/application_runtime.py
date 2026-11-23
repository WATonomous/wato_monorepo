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
"""Own one simulation launch process group inside an application container."""

import json
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from wato_lifecycle_manager.sim_presets import launch_command


class LaunchProcess:
    """Only this runtime's process group can be stopped; containers stay running."""

    def __init__(self):
        self.process = None

    def start(self, command):
        if self.process is not None:
            raise RuntimeError(
                "Stop the existing application group before starting another"
            )
        self.process = subprocess.Popen(command, start_new_session=True)

    def status(self):
        if self.process is None:
            return "stopped"
        code = self.process.poll()
        return "running" if code is None else f"exited ({code})"

    def stop(self):
        if self.process is None:
            return
        # ros2 launch and its descendants inherit the new session/process group.
        for sig, timeout in (
            (signal.SIGINT, 5),
            (signal.SIGTERM, 3),
            (signal.SIGKILL, 2),
        ):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            # A launch parent can exit before its children. Signal the entire group.
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            break
        self.process.wait(timeout=2)
        self.process = None


class ApplicationRuntime(Node):
    """A small ROS interface for starting and stopping reviewed launch groups."""

    def __init__(self):
        self.role = os.environ["WATOD_SIM_RUNTIME_ROLE"]
        if self.role not in {"action", "world"}:
            raise ValueError("Runtime role must be action or world")
        super().__init__("application_runtime", namespace=f"/sim/runtime/{self.role}")
        self.session_id = os.environ["WATOD_SIM_SESSION_ID"]
        self.declare_parameter("configuration", "{}")
        self.configuration = {}
        self.launch = LaunchProcess()
        for command in ("start", "stop", "status"):
            self.create_service(Trigger, command, getattr(self, f"_{command}"))

    def _snapshot(self):
        return json.dumps(
            {
                "session_id": self.session_id,
                "role": self.role,
                "process": self.launch.status(),
                "attempt_id": self.configuration.get("attempt_id"),
            }
        )

    def _start(self, request, response):
        try:
            configuration = json.loads(self.get_parameter("configuration").value)
            if configuration.get("session_id") != self.session_id:
                raise ValueError("Session ownership mismatch")
            command = launch_command(self.role, configuration)
            self.launch.start(command)
            self.configuration = configuration
            response.success, response.message = True, self._snapshot()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def _stop(self, request, response):
        try:
            self.launch.stop()
            self.configuration = {}
            response.success, response.message = True, self._snapshot()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def _status(self, request, response):
        response.success, response.message = True, self._snapshot()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ApplicationRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.launch.stop()
        node.destroy_node()
        rclpy.shutdown()
