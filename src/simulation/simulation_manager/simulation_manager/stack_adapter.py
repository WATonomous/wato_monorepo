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
"""Coordinate reviewed application launches through their owning runtimes."""

import json
import time

from carla_msgs.srv import BindControl
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from wato_lifecycle_manager.sim_presets import PRESETS, command_topic, launch_command


class SimulationStackAdapter:
    """Own startup ordering and process replacement, without transitioning nodes."""

    def __init__(self, call, session_id):
        self.call = call
        self.session_id = session_id
        self.roles = []
        self.preset = None
        self.command_topic = None
        self.prepared = False

    def _call(self, *args, timeout=15.0, deadline=None):
        if deadline is not None:
            timeout = min(timeout, deadline - time.monotonic())
            if timeout <= 0:
                raise RuntimeError("Application stack readiness deadline expired")
        return self.call(*args, timeout=timeout)

    def _runtime(self, role, operation, request=None, timeout=15.0, deadline=None):
        endpoint = f"/sim/runtime/{role}"
        status = self._call(
            Trigger, endpoint + "/status", timeout=3.0, deadline=deadline
        )
        identity = json.loads(status.message)
        if identity["session_id"] != self.session_id or identity["role"] != role:
            raise RuntimeError(f"Runtime ownership mismatch for {role}")
        if operation == "status":
            return identity
        return self._call(
            Trigger,
            endpoint + "/" + operation,
            request,
            timeout=timeout,
            deadline=deadline,
        )

    def prepare(self, definition, attempt_id, timeout_s=60.0):
        """Start the world-model group before action, then use their lifecycle owners."""
        if self.roles:
            raise RuntimeError(
                "Stop the existing application stack before preparing another"
            )
        deadline = time.monotonic() + timeout_s
        preset = definition["stack"]
        configuration = {
            "session_id": self.session_id,
            "preset": preset,
            "attempt_id": attempt_id,
            "inputs": definition["inputs"],
        }
        # Validate both groups before starting either one.
        for role in PRESETS[preset]:
            launch_command(role, configuration)
        self.preset = preset
        self.command_topic = command_topic(attempt_id)
        self._call(
            BindControl,
            "/carla/ackermann_control/bind_control",
            BindControl.Request(command_topic=self.command_topic),
            deadline=deadline,
        )
        try:
            for role, group in PRESETS[preset].items():
                status = self._runtime(role, "status", deadline=deadline)
                if status["process"] != "stopped":
                    raise RuntimeError(
                        f"Runtime {role} already owns a launch group; close the experiment first"
                    )
                request = SetParametersAtomically.Request(
                    parameters=[
                        Parameter(
                            name="configuration",
                            value=ParameterValue(
                                type=ParameterType.PARAMETER_STRING,
                                string_value=json.dumps(configuration),
                            ),
                        ),
                    ]
                )
                result = self._call(
                    SetParametersAtomically,
                    f"/sim/runtime/{role}/application_runtime/set_parameters_atomically",
                    request,
                    deadline=deadline,
                )
                if not result.result.successful:
                    raise RuntimeError(f"Runtime {role}: {result.result.reason}")
                # Track ownership before a start response can time out.
                self.roles.append(role)
                self._runtime(role, "start", deadline=deadline)
                self._call(
                    Trigger,
                    group["manager"] + "/startup",
                    timeout=60.0,
                    deadline=deadline,
                )
            self.prepared = True
        except Exception as start_error:
            try:
                self.stop()
            except Exception as stop_error:
                raise RuntimeError(
                    f"{start_error}; cleanup: {stop_error}"
                ) from start_error
            raise

    def pending(self):
        """Report runtime or lifecycle readiness without changing lifecycle state."""
        if not self.prepared:
            return "application stack has not been prepared"
        try:
            for role in self.roles:
                if self._runtime(role, "status")["process"] != "running":
                    return f"application launch group {role} is not running"
                manager = PRESETS[self.preset][role]["manager"]
                self.call(Trigger, manager + "/is_active", timeout=3.0)
        except RuntimeError as error:
            return str(error)
        return None

    def stop(self):
        """Stop owned launch groups; preserve failed groups for a close retry."""
        errors = []
        for role in reversed(self.roles[:]):
            try:
                self._runtime(role, "stop")
                self.roles.remove(role)
            except Exception as error:
                errors.append(str(error))
        self.prepared = False
        if errors:
            raise RuntimeError("; ".join(errors))
