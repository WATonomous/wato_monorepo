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
"""Runtime ownership, launch validation and process cleanup without containers."""

import signal
import subprocess
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from wato_lifecycle_manager.application_runtime import ApplicationRuntime, LaunchProcess
from wato_lifecycle_manager.sim_presets import launch_command


class TestLaunchProcess(unittest.TestCase):
    def test_stop_targets_owned_group_and_escalates(self):
        launch = LaunchProcess()
        with patch("subprocess.Popen") as popen, patch("os.killpg") as kill:
            process = popen.return_value
            process.pid = 123
            process.wait.side_effect = [subprocess.TimeoutExpired("launch", 5), 0, 0]
            launch.start(["ros2", "launch", "reviewed", "stack.launch.yaml"])
            popen.assert_called_once_with(
                ["ros2", "launch", "reviewed", "stack.launch.yaml"],
                start_new_session=True,
            )
            with self.assertRaises(RuntimeError):
                launch.start(["another"])
            launch.stop()
            self.assertEqual(
                [c.args for c in kill.call_args_list],
                [(123, signal.SIGINT), (123, signal.SIGTERM), (123, signal.SIGKILL)],
            )
            self.assertIsNone(launch.process)
            launch.stop()
            self.assertEqual(popen.call_count, 1)

    def test_exited_parent_does_not_skip_descendant_cleanup(self):
        launch = LaunchProcess()
        launch.process = MagicMock(pid=456)
        launch.process.poll.return_value = 1
        self.assertEqual(launch.status(), "exited (1)")
        with patch("os.killpg") as kill:
            launch.stop()
        self.assertEqual(kill.call_args_list[0].args, (456, signal.SIGINT))


class TestReviewedLaunches(unittest.TestCase):
    def test_controller_is_remapped_without_node_changes(self):
        command = launch_command(
            "action",
            {
                "preset": "controller_pure_pursuit",
                "attempt_id": "attempt_2",
                "inputs": {"maneuver": "steady_state_slalom"},
            },
        )
        self.assertIn("autostart:=false", command)
        self.assertIn("ackermann_topic:=/sim/attempts/attempt_2/command", command)

    def test_rejects_unreviewed_launch_inputs(self):
        configuration = {
            "preset": "autonomy",
            "attempt_id": "attempt_2",
            "inputs": {"map": "oval", "max_speed_mps": 3.0, "stop_distance_m": 2.0},
        }
        for field, value, role in (
            ("attempt_id", "../old", "action"),
            ("preset", "arbitrary", "action"),
            ("map", "$(command)", "world"),
            ("max_speed_mps", float("nan"), "action"),
        ):
            changed = {**configuration, "inputs": dict(configuration["inputs"])}
            (changed if field in configuration else changed["inputs"])[field] = value
            with self.assertRaises(ValueError):
                launch_command(role, changed)

    def test_runtime_rejects_other_session_without_launching(self):
        node = SimpleNamespace(
            role="action",
            session_id="owner",
            launch=MagicMock(),
            get_parameter=lambda _: SimpleNamespace(value='{"session_id": "other"}'),
        )
        response = ApplicationRuntime._start(node, None, SimpleNamespace())
        self.assertFalse(response.success)
        node.launch.start.assert_not_called()
