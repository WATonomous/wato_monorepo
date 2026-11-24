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

import os
import signal
import subprocess
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.validate_full_topic_name import validate_full_topic_name
from simulation_runtime.application_runtime import (
    ApplicationRuntime,
    LaunchProcess,
    main,
)
from simulation_runtime.sim_presets import command_topic, launch_command


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

    def test_attempt_topics_are_valid_and_do_not_alias_raw_ids(self):
        timestamp = "20261124T012353_3521a8663d94"
        mappings = {
            timestamp: f"_attempt_{timestamp}",
            "attempt_2": "attempt_2",
            "old": "old",
            "new": "new",
            "_private": "_private",
            "2": "_attempt_2",
            f"attempt_{timestamp}": f"attempt_{timestamp}",
            f"_attempt_{timestamp}": f"_attempt__attempt_{timestamp}",
            "_attempt__attempt_2": "_attempt__attempt__attempt_2",
            "_attempt_2": "_attempt__attempt_2",
        }
        topics = []
        for attempt, token in mappings.items():
            with self.subTest(attempt=attempt):
                topic = command_topic(attempt)
                self.assertEqual(topic, f"/sim/attempts/{token}/command")
                validate_full_topic_name(topic)
                topics.append(topic)
        self.assertEqual(len(set(topics)), len(mappings))

    def test_both_presets_remap_numeric_attempt_topics(self):
        expected = "/sim/attempts/_attempt_20261124T012353_3521a8663d94/command"
        for preset, inputs, argument in (
            (
                "controller_pure_pursuit",
                {"maneuver": "steady_state_slalom"},
                "ackermann_topic",
            ),
            (
                "autonomy",
                {"max_speed_mps": 3.0, "stop_distance_m": 2.0},
                "command_topic",
            ),
        ):
            with self.subTest(preset=preset):
                command = launch_command(
                    "action",
                    {
                        "preset": preset,
                        "attempt_id": "20261124T012353_3521a8663d94",
                        "inputs": inputs,
                    },
                )
                self.assertIn(f"{argument}:={expected}", command)

    def test_malformed_and_overlong_ids_are_rejected(self):
        for attempt in (
            None,
            123,
            "",
            "../old",
            "a/b",
            "a-b",
            "a b",
            "a\n",
            "é",
            "a" * 226,
        ):
            with self.subTest(attempt=attempt):
                with self.assertRaises(ValueError):
                    command_topic(attempt)
        # Include the normalized prefix in the limit; do not truncate IDs.
        for attempt in ("a" * 225, "1" * 216):
            validate_full_topic_name(command_topic(attempt))
        with self.assertRaises(ValueError):
            command_topic("1" * 217)

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


class TestRuntimeShutdown(unittest.TestCase):
    def test_already_shutdown_ros_context_exits_cleanly(self):
        def interrupted_spin(node):
            # Jazzy's SIGINT handler shuts down the context before spin exits.
            rclpy.shutdown()
            raise KeyboardInterrupt

        with (
            patch.dict(
                os.environ,
                {"WATOD_SIM_RUNTIME_ROLE": "action", "WATOD_SIM_SESSION_ID": "test"},
            ),
            patch(
                "simulation_runtime.application_runtime.rclpy.spin", interrupted_spin
            ),
        ):
            main(args=[])
        self.assertFalse(rclpy.ok())
