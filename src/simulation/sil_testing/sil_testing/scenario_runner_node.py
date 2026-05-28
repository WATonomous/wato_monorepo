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
"""Orchestrates a fixed-duration SIL run on simulation time.

Waits for /clock to start, then runs for the scenario's ``duration_s`` of simulation
time and exits. The launch file ties the recorder and the rest of the SIL stack to this
node's lifetime (on_exit -> Shutdown), so its exit cleanly stops recording.
"""

import os

import rclpy
from rclpy.node import Node

from sil_testing.sil_scenario_schema import load_scenario


class ScenarioRunnerNode(Node):
    def __init__(self):
        super().__init__("scenario_runner_node")
        self.declare_parameter("scenario_file", os.environ.get("SIL_SCENARIO_FILE", ""))
        self.declare_parameter("startup_grace_s", 3.0)

        scenario_file = self.get_parameter("scenario_file").value
        if not scenario_file:
            raise RuntimeError("scenario_file/SIL_SCENARIO_FILE not provided")
        scenario = load_scenario(scenario_file)

        self.duration_s = scenario.duration_s
        self.startup_grace_s = self.get_parameter("startup_grace_s").value
        self.start_time = None
        self.finished = False

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"SIL run '{scenario.name}' will record for {self.duration_s:.1f}s of sim time"
        )

    def _tick(self):
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # Wait for sim clock to actually start advancing before timing the run.
        if now <= 0.0:
            return
        if self.start_time is None:
            self.start_time = now + self.startup_grace_s
            return
        if now < self.start_time:
            return

        elapsed = now - self.start_time
        if elapsed >= self.duration_s:
            self.finished = True
            self.get_logger().info(
                f"SIL run complete after {elapsed:.1f}s of sim time; shutting down"
            )
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunnerNode()
    try:
        # Flag-driven loop so process exit is reliable across rclpy versions; the launch
        # ties recorder/stack shutdown to this process exiting.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
