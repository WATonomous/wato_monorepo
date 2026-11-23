# Copyright (c) 2026-present WATonomous. All rights reserved.
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
"""Host cleanup regressions using inert Docker inspection records."""

import importlib.util
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location(
    "watod_sim", Path(__file__).resolve().parents[1] / "watod-sim.py"
)
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def session():
    node = HELPER.Session.__new__(HELPER.Session)
    node.root = Path(__file__).resolve().parents[2]
    node.results = node.root / ".cache/watod-sim/results"
    node.directory = node.results.parent
    node.project = "watod_test_sim_workspace"
    node.environment = {}
    node.verbose = False
    node.state = None
    node.save = Mock()
    node.metadata = Mock()
    node.metadata.exists.return_value = False
    node.run = Mock()
    node.inspect = Mock()
    node.rpc = Mock()
    return node


def container(node, service, identity=None):
    identity = identity or service + "-id"
    return {
        "Id": identity,
        "Name": "/" + service,
        "State": {"Running": True},
        "Config": {
            "Image": "test-image",
            "Env": ["WATOD_SIM_SESSION_ID=session-id", "WATOD_SIM_REVISION=revision"],
            "Labels": {
                "com.docker.compose.project": node.project,
                "com.docker.compose.service": service,
                "com.docker.compose.project.working_dir": str(node.root / "modules"),
                "com.docker.compose.project.config_files": ",".join(
                    str(node.root / "modules" / name)
                    for name in (
                        "docker-compose.yaml",
                        "docker-compose.watcloud.yaml",
                        "docker-compose.dep.yaml",
                    )
                ),
                "com.docker.compose.oneoff": "False",
            },
        },
        "Mounts": [
            {
                "Type": "bind",
                "Source": str(node.results),
                "Destination": "/opt/watonomous/sim_results",
            }
        ],
        "HostConfig": {"NetworkMode": "host"},
    }


class TestOrphanRecovery(unittest.TestCase):
    def test_recovers_only_cleanup_state_in_reverse_dependency_order(self):
        node = session()
        names = (
            "sim_session",
            "infrastructure_bringup",
            "carla_sim",
            "network_namespace",
        )
        records = {name: container(node, name) for name in names}
        node.run.return_value = SimpleNamespace(stdout="\n".join(names))
        node.inspect.side_effect = records.get

        self.assertTrue(node.recover_for_stop())

        self.assertEqual(
            list(node.state["containers"]),
            ["network_namespace", "carla_sim", "infrastructure_bringup", "sim_session"],
        )
        self.assertTrue(node.state["recovered_for_stop"])
        self.assertEqual(node.state["session_id"], "session-id")
        node.save.assert_called_once()
        node.rpc.assert_not_called()
        self.assertEqual(node.run.call_count, 1)

    def test_foreign_or_unexpected_containers_are_rejected_before_saving(self):
        for label, value in (
            ("com.docker.compose.project", "another-project"),
            ("com.docker.compose.project.working_dir", "/another/workspace/modules"),
            ("com.docker.compose.project.config_files", "/another/compose.yaml"),
            ("com.docker.compose.service", "perception_bringup"),
            ("com.docker.compose.oneoff", "True"),
        ):
            with self.subTest(label=label):
                node = session()
                info = container(node, "sim_session")
                info["Config"]["Labels"][label] = value
                node.run.return_value = SimpleNamespace(stdout="candidate")
                node.inspect.return_value = info

                with self.assertRaises(HELPER.SessionError):
                    node.recover_for_stop()

                node.save.assert_not_called()
                node.rpc.assert_not_called()

    def test_results_from_a_different_workspace_are_rejected(self):
        node = session()
        info = container(node, "sim_session")
        info["Mounts"][0]["Source"] = "/another/results"
        node.run.return_value = SimpleNamespace(stdout="candidate")
        node.inspect.return_value = info

        with self.assertRaisesRegex(HELPER.SessionError, "Results mount"):
            node.recover_for_stop()

        node.save.assert_not_called()

    def test_duplicate_services_are_rejected_before_saving(self):
        node = session()
        node.run.return_value = SimpleNamespace(stdout="first\nsecond")
        node.inspect.return_value = container(node, "sim_session")

        with self.assertRaises(HELPER.SessionError):
            node.recover_for_stop()

        node.save.assert_not_called()

    def test_recovered_state_cannot_be_reused_as_a_running_session(self):
        node = session()
        node.run.return_value = SimpleNamespace(stdout="candidate")
        node.inspect.return_value = container(node, "sim_session")
        node.recover_for_stop()
        node.metadata.exists.return_value = True
        node.metadata.read_text.return_value = json.dumps(node.state)

        with self.assertRaisesRegex(HELPER.SessionError, "cleanup only"):
            node.load()

    def test_empty_stop_is_idempotent(self):
        node = session()
        node.run.return_value = SimpleNamespace(stdout="")

        node.stop()

        node.save.assert_not_called()
        node.rpc.assert_not_called()
        node.metadata.unlink.assert_not_called()


class TestScopedRemoval(unittest.TestCase):
    def test_failed_graceful_stop_removes_only_verified_id(self):
        for failure in (SimpleNamespace(returncode=1), HELPER.SessionError("timeout")):
            with self.subTest(failure=failure):
                node = session()
                node.state = {"containers": {"sim_session": "owned-id"}}
                node.inspect.return_value = container(node, "sim_session", "owned-id")
                node.run.side_effect = [failure, SimpleNamespace(returncode=0)]

                node.remove(["sim_session"])

                self.assertEqual(
                    node.run.call_args_list[-1].args[0],
                    ["docker", "rm", "--force", "owned-id"],
                )
                self.assertEqual(node.inspect.call_count, 2)
                self.assertEqual(node.state["containers"], {})

    def test_foreign_id_is_never_stopped(self):
        node = session()
        node.state = {"containers": {"sim_session": "foreign-id"}}
        info = container(node, "sim_session", "foreign-id")
        info["Config"]["Labels"]["com.docker.compose.project"] = "another-project"
        node.inspect.return_value = info

        with self.assertRaises(HELPER.SessionError):
            node.remove(["sim_session"])

        node.run.assert_not_called()

    def test_shared_namespace_is_preserved_for_foreign_consumers(self):
        node = session()
        namespace = container(node, "network_namespace", "network-id")
        consumer = container(node, "perception_bringup", "foreign-id")
        consumer["HostConfig"]["NetworkMode"] = "container:network-id"
        consumer["Config"]["Labels"]["com.docker.compose.project"] = "another-project"
        node.state = {"containers": {"network_namespace": "network-id"}}
        node.inspect.side_effect = {"network-id": namespace, "foreign-id": consumer}.get
        node.run.return_value = SimpleNamespace(stdout="network-id\nforeign-id")

        node.remove(["network_namespace"])

        node.run.assert_called_once_with(["docker", "ps", "--all", "--quiet"])

    def test_orphan_stop_closes_best_effort_then_removes_in_order(self):
        node = session()
        node.state = {"containers": {"carla_sim": "car-id", "sim_session": "sim-id"}}
        node.inspect.return_value = container(node, "sim_session", "sim-id")
        node.rpc.side_effect = HELPER.SessionError("expired SLURM cgroup")
        with (
            patch.object(node, "recover_for_stop", return_value=True),
            patch.object(node, "remove") as remove,
        ):
            node.stop()

        node.rpc.assert_called_once_with("close", timeout=30)
        self.assertEqual(list(remove.call_args.args[0]), ["sim_session", "carla_sim"])
        node.metadata.unlink.assert_called_once_with(missing_ok=True)


class TestRuntimeHandoff(unittest.TestCase):
    def test_open_starts_runtimes_and_leaves_launches_to_ros_manager(self):
        node = session()
        node.state = {"containers": {"sim_session": "session-container"}}
        node.owned = Mock(return_value={"Config": {"Image": "test"}, "Image": "id"})
        node.preflight = Mock(
            return_value={
                "definition": {"stack": "autonomy"},
                "groups": {"world": {}, "action": {}},
            }
        )
        node.rpc.side_effect = [
            {"state": "IDLE"},
            {},
            {"state": "PREPARING", "stack": "autonomy"},
            {"state": "READY"},
        ]
        node.start, node.remove = Mock(), Mock()

        snapshot = node.open("scenario", "parked_car_straight")

        self.assertEqual(snapshot["state"], "READY")
        self.assertEqual(
            [call.args[0] for call in node.start.call_args_list],
            ["sim_world", "sim_action"],
        )
        for call in node.start.call_args_list:
            self.assertEqual(
                call.kwargs["command"],
                "exec ros2 run wato_lifecycle_manager application_runtime",
            )
        self.assertEqual(
            [call.args[0] for call in node.rpc.call_args_list],
            ["status", "runtime", "open", "ready"],
        )

    def test_failed_readiness_closes_ros_groups_before_removing_containers(self):
        node = session()
        node.state = {"containers": {}}
        node.preflight = Mock(
            return_value={"definition": {"stack": "autonomy"}, "groups": {"action": {}}}
        )
        node.start, node.remove = Mock(), Mock()
        node.rpc.side_effect = [
            {"state": "IDLE"},
            {},
            {"stack": "autonomy"},
            HELPER.SessionError("readiness failed"),
            {},
        ]
        calls = Mock()
        calls.attach_mock(node.rpc, "rpc")
        calls.attach_mock(node.remove, "remove")

        with self.assertRaisesRegex(HELPER.SessionError, "readiness failed"):
            node.open("scenario", "parked_car_straight")

        self.assertEqual(calls.mock_calls[-2].args[0], "close")
        self.assertEqual(calls.mock_calls[-1].args[0], ["sim_action", "sim_world"])

    def test_preflight_uses_installed_preset_packages(self):
        node = session()
        node.state = {"containers": {"sim_session": "base"}}
        preparation = {
            "definition": {"stack": "controller_pure_pursuit"},
            "groups": {"action": {"packages": ["reviewed_package"]}},
        }
        node.run.side_effect = [
            SimpleNamespace(stdout=json.dumps(preparation)),
            SimpleNamespace(returncode=0),
        ]

        def start(service, **kwargs):
            node.state["containers"][service] = service + "-id"

        node.start, node.check_images = Mock(side_effect=start), Mock()

        result = node.preflight("scenario", "controller_snake")

        self.assertEqual(result, preparation)
        node.check_images.assert_called_once_with(["sim_action"])
        checks = node.run.call_args.args[0][-1]
        self.assertIn("ros2 pkg prefix reviewed_package", checks)
        self.assertIn("wato_lifecycle_manager/application_runtime", checks)
