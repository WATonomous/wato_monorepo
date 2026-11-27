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
"""Isolated bootstrap, tested with a local wheel and no package-index requests."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src/simulation/simulation_generation"))

from simulation_generation.environment import (  # noqa: E402
    ensure_environment,
    environment_identity,
)


class EnvironmentTests(unittest.TestCase):
    def test_local_wheel_installs_only_in_private_environment_and_is_reused(self):
        with tempfile.TemporaryDirectory(prefix="watod isolated test ") as temporary:
            root = Path(temporary)
            wheel = root / "watod_isolation_fixture-0.0.1-py3-none-any.whl"
            dist = "watod_isolation_fixture-0.0.1.dist-info/"
            files = {
                "watod_isolation_fixture/__init__.py": "VALUE = 42\n",
                dist
                + "METADATA": "Metadata-Version: 2.1\nName: watod-isolation-fixture\nVersion: 0.0.1\n",
                dist
                + "WHEEL": "Wheel-Version: 1.0\nGenerator: watod-test\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
            }
            files[dist + "RECORD"] = "".join(
                name + ",,\n" for name in [*files, dist + "RECORD"]
            )
            with zipfile.ZipFile(wheel, "w") as archive:
                for name, value in files.items():
                    archive.writestr(name, value)
            requirements = (
                root / "src/simulation/simulation_generation/requirements.txt"
            )
            requirements.parent.mkdir(parents=True)
            requirements.write_text(wheel.as_uri() + "\n")
            host = root / "host-packages"
            host.mkdir()
            (host / "watod_host_only.py").write_text("VALUE = 1\n")
            with patch.dict(
                os.environ,
                {
                    "PYTHONPATH": str(host),
                    "PIP_TARGET": str(root / "escaped"),
                    "PIP_USER": "1",
                },
            ):
                python = ensure_environment(root)
                result = subprocess.run(
                    [
                        str(python),
                        "-I",
                        "-c",
                        "import sys, site, json, importlib.util, watod_isolation_fixture; print(json.dumps({'prefix': sys.prefix, 'base': sys.base_prefix, 'user_site': site.ENABLE_USER_SITE, 'host_package': importlib.util.find_spec('watod_host_only') is not None, 'value': watod_isolation_fixture.VALUE}))",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
            state = json.loads(result.stdout)
            self.assertEqual(state["prefix"], str(python.parent.parent))
            self.assertNotEqual(state["prefix"], state["base"])
            self.assertFalse(state["user_site"])
            self.assertFalse(state["host_package"])
            self.assertEqual(state["value"], 42)
            self.assertFalse((root / "escaped").exists())
            self.assertIn(
                "include-system-site-packages = false",
                (python.parent.parent / "pyvenv.cfg").read_text(),
            )
            with patch("simulation_generation.environment.subprocess.run") as run:
                self.assertEqual(ensure_environment(root), python)
            run.assert_not_called()

    def test_requirements_changes_select_a_different_environment(self):
        self.assertNotEqual(environment_identity(b"one"), environment_identity(b"two"))

    def test_failed_install_is_not_marked_ready_or_replaced_by_global_pip(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            requirements = (
                root / "src/simulation/simulation_generation/requirements.txt"
            )
            requirements.parent.mkdir(parents=True)
            requirements.write_text("missing-package\n")
            with patch(
                "simulation_generation.environment.subprocess.run",
                side_effect=[None, subprocess.CalledProcessError(1, ["pip"])],
            ) as run:
                with self.assertRaises(subprocess.CalledProcessError):
                    ensure_environment(root)
            installer = run.call_args_list[-1].args[0]
            self.assertTrue(
                Path(installer[0]).is_relative_to(root / ".cache/watod-sim/generation")
            )
            self.assertIn("--require-virtualenv", installer)
            self.assertFalse(
                list(
                    (root / ".cache/watod-sim/generation").glob(
                        "env-*/requirements.sha256"
                    )
                )
            )


if __name__ == "__main__":
    unittest.main()
