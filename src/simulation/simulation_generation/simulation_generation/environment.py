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
"""Prepare and run the host generator in a checkout-local Python environment."""

import fcntl
import hashlib
import os
import platform
import subprocess
import sys
from pathlib import Path

ENTRYPOINT = (
    "import sys; sys.path.insert(0, sys.argv.pop(1)); "
    "from simulation_generation.cli import main; sys.exit(main())"
)


def environment_identity(requirements):
    identity = b"\0".join(
        (
            requirements,
            sys.version.encode(),
            str(Path(sys.base_prefix).resolve()).encode(),
            sys.platform.encode(),
            platform.machine().encode(),
        )
    )
    return hashlib.sha256(identity).hexdigest()


def ensure_environment(root):
    root = Path(root).resolve()
    requirements = root / "src/simulation/simulation_generation/requirements.txt"
    cache = root / ".cache/watod-sim/generation"
    cache.mkdir(parents=True, exist_ok=True)
    # Serialize installation separately from running generation or session commands.
    with (cache / "environment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        identity = environment_identity(requirements.read_bytes())
        environment = cache / ("env-" + identity[:16])
        python = environment / "bin/python"
        marker = environment / "requirements.sha256"
        if (
            python.is_file()
            and marker.is_file()
            and marker.read_text().strip() == identity
        ):
            return python
        print(
            "Preparing isolated scenario-generator environment…",
            file=sys.stderr,
            flush=True,
        )
        setup_environment = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith(("PIP_", "PYTHON")) and key != "OPENAI_API_KEY"
        }
        # Disable pip configuration files as well as environment/user settings.
        setup_environment["PIP_CONFIG_FILE"] = os.devnull
        subprocess.run(
            [sys.executable, "-I", "-m", "venv", str(environment)],
            check=True,
            env=setup_environment,
        )
        subprocess.run(
            [
                str(python),
                "-I",
                "-m",
                "pip",
                "--isolated",
                "install",
                "--require-virtualenv",
                "--disable-pip-version-check",
                "--no-input",
                "--only-binary=:all:",
                "--cache-dir",
                str(cache / "pip-cache"),
                "--requirement",
                str(requirements),
            ],
            check=True,
            env=setup_environment,
        )
        # Failed or interrupted installs never become reusable environments.
        marker.write_text(identity + "\n")
        return python


def run_isolated(root, arguments):
    try:
        python = ensure_environment(root)
        environment = dict(os.environ)
        environment.pop("PYTHONHOME", None)
        environment.pop("PYTHONPATH", None)
        environment["VIRTUAL_ENV"] = str(python.parent.parent)
        return subprocess.run(
            [
                str(python),
                "-I",
                "-c",
                ENTRYPOINT,
                str(Path(root) / "src/simulation/simulation_generation"),
                *arguments,
            ],
            env=environment,
            check=False,
        ).returncode
    except (OSError, subprocess.CalledProcessError):
        print(
            "Unable to prepare or run the isolated generator. Check the error above, "
            "Python venv support, and package availability, then retry watod sim generate.",
            file=sys.stderr,
        )
        return 2
