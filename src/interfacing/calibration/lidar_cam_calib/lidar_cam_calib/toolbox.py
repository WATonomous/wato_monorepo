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
"""Running direct_visual_lidar_calibration tools and editing their calib.json."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import yaml

PKG = "direct_visual_lidar_calibration"
CALIB = "calib.json"


def headless_prefix(mode: str) -> list[str]:
    """xvfb-run wrapper for tools that open an OpenGL window, when there is no display."""
    if mode == "never" or (mode == "auto" and os.environ.get("DISPLAY")):
        return []
    if shutil.which("xvfb-run"):
        return ["xvfb-run", "-a", "-s", "-screen 0 1280x1024x24"]
    return []


def run_tool(
    tool: str,
    args: list[str],
    log: Path,
    headless: str,
    dry_run: bool = False,
    timeout: float | None = None,
) -> int:
    cmd = headless_prefix(headless) + ["ros2", "run", PKG, tool, *args]
    print("  $ " + " ".join(shlex.quote(c) for c in cmd), flush=True)
    if dry_run:
        return 0
    log.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "w") as f:
        try:
            return subprocess.run(
                cmd, stdout=f, stderr=subprocess.STDOUT, timeout=timeout
            ).returncode
        except subprocess.TimeoutExpired:
            f.write(f"\n[lidar_cam_calib] timed out after {timeout} s\n")
            return 124


def rectified_intrinsics(camera_yaml: Path) -> list[float]:
    """fx, fy, cx, cy of the rectified image: the P matrix of a camera_calib YAML."""
    with open(camera_yaml) as f:
        p = yaml.safe_load(f)["projection_matrix"]["data"]
    return [float(p[0]), float(p[5]), float(p[2]), float(p[6])]


def read_calib(d: Path) -> dict:
    with open(d / CALIB) as f:
        return json.load(f)


def write_calib(d: Path, data: dict) -> None:
    with open(d / CALIB, "w") as f:
        json.dump(data, f, indent=2)


def make_trial(
    base: Path, trial: Path, bag_names: list[str], init_vec7: list[float]
) -> None:
    """A trial folder: links to base's preprocessed files, its own calib.json.

    calibrate reads meta.bag_names and results.init_T_lidar_camera from
    calib.json and writes results.T_lidar_camera back into it.
    """
    if trial.exists():
        shutil.rmtree(trial)
    trial.mkdir(parents=True)
    for f in base.iterdir():
        if f.name != CALIB and f.is_file():
            (trial / f.name).symlink_to(os.path.relpath(f, trial))
    calib = read_calib(base)
    calib["meta"]["bag_names"] = list(bag_names)
    results = calib.setdefault("results", {})
    for k in ("T_lidar_camera", "init_T_lidar_camera_auto"):
        results.pop(k, None)
    results["init_T_lidar_camera"] = list(init_vec7)
    write_calib(trial, calib)


def trial_result(trial: Path) -> list[float] | None:
    try:
        return read_calib(trial).get("results", {}).get("T_lidar_camera")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
