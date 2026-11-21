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
"""Loading config/calib.yaml and resolving repo-relative paths."""

from __future__ import annotations

from pathlib import Path

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_DIR / "config" / "calib.yaml"


def repo_root() -> Path:
    """The monorepo root (…/src/interfacing/calibration/lidar_cam_calib -> repo)."""
    return PACKAGE_DIR.parents[3]


def load_config(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_CONFIG) as f:
        return yaml.safe_load(f)


def resolve(path: str | Path) -> Path:
    """Absolute paths stay as they are; relative ones are relative to the repo root."""
    p = Path(path)
    return p if p.is_absolute() else repo_root() / p


def image_topic(cfg: dict, camera: str) -> str:
    return cfg["camera_image_topic"].format(camera=camera)


def info_topic(cfg: dict, camera: str) -> str:
    return cfg["camera_info_topic"].format(camera=camera)
