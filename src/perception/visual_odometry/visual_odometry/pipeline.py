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
"""Frame set -> pose, shared by the live node and the offline runner (no ROS types needed)."""

import os
from typing import Callable, Dict, List, Mapping, Optional

import numpy as np
import yaml

from visual_odometry.geometry import make_transform, split_transform
from visual_odometry.grouping import FrameSet
from visual_odometry.rectify import RigPreprocessor
from visual_odometry.vo_core import CameraSpec, TrackResult, VoCore

# (target, source) -> T_target_source, like tf2
Lookup = Callable[[str, str], np.ndarray]


def parse_pairs(pairs) -> List[List[str]]:
    """Stereo pairs from params: ["cam_a:cam_b", ...] (ROS-param friendly) or [[cam_a, cam_b], ...]."""
    out = []
    for pair in pairs or []:
        names = pair.split(":") if isinstance(pair, str) else list(pair)
        if len(names) != 2 or not all(names):
            raise ValueError(f"stereo pair {pair!r} must be 'cam_a:cam_b'")
        out.append([n.strip() for n in names])
    return out


def load_extrinsics(path: str, rig_frame: str) -> Dict[str, np.ndarray]:
    """Read a calibrate_pairs output file: {camera: rig_from_camera 4x4}. Empty path -> {}."""
    if not path:
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if data.get("rig_frame", rig_frame) != rig_frame:
        raise ValueError(
            f"{path} is for rig frame {data.get('rig_frame')!r}, not {rig_frame!r}"
        )
    out = {}
    for cam, entry in (data.get("cameras") or {}).items():
        pose = entry["rig_from_camera"]
        out[cam] = make_transform(pose["translation"], pose["rotation_xyzw"])
    return out


def dump_extrinsics(
    path: str,
    rig_frame: str,
    cameras: Mapping[str, np.ndarray],
    header: str,
    info: Mapping,
) -> None:
    """Write corrected extrinsics (and fit statistics) in the format load_extrinsics reads."""
    doc = {"rig_frame": rig_frame, "cameras": {}}
    for cam, transform in cameras.items():
        t, q = split_transform(transform)
        doc["cameras"][cam] = {
            "rig_from_camera": {
                "translation": [round(float(v), 6) for v in t],
                "rotation_xyzw": [round(float(v), 9) for v in q],
            },
            **info.get(cam, {}),
        }
    with open(path, "w") as f:
        f.write(header)
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)


def resolve_path(path: str, base_dir: str) -> str:
    """Resolve a config-relative path (e.g. extrinsics_file) against the config file's directory."""
    if not path or os.path.isabs(path):
        return path
    return os.path.join(base_dir, path)


def physical_specs(
    params: Mapping, infos: Mapping, lookup: Lookup, overrides: Mapping[str, np.ndarray]
) -> List[CameraSpec]:
    """CameraSpecs of the physical cameras from CameraInfo P + TF, with calibrated overrides applied."""
    specs = []
    for cam in params["cameras"]:
        info = infos[cam]
        rig_from_camera = overrides.get(cam)
        if rig_from_camera is None:
            rig_from_camera = lookup(params["rig_frame"], info.header.frame_id or cam)
        specs.append(
            CameraSpec.from_projection(
                cam,
                info.width,
                info.height,
                info.p,
                rig_from_camera,
                downscale=int(params["downscale"]),
                border=tuple(params["border"]),
            )
        )
    return specs


class VoPipeline:
    """Decode -> (rectify) -> cuVSLAM for one camera configuration."""

    def __init__(self, params: Mapping, specs: List[CameraSpec], decode: Callable):
        self.params = params
        self.physical = specs
        self._decode = decode
        self._downscale = int(params["downscale"])
        self.pre = RigPreprocessor(specs, parse_pairs(params.get("stereo_pairs")))
        self.core = VoCore(
            self.pre.specs,
            multicam_mode=params["multicam_mode"],
            async_sba=bool(params["async_sba"]),
            use_motion_model=bool(params["use_motion_model"]),
            use_denoising=bool(params["use_denoising"]),
            rectified_stereo_camera=bool(params["rectified_stereo_camera"])
            and bool(self.pre.pairs),
            masks=self.pre.masks,
            verbosity=int(params["verbosity"]),
        )

    def describe(self) -> List[str]:
        lines = []
        for s in self.physical:
            lines.append(
                f"{s.name}: {s.width}x{s.height} f=({s.fx:.1f},{s.fy:.1f}) c=({s.cx:.1f},{s.cy:.1f}) "
                f"t_rig={np.round(s.rig_from_camera[:3, 3], 3).tolist()}"
            )
        for pair in self.pre.pairs:
            f = pair.frame
            yaw = np.degrees(np.arctan2(f.r_rig_rect[1, 2], f.r_rig_rect[0, 2]))
            lines.append(
                f"rectified pair {pair.left.source.name} (left) / {pair.right.source.name} (right): "
                f"baseline {f.baseline:.3f} m, virtual view yaw {yaw:+.1f} deg in {self.params['rig_frame']}"
            )
        return lines

    def reset(self) -> None:
        self.core.reset()

    def process(self, frame_set: FrameSet) -> TrackResult:
        """Track one frame set whose payloads are CompressedImage data. Raises ValueError on bad stamps."""
        images: List[Optional[np.ndarray]] = [
            self._decode(f[1], self._downscale) if f is not None else None
            for f in frame_set.frames
        ]
        return self.core.track(frame_set.stamp_ns, self.pre.apply(images))
