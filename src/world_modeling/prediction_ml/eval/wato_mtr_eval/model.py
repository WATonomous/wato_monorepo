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
"""Checkpoint loading and forward pass against the official MTR implementation.

Imports torch; must never import rosbag2_py (see tests/test_env_split.py).
The MTR package itself is imported lazily so that merely importing this module
does not require the CUDA extensions to be built.
"""

from __future__ import annotations

import dataclasses
import pathlib
import sys
from typing import Any

import numpy as np
import torch

from . import representation as rep


@dataclasses.dataclass(frozen=True)
class LoadReport:
    """State-dictionary compatibility information from a checkpoint load."""

    missing_keys: list[str]
    unexpected_keys: list[str]
    epoch: int


def _import_mtr(mtr_root: pathlib.Path):
    """Import the pinned official MTR checkout without requiring installation."""
    root = pathlib.Path(mtr_root).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"MTR root does not exist: {root}")
    root_text = str(root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)
    try:
        from mtr.config import cfg, cfg_from_yaml_file
        from mtr.models import model as mtr_model
    except ImportError as exc:
        raise RuntimeError(
            f"could not import official MTR from {root}; build its CUDA extensions first"
        ) from exc
    return cfg, cfg_from_yaml_file, mtr_model


def load_checkpoint(
    checkpoint: pathlib.Path,
    cfg_path: pathlib.Path,
    device: str = "cuda",
    mtr_root: pathlib.Path | None = None,
    return_report: bool = False,
):
    """Build MTR from its official config and load the checkpoint into it.

    Missing or unexpected keys are surfaced rather than swallowed: a silent key
    mismatch is exactly how a wrong-config load yields plausible-looking garbage.
    """
    checkpoint = pathlib.Path(checkpoint).expanduser().resolve()
    cfg_path = pathlib.Path(cfg_path).expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"MTR checkpoint does not exist: {checkpoint}")
    if not cfg_path.is_file():
        raise FileNotFoundError(f"MTR config does not exist: {cfg_path}")

    inferred_root = (
        cfg_path.parents[3] if len(cfg_path.parents) > 3 else cfg_path.parent
    )
    root = (
        pathlib.Path(mtr_root).expanduser() if mtr_root is not None else inferred_root
    )
    cfg, cfg_from_yaml_file, mtr_model = _import_mtr(root)
    cfg_from_yaml_file(str(cfg_path), cfg)

    net = mtr_model.MotionTransformer(config=cfg.MODEL)
    raw = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if not isinstance(raw, dict) or "model_state" not in raw:
        keys = sorted(raw) if isinstance(raw, dict) else []
        raise ValueError(f"checkpoint has no 'model_state'; found keys: {keys}")
    result = net.load_state_dict(raw["model_state"], strict=False)
    if result.missing_keys or result.unexpected_keys:
        raise ValueError(
            "checkpoint state does not exactly match the configured MTR model: "
            f"missing={list(result.missing_keys)[:5]}, "
            f"unexpected={list(result.unexpected_keys)[:5]}"
        )
    net.to(device).eval()

    report = LoadReport(
        missing_keys=list(result.missing_keys),
        unexpected_keys=list(result.unexpected_keys),
        epoch=int(raw.get("epoch", -1)),
    )
    return (net, report) if return_report else net


def _prediction_pair(out: Any) -> tuple[torch.Tensor, torch.Tensor]:
    """Normalize common official-MTR output containers to two tensors."""
    if isinstance(out, dict):
        if "pred_scores" in out and "pred_trajs" in out:
            scores, trajs = out["pred_scores"], out["pred_trajs"]
            if torch.is_tensor(scores) and scores.ndim == 1:
                scores = scores.unsqueeze(0)
            if torch.is_tensor(trajs) and trajs.ndim == 3:
                trajs = trajs.unsqueeze(0)
            return scores, trajs
        if "pred_list" in out and out["pred_list"]:
            return _prediction_pair(out["pred_list"][-1])
        if "pred_dicts" in out and out["pred_dicts"]:
            pairs = [_prediction_pair(item) for item in out["pred_dicts"]]
            return (
                torch.cat([pair[0] for pair in pairs], dim=0),
                torch.cat([pair[1] for pair in pairs], dim=0),
            )
    if isinstance(out, (tuple, list)):
        if len(out) >= 2 and torch.is_tensor(out[0]) and torch.is_tensor(out[1]):
            return out[0], out[1]
        for item in out:
            try:
                return _prediction_pair(item)
            except (TypeError, ValueError):
                continue
    raise TypeError(f"unsupported MTR output container: {type(out).__name__}")


@torch.no_grad()
def forward_batch(net, batch: dict) -> tuple[np.ndarray, np.ndarray]:
    """Run MTR. Returns ``pred_scores [A,6]`` and ``pred_trajs [A,6,80,7]``."""
    scores, trajs = _prediction_pair(net(batch))
    if not torch.is_tensor(scores) or not torch.is_tensor(trajs):
        raise TypeError("MTR predictions must be torch tensors")
    scores_np = scores.float().detach().cpu().numpy()
    trajs_np = trajs.float().detach().cpu().numpy()
    expected_scores = (scores_np.shape[0], rep.NUM_MODES)
    expected_trajs = (
        scores_np.shape[0],
        rep.NUM_MODES,
        rep.NUM_FUTURE_FRAMES,
        rep.TRAJ_CHANNELS,
    )
    if scores_np.shape != expected_scores:
        raise ValueError(
            f"pred_scores must have shape [A,{rep.NUM_MODES}], got {scores_np.shape}"
        )
    if trajs_np.shape != expected_trajs:
        raise ValueError(
            "pred_trajs must have shape "
            f"[A,{rep.NUM_MODES},{rep.NUM_FUTURE_FRAMES},{rep.TRAJ_CHANNELS}], "
            f"got {trajs_np.shape}"
        )
    return scores_np, trajs_np
