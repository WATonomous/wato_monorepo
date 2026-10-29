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
"""Displacement metrics and paired delta distributions.

Rung-to-rung comparison is paired: the same scenarios and the same target
agents pass through both pipelines, so the per-agent difference is directly
available. Reporting only the difference of aggregates discards that. A mean
delta of 0.01 m could be every agent at 0.01 (uniform float precision loss,
benign) or 94% at zero and 6% at 0.19 (one code path broken). Those demand
opposite responses and no aggregate separates them.
"""

from __future__ import annotations

import dataclasses

import numpy as np

from . import representation as rep

NEAR_ZERO_M = 1e-3


def _displacement(pred: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """[A,K,T,2] against [A,T,2] -> [A,K,T] L2 distance."""
    return np.linalg.norm(pred[..., :2] - gt[:, None, :, :2], axis=-1)


def min_ade(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Minimum-over-modes average displacement error, per agent. NaN if no valid step."""
    dist = _displacement(pred, gt)
    mask = valid[:, None, :]
    counts = mask.sum(axis=-1)
    with np.errstate(invalid="ignore", divide="ignore"):
        ade = np.where(
            counts > 0,
            np.where(mask, dist, 0.0).sum(axis=-1) / counts,
            np.inf,
        )
    if ade.shape[0] == 0:
        return np.empty((0,), dtype=ade.dtype)
    result = ade.min(axis=-1)
    result[~valid.any(axis=-1)] = np.nan
    return result


def min_fde(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Minimum-over-modes final displacement error at each agent's LAST VALID step."""
    dist = _displacement(pred, gt)
    num_agents = dist.shape[0]
    out = np.full(num_agents, np.nan)
    for a in range(num_agents):
        idx = np.flatnonzero(valid[a])
        if idx.size == 0:
            continue
        out[a] = dist[a, :, idx[-1]].min()
    return out


def to_deployed(pred: np.ndarray) -> np.ndarray:
    """[A,K,80,C] model output -> [A,K,15,2], matching output_converter.cpp.

    Stride = DEPLOYED_STEP_S / MODEL_STEP_S, then clamp to DEPLOYED_STEPS. The
    first emitted waypoint is model step (stride - 1), i.e. the end of the first
    deployed interval, not step 0.
    """
    stride = int(round(rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S))
    strided = pred[:, :, stride - 1 :: stride, :2]
    return strided[:, :, : rep.DEPLOYED_STEPS]


@dataclasses.dataclass(frozen=True)
class WorstRow:
    scenario_id: str
    agent_id: str
    delta: float


@dataclasses.dataclass(frozen=True)
class PairedDelta:
    count: int
    mean: float
    median: float
    p90: float
    p99: float
    max: float
    near_zero_fraction: float
    worst: list[WorstRow]


def paired_delta(
    a: np.ndarray,
    b: np.ndarray,
    agent_ids: list[str],
    scenario_ids: list[str],
    n_worst: int = 10,
) -> PairedDelta:
    """Distribution of per-agent (a - b). Pairs where either side is NaN are dropped."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if not (len(a) == len(b) == len(agent_ids) == len(scenario_ids)):
        raise ValueError("a, b, agent_ids and scenario_ids must have the same length")

    keep = np.isfinite(a) & np.isfinite(b)
    delta = a[keep] - b[keep]
    kept_agents = [agent_ids[i] for i in np.flatnonzero(keep)]
    kept_scen = [scenario_ids[i] for i in np.flatnonzero(keep)]

    if delta.size == 0:
        return PairedDelta(0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, [])

    order = np.argsort(-np.abs(delta))[:n_worst]
    worst = [WorstRow(kept_scen[i], kept_agents[i], float(delta[i])) for i in order]

    return PairedDelta(
        count=int(delta.size),
        mean=float(delta.mean()),
        median=float(np.median(delta)),
        p90=float(np.percentile(delta, 90)),
        p99=float(np.percentile(delta, 99)),
        max=float(delta[np.argmax(np.abs(delta))]),
        near_zero_fraction=float((np.abs(delta) < NEAR_ZERO_M).mean()),
        worst=worst,
    )


def format_paired_delta(d: PairedDelta, label: str, units: str = "m") -> str:
    """Human-readable report. The worst rows are the debugging entry point."""
    lines = [
        f"{label}  paired over {d.count} target agents",
        f"  mean      {d.mean:+.4f} {units}",
        f"  median    {d.median:+.4f} {units}",
        f"  p90       {d.p90:+.4f} {units}",
        f"  p99       {d.p99:+.4f} {units}",
        f"  max       {d.max:+.4f} {units}",
        f"  |delta| < 1e-3:  {d.near_zero_fraction * d.count:.0f} / {d.count}"
        f"  ({d.near_zero_fraction:.1%})",
        "",
        f"  worst {len(d.worst)} agents:",
        f"    {'scenario_id':<24} {'agent_id':<12} {'delta':>10}",
    ]
    lines += [
        f"    {r.scenario_id:<24} {r.agent_id:<12} {r.delta:>+10.4f}" for r in d.worst
    ]
    return "\n".join(lines)
