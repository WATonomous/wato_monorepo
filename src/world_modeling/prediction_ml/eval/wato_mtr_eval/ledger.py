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
"""Information-loss accounting.

Every drop point already exists in the pipeline, so instrumenting is cheap.
The payoff is that when deployed minFDE moves 1.22 -> 1.61, cross-referencing
the worst-N agents against these counts usually names the cause immediately:
if the worst rows all carry a context overflow, the bug is in ranking or
truncation rather than in packing.

Scoped to drop points that CAN change -- budgets, filters, policies. Static
facts belong in REPRESENTATION.md, not in per-run output.
"""
from __future__ import annotations

import dataclasses
import json
import pathlib

from . import representation as rep


@dataclasses.dataclass(frozen=True)
class MapStats:
    radius_m: float
    nearby_lanes: int
    radius_only_emitted: int
    reachable_emitted: int
    points_before: int
    points_after: int
    omitted_by_type: dict[str, int]


@dataclasses.dataclass(frozen=True)
class Ledger:
    scenario_id: str
    agents: dict
    targets: dict
    history: dict
    map_: dict
    output: dict

    def format(self) -> str:
        a, t, h, m, o = (
            self.agents,
            self.targets,
            self.history,
            self.map_,
            self.output,
        )
        dropped = (
            ", ".join(
                f"{key} {value}"
                for key, value in sorted(a["dropped_by_type"].items())
            )
            or "none"
        )
        omitted = "  ".join(
            f"{key} {value}"
            for key, value in sorted(m["omitted_semantics"].items())
        )
        return "\n".join(
            [
                f"scenario {self.scenario_id}",
                f"AGENTS   waymo tracks {a['waymo_tracks']} -> type-mapped "
                f"{a['type_mapped']} ({a['dropped']} dropped: {dropped})",
                f"         -> context capacity {a['context_capacity']}, used "
                f"{a['context_used']}, overflow {a['context_overflow']}",
                f"TARGETS  candidates {t['candidates']} -> tracks_to_predict "
                f"{t['selected']} -> selected {t['selected']} "
                f"({'pinned' if t['pinned'] else 'heuristic'})",
                f"         [heuristic would have selected "
                f"{t['heuristic_agreement']}/{t['selected']} of these]",
                f"HISTORY  {h['slots']} slots x {h['tracks']} tracks = "
                f"{h['total']} -> valid {h['valid']}, masked {h['masked']} "
                f"({h['masked_fraction']:.1%})",
                f"MAP      radius {m['radius_m']:.0f} m; nearby lanes "
                f"{m['nearby_lanes']} -> radius_only "
                f"{m['radius_only_emitted']} -> reachable "
                f"{m['reachable_emitted']}",
                f"         centerline points {m['centerline_points_before']} -> "
                f"{m['centerline_points_after']}",
                f"         polyline capacity {m['polyline_capacity']}, used "
                f"{m['polylines_used']}, overflow {m['polyline_overflow']}",
                f"         omitted semantics: {omitted}",
                f"         global_type: {m['global_type_collapse']}",
                f"OUTPUT   {o['model_steps']} steps @ {o['model_step_s']} s -> "
                f"stride {o['stride']} -> {o['strided_steps']} -> horizon clamp "
                f"-> {o['emitted_steps']} @ {o['emitted_step_s']} s",
            ]
        )

    def to_json(self, path: pathlib.Path) -> None:
        payload = {
            "scenario_id": self.scenario_id,
            "agents": self.agents,
            "targets": self.targets,
            "history": self.history,
            "map": self.map_,
            "output": self.output,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def build_ledger(
    *,
    scenario_id: str,
    num_waymo_tracks: int,
    num_type_mapped: int,
    dropped_by_type: dict[str, int],
    num_candidates: int,
    selected_target_indices: list[int],
    heuristic_target_indices: list[int],
    history_valid: int,
    history_total: int,
    map_stats: MapStats,
) -> Ledger:
    stride = int(round(rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S))
    strided = rep.NUM_FUTURE_FRAMES // stride
    agreement = len(set(selected_target_indices) & set(heuristic_target_indices))
    selected = len(selected_target_indices)
    used_map = min(map_stats.reachable_emitted, rep.NUM_SRC_POLYLINES)

    return Ledger(
        scenario_id=scenario_id,
        agents={
            "waymo_tracks": num_waymo_tracks,
            "type_mapped": num_type_mapped,
            "dropped": num_waymo_tracks - num_type_mapped,
            "dropped_by_type": dict(dropped_by_type),
            "context_capacity": rep.CONTEXT_CAPACITY,
            "context_used": min(num_type_mapped, rep.CONTEXT_CAPACITY),
            "context_overflow": max(
                0, num_type_mapped - rep.CONTEXT_CAPACITY
            ),
        },
        targets={
            "candidates": num_candidates,
            "selected": selected,
            "pinned": True,
            "heuristic_agreement": agreement,
            "heuristic_agreement_fraction": (
                agreement / selected if selected else 0.0
            ),
        },
        history={
            "slots": rep.HISTORY_STEPS,
            "tracks": (
                history_total // rep.HISTORY_STEPS if rep.HISTORY_STEPS else 0
            ),
            "total": history_total,
            "valid": history_valid,
            "masked": history_total - history_valid,
            "masked_fraction": (
                (history_total - history_valid) / history_total
                if history_total
                else 0.0
            ),
        },
        map_={
            "radius_m": map_stats.radius_m,
            "nearby_lanes": map_stats.nearby_lanes,
            "radius_only_emitted": map_stats.radius_only_emitted,
            "reachable_emitted": map_stats.reachable_emitted,
            "dropped_by_connectivity": (
                map_stats.radius_only_emitted - map_stats.reachable_emitted
            ),
            "centerline_points_before": map_stats.points_before,
            "centerline_points_after": map_stats.points_after,
            "polyline_capacity": rep.NUM_SRC_POLYLINES,
            "polylines_used": used_map,
            "polyline_overflow": max(
                0, map_stats.reachable_emitted - rep.NUM_SRC_POLYLINES
            ),
            "omitted_semantics": dict(map_stats.omitted_by_type),
            "global_type_collapse": "20 waymo types -> 1 wato token",
        },
        output={
            "model_steps": rep.NUM_FUTURE_FRAMES,
            "model_step_s": rep.MODEL_STEP_S,
            "stride": stride,
            "strided_steps": strided,
            "emitted_steps": min(strided, rep.DEPLOYED_STEPS),
            "emitted_step_s": rep.DEPLOYED_STEP_S,
            "emitted_horizon_s": (
                min(strided, rep.DEPLOYED_STEPS) * rep.DEPLOYED_STEP_S
            ),
        },
    )
