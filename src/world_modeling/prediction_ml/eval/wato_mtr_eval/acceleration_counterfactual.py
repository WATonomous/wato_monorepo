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
"""Official-MTR acceleration-rule counterfactual for the WATO representation.

This evaluation-only intervention compares current WATO against two fixed
counterfactual arms. One applies upstream MTR's acceleration formula with
raw-invalid velocity semantics on the fixed WATO representation; the deployable
arm treats unavailable invalid values as zero. Both replace only
``obj_trajs[..., 27:29]``: difference every adjacent velocity at 10 Hz, copy
slot 1 into slot 0, then zero invalid rows. Production SceneBuilder behavior is
not changed.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
from collections.abc import Iterable, Sequence

import numpy as np
import torch

from . import (
    lanelet_query as lq,
)
from . import (
    metrics,
    model,
    reference_pack,
    rung1,
    scenario,
)
from . import (
    representation as rep,
)


@dataclasses.dataclass(frozen=True)
class InvalidVelocityAudit:
    scenarios: int
    typed_history_slots: int
    invalid_history_slots: int
    nonzero_invalid_velocity_slots: int
    nonfinite_invalid_velocity_slots: int
    max_invalid_speed_mps: float

    @property
    def all_invalid_velocities_are_finite_zero(self) -> bool:
        return (
            self.nonzero_invalid_velocity_slots == 0
            and self.nonfinite_invalid_velocity_slots == 0
        )

    def format(self) -> str:
        return (
            "Raw-shard invalid-slot velocity prevalence\n"
            f"  scenarios: {self.scenarios}\n"
            f"  typed history slots: {self.typed_history_slots}\n"
            f"  invalid history slots: {self.invalid_history_slots}\n"
            "  invalid slots with nonzero velocity: "
            f"{self.nonzero_invalid_velocity_slots}\n"
            "  invalid slots with nonfinite velocity: "
            f"{self.nonfinite_invalid_velocity_slots}\n"
            f"  maximum invalid-slot speed: {self.max_invalid_speed_mps:.6f} m/s\n"
            "  raw-shard invalid velocities are all finite zero values: "
            f"{'YES' if self.all_invalid_velocities_are_finite_zero else 'NO'}"
        )


@dataclasses.dataclass(frozen=True)
class CounterfactualReport:
    audit: InvalidVelocityAudit
    num_scenarios: int
    num_targets: int
    current_minade: float
    current_minfde: float
    exact_minade: float
    exact_minfde: float
    deployable_minade: float
    deployable_minfde: float
    exact_minus_current_minade: metrics.PairedDelta
    exact_minus_current_minfde: metrics.PairedDelta
    deployable_minus_current_minade: metrics.PairedDelta
    deployable_minus_current_minfde: metrics.PairedDelta
    exact_minus_deployable_minade: metrics.PairedDelta
    exact_minus_deployable_minfde: metrics.PairedDelta
    deployable_minade_changed_context: metrics.PairedDelta
    deployable_minfde_changed_context: metrics.PairedDelta
    deployable_minade_changed_target: metrics.PairedDelta
    deployable_minfde_changed_target: metrics.PairedDelta
    exact_changed_context_targets: int
    exact_changed_target_row_targets: int
    exact_changed_acceleration_values: int
    deployable_changed_context_targets: int
    deployable_changed_target_row_targets: int
    deployable_changed_acceleration_values: int

    def format(self) -> str:
        sections = [
            self.audit.format(),
            "",
            "Acceleration counterfactual (three fixed-pairing arms)",
            f"  scenarios: {self.num_scenarios}",
            f"  targets: {self.num_targets}",
            (
                "  B upstream-formula/raw-invalid targets with any changed "
                "context acceleration: "
                f"{self.exact_changed_context_targets}"
            ),
            (
                "  B upstream-formula/raw-invalid targets with changed "
                "selected-target row: "
                f"{self.exact_changed_target_row_targets}"
            ),
            (
                "  B upstream-formula/raw-invalid changed scalar acceleration values: "
                f"{self.exact_changed_acceleration_values}"
            ),
            (
                "  C deployable targets with any changed context acceleration: "
                f"{self.deployable_changed_context_targets}"
            ),
            (
                "  C deployable targets with changed selected-target row: "
                f"{self.deployable_changed_target_row_targets}"
            ),
            (
                "  C deployable changed scalar acceleration values: "
                f"{self.deployable_changed_acceleration_values}"
            ),
            (
                f"  current WATO minADE/minFDE: "
                f"{self.current_minade:.4f} / {self.current_minfde:.4f} m"
            ),
            (
                f"  upstream formula with raw-invalid semantics minADE/minFDE: "
                f"{self.exact_minade:.4f} / {self.exact_minfde:.4f} m"
            ),
            (
                f"  deployable zero-fill minADE/minFDE: "
                f"{self.deployable_minade:.4f} / {self.deployable_minfde:.4f} m"
            ),
            "  A/B/C finite metric masks: IDENTICAL (enforced)",
            "",
            metrics.format_paired_delta(
                self.exact_minus_current_minade,
                "B - A  upstream formula/raw-invalid minus current WATO  minADE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.exact_minus_current_minfde,
                "B - A  upstream formula/raw-invalid minus current WATO  minFDE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minus_current_minade,
                "C - A  deployable zero-fill minus current WATO  minADE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minus_current_minfde,
                "C - A  deployable zero-fill minus current WATO  minFDE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.exact_minus_deployable_minade,
                "B - C  upstream formula/raw-invalid minus deployable "
                "zero-fill  minADE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.exact_minus_deployable_minfde,
                "B - C  upstream formula/raw-invalid minus deployable "
                "zero-fill  minFDE  (all)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minade_changed_context,
                "C - A  deployable zero-fill minus current WATO  minADE  "
                "(any of 128 context rows changed)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minfde_changed_context,
                "C - A  deployable zero-fill minus current WATO  minFDE  "
                "(any of 128 context rows changed)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minade_changed_target,
                "C - A  deployable zero-fill minus current WATO  minADE  "
                "(selected-target row changed)",
            ),
            "",
            metrics.format_paired_delta(
                self.deployable_minfde_changed_target,
                "C - A  deployable zero-fill minus current WATO  minFDE  "
                "(selected-target row changed)",
            ),
        ]
        return "\n".join(sections)


@dataclasses.dataclass
class _AuditAccumulator:
    scenarios: int = 0
    typed_history_slots: int = 0
    invalid_history_slots: int = 0
    nonzero_invalid_velocity_slots: int = 0
    nonfinite_invalid_velocity_slots: int = 0
    max_invalid_speed_mps: float = 0.0

    def add(self, scen: scenario.Scenario) -> None:
        self.scenarios += 1
        start = max(0, scen.current_time_index - rep.HISTORY_STEPS + 1)
        stop = scen.current_time_index + 1
        for track in scen.tracks:
            if track.object_type not in rep.MTR_TYPE_CODES:
                continue
            history = track.states[start:stop]
            velocity = history[:, [scenario.STATE_COL["vx"], scenario.STATE_COL["vy"]]]
            invalid = history[:, scenario.STATE_COL["valid"]] == 0.0
            self.typed_history_slots += len(history)
            self.invalid_history_slots += int(invalid.sum())
            if not invalid.any():
                continue
            invalid_velocity = velocity[invalid]
            finite = np.isfinite(invalid_velocity).all(axis=1)
            self.nonfinite_invalid_velocity_slots += int((~finite).sum())
            if finite.any():
                speed = np.linalg.norm(invalid_velocity[finite], axis=1)
                self.nonzero_invalid_velocity_slots += int((speed != 0.0).sum())
                self.max_invalid_speed_mps = max(
                    self.max_invalid_speed_mps,
                    float(speed.max(initial=0.0)),
                )

    def result(self) -> InvalidVelocityAudit:
        return InvalidVelocityAudit(**dataclasses.asdict(self))


def official_acceleration(
    obj_trajs: np.ndarray,
    obj_trajs_mask: np.ndarray,
) -> np.ndarray:
    """Apply the pinned MTR formula to WATO's zero-filled velocity grid."""
    obj_trajs = np.asarray(obj_trajs)
    mask = np.asarray(obj_trajs_mask).astype(bool, copy=False)
    if obj_trajs.ndim != 4 or obj_trajs.shape[-2:] != (
        rep.HISTORY_STEPS,
        rep.OBJ_FEATURE_DIM,
    ):
        raise ValueError(
            "obj_trajs must have shape [targets,context,"
            f"{rep.HISTORY_STEPS},{rep.OBJ_FEATURE_DIM}]"
        )
    if mask.shape != obj_trajs.shape[:-1]:
        raise ValueError(
            f"obj_trajs_mask shape {mask.shape} does not match {obj_trajs.shape[:-1]}"
        )
    velocity = obj_trajs[..., rep.OBJ_COL["velocity"]]
    previous = np.roll(velocity, shift=1, axis=2)
    acceleration = (velocity - previous) * np.asarray(
        rep.HISTORY_RATE_HZ, dtype=obj_trajs.dtype
    )
    acceleration[:, :, 0, :] = acceleration[:, :, 1, :]
    acceleration[~mask] = 0.0
    return acceleration


def counterfactual_bundle(
    tensors: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Apply the formula to packed float32 velocity for focused unit tests.

    The evaluation does not use this helper: recomputing from packed velocity
    can introduce a float32 precision confound even when history is gap-free.
    """
    required = {"obj_trajs", "obj_trajs_mask"}
    if not required <= tensors.keys():
        raise ValueError(
            f"tensor bundle is missing {sorted(required - tensors.keys())}"
        )
    output = {name: np.asarray(value).copy() for name, value in tensors.items()}
    output["obj_trajs"][..., rep.OBJ_COL["acceleration"]] = official_acceleration(
        tensors["obj_trajs"],
        tensors["obj_trajs_mask"],
    )
    return output


def _source_velocity_grid(
    scen: scenario.Scenario,
    tensors: dict[str, np.ndarray],
    selections: Sequence[reference_pack.TargetSelection] | None = None,
    *,
    step: int | None = None,
    include_raw_invalid: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """Reconstruct target-frame velocity before its final float32 cast."""
    step = scen.current_time_index if step is None else step
    selections = (
        reference_pack.target_selection(scen, step)
        if selections is None
        else list(selections)
    )
    velocity = np.zeros(
        (*tensors["obj_trajs"].shape[:-1], 2),
        dtype=np.float64,
    )
    mask = tensors["obj_trajs_mask"].astype(bool, copy=False)
    if not selections:
        return velocity, mask

    mandatory = {selection.scene_builder_id for selection in selections}
    context = reference_pack._context_tracks(scen, step, mandatory)
    slots = reference_pack._resampled_slots(scen, step)
    source_steps = (
        [tuple(range(step + 1)) for _ in context]
        if include_raw_invalid
        else [context_track.source_steps for context_track in context]
    )
    sources = [
        [
            reference_pack._nearest_source_step(
                scen,
                available_steps,
                float(desired_time),
            )
            for desired_time in slots
        ]
        for available_steps in source_steps
    ]

    for selection in selections:
        target_state = scen.tracks[selection.track_index].states[step]
        heading = float(target_state[scenario.STATE_COL["heading"]])
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        for context_slot, context_track in enumerate(context):
            if not include_raw_invalid:
                reconstructed_mask = np.asarray(
                    [source is not None for source in sources[context_slot]],
                    dtype=bool,
                )
                if not np.array_equal(
                    reconstructed_mask,
                    mask[selection.slot, context_slot],
                ):
                    raise RuntimeError(
                        "deployable source reconstruction changed the packed mask"
                    )
            for history_slot, source_step in enumerate(sources[context_slot]):
                if source_step is None:
                    continue
                state = scen.tracks[context_track.track_index].states[source_step]
                vx = float(state[scenario.STATE_COL["vx"]])
                vy = float(state[scenario.STATE_COL["vy"]])
                velocity[selection.slot, context_slot, history_slot] = (
                    vx * cos_h + vy * sin_h,
                    -vx * sin_h + vy * cos_h,
                )
    return velocity, mask


def _mtr_acceleration_from_velocity(
    velocity: np.ndarray,
    mask: np.ndarray,
) -> np.ndarray:
    """Apply pinned MTR's adjacent-slot formula before masking."""
    previous = np.roll(velocity, shift=1, axis=2)
    acceleration = (velocity - previous) * rep.HISTORY_RATE_HZ
    acceleration[:, :, 0, :] = acceleration[:, :, 1, :]
    acceleration[~mask] = 0.0
    return acceleration


def upstream_raw_invalid_acceleration_for_scenario(
    scen: scenario.Scenario,
    tensors: dict[str, np.ndarray],
    selections: Sequence[reference_pack.TargetSelection] | None = None,
    *,
    step: int | None = None,
) -> np.ndarray:
    """Apply upstream's acceleration formula with raw-invalid semantics.

    Context selection, slot resampling, target frames, and all other features
    remain fixed to the WATO representation. This is not a claim of exact
    upstream preprocessing parity.
    """
    velocity, mask = _source_velocity_grid(
        scen,
        tensors,
        selections,
        step=step,
        include_raw_invalid=True,
    )
    return _mtr_acceleration_from_velocity(velocity, mask)


def deployable_acceleration_for_scenario(
    scen: scenario.Scenario,
    tensors: dict[str, np.ndarray],
    selections: Sequence[reference_pack.TargetSelection] | None = None,
    *,
    step: int | None = None,
) -> np.ndarray:
    """Apply MTR's formula to float64 valid-source velocities and zero gaps."""
    velocity, mask = _source_velocity_grid(
        scen,
        tensors,
        selections,
        step=step,
        include_raw_invalid=False,
    )
    return _mtr_acceleration_from_velocity(velocity, mask)


def upstream_raw_invalid_scenario_bundle(
    scen: scenario.Scenario,
    tensors: dict[str, np.ndarray],
    selections: Sequence[reference_pack.TargetSelection] | None = None,
    *,
    step: int | None = None,
) -> dict[str, np.ndarray]:
    """Clone WATO and patch only upstream-formula/raw-invalid acceleration."""
    output = {name: np.asarray(value).copy() for name, value in tensors.items()}
    output["obj_trajs"][..., rep.OBJ_COL["acceleration"]] = (
        upstream_raw_invalid_acceleration_for_scenario(
            scen,
            tensors,
            selections,
            step=step,
        )
    )
    return output


def deployable_scenario_bundle(
    scen: scenario.Scenario,
    tensors: dict[str, np.ndarray],
    selections: Sequence[reference_pack.TargetSelection] | None = None,
    *,
    step: int | None = None,
) -> dict[str, np.ndarray]:
    """Clone WATO and patch only float64-source, zero-fill acceleration."""
    output = {name: np.asarray(value).copy() for name, value in tensors.items()}
    output["obj_trajs"][..., rep.OBJ_COL["acceleration"]] = (
        deployable_acceleration_for_scenario(
            scen,
            tensors,
            selections,
            step=step,
        )
    )
    return output


def _audit_scenarios(scenarios: Iterable[scenario.Scenario]) -> InvalidVelocityAudit:
    accumulator = _AuditAccumulator()
    for scen in scenarios:
        accumulator.add(scen)
    return accumulator.result()


def audit(
    shard: pathlib.Path,
    limit: int | None = None,
) -> InvalidVelocityAudit:
    """Audit raw invalid-slot velocities over each scenario's 11-step history."""
    shard = pathlib.Path(shard).expanduser().resolve()
    if not shard.is_file():
        raise FileNotFoundError(f"Waymo scenario shard does not exist: {shard}")
    return _audit_scenarios(scenario.load_scenarios(shard, limit=limit))


def _finite_mean(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    return float(finite.mean()) if finite.size else float("nan")


def _require_identical_finite_masks(
    label: str,
    current: np.ndarray,
    upstream_raw_invalid: np.ndarray,
    deployable: np.ndarray,
) -> None:
    masks = [
        np.isfinite(values) for values in (current, upstream_raw_invalid, deployable)
    ]
    if not (np.array_equal(masks[0], masks[1]) and np.array_equal(masks[0], masks[2])):
        counts = "/".join(str(int(mask.sum())) for mask in masks)
        raise RuntimeError(
            f"{label} finite masks differ across A/B/C (finite counts {counts})"
        )


def _subset_delta(
    current: np.ndarray,
    counterfactual: np.ndarray,
    keep: np.ndarray,
    agent_ids: list[str],
    scenario_ids: list[str],
) -> metrics.PairedDelta:
    indices = np.flatnonzero(keep)
    return metrics.paired_delta(
        counterfactual[indices],
        current[indices],
        [agent_ids[index] for index in indices],
        [scenario_ids[index] for index in indices],
    )


def run(
    shard: pathlib.Path,
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    limit: int | None = None,
    *,
    cfg_name: str = "mtr+100_percent_data.yaml",
    device: str = "cuda",
) -> CounterfactualReport:
    """Run fixed-pairing current, raw-invalid-formula, and deployable scoring."""
    if device not in {"cuda", "cuda:0"}:
        raise ValueError("acceleration counterfactual requires device='cuda'")
    if not torch.cuda.is_available():
        raise RuntimeError("acceleration counterfactual requires CUDA")
    shard = pathlib.Path(shard).expanduser().resolve()
    checkpoint = pathlib.Path(checkpoint).expanduser().resolve()
    mtr_root = pathlib.Path(mtr_root).expanduser().resolve()
    cfg = mtr_root / "tools/cfgs/waymo" / cfg_name
    for path, label in (
        (shard, "Waymo scenario shard"),
        (checkpoint, "MTR checkpoint"),
        (cfg, "MTR config"),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{label} does not exist: {path}")

    net = model.load_checkpoint(
        checkpoint,
        cfg,
        device=device,
        mtr_root=mtr_root,
    )
    query = lq.QueryConfig(strategy="radius")
    audit_accumulator = _AuditAccumulator()
    current_ade: list[np.ndarray] = []
    current_fde: list[np.ndarray] = []
    exact_ade: list[np.ndarray] = []
    exact_fde: list[np.ndarray] = []
    deployable_ade: list[np.ndarray] = []
    deployable_fde: list[np.ndarray] = []
    exact_context_changed: list[bool] = []
    exact_target_changed: list[bool] = []
    deployable_context_changed: list[bool] = []
    deployable_target_changed: list[bool] = []
    agent_ids: list[str] = []
    scenario_ids: list[str] = []
    exact_changed_values = 0
    deployable_changed_values = 0
    num_scenarios = 0

    for scen in scenario.load_scenarios(shard, limit=limit):
        audit_accumulator.add(scen)
        selections = reference_pack.target_selection(scen)
        if not selections:
            continue
        current = reference_pack.pack(scen, query)
        exact = upstream_raw_invalid_scenario_bundle(
            scen,
            current,
            selections,
        )
        deployable = deployable_scenario_bundle(
            scen,
            current,
            selections,
        )
        current_acceleration = current["obj_trajs"][..., rep.OBJ_COL["acceleration"]]
        exact_acceleration = exact["obj_trajs"][..., rep.OBJ_COL["acceleration"]]
        deployable_acceleration = deployable["obj_trajs"][
            ..., rep.OBJ_COL["acceleration"]
        ]
        exact_changed = current_acceleration != exact_acceleration
        deployable_changed = current_acceleration != deployable_acceleration
        count = len(selections)
        exact_changed_values += int(exact_changed[:count].sum())
        deployable_changed_values += int(deployable_changed[:count].sum())
        for selection in selections:
            exact_context_changed.append(bool(exact_changed[selection.slot].any()))
            deployable_context_changed.append(
                bool(deployable_changed[selection.slot].any())
            )
            target_row = int(current["track_index_to_predict"][selection.slot])
            exact_target_changed.append(
                bool(exact_changed[selection.slot, target_row].any())
            )
            deployable_target_changed.append(
                bool(deployable_changed[selection.slot, target_row].any())
            )
            agent_ids.append(selection.waymo_id)
            scenario_ids.append(scen.scenario_id)

        gt, valid = rung1._deployed_ground_truth(scen, selections)
        _, current_native = model.forward_batch(
            net,
            rung1._model_batch(current, selections, scen.scenario_id),
        )
        _, exact_native = model.forward_batch(
            net,
            rung1._model_batch(exact, selections, scen.scenario_id),
        )
        _, deployable_native = model.forward_batch(
            net,
            rung1._model_batch(deployable, selections, scen.scenario_id),
        )
        current_deployed = metrics.to_deployed(current_native)
        exact_deployed = metrics.to_deployed(exact_native)
        deployable_deployed = metrics.to_deployed(deployable_native)
        current_ade.append(metrics.min_ade(current_deployed, gt, valid))
        current_fde.append(metrics.min_fde(current_deployed, gt, valid))
        exact_ade.append(metrics.min_ade(exact_deployed, gt, valid))
        exact_fde.append(metrics.min_fde(exact_deployed, gt, valid))
        deployable_ade.append(metrics.min_ade(deployable_deployed, gt, valid))
        deployable_fde.append(metrics.min_fde(deployable_deployed, gt, valid))
        num_scenarios += 1

    if not current_ade:
        raise RuntimeError(f"no scorable scenarios were read from {shard}")
    current_ade_values = np.concatenate(current_ade)
    current_fde_values = np.concatenate(current_fde)
    exact_ade_values = np.concatenate(exact_ade)
    exact_fde_values = np.concatenate(exact_fde)
    deployable_ade_values = np.concatenate(deployable_ade)
    deployable_fde_values = np.concatenate(deployable_fde)
    _require_identical_finite_masks(
        "minADE",
        current_ade_values,
        exact_ade_values,
        deployable_ade_values,
    )
    _require_identical_finite_masks(
        "minFDE",
        current_fde_values,
        exact_fde_values,
        deployable_fde_values,
    )
    exact_context_changed_values = np.asarray(exact_context_changed, dtype=bool)
    exact_target_changed_values = np.asarray(exact_target_changed, dtype=bool)
    deployable_context_changed_values = np.asarray(
        deployable_context_changed, dtype=bool
    )
    deployable_target_changed_values = np.asarray(deployable_target_changed, dtype=bool)

    return CounterfactualReport(
        audit=audit_accumulator.result(),
        num_scenarios=num_scenarios,
        num_targets=len(agent_ids),
        current_minade=_finite_mean(current_ade_values),
        current_minfde=_finite_mean(current_fde_values),
        exact_minade=_finite_mean(exact_ade_values),
        exact_minfde=_finite_mean(exact_fde_values),
        deployable_minade=_finite_mean(deployable_ade_values),
        deployable_minfde=_finite_mean(deployable_fde_values),
        exact_minus_current_minade=metrics.paired_delta(
            exact_ade_values, current_ade_values, agent_ids, scenario_ids
        ),
        exact_minus_current_minfde=metrics.paired_delta(
            exact_fde_values, current_fde_values, agent_ids, scenario_ids
        ),
        deployable_minus_current_minade=metrics.paired_delta(
            deployable_ade_values, current_ade_values, agent_ids, scenario_ids
        ),
        deployable_minus_current_minfde=metrics.paired_delta(
            deployable_fde_values, current_fde_values, agent_ids, scenario_ids
        ),
        exact_minus_deployable_minade=metrics.paired_delta(
            exact_ade_values, deployable_ade_values, agent_ids, scenario_ids
        ),
        exact_minus_deployable_minfde=metrics.paired_delta(
            exact_fde_values, deployable_fde_values, agent_ids, scenario_ids
        ),
        deployable_minade_changed_context=_subset_delta(
            current_ade_values,
            deployable_ade_values,
            deployable_context_changed_values,
            agent_ids,
            scenario_ids,
        ),
        deployable_minfde_changed_context=_subset_delta(
            current_fde_values,
            deployable_fde_values,
            deployable_context_changed_values,
            agent_ids,
            scenario_ids,
        ),
        deployable_minade_changed_target=_subset_delta(
            current_ade_values,
            deployable_ade_values,
            deployable_target_changed_values,
            agent_ids,
            scenario_ids,
        ),
        deployable_minfde_changed_target=_subset_delta(
            current_fde_values,
            deployable_fde_values,
            deployable_target_changed_values,
            agent_ids,
            scenario_ids,
        ),
        exact_changed_context_targets=int(exact_context_changed_values.sum()),
        exact_changed_target_row_targets=int(exact_target_changed_values.sum()),
        exact_changed_acceleration_values=exact_changed_values,
        deployable_changed_context_targets=int(deployable_context_changed_values.sum()),
        deployable_changed_target_row_targets=int(
            deployable_target_changed_values.sum()
        ),
        deployable_changed_acceleration_values=deployable_changed_values,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shard", type=pathlib.Path, required=True)
    parser.add_argument("--checkpoint", type=pathlib.Path)
    parser.add_argument("--mtr-root", type=pathlib.Path)
    parser.add_argument("--cfg", default="mtr+100_percent_data.yaml")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--audit-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.audit_only:
        print(audit(args.shard, limit=args.limit).format())
        return 0
    if args.checkpoint is None or args.mtr_root is None:
        raise SystemExit("--checkpoint and --mtr-root are required unless --audit-only")
    report = run(
        args.shard,
        args.checkpoint,
        args.mtr_root,
        limit=args.limit,
        cfg_name=args.cfg,
    )
    print(report.format())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
