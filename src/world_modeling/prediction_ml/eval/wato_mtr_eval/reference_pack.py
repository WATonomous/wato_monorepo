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
"""Independent NumPy implementation of the WATO MTR input representation.

This intentionally does not share packing code with ``scene_builder.cpp``.
Both implementations read the representation contract, so disagreement can
expose a regression instead of allowing one implementation's bug to validate
itself.

The reference models replay through ``step``. In particular it includes the
synthetic ``__ego__`` history row that SceneBuilder creates from pose/odometry.
Fixture replay adds ``REPLAY_TIME_OFFSET_S`` to every header timestamp because
SceneBuilder deliberately rejects timestamp zero.
"""

from __future__ import annotations

import dataclasses
import math
import pathlib

import numpy as np

from . import lanelet_query as lq, representation as rep, scenario

S = scenario.STATE_COL

_EGO_TRACK_ID = "__ego__"
_MAP_CENTER_OFFSET = (30.0, 0.0)
_TIMESTAMP_EPSILON = 0.0
_MAP_DIRECTION_EPSILON = 1.0e-6
REPLAY_TIME_OFFSET_S = 1.0


@dataclasses.dataclass(frozen=True)
class _ContextTrack:
    detection_id: str
    object_type: str
    track_index: int
    source_steps: tuple[int, ...]
    is_sdc: bool = False


@dataclasses.dataclass(frozen=True)
class _MapChunk:
    points: np.ndarray
    directions: np.ndarray
    center: np.ndarray


@dataclasses.dataclass(frozen=True)
class TargetSelection:
    """One accepted packed target and its row in ground_truth.npz."""

    slot: int
    gt_row: int
    track_index: int
    waymo_id: str
    scene_builder_id: str


def target_ids(scen: scenario.Scenario) -> list[str]:
    """Return accepted packed target IDs in pinned order."""
    return [
        item.scene_builder_id
        for item in target_selection(scen)
    ]


def raw_target_ids(scen: scenario.Scenario) -> list[str]:
    """Return accepted Waymo IDs in the same slot order as :func:`target_ids`."""
    return [item.waymo_id for item in target_selection(scen)]


def _target_detection_id(
    scen: scenario.Scenario,
    track_index: int,
) -> str:
    """Map Waymo's SDC identity to SceneBuilder's sole reserved ego identity."""
    if track_index == scen.sdc_track_index:
        return _EGO_TRACK_ID
    return str(scen.tracks[track_index].object_id)


def write_replay_sidecars(
    fixture_dir: pathlib.Path,
    scen: scenario.Scenario,
) -> None:
    """Write the dependency-free metadata consumed by the C++ parity test."""
    fixture_dir = pathlib.Path(fixture_dir)
    fixture_dir.mkdir(parents=True, exist_ok=True)
    selection = target_selection(scen)
    ids = [item.scene_builder_id for item in selection]
    (fixture_dir / "target_ids.txt").write_text(
        "".join(f"{detection_id}\n" for detection_id in ids),
        encoding="utf-8",
    )
    (fixture_dir / "target_id_map.tsv").write_text(
        "slot\tgt_row\twaymo_id\tscene_builder_id\n"
        + "".join(
            f"{item.slot}\t{item.gt_row}\t{item.waymo_id}\t"
            f"{item.scene_builder_id}\n"
            for item in selection
        ),
        encoding="utf-8",
    )
    (fixture_dir / "replay_step.txt").write_text(
        f"{scen.current_time_index}\n",
        encoding="utf-8",
    )


def _ros_time(timestamp_s: float) -> float:
    """Round like ROS serialization and apply the fixture replay epoch offset."""
    offset_ns = int(round(REPLAY_TIME_OFFSET_S * 1.0e9))
    return (
        round(float(timestamp_s) * 1.0e9) + offset_ns
    ) / 1.0e9


def _resampled_slots(scen: scenario.Scenario, step: int) -> np.ndarray:
    """SceneBuilder's fixed history timestamps, oldest first."""
    now = _ros_time(scen.timestamps_s[step])
    offsets = np.arange(rep.HISTORY_STEPS - 1, -1, -1, dtype=np.float64)
    return now - offsets / rep.HISTORY_RATE_HZ


def _validate_step(scen: scenario.Scenario, step: int) -> None:
    if not 0 <= step < len(scen.timestamps_s):
        raise IndexError(f"step {step} is outside scenario timestamps")
    if scen.sdc_track_index not in range(len(scen.tracks)):
        raise ValueError("sdc_track_index is outside scenario tracks")
    if any(step >= len(track.states) for track in scen.tracks):
        raise ValueError(f"not every track has a state at step {step}")
    timestamp = _ros_time(scen.timestamps_s[step])
    if not math.isfinite(timestamp) or timestamp <= _TIMESTAMP_EPSILON:
        raise ValueError("reference step must have a finite, positive timestamp")


def _normal_source_steps(
    scen: scenario.Scenario,
    track: scenario.Track,
    step: int,
    oldest_allowed: float,
) -> tuple[int, ...]:
    if track.object_type not in rep.MTR_TYPE_CODES:
        return ()
    output = []
    for source_step in range(step + 1):
        timestamp = _ros_time(scen.timestamps_s[source_step])
        state = track.states[source_step]
        if (
            timestamp > _TIMESTAMP_EPSILON
            and timestamp >= oldest_allowed
            and state[S["valid"]] != 0.0
        ):
            output.append(source_step)
    return tuple(output)


def _context_tracks(
    scen: scenario.Scenario,
    step: int,
    mandatory_ids: set[str],
) -> list[_ContextTrack]:
    """Rank retained SceneBuilder histories, including the synthetic ego row."""
    now = _ros_time(scen.timestamps_s[step])
    window = (rep.HISTORY_STEPS - 1) / rep.HISTORY_RATE_HZ
    oldest_allowed = now - window - 0.5 / rep.HISTORY_RATE_HZ

    context: list[_ContextTrack] = []
    for track_index, track in enumerate(scen.tracks):
        # Fixture bags omit the raw Waymo SDC detection. SceneBuilder injects
        # exactly one ego identity below from pose/odometry.
        if track_index == scen.sdc_track_index:
            continue
        source_steps = _normal_source_steps(
            scen, track, step, oldest_allowed
        )
        if source_steps:
            context.append(
                _ContextTrack(
                    detection_id=str(track.object_id),
                    object_type=track.object_type,
                    track_index=track_index,
                    source_steps=source_steps,
                )
            )

    # build() injects ego pose/odometry at every replayed positive timestamp,
    # independently of whether Waymo's ordinary SDC detection is valid.
    ego_steps = tuple(
        source_step
        for source_step in range(step + 1)
        if (
            _ros_time(scen.timestamps_s[source_step]) > _TIMESTAMP_EPSILON
            and _ros_time(scen.timestamps_s[source_step]) >= oldest_allowed
        )
    )
    if ego_steps:
        context.append(
            _ContextTrack(
                detection_id=_EGO_TRACK_ID,
                object_type="vehicle",
                track_index=scen.sdc_track_index,
                source_steps=ego_steps,
                is_sdc=True,
            )
        )

    ego_state = scen.tracks[scen.sdc_track_index].states[step]

    def rank(item: _ContextTrack) -> tuple[float, str]:
        latest = scen.tracks[item.track_index].states[item.source_steps[-1]]
        distance_sq = (
            (float(latest[S["x"]]) - float(ego_state[S["x"]])) ** 2
            + (float(latest[S["y"]]) - float(ego_state[S["y"]])) ** 2
        )
        return distance_sq, item.detection_id

    context.sort(key=rank)
    if len(context) <= rep.CONTEXT_CAPACITY:
        return context

    retained = context[: rep.CONTEXT_CAPACITY]
    retained_ids = {item.detection_id for item in retained}
    by_id = {item.detection_id: item for item in context}
    for mandatory_id in mandatory_ids:
        if mandatory_id in retained_ids or mandatory_id not in by_id:
            continue
        eviction = next(
            (
                index
                for index in range(len(retained) - 1, -1, -1)
                if retained[index].detection_id not in mandatory_ids
            ),
            None,
        )
        if eviction is None:
            raise ValueError("mandatory targets exceed context capacity")
        retained_ids.remove(retained[eviction].detection_id)
        retained[eviction] = by_id[mandatory_id]
        retained_ids.add(mandatory_id)

    retained.sort(key=rank)
    return retained


def target_selection(
    scen: scenario.Scenario,
    step: int | None = None,
) -> list[TargetSelection]:
    """Mirror pinned selection and retain the ground-truth row mapping."""
    step = scen.current_time_index if step is None else step
    selected: list[TargetSelection] = []
    accepted: set[str] = set()
    for gt_row, track_index in enumerate(scen.tracks_to_predict):
        if len(selected) >= rep.TARGET_CAPACITY:
            break
        if track_index not in range(len(scen.tracks)):
            raise ValueError(
                f"target track index {track_index} is outside scenario tracks"
            )
        track = scen.tracks[track_index]
        detection_id = _target_detection_id(scen, track_index)
        state = track.states[step]
        if detection_id in accepted:
            continue
        is_sdc = track_index == scen.sdc_track_index
        if not is_sdc and (
            track.object_type not in rep.MTR_TYPE_CODES
            or state[S["valid"]] == 0.0
        ):
            continue
        selected.append(
            TargetSelection(
                slot=len(selected),
                gt_row=gt_row,
                track_index=track_index,
                waymo_id=str(track.object_id),
                scene_builder_id=detection_id,
            )
        )
        accepted.add(detection_id)
    return selected


def _nearest_source_step(
    scen: scenario.Scenario,
    source_steps: tuple[int, ...],
    desired_time: float,
) -> int | None:
    """Mirror nearestSampleForSlot, including its later-sample tie break."""
    if not source_steps or desired_time <= _TIMESTAMP_EPSILON:
        return None
    tolerance = 0.5 / rep.HISTORY_RATE_HZ
    best: int | None = None
    best_delta = 0.0
    best_timestamp = 0.0
    for source_step in source_steps:
        timestamp = _ros_time(scen.timestamps_s[source_step])
        delta = abs(timestamp - desired_time)
        if delta > tolerance:
            continue
        if (
            best is None
            or delta < best_delta
            or (delta == best_delta and timestamp > best_timestamp)
        ):
            best = source_step
            best_delta = delta
            best_timestamp = timestamp
    return best


def _track_state(
    scen: scenario.Scenario,
    context_track: _ContextTrack,
    source_step: int,
) -> tuple[np.ndarray, tuple[float, float, float]]:
    state = scen.tracks[context_track.track_index].states[source_step]
    dimensions = (
        rep.EGO_DIMS
        if context_track.is_sdc
        else (
            float(state[S["length"]]),
            float(state[S["width"]]),
            float(state[S["height"]]),
        )
    )
    return state, dimensions


def _pack_objects(
    scen: scenario.Scenario,
    step: int,
    targets: list[int],
    context: list[_ContextTrack],
) -> dict[str, np.ndarray]:
    obj_trajs = np.zeros(
        (
            rep.TARGET_CAPACITY,
            rep.CONTEXT_CAPACITY,
            rep.HISTORY_STEPS,
            rep.OBJ_FEATURE_DIM,
        ),
        dtype=np.float32,
    )
    obj_mask = np.zeros(
        (
            rep.TARGET_CAPACITY,
            rep.CONTEXT_CAPACITY,
            rep.HISTORY_STEPS,
        ),
        dtype=np.uint8,
    )
    last_pos = np.zeros(
        (rep.TARGET_CAPACITY, rep.CONTEXT_CAPACITY, 3),
        dtype=np.float32,
    )
    track_index = np.zeros(rep.TARGET_CAPACITY, dtype=np.int64)
    type_ids = np.zeros(rep.TARGET_CAPACITY, dtype=np.int64)

    context_index = {
        item.detection_id: index for index, item in enumerate(context)
    }
    slots = _resampled_slots(scen, step)
    now = _ros_time(scen.timestamps_s[step])

    for target_slot, target_track_index in enumerate(targets):
        target = scen.tracks[target_track_index]
        target_id = _target_detection_id(scen, target_track_index)
        if target_id not in context_index:
            raise ValueError(
                f"pinned target {target_id} falls outside the "
                f"{rep.CONTEXT_CAPACITY}-track context"
            )
        target_state = target.states[step]
        cx = float(target_state[S["x"]])
        cy = float(target_state[S["y"]])
        cz = float(target_state[S["z"]])
        heading = float(target_state[S["heading"]])
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        type_ids[target_slot] = rep.MTR_TYPE_CODES[target.object_type]
        track_index[target_slot] = context_index[target_id]

        for context_slot, context_track in enumerate(context):
            sources = [
                _nearest_source_step(
                    scen, context_track.source_steps, float(desired_time)
                )
                for desired_time in slots
            ]
            present = np.asarray(
                [source is not None for source in sources], dtype=bool
            )
            relative_velocity = np.zeros(
                (rep.HISTORY_STEPS, 2), dtype=np.float64
            )
            for history_slot, source_step in enumerate(sources):
                if source_step is None:
                    continue
                state, _ = _track_state(
                    scen, context_track, source_step
                )
                vx = float(state[S["vx"]])
                vy = float(state[S["vy"]])
                relative_velocity[history_slot] = (
                    vx * cos_h + vy * sin_h,
                    -vx * sin_h + vy * cos_h,
                )

            acceleration = np.zeros(
                (rep.HISTORY_STEPS, 2), dtype=np.float64
            )
            acceleration_valid = np.zeros(rep.HISTORY_STEPS, dtype=bool)
            for history_slot in range(1, rep.HISTORY_STEPS):
                if present[history_slot] and present[history_slot - 1]:
                    acceleration[history_slot] = (
                        relative_velocity[history_slot]
                        - relative_velocity[history_slot - 1]
                    ) * rep.HISTORY_RATE_HZ
                    acceleration_valid[history_slot] = True

            for history_slot in range(rep.HISTORY_STEPS):
                if not present[history_slot]:
                    continue
                if not acceleration_valid[history_slot]:
                    later = np.flatnonzero(
                        acceleration_valid[history_slot + 1 :]
                    )
                    if later.size:
                        acceleration[history_slot] = acceleration[
                            history_slot + 1 + int(later[0])
                        ]
                break

            for history_slot, source_step in enumerate(sources):
                if source_step is None:
                    continue
                state, dimensions = _track_state(
                    scen, context_track, source_step
                )
                obj_mask[
                    target_slot, context_slot, history_slot
                ] = np.uint8(1)

                px = float(state[S["x"]]) - cx
                py = float(state[S["y"]]) - cy
                relative_heading = _wrap_pi(
                    float(state[S["heading"]]) - heading
                )
                row = obj_trajs[
                    target_slot, context_slot, history_slot
                ]
                row[rep.OBJ_COL["position"]] = (
                    px * cos_h + py * sin_h,
                    -px * sin_h + py * cos_h,
                    float(state[S["z"]]) - cz,
                )
                row[rep.OBJ_COL["size"]] = dimensions
                type_code = rep.MTR_TYPE_CODES[context_track.object_type]
                row[6 + type_code] = 1.0
                row[rep.OBJ_COL["is_center"]] = (
                    1.0 if context_track.detection_id == target_id else 0.0
                )
                row[rep.OBJ_COL["is_sdc"]] = (
                    1.0 if context_track.is_sdc else 0.0
                )
                row[11 + history_slot] = 1.0
                row[rep.OBJ_COL["scalar_time"]] = (
                    float(slots[history_slot]) - now
                )
                row[rep.OBJ_COL["heading"]] = (
                    math.sin(relative_heading),
                    math.cos(relative_heading),
                )
                row[rep.OBJ_COL["velocity"]] = relative_velocity[
                    history_slot
                ]
                row[rep.OBJ_COL["acceleration"]] = acceleration[
                    history_slot
                ]

            for history_slot in range(rep.HISTORY_STEPS - 1, -1, -1):
                source_step = sources[history_slot]
                if source_step is None:
                    continue
                state, _ = _track_state(scen, context_track, source_step)
                px = float(state[S["x"]]) - cx
                py = float(state[S["y"]]) - cy
                last_pos[target_slot, context_slot] = (
                    px * cos_h + py * sin_h,
                    -px * sin_h + py * cos_h,
                    float(state[S["z"]]) - cz,
                )
                break

    return {
        "obj_trajs": obj_trajs,
        "obj_trajs_mask": obj_mask,
        "obj_trajs_last_pos": last_pos,
        "track_index_to_predict": track_index,
        "center_type_ids": type_ids,
    }


def _map_chunks(polylines: list[scenario.MapPolyline]) -> list[_MapChunk]:
    chunks: list[_MapChunk] = []
    for polyline in polylines:
        points = np.asarray(polyline.points, dtype=np.float64)
        if (
            points.ndim != 2
            or points.shape[1] < 3
            or len(points) < 2
            or not np.isfinite(points[:, :3]).all()
        ):
            continue
        points = points[:, :3]
        directions = np.zeros_like(points)
        differences = points[1:] - points[:-1]
        norms = np.linalg.norm(differences, axis=1)
        valid = norms > _MAP_DIRECTION_EPSILON
        directions[1:][valid] = differences[valid] / norms[valid, None]

        for start in range(0, len(points), rep.POINTS_PER_POLYLINE):
            stop = start + rep.POINTS_PER_POLYLINE
            chunk_points = points[start:stop]
            chunk_directions = directions[start:stop]
            chunks.append(
                _MapChunk(
                    points=chunk_points,
                    directions=chunk_directions,
                    center=chunk_points.mean(axis=0),
                )
            )
    return chunks


def _pack_map(
    scen: scenario.Scenario,
    step: int,
    targets: list[int],
    config: lq.QueryConfig,
) -> dict[str, np.ndarray]:
    polylines = np.zeros(
        (
            rep.TARGET_CAPACITY,
            rep.NUM_SRC_POLYLINES,
            rep.POINTS_PER_POLYLINE,
            rep.MAP_FEATURE_DIM,
        ),
        dtype=np.float32,
    )
    mask = np.zeros(
        (
            rep.TARGET_CAPACITY,
            rep.NUM_SRC_POLYLINES,
            rep.POINTS_PER_POLYLINE,
        ),
        dtype=np.uint8,
    )
    centers = np.zeros(
        (rep.TARGET_CAPACITY, rep.NUM_SRC_POLYLINES, 3),
        dtype=np.float32,
    )

    ego = scen.tracks[scen.sdc_track_index].states[step]
    kept, _ = lq.query(
        scen.map_polylines,
        (float(ego[S["x"]]), float(ego[S["y"]])),
        float(ego[S["heading"]]),
        config,
    )
    chunks = _map_chunks(kept)

    for target_slot, target_track_index in enumerate(targets):
        center = scen.tracks[target_track_index].states[step]
        cx = float(center[S["x"]])
        cy = float(center[S["y"]])
        cz = float(center[S["z"]])
        heading = float(center[S["heading"]])
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        offset_x = (
            cx
            + _MAP_CENTER_OFFSET[0] * cos_h
            - _MAP_CENTER_OFFSET[1] * sin_h
        )
        offset_y = (
            cy
            + _MAP_CENTER_OFFSET[0] * sin_h
            + _MAP_CENTER_OFFSET[1] * cos_h
        )
        ranked = sorted(
            range(len(chunks)),
            key=lambda index: (
                float(chunks[index].center[0]) - offset_x
            )
            ** 2
            + (
                float(chunks[index].center[1]) - offset_y
            )
            ** 2,
        )[: rep.NUM_SRC_POLYLINES]

        for polyline_slot, chunk_index in enumerate(ranked):
            chunk = chunks[chunk_index]
            count = len(chunk.points)
            local_positions = np.empty((count, 3), dtype=np.float64)
            local_directions = np.empty((count, 3), dtype=np.float64)
            for point_index, point in enumerate(chunk.points):
                dx = float(point[0]) - cx
                dy = float(point[1]) - cy
                local_positions[point_index] = (
                    dx * cos_h + dy * sin_h,
                    -dx * sin_h + dy * cos_h,
                    float(point[2]) - cz,
                )
                direction = chunk.directions[point_index]
                local_directions[point_index] = (
                    float(direction[0]) * cos_h
                    + float(direction[1]) * sin_h,
                    -float(direction[0]) * sin_h
                    + float(direction[1]) * cos_h,
                    float(direction[2]),
                )

            rows = polylines[target_slot, polyline_slot, :count]
            rows[:, rep.MAP_COL["position"]] = local_positions
            rows[:, rep.MAP_COL["direction"]] = local_directions
            rows[:, rep.MAP_COL["global_type"]] = rep.WATO_GLOBAL_TYPE
            rows[:, rep.MAP_COL["pre_position"]] = local_positions[
                np.maximum(np.arange(count) - 1, 0), :2
            ]
            mask[target_slot, polyline_slot, :count] = np.uint8(1)
            centers[target_slot, polyline_slot] = local_positions.mean(axis=0)

    return {
        "map_polylines": polylines,
        "map_polylines_mask": mask,
        "map_polylines_center": centers,
    }


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pack(
    scen: scenario.Scenario,
    config: lq.QueryConfig,
    step: int | None = None,
) -> dict[str, np.ndarray]:
    """Pack the eight fixed-shape MTR input tensors at one replay step."""
    step = scen.current_time_index if step is None else step
    _validate_step(scen, step)
    selection = target_selection(scen, step)
    targets = [item.track_index for item in selection]
    mandatory_ids = {
        _target_detection_id(scen, index) for index in targets
    }
    context = _context_tracks(scen, step, mandatory_ids)
    objects = _pack_objects(scen, step, targets, context)
    maps = _pack_map(scen, step, targets, config)
    return {**objects, **maps}
