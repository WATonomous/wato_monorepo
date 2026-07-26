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
"""Waymo Scenario protos flattened into plain dataclasses.

Everything downstream consumes these rather than the protos, so the converter,
the ablation, and the ledger share one representation of the source data.
Imports protobuf but neither ROS nor torch, so it loads in both environments.
"""

from __future__ import annotations

import dataclasses
import pathlib
from collections.abc import Iterator

import numpy as np
from waymo_open_dataset.protos import scenario_pb2

from . import tfrecord

STATE_COL = {
    "x": 0,
    "y": 1,
    "z": 2,
    "length": 3,
    "width": 4,
    "height": 5,
    "heading": 6,
    "vx": 7,
    "vy": 8,
    "valid": 9,
}
NUM_STATE_COLS = len(STATE_COL)

# Waymo Track.ObjectType -> the three types MTR predicts. Anything else is
# "other"; SceneBuilder's parseObjectClass returns nullopt for those, so they
# are dropped. The ledger counts the drops rather than letting them go silent.
_OBJECT_TYPE = {1: "vehicle", 2: "pedestrian", 3: "cyclist"}

_MAP_FEATURE_FIELDS = (
    ("lane", "lane"),
    ("road_line", "road_line"),
    ("road_edge", "road_edge"),
    ("crosswalk", "crosswalk"),
    ("speed_bump", "speed_bump"),
    ("driveway", "driveway"),
)


@dataclasses.dataclass(frozen=True)
class Track:
    """One Waymo agent and its complete state history."""

    object_id: int
    object_type: str
    states: np.ndarray  # [91, 10]


@dataclasses.dataclass(frozen=True)
class MapPolyline:
    """One typed Waymo map feature flattened to XYZ points."""

    feature_id: int
    feature_type: str
    points: np.ndarray  # [N, 3]


@dataclasses.dataclass(frozen=True)
class Scenario:
    """The subset of a Waymo Scenario shared by every harness rung."""

    scenario_id: str
    timestamps_s: np.ndarray
    current_time_index: int
    sdc_track_index: int
    tracks: list[Track]
    tracks_to_predict: list[int]
    map_polylines: list[MapPolyline]


def _track_from_proto(proto) -> Track:
    states = np.zeros((len(proto.states), NUM_STATE_COLS), dtype=np.float64)
    for index, state in enumerate(proto.states):
        states[index] = (
            state.center_x,
            state.center_y,
            state.center_z,
            state.length,
            state.width,
            state.height,
            state.heading,
            state.velocity_x,
            state.velocity_y,
            1.0 if state.valid else 0.0,
        )
    return Track(
        object_id=proto.id,
        object_type=_OBJECT_TYPE.get(proto.object_type, "other"),
        states=states,
    )


def _feature_points(feature_data) -> np.ndarray | None:
    for field_name in ("polyline", "polygon"):
        points = getattr(feature_data, field_name, None)
        if points is not None and len(points) > 0:
            return np.asarray([[p.x, p.y, p.z] for p in points], dtype=np.float64)
    return None


def _polylines_from_proto(proto) -> list[MapPolyline]:
    output: list[MapPolyline] = []
    for feature in proto.map_features:
        selected_field = feature.WhichOneof("feature_data")
        for field_name, label in _MAP_FEATURE_FIELDS:
            if selected_field != field_name:
                continue
            points = _feature_points(getattr(feature, field_name))
            if points is not None:
                output.append(
                    MapPolyline(
                        feature_id=feature.id,
                        feature_type=label,
                        points=points,
                    )
                )
            break
    return output


def load_scenarios(path: pathlib.Path, limit: int | None = None) -> Iterator[Scenario]:
    """Parse scenarios from a Waymo Motion TFRecord shard."""
    for index, payload in enumerate(tfrecord.read_records(path)):
        if limit is not None and index >= limit:
            return
        proto = scenario_pb2.Scenario()
        proto.ParseFromString(payload)
        yield Scenario(
            scenario_id=proto.scenario_id,
            timestamps_s=np.asarray(proto.timestamps_seconds, dtype=np.float64),
            current_time_index=proto.current_time_index,
            sdc_track_index=proto.sdc_track_index,
            tracks=[_track_from_proto(track) for track in proto.tracks],
            tracks_to_predict=[
                target.track_index for target in proto.tracks_to_predict
            ],
            map_polylines=_polylines_from_proto(proto),
        )
