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
"""Rung 4: score recorded full-runtime ROS output against fixture ground truth.

The scoring core is ROS-free. ROS imports are lazy and isolated to the two bag
reader functions, so unit tests and report inspection work in the model
environment. A run manifest records the exact pinned target IDs used for each
replay; they must equal the fixture sidecar or the comparison is rejected.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import pathlib
import re
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from . import metrics, representation as rep

OUTPUT_TOPIC = "/world_modeling/world_object_seeds"
TRACKS_TOPIC = "/perception/detections_3D_tracked"


@dataclasses.dataclass(frozen=True)
class RecordedMode:
    confidence: float
    source_stamp_ns: int
    pose_stamps_ns: np.ndarray
    xy: np.ndarray


@dataclasses.dataclass(frozen=True)
class RecordedObject:
    detection_id: str
    record_stamp_ns: int
    modes: tuple[RecordedMode, ...]


@dataclasses.dataclass(frozen=True)
class Rung4AgentRow:
    scenario_id: str
    waymo_id: str
    runtime_id: str
    minade: float
    minfde: float
    rung3_minfde: float
    delta_minfde: float
    mode_count: int


@dataclasses.dataclass(frozen=True)
class Rung4Report:
    paired_minfde: metrics.PairedDelta
    per_agent: tuple[Rung4AgentRow, ...]
    num_scenarios: int
    fallback_agents: int

    @property
    def mean_minade(self) -> float:
        return float(np.mean([row.minade for row in self.per_agent]))

    @property
    def mean_minfde(self) -> float:
        return float(np.mean([row.minfde for row in self.per_agent]))

    def gate_ok(self) -> bool:
        return bool(
            self.per_agent
            and self.paired_minfde.count == len(self.per_agent)
            and math.isfinite(self.paired_minfde.near_zero_fraction)
            and self.paired_minfde.near_zero_fraction >= 0.99
        )

    def format(self) -> str:
        return "\n\n".join(
            [
                f"Rung 4 — {self.num_scenarios} scenarios, "
                f"{len(self.per_agent)} pinned targets",
                f"deployed mean minADE={self.mean_minade:.4f} m, "
                f"mean minFDE={self.mean_minfde:.4f} m",
                f"one-mode fallback targets: {self.fallback_agents}",
                metrics.format_paired_delta(
                    self.paired_minfde, "R3-final -> R4  minFDE"
                ),
                f"RUNG 4 GATE: {'PASS' if self.gate_ok() else 'FAIL'}",
            ]
        )


@dataclasses.dataclass(frozen=True)
class _Target:
    slot: int
    gt_row: int
    waymo_id: str
    runtime_id: str


@dataclasses.dataclass(frozen=True)
class _RunConfig:
    recordings: dict[str, pathlib.Path]
    pinned_target_ids: dict[str, tuple[str, ...]]
    rung3_final_report: pathlib.Path
    topic: str


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _storage_id(bag: pathlib.Path) -> str:
    metadata = bag / "metadata.yaml"
    if not metadata.is_file():
        return ""
    match = re.search(
        r"^\s*storage_identifier:\s*(\S+)\s*$",
        metadata.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    return match.group(1) if match else ""


def _open_reader(bag: pathlib.Path):
    try:
        import rosbag2_py
    except ImportError as error:
        raise RuntimeError(
            "reading rung-4 bags requires the ROS environment with rosbag2_py"
        ) from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(pathlib.Path(bag).expanduser().resolve()),
            storage_id=_storage_id(pathlib.Path(bag)),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def read_recorded_objects(
    bag: pathlib.Path,
    topic: str = OUTPUT_TOPIC,
) -> list[RecordedObject]:
    """Normalize every published object sample from a recorded ROS bag."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "reading rung-4 bags requires sourced ROS message packages"
        ) from error

    reader = _open_reader(pathlib.Path(bag))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"{bag} does not contain required topic {topic!r}")
    message_type = get_message(topic_types[topic])
    output: list[RecordedObject] = []
    while reader.has_next():
        current_topic, serialized, record_stamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        for obj in message.objects:
            modes = []
            for prediction in obj.predictions:
                modes.append(
                    RecordedMode(
                        confidence=float(prediction.conf),
                        source_stamp_ns=_stamp_ns(prediction.header.stamp),
                        pose_stamps_ns=np.asarray(
                            [_stamp_ns(pose.header.stamp) for pose in prediction.poses],
                            dtype=np.int64,
                        ),
                        xy=np.asarray(
                            [
                                [pose.pose.position.x, pose.pose.position.y]
                                for pose in prediction.poses
                            ],
                            dtype=np.float64,
                        ),
                    )
                )
            output.append(
                RecordedObject(
                    detection_id=str(obj.detection.id),
                    record_stamp_ns=int(record_stamp_ns),
                    modes=tuple(modes),
                )
            )
    return output


def read_replay_source_stamp(fixture: pathlib.Path) -> int:
    """Read the tracked-object source stamp at the fixture's scoring step."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "reading fixture replay timestamps requires sourced ROS messages"
        ) from error

    fixture = pathlib.Path(fixture)
    replay_step = int((fixture / "replay_step.txt").read_text().strip())
    reader = _open_reader(fixture)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if TRACKS_TOPIC not in topic_types:
        raise ValueError(f"{fixture} does not contain {TRACKS_TOPIC!r}")
    message_type = get_message(topic_types[TRACKS_TOPIC])
    step = 0
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != TRACKS_TOPIC:
            continue
        if step == replay_step:
            message = deserialize_message(serialized, message_type)
            return _stamp_ns(message.header.stamp)
        step += 1
    raise ValueError(
        f"{fixture} has only {step} tracked-object frames; "
        f"cannot read replay step {replay_step}"
    )


def _read_targets(fixture: pathlib.Path, gt_ids: np.ndarray) -> list[_Target]:
    path = fixture / "target_id_map.tsv"
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        expected = ["slot", "gt_row", "waymo_id", "scene_builder_id"]
        if reader.fieldnames != expected:
            raise ValueError(f"{path} header must be {expected}")
        rows = list(reader)
    if not rows or len(rows) > rep.TARGET_CAPACITY:
        raise ValueError(f"{path} must contain 1..{rep.TARGET_CAPACITY} targets")
    ground_truth_ids = [str(value) for value in gt_ids.tolist()]
    targets = []
    for expected_slot, row in enumerate(rows):
        slot = int(row["slot"])
        gt_row = int(row["gt_row"])
        if slot != expected_slot or not 0 <= gt_row < len(ground_truth_ids):
            raise ValueError(f"{path} has invalid slot/gt_row at row {expected_slot}")
        if ground_truth_ids[gt_row] != row["waymo_id"]:
            raise ValueError(f"{path} row {slot} does not match ground_truth.npz")
        targets.append(
            _Target(
                slot=slot,
                gt_row=gt_row,
                waymo_id=row["waymo_id"],
                runtime_id=row["scene_builder_id"],
            )
        )
    if len({target.runtime_id for target in targets}) != len(targets):
        raise ValueError(f"{path} contains duplicate runtime target IDs")
    return targets


def _validate_mode(mode: RecordedMode, source_stamp_ns: int) -> None:
    expected_shape = (rep.DEPLOYED_STEPS, 2)
    if mode.xy.shape != expected_shape or not np.isfinite(mode.xy).all():
        raise ValueError(
            f"recorded trajectory must be finite {expected_shape}, got {mode.xy.shape}"
        )
    if not math.isfinite(mode.confidence) or not 0.0 < mode.confidence <= 1.0:
        raise ValueError(f"invalid recorded mode confidence {mode.confidence}")
    expected_stamps = source_stamp_ns + (
        np.arange(1, rep.DEPLOYED_STEPS + 1, dtype=np.int64)
        * int(round(rep.DEPLOYED_STEP_S * 1_000_000_000))
    )
    if mode.source_stamp_ns != source_stamp_ns:
        raise ValueError("recorded prediction has the wrong source stamp")
    if not np.array_equal(mode.pose_stamps_ns, expected_stamps):
        raise ValueError("recorded prediction timestamps do not match deployed cadence")


def _select_predictions(
    recorded: Sequence[RecordedObject],
    runtime_ids: Sequence[str],
    source_stamp_ns: int,
) -> dict[str, tuple[RecordedMode, ...]]:
    """Select one publication per pin, preferring MTR's multi-mode output."""
    wanted = set(runtime_ids)
    selected: dict[str, tuple[RecordedMode, ...]] = {}
    for obj in recorded:
        if obj.detection_id not in wanted or not obj.modes:
            continue
        source_modes = tuple(
            mode for mode in obj.modes if mode.source_stamp_ns == source_stamp_ns
        )
        if not source_modes:
            continue
        previous = selected.get(obj.detection_id)
        if previous is None or len(source_modes) >= len(previous):
            selected[obj.detection_id] = source_modes
    missing = wanted - selected.keys()
    if missing:
        raise ValueError(
            "recorded output is missing pinned target IDs at the scoring stamp: "
            f"{sorted(missing)}"
        )
    return selected


def score_scenario(
    fixture: pathlib.Path,
    recorded: Sequence[RecordedObject],
    source_stamp_ns: int,
    rung3_minfde: Mapping[tuple[str, str], float],
) -> list[Rung4AgentRow]:
    """Pure scoring core for one normalized runtime recording."""
    fixture = pathlib.Path(fixture)
    scenario_id = fixture.name
    with np.load(fixture / "ground_truth.npz", allow_pickle=False) as data:
        gt = np.asarray(data["gt"], dtype=np.float64)
        valid = np.asarray(data["valid"], dtype=bool)
        targets = _read_targets(fixture, data["agent_ids"])
    gt = gt[:, 1 : 1 + rep.DEPLOYED_STEPS * 2 : 2]
    valid = valid[:, 1 : 1 + rep.DEPLOYED_STEPS * 2 : 2]
    selected = _select_predictions(
        recorded, [target.runtime_id for target in targets], source_stamp_ns
    )

    rows = []
    for target in targets:
        key = (scenario_id, target.waymo_id)
        if key not in rung3_minfde:
            raise ValueError(f"rung3-final report is missing target {key}")
        modes = selected[target.runtime_id]
        for mode in modes:
            _validate_mode(mode, source_stamp_ns)
        pred = np.stack([mode.xy for mode in modes])[None, ...]
        target_gt = gt[target.gt_row][None, ...]
        target_valid = valid[target.gt_row][None, ...]
        minade = float(metrics.min_ade(pred, target_gt, target_valid)[0])
        minfde = float(metrics.min_fde(pred, target_gt, target_valid)[0])
        reference = float(rung3_minfde[key])
        rows.append(
            Rung4AgentRow(
                scenario_id=scenario_id,
                waymo_id=target.waymo_id,
                runtime_id=target.runtime_id,
                minade=minade,
                minfde=minfde,
                rung3_minfde=reference,
                delta_minfde=minfde - reference,
                mode_count=len(modes),
            )
        )
    return rows


def _load_config(path: pathlib.Path) -> _RunConfig:
    path = pathlib.Path(path).expanduser().resolve()
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("rung4 node config must be a JSON object")
    base = path.parent
    recordings_raw = raw.get("recordings")
    pins_raw = raw.get("pinned_target_ids")
    if not isinstance(recordings_raw, dict) or not isinstance(pins_raw, dict):
        raise ValueError("recordings and pinned_target_ids must be JSON objects")
    if any(
        not isinstance(value, list)
        or not value
        or any(not isinstance(item, str) or not item for item in value)
        for value in pins_raw.values()
    ):
        raise ValueError("each pinned_target_ids value must be a nonempty string list")

    def resolve(value: Any) -> pathlib.Path:
        return (base / str(value)).expanduser().resolve()

    return _RunConfig(
        recordings={str(key): resolve(value) for key, value in recordings_raw.items()},
        pinned_target_ids={
            str(key): tuple(str(item) for item in value)
            for key, value in pins_raw.items()
        },
        rung3_final_report=resolve(raw["rung3_final_report"]),
        topic=str(raw.get("topic", OUTPUT_TOPIC)),
    )


def _load_rung3(path: pathlib.Path) -> dict[tuple[str, str], float]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(raw, dict)
        or raw.get("schema_version") != 1
        or raw.get("rung") != "rung3"
    ):
        raise ValueError("rung3-final report must use rung3 schema version 1")
    rows = raw.get("per_agent")
    if not isinstance(rows, dict):
        raise ValueError("rung3-final report must contain a per_agent object")
    output: dict[tuple[str, str], float] = {}
    for serialized_key, row in rows.items():
        if not isinstance(row, dict):
            raise ValueError("rung3 per_agent entries must be objects")
        key = (str(row["scenario_id"]), str(row["waymo_id"]))
        if serialized_key != f"{key[0]}::{key[1]}":
            raise ValueError(
                f"rung3-final key {serialized_key!r} does not match row {key}"
            )
        if key in output:
            raise ValueError(f"duplicate rung3-final target {key}")
        value = float(row["engine_minfde_m"])
        if not math.isfinite(value):
            raise ValueError(f"non-finite rung3-final minFDE for {key}")
        output[key] = value
    return output


def run(
    fixtures: pathlib.Path,
    node_config: pathlib.Path,
) -> Rung4Report:
    """Read recorded bags and produce the paired rung3-final -> rung4 report."""
    fixtures = pathlib.Path(fixtures).expanduser().resolve()
    config = _load_config(node_config)
    fixture_dirs = sorted(
        path for path in fixtures.iterdir() if (path / "ground_truth.npz").is_file()
    )
    scenario_ids = {path.name for path in fixture_dirs}
    if set(config.recordings) != scenario_ids:
        raise ValueError("recording scenario IDs must exactly match fixture IDs")
    if set(config.pinned_target_ids) != scenario_ids:
        raise ValueError("pinned-target scenario IDs must exactly match fixture IDs")

    rung3 = _load_rung3(config.rung3_final_report)
    expected_keys: set[tuple[str, str]] = set()
    all_rows: list[Rung4AgentRow] = []
    for fixture in fixture_dirs:
        with np.load(fixture / "ground_truth.npz", allow_pickle=False) as data:
            targets = _read_targets(fixture, data["agent_ids"])
        expected_pins = tuple(target.runtime_id for target in targets)
        if config.pinned_target_ids[fixture.name] != expected_pins:
            raise ValueError(
                f"{fixture.name} pinned_target_ids must be {list(expected_pins)}"
            )
        expected_keys.update((fixture.name, target.waymo_id) for target in targets)
        source_stamp_ns = read_replay_source_stamp(fixture)
        recorded = read_recorded_objects(
            config.recordings[fixture.name], topic=config.topic
        )
        all_rows.extend(score_scenario(fixture, recorded, source_stamp_ns, rung3))
    if set(rung3) != expected_keys:
        raise ValueError(
            "rung3-final per_agent keys must exactly match fixture targets"
        )

    runtime_fde = np.asarray([row.minfde for row in all_rows])
    rung3_fde = np.asarray([row.rung3_minfde for row in all_rows])
    paired = metrics.paired_delta(
        runtime_fde,
        rung3_fde,
        [row.waymo_id for row in all_rows],
        [row.scenario_id for row in all_rows],
    )
    return Rung4Report(
        paired_minfde=paired,
        per_agent=tuple(all_rows),
        num_scenarios=len(fixture_dirs),
        fallback_agents=sum(row.mode_count == 1 for row in all_rows),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", type=pathlib.Path, required=True)
    parser.add_argument("--node-config", type=pathlib.Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run(args.fixtures, args.node_config)
    print(report.format())
    return 0 if report.gate_ok() else 1


if __name__ == "__main__":
    raise SystemExit(main())
