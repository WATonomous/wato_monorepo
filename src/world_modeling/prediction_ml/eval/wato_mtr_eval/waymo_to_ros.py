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
"""Waymo scenario -> MCAP bag of WATO ROS messages.

One artifact, two consumers: rung 2 reads the bag through rosbag2_cpp, rung 4
replays it with ``ros2 bag play``. Using ROS's own serializer means the Python
writer and the C++ reader agree by construction.

Imports rosbag2_py and ROS messages; must never import torch.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
from collections.abc import Sequence

import numpy as np
import rosbag2_py
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from lanelet_msgs.msg import Lanelet, LaneletAhead
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from . import ledger, lanelet_query as lq, representation as rep, scenario

TOPICS = {
    "tracks": "/perception/detections_3D_tracked",
    "pose": "/localization/pose",
    "odom": "/world_modeling/transform/odometry",
    "lanelet": "lanelet/lanelet_ahead",
}
TYPES = {
    "tracks": "vision_msgs/msg/Detection3DArray",
    "pose": "geometry_msgs/msg/PoseStamped",
    "odom": "nav_msgs/msg/Odometry",
    "lanelet": "lanelet_msgs/msg/LaneletAhead",
}
MAP_FRAME = "map"
EGO_CHILD_FRAME = "base_footprint"
# SceneBuilder treats ROS time zero as "unset". Shift fixture transport stamps
# by a constant epoch while preserving every relative Waymo interval.
ROS_TIME_OFFSET_S = 1.0
ROS_TIME_OFFSET_NS = int(ROS_TIME_OFFSET_S * 1_000_000_000)
S = scenario.STATE_COL
_SELECTORS = {"dense", "sparse", "gappy"}


@dataclasses.dataclass(frozen=True)
class ConversionResult:
    bag_path: pathlib.Path
    ledger: ledger.Ledger
    ground_truth: tuple[np.ndarray, np.ndarray, list[str], list[str]]


def transport_time_ns(timestamp_s: float) -> int:
    """Convert Waymo relative time to a positive ROS timestamp exactly."""
    return int(round(timestamp_s * 1e9)) + ROS_TIME_OFFSET_NS


def _stamp(msg, timestamp_s: float) -> None:
    total_ns = transport_time_ns(timestamp_s)
    seconds, nanoseconds = divmod(total_ns, 1_000_000_000)
    msg.header.stamp.sec = seconds
    msg.header.stamp.nanosec = nanoseconds
    msg.header.frame_id = MAP_FRAME


def _yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def _state_at(
    scen: scenario.Scenario, track: scenario.Track, step: int
) -> np.ndarray:
    if not 0 <= step < len(scen.timestamps_s):
        raise IndexError(f"step {step} is outside scenario timestamps")
    if step >= len(track.states):
        raise IndexError(
            f"track {track.object_id} has no state for scenario step {step}"
        )
    return track.states[step]


def detections_at(scen: scenario.Scenario, step: int) -> Detection3DArray:
    """Tracked detections at one timestep.

    The Waymo SDC is carried by the dedicated pose/odometry topics and becomes
    SceneBuilder's reserved ``__ego__`` history row.  Omitting it here avoids
    representing the same physical actor twice.
    """
    msg = Detection3DArray()
    _stamp(msg, float(scen.timestamps_s[step]))

    for track_index, track in enumerate(scen.tracks):
        if track_index == scen.sdc_track_index:
            continue
        state = _state_at(scen, track, step)
        if (
            state[S["valid"]] == 0.0
            or track.object_type not in rep.CLASS_ID_FOR_TYPE
        ):
            continue
        dimensions = state[[S["length"], S["width"], S["height"]]]
        if not np.isfinite(dimensions).all() or (dimensions <= 0.0).any():
            raise ValueError(
                f"valid track {track.object_id} has non-positive bbox dimensions"
            )

        detection = Detection3D()
        detection.header = msg.header
        detection.id = str(track.object_id)

        box = BoundingBox3D()
        box.center.position.x = float(state[S["x"]])
        box.center.position.y = float(state[S["y"]])
        box.center.position.z = float(state[S["z"]])
        box.center.orientation = _yaw_to_quat(float(state[S["heading"]]))
        box.size.x = float(state[S["length"]])
        box.size.y = float(state[S["width"]])
        box.size.z = float(state[S["height"]])
        detection.bbox = box

        object_class = ObjectHypothesisWithPose()
        object_class.hypothesis.class_id = rep.CLASS_ID_FOR_TYPE[
            track.object_type
        ]
        object_class.hypothesis.score = 1.0

        velocity = ObjectHypothesisWithPose()
        velocity.hypothesis.class_id = rep.VELOCITY_CLASS_ID
        velocity.hypothesis.score = 1.0
        velocity.pose.pose.position.x = float(state[S["vx"]])
        velocity.pose.pose.position.y = float(state[S["vy"]])
        velocity.pose.pose.position.z = 0.0

        detection.results = [object_class, velocity]
        msg.detections.append(detection)

    return msg


def ego_pose_at(scen: scenario.Scenario, step: int) -> PoseStamped:
    track = scen.tracks[scen.sdc_track_index]
    state = _state_at(scen, track, step)
    msg = PoseStamped()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.pose.position.x = float(state[S["x"]])
    msg.pose.position.y = float(state[S["y"]])
    msg.pose.position.z = float(state[S["z"]])
    msg.pose.orientation = _yaw_to_quat(float(state[S["heading"]]))
    return msg


def ego_odometry_at(scen: scenario.Scenario, step: int) -> Odometry:
    """Return odometry whose twist is expressed in the ego body frame."""
    track = scen.tracks[scen.sdc_track_index]
    state = _state_at(scen, track, step)
    heading = float(state[S["heading"]])
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    vx = float(state[S["vx"]])
    vy = float(state[S["vy"]])

    msg = Odometry()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.child_frame_id = EGO_CHILD_FRAME
    msg.pose.pose = ego_pose_at(scen, step).pose
    msg.twist.twist.linear.x = vx * cos_h + vy * sin_h
    msg.twist.twist.linear.y = -vx * sin_h + vy * cos_h
    msg.twist.twist.linear.z = 0.0
    return msg


def lanelet_ahead_at(
    scen: scenario.Scenario,
    step: int,
    config: lq.QueryConfig,
) -> tuple[LaneletAhead, ledger.MapStats]:
    track = scen.tracks[scen.sdc_track_index]
    state = _state_at(scen, track, step)
    kept, stats = lq.query(
        scen.map_polylines,
        (float(state[S["x"]]), float(state[S["y"]])),
        float(state[S["heading"]]),
        config,
    )

    msg = LaneletAhead()
    _stamp(msg, float(scen.timestamps_s[step]))
    msg.current_lanelet_id = -1
    msg.radius_m = config.radius_m
    for polyline in kept:
        lanelet = Lanelet()
        lanelet.id = int(polyline.feature_id)
        lanelet.lanelet_type = "road"
        lanelet.left_lane_id = -1
        lanelet.right_lane_id = -1
        for coordinates in polyline.points:
            lanelet.centerline.append(
                Point(
                    x=float(coordinates[0]),
                    y=float(coordinates[1]),
                    z=float(coordinates[2]),
                )
            )
        lanelet.centerline_curvature = [0.0] * len(lanelet.centerline)
        msg.lanelets.append(lanelet)
    return msg, stats


def ground_truth_of(
    scen: scenario.Scenario,
) -> tuple[np.ndarray, np.ndarray, list[str], list[str]]:
    """Future positions for every designated target over the native horizon."""
    start = scen.current_time_index + 1
    num_targets = len(scen.tracks_to_predict)
    gt = np.zeros(
        (num_targets, rep.NUM_FUTURE_FRAMES, 2), dtype=np.float64
    )
    valid = np.zeros(
        (num_targets, rep.NUM_FUTURE_FRAMES), dtype=bool
    )
    agent_ids: list[str] = []
    types: list[str] = []
    for output_index, track_index in enumerate(scen.tracks_to_predict):
        track = scen.tracks[track_index]
        states = track.states[start : start + rep.NUM_FUTURE_FRAMES]
        available = min(len(states), rep.NUM_FUTURE_FRAMES)
        if available:
            gt[output_index, :available] = states[
                :available, [S["x"], S["y"]]
            ]
            valid[output_index, :available] = (
                states[:available, S["valid"]] > 0.0
            )
        agent_ids.append(str(track.object_id))
        types.append(track.object_type)
    return gt, valid, agent_ids, types


def _write_sidecars(
    bag_path: pathlib.Path,
    conversion_ledger: ledger.Ledger,
    ground_truth: tuple[np.ndarray, np.ndarray, list[str], list[str]],
) -> None:
    conversion_ledger.to_json(bag_path / "ledger.json")
    gt, valid, agent_ids, types = ground_truth
    np.savez_compressed(
        bag_path / "ground_truth.npz",
        gt=gt,
        valid=valid,
        agent_ids=np.asarray(agent_ids),
        types=np.asarray(types),
    )


def convert(
    scen: scenario.Scenario,
    out_dir: pathlib.Path,
    config: lq.QueryConfig,
) -> ConversionResult:
    """Write one scenario to an MCAP bag plus ledger and ground-truth sidecars."""
    if "mcap" not in rosbag2_py.get_registered_writers():
        raise RuntimeError(
            "rosbag2 MCAP storage plugin is not installed; install "
            "ros-humble-rosbag2-storage-mcap before converting fixtures"
        )
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_path = out_dir / scen.scenario_id
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for key, topic in TOPICS.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=TYPES[key],
                serialization_format="cdr",
            )
        )

    current_map_stats = None
    history_valid = 0
    history_total = 0
    history_start = max(
        0, scen.current_time_index - rep.HISTORY_STEPS + 1
    )
    for step, timestamp_s in enumerate(scen.timestamps_s):
        timestamp_ns = transport_time_ns(float(timestamp_s))
        detections = detections_at(scen, step)
        lanelet_msg, map_stats = lanelet_ahead_at(scen, step, config)
        writer.write(
            TOPICS["pose"],
            serialize_message(ego_pose_at(scen, step)),
            timestamp_ns,
        )
        writer.write(
            TOPICS["odom"],
            serialize_message(ego_odometry_at(scen, step)),
            timestamp_ns,
        )
        writer.write(
            TOPICS["lanelet"], serialize_message(lanelet_msg), timestamp_ns
        )
        # The lifecycle node builds immediately when tracks arrive, so all
        # auxiliary state for this timestamp must be visible first.
        writer.write(
            TOPICS["tracks"], serialize_message(detections), timestamp_ns
        )

        if history_start <= step <= scen.current_time_index:
            history_total += len(scen.tracks)
            # Pose/odometry inject exactly one synthetic ego history sample.
            history_valid += len(detections.detections) + 1
        if step == scen.current_time_index:
            current_map_stats = map_stats

    del writer
    if current_map_stats is None:
        raise RuntimeError("scenario current_time_index was not written")

    typed = [
        track
        for track in scen.tracks
        if track.object_type in rep.CLASS_ID_FOR_TYPE
    ]
    dropped: dict[str, int] = {}
    for track in scen.tracks:
        if track.object_type not in rep.CLASS_ID_FOR_TYPE:
            dropped[track.object_type] = dropped.get(track.object_type, 0) + 1

    conversion_ledger = ledger.build_ledger(
        scenario_id=scen.scenario_id,
        num_waymo_tracks=len(scen.tracks),
        num_type_mapped=len(typed),
        dropped_by_type=dropped,
        num_candidates=len(typed),
        selected_target_indices=list(scen.tracks_to_predict),
        heuristic_target_indices=list(scen.tracks_to_predict),
        history_valid=history_valid,
        history_total=history_total,
        map_stats=current_map_stats,
    )
    ground_truth = ground_truth_of(scen)
    _write_sidecars(bag_path, conversion_ledger, ground_truth)
    return ConversionResult(bag_path, conversion_ledger, ground_truth)


def _history_gap_fraction(scen: scenario.Scenario) -> float:
    start = max(0, scen.current_time_index - rep.HISTORY_STEPS + 1)
    stop = scen.current_time_index + 1
    masks = [
        track.states[start:stop, S["valid"]] > 0.0
        for track in scen.tracks
        if track.object_type in rep.CLASS_ID_FOR_TYPE
    ]
    if not masks:
        return 1.0
    valid = np.concatenate(masks)
    return float(1.0 - valid.mean()) if valid.size else 1.0


def select_fixtures(
    scenarios: Sequence[scenario.Scenario],
    selectors: Sequence[str],
) -> dict[str, scenario.Scenario]:
    """Select distinct dense, sparse, and gappy scenarios deterministically."""
    requested = list(selectors)
    unknown = set(requested) - _SELECTORS
    if unknown:
        raise ValueError(f"unknown fixture selectors: {sorted(unknown)}")
    if len(requested) != len(set(requested)):
        raise ValueError("fixture selectors must be unique")
    if len(scenarios) < len(requested):
        raise ValueError("not enough distinct scenarios for fixture selection")

    rankings = {
        "dense": sorted(
            scenarios, key=lambda item: (-len(item.tracks), item.scenario_id)
        ),
        "sparse": sorted(
            scenarios, key=lambda item: (len(item.tracks), item.scenario_id)
        ),
        "gappy": sorted(
            scenarios,
            key=lambda item: (-_history_gap_fraction(item), item.scenario_id),
        ),
    }
    selected: dict[str, scenario.Scenario] = {}
    used: set[str] = set()
    for selector in requested:
        choice = next(
            item
            for item in rankings[selector]
            if item.scenario_id not in used
        )
        selected[selector] = choice
        used.add(choice.scenario_id)
    return selected


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shard", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    parser.add_argument("--select", default="dense,sparse,gappy")
    parser.add_argument(
        "--strategy", choices=["radius", "reachable"], default="radius"
    )
    parser.add_argument("--radius-m", type=float, default=80.0)
    parser.add_argument("--forward-half-angle-deg", type=float, default=90.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    selectors = [item.strip() for item in args.select.split(",") if item.strip()]
    scenarios = list(scenario.load_scenarios(args.shard))
    selected = select_fixtures(scenarios, selectors)
    config = lq.QueryConfig(
        radius_m=args.radius_m,
        strategy=args.strategy,
        forward_half_angle_deg=args.forward_half_angle_deg,
    )
    for selector, selected_scenario in selected.items():
        result = convert(selected_scenario, args.out, config)
        print(f"{selector}: {selected_scenario.scenario_id}")
        print(result.ledger.format())
        if selector == "dense" and len(selected_scenario.tracks) <= rep.CONTEXT_CAPACITY:
            print(
                "NOTE: densest scenario does not exceed context capacity; "
                "Task 12 needs a synthetic overflow fixture."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
