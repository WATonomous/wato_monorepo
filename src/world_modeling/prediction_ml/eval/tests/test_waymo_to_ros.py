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

import json

import numpy as np
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")

from wato_mtr_eval import (  # noqa: E402
    lanelet_query as lq,
    representation as rep,
    scenario,
    waymo_to_ros as w2r,
)

mcap_available = "mcap" in rosbag2_py.get_registered_writers()
needs_mcap = pytest.mark.skipif(
    not mcap_available, reason="rosbag2 MCAP storage plugin is not installed"
)


def _scenario(num_tracks=3, num_steps=91, scenario_id="test_scenario"):
    tracks = []
    for index in range(num_tracks):
        states = np.zeros((num_steps, 10))
        states[:, 0] = np.arange(num_steps) * 0.5 + index * 10
        states[:, 1] = float(index)
        states[:, 3:6] = [4.5, 2.0, 1.6]
        states[:, 6] = 0.0
        states[:, 7] = 5.0
        states[:, 9] = 1.0
        tracks.append(
            scenario.Track(
                object_id=100 + index,
                object_type=["vehicle", "pedestrian", "cyclist"][index % 3],
                states=states,
            )
        )
    lane = scenario.MapPolyline(
        feature_id=1,
        feature_type="lane",
        points=np.array(
            [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]
        ),
    )
    return scenario.Scenario(
        scenario_id=scenario_id,
        timestamps_s=np.arange(num_steps) * 0.1,
        current_time_index=10,
        sdc_track_index=0,
        tracks=tracks,
        tracks_to_predict=list(range(num_tracks)),
        map_polylines=[lane],
    )


def test_detections_use_class_id_strings_scene_builder_accepts():
    msg = w2r.detections_at(_scenario(), step=10)
    labels = {
        result.hypothesis.class_id
        for detection in msg.detections
        for result in detection.results
        if result.hypothesis.class_id != rep.VELOCITY_CLASS_ID
    }
    assert labels <= set(rep.CLASS_ID_FOR_TYPE.values())


def test_velocity_rides_in_the_hypothesis_pose_position():
    det = w2r.detections_at(_scenario(), step=10).detections[0]
    velocity = [
        result
        for result in det.results
        if result.hypothesis.class_id == rep.VELOCITY_CLASS_ID
    ]
    assert len(velocity) == 1
    assert velocity[0].pose.pose.position.x == pytest.approx(5.0)
    assert velocity[0].pose.pose.position.z == pytest.approx(0.0)
    assert velocity[0].hypothesis.score > 0.0


def test_detection_ids_are_stable_across_frames():
    scen = _scenario()
    earlier = {detection.id for detection in w2r.detections_at(scen, 8).detections}
    current = {
        detection.id for detection in w2r.detections_at(scen, 10).detections
    }
    assert earlier == current and earlier


def test_invalid_states_and_unmapped_types_are_omitted():
    scen = _scenario()
    scen.tracks[1].states[10, scenario.STATE_COL["valid"]] = 0.0
    scen.tracks[2] = scenario.Track(
        object_id=scen.tracks[2].object_id,
        object_type="other",
        states=scen.tracks[2].states,
    )

    ids = {detection.id for detection in w2r.detections_at(scen, 10).detections}

    assert str(scen.tracks[1].object_id) not in ids
    assert str(scen.tracks[2].object_id) not in ids


def test_bbox_sizes_are_positive_so_sample_from_detection_accepts_them():
    for detection in w2r.detections_at(_scenario(), 10).detections:
        assert detection.bbox.size.x > 0
        assert detection.bbox.size.y > 0
        assert detection.bbox.size.z > 0


def test_ego_odometry_twist_is_body_frame_and_round_trips():
    scen = _scenario()
    scen.tracks[0].states[:, scenario.STATE_COL["heading"]] = np.pi / 2
    scen.tracks[0].states[:, scenario.STATE_COL["vx"]] = 0.0
    scen.tracks[0].states[:, scenario.STATE_COL["vy"]] = 5.0

    odom = w2r.ego_odometry_at(scen, 10)

    assert odom.child_frame_id == w2r.EGO_CHILD_FRAME
    assert odom.twist.twist.linear.x == pytest.approx(5.0, abs=1e-6)
    assert odom.twist.twist.linear.y == pytest.approx(0.0, abs=1e-6)


def test_ego_pose_matches_the_sdc_track():
    scen = _scenario()
    pose = w2r.ego_pose_at(scen, 10)
    assert pose.header.frame_id == w2r.MAP_FRAME
    assert pose.pose.position.x == pytest.approx(
        scen.tracks[0].states[10, scenario.STATE_COL["x"]]
    )


def test_lanelet_ahead_matches_actual_schema_and_carries_only_centerlines():
    msg, _ = w2r.lanelet_ahead_at(_scenario(), 10, lq.QueryConfig())
    assert msg.current_lanelet_id == -1
    assert msg.lanelets
    for lanelet in msg.lanelets:
        assert len(lanelet.centerline) > 0
        assert len(lanelet.centerline_curvature) == len(lanelet.centerline)
        assert len(lanelet.left_boundary) == 0
        assert len(lanelet.right_boundary) == 0
        assert lanelet.left_lane_id == lanelet.right_lane_id == -1


@pytest.mark.skipif(
    mcap_available, reason="this diagnostic applies only without MCAP support"
)
def test_convert_explains_when_mcap_storage_is_unavailable(tmp_path):
    with pytest.raises(RuntimeError, match="MCAP storage plugin"):
        w2r.convert(_scenario(), tmp_path, lq.QueryConfig())


@needs_mcap
def test_convert_writes_a_readable_bag_with_all_four_topics(tmp_path):
    result = w2r.convert(_scenario(), tmp_path, lq.QueryConfig())
    assert result.bag_path.exists()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(result.bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {topic.name for topic in reader.get_all_topics_and_types()}
    assert topics == set(w2r.TOPICS.values())


@needs_mcap
def test_convert_emits_one_message_per_topic_per_step_and_sidecars(tmp_path):
    scen = _scenario()
    result = w2r.convert(scen, tmp_path, lq.QueryConfig())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(result.bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    counts = {}
    while reader.has_next():
        topic, _, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
    assert set(counts.values()) == {len(scen.timestamps_s)}
    assert json.loads((result.bag_path / "ledger.json").read_text())[
        "scenario_id"
    ] == scen.scenario_id
    truth = np.load(result.bag_path / "ground_truth.npz")
    np.testing.assert_array_equal(truth["gt"], result.ground_truth[0])
    np.testing.assert_array_equal(truth["valid"], result.ground_truth[1])


def test_ground_truth_covers_designated_targets_and_native_horizon():
    gt, valid, ids, types = w2r.ground_truth_of(_scenario())
    assert gt.shape == (3, rep.NUM_FUTURE_FRAMES, 2)
    assert valid.shape == (3, rep.NUM_FUTURE_FRAMES)
    assert len(ids) == len(types) == 3


def test_conversion_functions_are_lossless_for_claimed_detection_fields():
    scen = _scenario()
    for step in range(len(scen.timestamps_s)):
        by_id = {
            detection.id: detection
            for detection in w2r.detections_at(scen, step).detections
        }
        for track in scen.tracks:
            state = track.states[step]
            if (
                state[scenario.STATE_COL["valid"]] == 0.0
                or track.object_type == "other"
            ):
                assert str(track.object_id) not in by_id
                continue
            detection = by_id[str(track.object_id)]
            assert detection.bbox.center.position.x == pytest.approx(
                state[scenario.STATE_COL["x"]], abs=1e-9
            )
            assert detection.bbox.center.position.y == pytest.approx(
                state[scenario.STATE_COL["y"]], abs=1e-9
            )
            assert detection.bbox.size.x == pytest.approx(
                state[scenario.STATE_COL["length"]], abs=1e-9
            )
            velocity = next(
                result
                for result in detection.results
                if result.hypothesis.class_id == rep.VELOCITY_CLASS_ID
            )
            assert velocity.pose.pose.position.x == pytest.approx(
                state[scenario.STATE_COL["vx"]], abs=1e-9
            )
            assert velocity.pose.pose.position.y == pytest.approx(
                state[scenario.STATE_COL["vy"]], abs=1e-9
            )


def test_fixture_selection_is_deterministic_and_code_path_driven():
    sparse = _scenario(1, scenario_id="sparse")
    dense = _scenario(5, scenario_id="dense")
    gappy = _scenario(3, scenario_id="gappy")
    gappy.tracks[1].states[:11, scenario.STATE_COL["valid"]] = 0.0

    selected = w2r.select_fixtures(
        [gappy, dense, sparse], ["dense", "sparse", "gappy"]
    )

    assert {name: scen.scenario_id for name, scen in selected.items()} == {
        "dense": "dense",
        "sparse": "sparse",
        "gappy": "gappy",
    }
