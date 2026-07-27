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

from __future__ import annotations

import numpy as np

from wato_mtr_eval import lanelet_query as lq
from wato_mtr_eval import reference_pack, representation, scenario

S = scenario.STATE_COL


def _states(
    x: float,
    *,
    valid: np.ndarray | None = None,
    length: float = 5.0,
) -> np.ndarray:
    states = np.zeros((11, scenario.NUM_STATE_COLS), dtype=np.float64)
    states[:, S["x"]] = x + np.arange(11, dtype=np.float64) * 0.1
    states[:, S["length"]] = length
    states[:, S["width"]] = 2.0
    states[:, S["height"]] = 1.5
    states[:, S["vx"]] = 1.0
    states[:, S["valid"]] = 1.0 if valid is None else valid
    return states


def _scenario() -> scenario.Scenario:
    # The first lane is behind the target-forward offset; the second is near it.
    # Its 25 points also exercise direction continuity across a 20-point chunk.
    behind = np.column_stack(
        (np.linspace(-20.0, -10.0, 4), np.zeros(4), np.zeros(4))
    )
    ahead = np.column_stack(
        (np.linspace(20.0, 44.0, 25), np.zeros(25), np.zeros(25))
    )
    stale_valid = np.ones(11)
    stale_valid[-1] = 0.0
    return scenario.Scenario(
        scenario_id="reference",
        timestamps_s=np.arange(11, dtype=np.float64) * 0.1,
        current_time_index=10,
        sdc_track_index=0,
        tracks=[
            scenario.Track(20, "vehicle", _states(0.0, length=7.0)),
            scenario.Track(10, "vehicle", _states(8.0)),
            scenario.Track(
                30,
                "pedestrian",
                _states(4.0, valid=stale_valid, length=0.5),
            ),
        ],
        tracks_to_predict=[1],
        map_polylines=[
            scenario.MapPolyline(1, "lane", behind),
            scenario.MapPolyline(2, "lane", ahead),
        ],
    )


def test_pack_shapes_dtypes_and_positive_replay_epoch():
    tensors = reference_pack.pack(
        _scenario(), lq.QueryConfig(radius_m=100.0)
    )

    assert tensors["obj_trajs"].shape == (8, 128, 11, 29)
    assert tensors["obj_trajs"].dtype == np.float32
    assert tensors["obj_trajs_mask"].dtype == np.uint8
    assert tensors["obj_trajs_last_pos"].shape == (8, 128, 3)
    assert tensors["track_index_to_predict"].dtype == np.int64
    assert tensors["center_type_ids"].dtype == np.int64
    assert tensors["map_polylines"].shape == (8, 768, 20, 9)
    assert tensors["map_polylines_mask"].dtype == np.uint8
    assert tensors["map_polylines_center"].shape == (8, 768, 3)
    assert tensors["obj_trajs_mask"][0, :, 0].any()


def test_context_contains_synthetic_ego_and_retained_stale_track():
    tensors = reference_pack.pack(
        _scenario(), lq.QueryConfig(radius_m=100.0)
    )
    target_row = int(tensors["track_index_to_predict"][0])

    # The raw Waymo SDC is omitted; pose/odometry contributes the only ego row.
    assert tensors["obj_trajs"][0, 0, -1, 10] == 1.0
    np.testing.assert_allclose(
        tensors["obj_trajs"][0, 0, -1, 3:6],
        np.asarray([4.0936, 1.3579, 1.4090]),
    )
    assert target_row == 2
    assert tensors["obj_trajs"][0, 1, -1].sum() == 0.0
    assert tensors["obj_trajs_mask"][0, 1, -2] == 1


def test_map_chunks_are_ranked_and_keep_cross_chunk_direction():
    tensors = reference_pack.pack(
        _scenario(), lq.QueryConfig(radius_m=100.0)
    )
    packed = tensors["map_polylines"][0]
    mask = tensors["map_polylines_mask"][0]

    # Ahead chunks rank before the geometrically behind lane.
    assert tensors["map_polylines_center"][0, 0, 0] > 0.0
    assert tensors["map_polylines_center"][0, 1, 0] > 0.0
    # First point of the five-point tail window keeps the full-lane backward diff.
    tail = next(index for index in range(3) if int(mask[index].sum()) == 5)
    np.testing.assert_allclose(packed[tail, 0, 3:6], [1.0, 0.0, 0.0])


def test_pinned_target_is_mandatory_context_member():
    scen = _scenario()
    tracks = list(scen.tracks)
    for object_id in range(1000, 1130):
        tracks.append(
            scenario.Track(
                object_id,
                "vehicle",
                _states(float(object_id - 1000)),
            )
        )
    far_target = scenario.Track(9999, "vehicle", _states(1000.0))
    tracks.append(far_target)
    scen = scenario.Scenario(
        scenario_id=scen.scenario_id,
        timestamps_s=scen.timestamps_s,
        current_time_index=scen.current_time_index,
        sdc_track_index=scen.sdc_track_index,
        tracks=tracks,
        tracks_to_predict=[len(tracks) - 1],
        map_polylines=scen.map_polylines,
    )

    tensors = reference_pack.pack(
        scen, lq.QueryConfig(radius_m=100.0)
    )
    assert 0 <= tensors["track_index_to_predict"][0] < 128
    target_row = int(tensors["track_index_to_predict"][0])
    assert tensors["obj_trajs"][0, target_row, -1, 9] == 1.0


def test_replay_sidecars_preserve_pinned_order(tmp_path):
    scen = _scenario()
    scen.tracks_to_predict[:] = [1, 0]
    reference_pack.write_replay_sidecars(tmp_path, scen)

    assert (tmp_path / "target_ids.txt").read_text().splitlines() == [
        "10",
        "__ego__",
    ]
    assert (tmp_path / "target_id_map.tsv").read_text().splitlines() == [
        "slot\tgt_row\twaymo_id\tscene_builder_id",
        "0\t0\t10\t10",
        "1\t1\t20\t__ego__",
    ]
    assert (tmp_path / "target_frames.tsv").read_text().splitlines() == [
        "slot\tgt_row\tcenter_x\tcenter_y\theading",
        "0\t0\t9\t0\t0",
        "1\t1\t1\t0\t0",
    ]
    assert (tmp_path / "replay_step.txt").read_text() == "10\n"


def test_sdc_target_uses_the_only_synthetic_ego_row():
    scen = _scenario()
    scen.tracks_to_predict[:] = [scen.sdc_track_index]
    tensors = reference_pack.pack(
        scen, lq.QueryConfig(radius_m=100.0)
    )

    assert tensors["track_index_to_predict"][0] == 0
    assert tensors["center_type_ids"][0] == 0
    assert tensors["obj_trajs"][0, 0, -1, 9] == 1.0
    assert tensors["obj_trajs"][0, 0, -1, 10] == 1.0
