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
"""Tests for lanelet_ground_truth. Run with: python -m pytest test_lanelet_ground_truth.py"""

import math

import numpy as np
import pytest

import lanelet_ground_truth as gt

# Two adjacent lanes along +x from x=0 to x=40:
#   right lane y in [0, 4] (bounded by ways 10 (y=0) and 11 (y=4))
#   left lane  y in [4, 8] (bounded by ways 11 (y=4) and 12 (y=8))
# Way 11 is shared -> divider. Way 12 is stored reversed to exercise boundary flipping.
# A stop line (way 13) crosses the right lane at x=30.
TWO_LANE_OSM = """<?xml version="1.0"?>
<osm>
  <node id="1"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
  <node id="2"><tag k="local_x" v="40"/><tag k="local_y" v="0"/></node>
  <node id="3"><tag k="local_x" v="0"/><tag k="local_y" v="4"/></node>
  <node id="4"><tag k="local_x" v="40"/><tag k="local_y" v="4"/></node>
  <node id="5"><tag k="local_x" v="0"/><tag k="local_y" v="8"/></node>
  <node id="6"><tag k="local_x" v="40"/><tag k="local_y" v="8"/></node>
  <node id="7"><tag k="local_x" v="30"/><tag k="local_y" v="0"/></node>
  <node id="8"><tag k="local_x" v="30"/><tag k="local_y" v="4"/></node>
  <way id="10"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>
  <way id="12"><nd ref="6"/><nd ref="5"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>
  <way id="13"><nd ref="7"/><nd ref="8"/><tag k="type" v="stop_line"/><tag k="subtype" v="solid"/></way>
  <relation id="100">
    <member type="way" ref="11" role="left"/><member type="way" ref="10" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
  <relation id="101">
    <member type="way" ref="12" role="left"/><member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
</osm>
"""


@pytest.fixture
def two_lane_map(tmp_path):
    path = tmp_path / "two_lane.osm"
    path.write_text(TWO_LANE_OSM)
    return gt.load_osm(str(path))


def cell(x_forward, y_left):
    """Grid (row, col) of a point in the LiDAR frame."""
    return (
        int((x_forward - gt.ORIGIN) / gt.RESOLUTION),
        int((y_left - gt.ORIGIN) / gt.RESOLUTION),
    )


def near(mask, x_forward, y_left):
    """True if the mask is set within one cell of the point (lines are 1 cell wide)."""
    r, c = cell(x_forward, y_left)
    return bool(np.any(mask[r - 1 : r + 2, c - 1 : c + 2] == 1))


def test_load_osm(two_lane_map):
    assert len(two_lane_map.nodes) == 8
    assert len(two_lane_map.lanelets) == 2


def test_shared_boundary_is_divider_and_edges_are_not(two_lane_map):
    assert gt.divider_way_ids(two_lane_map) == {"11"}


def test_drivable_ahead_not_behind(two_lane_map):
    # Car in the right lane at x=5 facing +x.
    masks = gt.rasterize(two_lane_map, (5.0, 2.0, 0.0))
    assert masks[gt.DRIVABLE][cell(10, 0)] == 1  # 10 m ahead, same lane
    assert masks[gt.DRIVABLE][cell(-10, 0)] == 0  # 10 m behind: road starts at x=0
    assert (
        masks[gt.DRIVABLE][cell(10, 4)] == 1
    )  # left lane (reversed boundary way still fills)


def test_divider_on_left_of_car_in_right_lane(two_lane_map):
    masks = gt.rasterize(two_lane_map, (5.0, 2.0, 0.0))
    divider = masks[gt.DIVIDER]
    assert near(divider, 10, 2.0)  # shared line at map y=4 -> 2 m to the car's left
    assert not near(divider, 10, -2.0)  # road edge at map y=0 is not a divider


def test_rotated_pose(two_lane_map):
    # Same spot, car facing -x: the road is now behind and the divider on its right.
    masks = gt.rasterize(two_lane_map, (5.0, 2.0, math.pi))
    assert masks[gt.DRIVABLE][cell(-10, 0)] == 1
    assert near(masks[gt.DIVIDER], -10, -2.0)
    assert not near(masks[gt.DIVIDER], -10, 2.0)


def test_stop_line(two_lane_map):
    masks = gt.rasterize(two_lane_map, (5.0, 2.0, 0.0))
    assert near(masks[gt.STOP_LINE], 25, 0)  # stop line at map x=30 -> 25 m ahead
    assert not near(masks[gt.STOP_LINE], 10, 0)


def test_unlabeled_classes_are_ignored(two_lane_map):
    masks = gt.rasterize(two_lane_map, (5.0, 2.0, 0.0))
    for cls in gt.UNLABELED_CLASSES:
        assert np.all(masks[cls] == gt.IGNORE)
    assert masks.shape == (len(gt.CLASS_NAMES), gt.GRID_SIZE, gt.GRID_SIZE)


def test_sampled_pose_heads_along_lane(two_lane_map):
    for x, y, yaw in gt.sample_lanelet_poses(two_lane_map, 2):
        assert 10 < x < 30 and 0 < y < 8  # mid-lane, not at an end
        assert abs(math.sin(yaw)) < 1e-6  # both lanes run along x
