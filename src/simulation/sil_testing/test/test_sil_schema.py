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
import pytest

from sil_testing.sil_scenario_schema import parse_scenario


def _valid_scenario():
    return {
        "name": "test",
        "component": "motion_control",
        "duration_s": 20.0,
        "route": {
            "target_speed_mps": 8.0,
            "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}],
        },
        "limits": {"max_speed_mps": 10.0, "min_distance_to_actor_m": 2.0},
    }


def test_parse_valid_scenario():
    s = parse_scenario(_valid_scenario())
    assert s.name == "test"
    assert s.duration_s == 20.0
    assert len(s.route.waypoints) == 2
    assert s.limits["max_speed_mps"] == 10.0


def test_default_duration_and_component():
    data = _valid_scenario()
    del data["duration_s"]
    del data["component"]
    s = parse_scenario(data)
    assert s.duration_s == 30.0
    assert s.component == "motion_control"


def test_missing_name_raises():
    data = _valid_scenario()
    del data["name"]
    with pytest.raises(ValueError, match="name"):
        parse_scenario(data)


def test_motion_control_requires_route():
    data = _valid_scenario()
    del data["route"]
    with pytest.raises(ValueError, match="route"):
        parse_scenario(data)


def test_route_requires_two_waypoints():
    data = _valid_scenario()
    data["route"]["waypoints"] = [{"x": 0.0, "y": 0.0}]
    with pytest.raises(ValueError, match="2 waypoints"):
        parse_scenario(data)


def test_limit_key_must_have_direction_prefix():
    data = _valid_scenario()
    data["limits"] = {"lateral_accel": 4.0}
    with pytest.raises(ValueError, match="max_.*min_"):
        parse_scenario(data)
