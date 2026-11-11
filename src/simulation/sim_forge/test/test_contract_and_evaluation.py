import copy
import math
import pytest
from sim_forge.contract import ContractError, Mapper, command_to_control
from sim_forge.score import assess
from sim_forge.tools import check_repeatability


def contract():
    return {"reviewed": True, "time": "/info/t_s",
            "ego": {"x": "/info/ego/x", "y": "/info/ego/y",
                    "yaw": "/info/ego/yaw", "speed": "/info/ego/speed"},
            "plane_matrix": [[1, 0], [0, -1]], "map_offset_m": [10, 20],
            "yaw_scale": 1, "yaw_offset_rad": 0, "speed_scale": 1,
            "runtime_origin_to_base_m": [-1, 0]}


def test_reflection_and_reference_offset_together():
    raw = {"info": {"t_s": 1, "ego": {"x": 2, "y": 3, "yaw": math.pi/2, "speed": 4}}}
    ego = Mapper(contract()).ego(raw)
    assert (ego.x, ego.y) == pytest.approx((12, 18))
    assert ego.yaw == pytest.approx(-math.pi/2)
    assert ego.speed == 4


def test_missing_coordinates_and_unreviewed_profiles_do_not_default_to_zero():
    with pytest.raises(ContractError):
        Mapper({"reviewed": False})
    with pytest.raises(ContractError):
        Mapper(contract()).ego({"info": {"t_s": 1, "ego": {}}})
    with pytest.raises(ContractError):
        Mapper(contract()).actors({})


def test_reverse_and_nonfinite_commands_are_rejected():
    cfg = {"reviewed": True, "max_speed_mps": 3, "steer_limit_rad": 0.5,
           "steer_sign": -1, "speed_kp": 0.4}
    assert command_to_control(1, 0.25, 0, cfg) == pytest.approx([0.4, 0, -0.5])
    assert command_to_control(0, 0, 2, cfg) == [0, 1, 0]
    for speed in (-1, float("nan"), 4):
        with pytest.raises(ContractError):
            command_to_control(speed, 0, 0, cfg)


def trace():
    return [{"ego": {"t": 0, "x": 0, "y": 0}, "events": []},
            {"ego": {"t": 1, "x": 2, "y": 0}, "events": []}]


def evaluate(rows, kind="wato"):
    return assess({"completed": True, "controller_kind": kind, "actor_truth_enabled": True},
                  rows, minimum_duration=1, minimum_travel=1, collision_kinds={"collision"})


def test_collision_in_final_frame_fails_and_missing_channel_is_an_error():
    rows = trace()
    rows[-1]["events"] = [{"kind": "collision"}]
    assert evaluate(rows)["passed"] is False
    rows[-1]["events"] = None
    with pytest.raises(ValueError, match="unavailable"):
        evaluate(rows)


def test_parked_car_and_diagnostic_driver_cannot_pass_driving_checks():
    rows = trace()
    rows[-1]["ego"]["x"] = 0
    assert evaluate(rows)["passed"] is False
    with pytest.raises(ValueError, match="WATO"):
        evaluate(trace(), "diagnostic")


def test_reset_repeatability_detects_changes_and_frozen_time():
    first = [{"info": {"t_s": 0}}, {"info": {"t_s": 1}}]
    check_repeatability(first, copy.deepcopy(first))
    second = copy.deepcopy(first)
    second[-1]["info"]["t_s"] = 2
    with pytest.raises(ValueError, match="different traces"):
        check_repeatability(first, second)
    with pytest.raises(ValueError, match="monotonically"):
        check_repeatability([first[0], first[0]], [first[0], first[0]])


@pytest.mark.parametrize("threshold", [float("nan"), float("inf"), 0, -1])
def test_invalid_scoring_threshold_is_an_evaluation_error(threshold):
    with pytest.raises(ValueError, match="threshold"):
        assess({"completed": True, "controller_kind": "wato", "actor_truth_enabled": True},
               trace(), minimum_duration=threshold, minimum_travel=1,
               collision_kinds={"collision"})


def test_truncated_trace_cannot_pass_even_with_inconsistent_run_metadata():
    rows = trace()
    rows[-1]["truncated"] = True
    with pytest.raises(ValueError, match="Truncated"):
        evaluate(rows)


def test_actor_truth_maps_boxes_and_rejects_duplicate_ids():
    cfg = contract()
    keys = ("id", "class", "x", "y", "z", "yaw", "length", "width", "height", "signal_state")
    cfg["actors"] = {"reviewed": True, "list": "/info/actors",
                     "fields": {key: "/" + key for key in keys},
                     "class_map": {"signal": "traffic_light"}}
    actor = {"id": "light_1", "class": "signal", "x": 2, "y": 3, "z": 4,
             "yaw": math.pi / 2, "length": 1, "width": 2, "height": 3,
             "signal_state": "green"}
    result = Mapper(cfg).actors({"info": {"actors": [actor]}})[0]
    assert (result["x"], result["y"], result["z"]) == (12, 17, 4)
    assert result["yaw"] == pytest.approx(-math.pi / 2)
    assert result["class"] == "traffic_light"
    with pytest.raises(ContractError, match="unique"):
        Mapper(cfg).actors({"info": {"actors": [actor, actor]}})
    actor["signal_state"] = "unknown"
    with pytest.raises(ContractError, match="signal"):
        Mapper(cfg).actors({"info": {"actors": [actor]}})
