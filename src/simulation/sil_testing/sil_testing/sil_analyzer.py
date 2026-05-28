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
"""Post-hoc SIL bag analyzer.

Reads the recorded MCAP, reconstructs the ego/command/actor time-series, evaluates them
against the scenario's limits, writes report.json, prints a colored table, and exits
non-zero if any limit was breached. This is the authoritative pass/fail verdict.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

from sil_testing import sil_metrics
from sil_testing.sil_scenario_schema import load_scenario

ODOM_TOPIC = "/ego/odom"
COMMAND_TOPIC = "/carla/ackermann_control/command"
DETECTIONS_TOPIC = "/carla/detections_3d"


def _quat_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _read_bag(bag_path):
    """Read the SIL topics from an MCAP bag into per-topic arrays."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    odom = {"t": [], "speed": [], "x": [], "y": [], "yaw": []}
    cmd = {"t": [], "steer": []}
    det = {"t": [], "actors": []}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in type_map:
            continue
        ts = t * 1e-9
        msg = deserialize_message(data, get_message(type_map[topic]))
        if topic == ODOM_TOPIC:
            odom["t"].append(ts)
            odom["speed"].append(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
            odom["x"].append(msg.pose.pose.position.x)
            odom["y"].append(msg.pose.pose.position.y)
            odom["yaw"].append(_quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        elif topic == COMMAND_TOPIC:
            cmd["t"].append(ts)
            cmd["steer"].append(msg.drive.steering_angle)
        elif topic == DETECTIONS_TOPIC:
            det["t"].append(ts)
            det["actors"].append([(d.bbox.center.position.x, d.bbox.center.position.y)
                                  for d in msg.detections])
    return odom, cmd, det


def _actors_aligned_to(odom_t, det):
    """Nearest-in-time actor lists for each odom sample."""
    if not det["t"]:
        return [[] for _ in odom_t]
    det_t = np.asarray(det["t"])
    aligned = []
    for ts in odom_t:
        aligned.append(det["actors"][int(np.argmin(np.abs(det_t - ts)))])
    return aligned


def _build_series(odom, cmd, det, route_xy, wheelbase):
    """Map each supported limit name to (series, times) or None if inputs are missing."""
    odom_t = np.asarray(odom["t"])
    speed = np.asarray(odom["speed"])
    series = {}

    if odom_t.size:
        series["max_speed_mps"] = (speed, odom_t)
        if route_xy is not None:
            cte = sil_metrics.cross_track_error(odom["x"], odom["y"], route_xy)
            series["max_cross_track_error_m"] = (cte, odom_t)
        actors = _actors_aligned_to(odom["t"], det)
        ego_xy = np.column_stack([odom["x"], odom["y"]]) if odom_t.size else np.empty((0, 2))
        series["min_distance_to_actor_m"] = (
            sil_metrics.min_distance_to_actors(ego_xy, actors), odom_t
        )

    if cmd["t"] and odom_t.size:
        cmd_t = np.asarray(cmd["t"])
        steer = np.asarray(cmd["steer"])
        steer_on_odom = np.interp(odom_t, cmd_t, steer)
        # Lateral accel is signed (negative on left turns); check its magnitude.
        series["max_lateral_accel_mps2"] = (
            np.abs(sil_metrics.lateral_acceleration_from_steering(speed, steer_on_odom, wheelbase)),
            odom_t,
        )
        series["max_steering_rate_radps"] = (
            np.abs(sil_metrics.steering_rate(steer, cmd_t)), cmd_t
        )

    return series


def analyze(bag_path, scenario_path, wheelbase=2.5667):
    scenario = load_scenario(scenario_path)
    route_xy = [[wp.x, wp.y] for wp in scenario.route.waypoints] if scenario.route else None

    odom, cmd, det = _read_bag(bag_path)
    series = _build_series(odom, cmd, det, route_xy, wheelbase)

    results = []
    for name, limit in scenario.limits.items():
        if name not in series:
            results.append({
                "name": name, "limit": limit, "observed": None,
                "passed": False, "first_violation_time": None,
                "message": "no data to evaluate (missing topic/inputs)",
            })
            continue
        values, times = series[name]
        r = sil_metrics.check_limit(name, values, times, limit)
        results.append({
            "name": r.name, "kind": r.kind, "limit": r.limit, "observed": r.observed,
            "passed": r.passed, "first_violation_time": r.first_violation_time,
            "message": "ok" if r.passed else "VIOLATION",
        })

    passed = all(r["passed"] for r in results) if results else True
    return {
        "scenario": scenario.name,
        "bag": bag_path,
        "passed": passed,
        "limits": results,
        "samples": {"odom": len(odom["t"]), "command": len(cmd["t"]), "detections": len(det["t"])},
    }


def _print_report(report):
    green, red, yellow, reset, bold = "\033[0;32m", "\033[0;31m", "\033[0;33m", "\033[0m", "\033[1m"
    status = f"{green}PASS{reset}" if report["passed"] else f"{red}FAIL{reset}"
    print(f"\n{bold}SIL Report: {report['scenario']}{reset}  [{status}]")
    print(f"  bag: {report['bag']}")
    print(f"  samples: {report['samples']}")
    print(f"  {'limit':<28}{'observed':>12}{'threshold':>12}   result")
    for r in report["limits"]:
        observed = "n/a" if r["observed"] is None else f"{r['observed']:.4f}"
        mark = f"{green}ok{reset}" if r["passed"] else f"{red}{r['message']}{reset}"
        extra = ""
        if r.get("first_violation_time") is not None:
            extra = f"{yellow} @ t={r['first_violation_time']:.2f}s{reset}"
        print(f"  {r['name']:<28}{observed:>12}{r['limit']:>12.4f}   {mark}{extra}")
    print()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Analyze a SIL run bag against scenario limits")
    parser.add_argument("--bag", default=os.environ.get("SIL_BAG_PATH", ""),
                        help="Path to the recorded MCAP bag directory")
    parser.add_argument("--scenario", default=os.environ.get("SIL_SCENARIO_FILE", ""),
                        help="Path to the scenario YAML")
    parser.add_argument("--report", default=os.environ.get("SIL_REPORT_PATH", ""),
                        help="Where to write report.json (default: <bag>/report.json)")
    parser.add_argument("--wheelbase", type=float, default=2.5667)
    args = parser.parse_args(argv)

    if not args.bag or not args.scenario:
        parser.error("both --bag and --scenario (or SIL_BAG_PATH/SIL_SCENARIO_FILE) are required")

    report = analyze(args.bag, args.scenario, args.wheelbase)

    report_path = args.report or os.path.join(args.bag, "report.json")
    try:
        with open(report_path, "w") as f:
            json.dump(report, f, indent=2)
        report["report_path"] = report_path
    except OSError as e:
        print(f"Warning: could not write report to {report_path}: {e}", file=sys.stderr)

    _print_report(report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
