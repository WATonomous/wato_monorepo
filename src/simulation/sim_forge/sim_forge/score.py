"""Two explicit prototype checks. Never claim route or localization coverage."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .tools import dump


def assess(run, rows, *, minimum_duration, minimum_travel, collision_kinds):
    if not all(math.isfinite(x) and x > 0 for x in (minimum_duration, minimum_travel)):
        raise ValueError("Duration and travel thresholds must be finite and positive")
    if any(row.get("truncated") for row in rows):
        raise ValueError("Truncated trace cannot pass these checks")
    if not run.get("completed") or run.get("controller_kind") != "wato":
        raise ValueError("Only a completed WATO controlled episode may be scored as WATO driving")
    if not run.get("actor_truth_enabled"):
        raise ValueError("Missing actor truth")
    if run.get("end") == "truncated":
        raise ValueError("Truncated episode requires review and cannot pass these checks")
    if not collision_kinds or len(rows) < 2:
        raise ValueError("Configure reviewed collision event kinds and capture transitions")
    events = []
    for row in rows:
        if not isinstance(row.get("events"), list):
            raise ValueError("Collision event channel unavailable; cannot report zero collisions")
        for event in row["events"]:
            if not isinstance(event, dict) or not isinstance(event.get("kind"), str):
                raise ValueError("Invalid event contract")
        events.extend(row["events"])
    times = [float(row["ego"]["t"]) for row in rows]
    if not all(math.isfinite(t) for t in times) or not all(b > a for a, b in zip(times, times[1:])):
        raise ValueError("Invalid or nonincreasing simulation time")
    duration = times[-1]-times[0]
    travel = sum(math.hypot(b["ego"]["x"]-a["ego"]["x"], b["ego"]["y"]-a["ego"]["y"])
                 for a, b in zip(rows, rows[1:]))
    if not math.isfinite(travel):
        raise ValueError("Nonfinite trajectory")
    collision_seen = any(event["kind"] in collision_kinds for event in events)
    checks = {"no_collision_event": not collision_seen,
              "minimum_duration": duration >= minimum_duration,
              "minimum_travel": travel >= minimum_travel}
    return {"passed": all(checks.values()), "checks": checks,
            "duration_s": duration, "travel_m": travel,
            "coverage": "prototype collision events and nontrivial motion only",
            "unmeasured": ["route completion", "lane compliance", "stop line compliance",
                           "comfort", "real vehicle dynamics", "perception", "SLAM"]}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("episode")
    p.add_argument("--collision-kind", action="append", required=True)
    p.add_argument("--minimum-duration", type=float, required=True)
    p.add_argument("--minimum-travel", type=float, required=True)
    a = p.parse_args()
    if a.minimum_duration <= 0 or a.minimum_travel <= 0:
        p.error("Set positive duration and travel thresholds for this scenario")
    root = Path(a.episode)
    code = 2
    try:
        run = json.loads((root / "run.json").read_text())
        rows = [json.loads(line) for line in (root / "trace.jsonl").read_text().splitlines()]
        report = assess(run, rows, minimum_duration=a.minimum_duration,
                        minimum_travel=a.minimum_travel, collision_kinds=set(a.collision_kind))
        code = 0 if report["passed"] else 1
    except Exception as exc:
        report = {"passed": False, "error": str(exc), "status": "evaluation_error"}
    dump(root / "evaluation.json", report)
    suite = ET.Element("testsuite", name="simforge_prototype", tests="1",
                       failures=str(int(code == 1)), errors=str(int(code == 2)))
    case = ET.SubElement(suite, "testcase", name="collision_and_motion")
    if code:
        ET.SubElement(case, "error" if code == 2 else "failure").text = json.dumps(report)
    ET.ElementTree(suite).write(root / "junit.xml", encoding="utf-8", xml_declaration=True)
    print(json.dumps(report))
    raise SystemExit(code)
