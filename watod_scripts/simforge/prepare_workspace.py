"""Use the installed SDK's own template and map pin rather than inventing scenario JSON."""
import argparse
import json
from pathlib import Path
from simforge_gym import ScenarioSet, template_workspace

p = argparse.ArgumentParser()
p.add_argument("--task-set", default="richmond-field-station")
p.add_argument("--scenario", default="lead-hard-brake")
p.add_argument("--out", required=True)
p.add_argument("--list", action="store_true")
a = p.parse_args()
tasks = ScenarioSet.bundled(a.task_set)
if a.list:
    print(json.dumps({"map": tasks.map.__dict__, "scenarios": [s.name for s in tasks.scenarios]}, indent=2))
else:
    candidates = [s for s in tasks.scenarios if s.name == a.scenario or s.name.split("@")[0] == a.scenario]
    if not candidates:
        raise ValueError("No matching task. Run with --list first")
    scenario = sorted(candidates, key=lambda s: s.name)[0]
    if scenario.template is None or tasks.map is None:
        raise ValueError("Expected a template task with an exact map pin")
    path = Path(a.out)
    if path.exists() and any(path.iterdir()):
        raise FileExistsError("Choose a new empty workspace")
    path = template_workspace(scenario.template, None, path,
        map_id=tasks.map.name, xodr_sha256=tasks.map.xodr_sha256, site=scenario.site)
    print(json.dumps({"workspace": str(path), "map": tasks.map.__dict__,
                      "scenario": scenario.name, "control": "bundled authored ego; diagnostic only"}, indent=2))
