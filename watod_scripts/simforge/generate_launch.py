"""Generate reviewed starting copies of existing launches. Does not prove stack readiness."""
import argparse
import shutil
from pathlib import Path
import yaml

p = argparse.ArgumentParser()
p.add_argument("--repo", default=".")
p.add_argument("--map-config", required=True)
p.add_argument("--output", required=True)
a = p.parse_args()
root, out = Path(a.repo).resolve(), Path(a.output).resolve()
if out.exists() and any(out.iterdir()):
    raise FileExistsError("Choose an empty output directory")
out.mkdir(parents=True, exist_ok=True)
world = yaml.safe_load((root / "src/world_modeling/world_modeling_bringup/launch/world_modeling_sim.launch.yaml").read_text())
action = yaml.safe_load((root / "src/action/action_bringup/launch/action_sim.launch.yaml").read_text())
for entry in world["launch"]:
    arg = entry.get("arg")
    if arg and arg["name"] == "map_config_file":
        arg["default"] = str(Path(a.map_config).resolve())
    if arg and arg["name"] == "use_eidos":
        arg["default"] = "false"
for document in (world, action):
    for entry in document["launch"]:
        node = entry.get("node")
        if node:
            params = node.setdefault("param", [])
            for param in params:
                if param.get("name") == "use_sim_time":
                    param["value"] = True
                    break
            else:
                params.append({"name": "use_sim_time", "value": True})
            for remap in node.get("remap", []):
                if remap.get("to") == "/carla/tracked_detections_3d":
                    remap["to"] = "/simforge/tracked_detections_3d"
                if remap.get("to") == "/carla/ackermann_control/command":
                    remap["to"] = "/simforge/command"
for filename, data in (("world_modeling_simforge.launch.yaml", world), ("action_simforge.launch.yaml", action)):
    (out / filename).write_text(yaml.safe_dump(data, sort_keys=False))
print(f"Generated starting launches in {out}; finish map, truth, costmap and synchronization gates in the handoff")

shutil.copyfile(root / "watod_scripts/simforge/combined_launch.template.py",
                out / "combined_simforge.launch.py")
