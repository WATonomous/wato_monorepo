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
"""Host CLI integration; no Docker, ROS installation, or OpenAI SDK required."""

import argparse
import datetime
import json
import os
import sys
import tempfile
import uuid
from pathlib import Path


def add_arguments(parser):
    parser.add_argument("kind", choices=("scenario", "map-context"))
    parser.add_argument("--prompt", "-prompt", help="Describe a custom controller path")
    parser.add_argument(
        "--name", help="New scenario and maneuver ID; defaults to a unique ID"
    )
    parser.add_argument("--map", default="oval", help="Playground ID (default: oval)")
    parser.add_argument(
        "--boundary-clearance-m",
        type=float,
        default=3.0,
        help="Minimum reference/actor center clearance from map edges (default: 3 m)",
    )
    parser.add_argument(
        "--model",
        default=os.environ.get("OPENAI_MODEL", "gpt-4.1"),
        help="OpenAI model ID",
    )
    parser.add_argument("--max-rounds", type=int, choices=range(1, 9), default=4)
    parser.add_argument(
        "--dry-run", action="store_true", help="Save preview files without installing"
    )
    parser.add_argument(
        "--candidate",
        type=Path,
        help="Validate a proposal JSON offline instead of calling OpenAI",
    )


def publish(workspace, maneuver_text, scenario_text):
    """Publish data with exclusive creates; scenario appears after its maneuver.

    Concurrent runs cannot replace an existing file. Roll back only files this
    invocation created if either write fails. This does not replace the session
    transition lock and never calls session services.
    """
    created = []
    try:
        for path, content in zip(workspace.targets, (maneuver_text, scenario_text)):
            with path.open("x", encoding="utf-8") as stream:
                created.append(path)
                stream.write(content)
    except BaseException:
        for path in reversed(created):
            path.unlink()
        raise


def run_generation(arguments, root):
    for package in (
        root / "src/simulation/simulation_runtime",
        root / "src/simulation/carla_ros_bridge/carla_scenarios",
    ):
        sys.path.insert(0, str(package))
    try:
        import yaml

        from .generator import Workspace, generate, read_json
        from .trajectory import preview_svg
    except ModuleNotFoundError as error:
        print(
            f"Generation dependency missing: {error.name}. Run through watod sim generate "
            "to prepare its isolated environment.",
            file=sys.stderr,
        )
        return 2
    try:
        prompt = arguments.prompt or ""
        if (
            arguments.kind == "scenario"
            and not arguments.candidate
            and not prompt.strip()
        ):
            raise ValueError("--prompt is required unless --candidate is supplied")
        if len(prompt) > 16000:
            raise ValueError("--prompt must be at most 16000 characters")
        name = arguments.name or (
            "generated_"
            + datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S_")
            + uuid.uuid4().hex[:6]
        )
        workspace = Workspace(
            root,
            name,
            arguments.map,
            boundary_clearance_m=arguments.boundary_clearance_m,
        )
        if arguments.kind == "map-context":
            output = root / ".cache/watod-sim/map-context" / (arguments.map + ".json")
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(workspace.geometry.context(), indent=2, allow_nan=False)
                + "\n"
            )
            print(f"Map geometry context: {output}")
            return 0
        if not arguments.dry_run:
            workspace.check_targets()
        if arguments.candidate:
            if arguments.candidate.stat().st_size > 64000:
                raise ValueError("Candidate exceeds 64 KB")
            proposal = read_json(arguments.candidate.read_text(encoding="utf-8"))
            metadata = {"model": None, "source": "offline_candidate"}
        else:
            print(
                "Generating and validating a controller trajectory and actors…",
                file=sys.stderr,
                flush=True,
            )
            proposal, metadata = generate(
                workspace, prompt, arguments.model, max_rounds=arguments.max_rounds
            )
        scenario, maneuver, points, summary = workspace.validate(proposal)
        scenario_text = yaml.safe_dump(scenario, sort_keys=False)
        maneuver_text = json.dumps(maneuver, indent=2, allow_nan=False) + "\n"
        artifacts = root / ".cache/watod-sim/generated"
        artifacts.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix=name + "-", dir=artifacts))
        (output / "proposal.json").write_text(
            json.dumps(proposal, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        (output / "scenario.yaml").write_text(scenario_text, encoding="utf-8")
        (output / "map_context.json").write_text(
            json.dumps(workspace.geometry.context(), indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        (output / "maneuver.json").write_text(maneuver_text, encoding="utf-8")
        (output / "preview.svg").write_text(
            preview_svg(points, summary), encoding="utf-8"
        )
        report = {
            **metadata,
            "prompt": prompt,
            "name": name,
            "map": arguments.map,
            "geometry": summary,
            "actors": scenario["world"]["actors"],
            "validation": "schema, bounded primitives, anchored path including stop pad and actor centers contained in projected driving polygons with boundary clearance; no CARLA driving verification",
            "installed": False,
        }
        report_path = output / "report.json"
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        if not arguments.dry_run:
            publish(workspace, maneuver_text, scenario_text)
            report["installed"] = True
            report["installed_files"] = [str(path) for path in workspace.targets]
            report_path.write_text(
                json.dumps(report, indent=2) + "\n", encoding="utf-8"
            )
        print(
            f"Validated {name}: {summary['authored_length_m']:.1f} m authored path, "
            f"{len(scenario['world']['actors'])} vehicle actor(s)"
        )
        print(f"Preview and generation record: {output}")
        print(
            f"Map boundary check passed with {workspace.geometry.clearance:g} m center clearance, including the stop pad. Controller performance still requires a CARLA run."
        )
        if arguments.dry_run:
            print("Dry run: catalog and maneuver directories were not changed.")
        else:
            print(f"Open: watod sim open scenario {name}")
            print("Play: watod sim play")
        return 0
    except (ValueError, OSError, yaml.YAMLError) as error:
        print(f"Generation failed: {error}", file=sys.stderr)
        return 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_arguments(parser)
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    return run_generation(arguments, root)


if __name__ == "__main__":
    sys.exit(main())
