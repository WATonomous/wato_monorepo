"""Capture provenance without copying credentials or account caches.

Run in the adapter container. Runtime command failures are recorded as evidence,
not silently represented as successful checks. Supply actual installed CLI
inspection commands through --command-json if that release differs.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys


def capture(argv, cwd=None):
    try:
        result = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=30)
        return {"argv": argv, "exit_code": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"argv": argv, "error": str(exc)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", required=True)
    p.add_argument("--repo", default="/repo")
    p.add_argument("--contract")
    p.add_argument("--workspace")
    p.add_argument("--image-id", help="Image ID from docker image inspect on the host")
    p.add_argument("--command-json", action="append", default=[],
                   help='Additional inspection argv as JSON, e.g. ["simforge", "doctor"]')
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    commands = [["simforge", "--version"], ["simforge", "runtime", "status", "--available"],
                ["simforge", "--help"], ["simforge", "doctor"]]
    for command in a.command_json:
        argv = json.loads(command)
        if not isinstance(argv, list) or not argv or not all(isinstance(v, str) for v in argv):
            p.error("--command-json must contain a nonempty string argument array")
        commands.append(argv)
    git = {name: capture(["git", *argv], a.repo) for name, argv in {
        "commit": ["rev-parse", "HEAD"], "status": ["status", "--porcelain"],
        "diff": ["diff", "HEAD", "--", "src/simulation/sim_forge", "watod_scripts/simforge",
                 "docker/simforge.Dockerfile", "modules/docker-compose.simforge.yaml"]}.items()}
    assets = {}
    if a.workspace:
        workspace = Path(a.workspace)
        if not workspace.is_dir():
            p.error("--workspace must be an existing scenario directory")
        for path in sorted(workspace.rglob("*")):
            if path.is_file():
                assets[str(path.relative_to(workspace))] = hashlib.sha256(path.read_bytes()).hexdigest()
    if a.contract:
        (out / "contract.json").write_bytes(Path(a.contract).read_bytes())
    report = {"platform": platform.platform(), "python": sys.version,
              "image_id": a.image_id, "git": git, "workspace_sha256": assets,
              "dependencies": {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()},
              "commands": [capture(command) for command in commands],
              "limitations": ["Map and runtime asset digests must be collected from installed CLI",
                              "Untracked source files are listed but not included in git diff"]}
    (out / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
