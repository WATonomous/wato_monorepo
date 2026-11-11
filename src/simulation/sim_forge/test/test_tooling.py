"""Lightweight lifecycle and launch integration checks, without Docker or ROS."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]


def runner():
    spec = importlib.util.spec_from_file_location("run_suite", REPO / "watod_scripts/simforge/run_suite.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_generation_routes_stack_and_copies_combined_launch(tmp_path):
    out = tmp_path / "launch"
    subprocess.run([sys.executable, str(REPO / "watod_scripts/simforge/generate_launch.py"),
                    "--repo", str(REPO), "--map-config", "/repo/maps/reviewed.yaml",
                    "--output", str(out)], check=True)
    world = yaml.safe_load((out / "world_modeling_simforge.launch.yaml").read_text())
    action = yaml.safe_load((out / "action_simforge.launch.yaml").read_text())
    args = {item["arg"]["name"]: item["arg"]["default"] for item in world["launch"] if "arg" in item}
    assert args["use_eidos"] == "false"
    assert args["map_config_file"] == "/repo/maps/reviewed.yaml"
    targets = []
    for doc in (world, action):
        for entry in doc["launch"]:
            if "node" in entry:
                node = entry["node"]
                clocks = [p for p in node["param"] if p.get("name") == "use_sim_time"]
                assert clocks == [{"name": "use_sim_time", "value": True}]
                targets.extend(r["to"] for r in node.get("remap", []))
    assert "/simforge/command" in targets
    assert "/simforge/tracked_detections_3d" in targets
    assert not any("/carla/" in t for t in targets)
    assert (out / "combined_simforge.launch.py").is_file()


def test_runner_cleans_up_when_server_fails(tmp_path):
    cleanup = tmp_path / "cleaned"
    case = {"socket": str(tmp_path / "missing.sock"), "wall_timeout_s": 2,
            "server_argv": [sys.executable, "-c", "raise SystemExit(3)"],
            "stack_argv": [sys.executable, "-c", "import time; time.sleep(30)"],
            "bridge_argv": ["unused"], "score_argv": ["unused"],
            "cleanup_argv": [sys.executable, "-c", f"from pathlib import Path; Path({str(cleanup)!r}).touch()"]}
    with pytest.raises(RuntimeError, match="socket"):
        runner().execute(case, tmp_path / "run")
    assert cleanup.exists()


def test_runner_detects_dead_stack_before_stage_completion():
    module = runner()
    service = subprocess.Popen([sys.executable, "-c", "raise SystemExit(1)"])
    stage = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    service.wait()
    try:
        with pytest.raises(RuntimeError, match="exited"):
            module.wait_stage(stage, [service], 2)
    finally:
        module.stop(stage)
    assert stage.poll() is not None


@pytest.fixture
def short_socket():
    # macOS AF_UNIX paths are limited to 104 bytes; pytest roots may be longer.
    with tempfile.TemporaryDirectory(prefix="sf-", dir="/tmp") as directory:
        yield str(Path(directory) / "env.sock")


@pytest.mark.parametrize("score_exit, expected", [(0, "passed"), (1, "failed"), (2, "error")])
def test_runner_success_and_scoring_exit_classification(tmp_path, short_socket, score_exit, expected):
    # Real Unix socket and subprocesses, but no simulator or heavy compute.
    socket_path = short_socket
    cleanup = tmp_path / "cleaned"
    server = ("import socket,time; s=socket.socket(socket.AF_UNIX); "
              f"s.bind({socket_path!r}); s.listen(); time.sleep(30)")
    case = {"socket": socket_path, "wall_timeout_s": 2,
            "server_argv": [sys.executable, "-c", server],
            "stack_argv": [sys.executable, "-c", "import time; time.sleep(30)"],
            "bridge_argv": [sys.executable, "-c", "pass"],
            "score_argv": [sys.executable, "-c", f"raise SystemExit({score_exit})"],
            "cleanup_argv": [sys.executable, "-c", f"from pathlib import Path; Path({str(cleanup)!r}).touch()"]}
    assert runner().execute(case, tmp_path / "run")["status"] == expected
    assert cleanup.exists()
