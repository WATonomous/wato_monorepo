"""Inspect the real SDK contract and test simulator repeatability, without ROS."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path


def jsonable(value):
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, bytes):
        return {"bytes": len(value), "sha256": hashlib.sha256(value).hexdigest()}
    return value


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + "\n")


def connect(path, *, timeout=20.0):
    from simforge_gym import SimForgeEnv, SocketTransport
    version = importlib.metadata.version("simforge-gym")
    if version != "0.2.0":
        raise RuntimeError(f"Review this adapter before using simforge-gym {version}")
    return SimForgeEnv(SocketTransport(path, timeout=timeout),
                       action_mode="control", observation="state", info_channel=True)


def args(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--socket", default="/run/simforge/env.sock")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--out", required=True)
    return parser


def probe():
    p = args("Capture the runtime's actual state contract. Authored control only.")
    a = p.parse_args()
    env = connect(a.socket)
    root = Path(a.out)
    try:
        dump(root / "hello.json", env.hello)
        obs, info = env.reset(seed=a.seed)
        dump(root / "reset.json", {"observation": obs, "info": info})
        for i in range(a.steps):
            obs, reward, terminated, truncated, info = env.step(None)
            dump(root / f"step_{i:04d}.json", {
                "observation": obs, "info": info, "reward": reward,
                "terminated": terminated, "truncated": truncated,
            })
            if terminated or truncated:
                break
    finally:
        env.close()
    print(f"Contract captured in {root}; this was not a WATO driving evaluation.")


def episode(env, seed, steps):
    obs, info = env.reset(seed=seed)
    rows = []
    def append(reward=None, terminated=False, truncated=False):
        # Deliberately exclude wall time and render timing from repeatability.
        rows.append(jsonable({"observation": obs, "info": {
            k: info[k] for k in ("t_s", "ego", "object_ids", "reward_terms", "events")
            if k in info}, "reward": reward,
            "terminated": terminated, "truncated": truncated}))
    append()
    for _ in range(steps):
        obs, reward, terminated, truncated, info = env.step(None)
        append(reward, terminated, truncated)
        if terminated or truncated:
            break
    return rows


def digest(rows):
    data = json.dumps(rows, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(data).hexdigest()


def check_repeatability(first, second):
    if len(first) < 2:
        raise ValueError("Episode returned no transitions")
    for rows in (first, second):
        times = [float(row["info"]["t_s"]) for row in rows]
        if not all(b > a for a, b in zip(times, times[1:])):
            raise ValueError("Simulation time did not advance monotonically")
    if digest(first) != digest(second):
        raise ValueError("Same seed and authored actions produced different traces")


def smoke():
    a = args("Check reset and same seed repeatability; not WATO driving quality").parse_args()
    if a.steps < 1:
        raise ValueError("steps must be positive")
    root = Path(a.out)
    env = connect(a.socket)
    report = {"kind": "runtime_smoke", "wato_driving_evaluated": False,
              "seed": a.seed, "gym_version": importlib.metadata.version("simforge-gym"),
              "passed": False}
    try:
        dump(root / "hello.json", env.hello)
        first = episode(env, a.seed, a.steps)
        second = episode(env, a.seed, a.steps)
        dump(root / "first.json", first)
        dump(root / "second.json", second)
        check_repeatability(first, second)
        report.update(passed=True, trace_sha256=digest(first), samples=len(first))
    except Exception as exc:
        report["error"] = str(exc)
        raise
    finally:
        env.close()
        dump(root / "summary.json", report)
    print(json.dumps(report))
