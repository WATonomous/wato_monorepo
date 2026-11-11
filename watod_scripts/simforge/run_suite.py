"""Sequential cases with fresh server and stack process groups, including cleanup on failure."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop(process):
    if process is None:
        return
    # Signal the group even when its original parent exited.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def argv_for(spec, name, out):
    argv = spec[name + "_argv"]
    if not isinstance(argv, list) or not argv or not all(isinstance(x, str) and x for x in argv):
        raise ValueError(f"{name}_argv must be a nonempty argument array")
    return [str(out) if item == "{out}" else item for item in argv]


def wait_stage(proc, services, timeout):
    deadline = time.monotonic() + timeout
    while True:
        if any(service.poll() is not None for service in services):
            raise RuntimeError("Server or stack exited during evaluation")
        code = proc.poll()
        if code is not None:
            return code
        if time.monotonic() >= deadline:
            raise TimeoutError("Stage exceeded wall_timeout_s")
        time.sleep(0.05)


def execute(spec, out):
    out.mkdir(parents=True, exist_ok=False)
    processes, logs = [], []
    timeout = float(spec.get("wall_timeout_s", 300))
    if not 0 < timeout < float("inf"):
        raise ValueError("wall_timeout_s must be finite and positive")
    for name in ("server", "stack", "bridge", "score"):
        argv_for(spec, name, out)
    try:
        for name in ("server", "stack"):
            argv = argv_for(spec, name, out)
            log = (out / f"{name}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            if name == "server":
                deadline = time.monotonic()+min(60, timeout)
                while not Path(spec["socket"]).is_socket():
                    if processes[-1].poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError("Server did not create its socket; inspect server.log")
                    time.sleep(0.1)
        # Placeholders only replace entire arguments. No shell evaluation.
        for name in ("bridge", "score"):
            argv = argv_for(spec, name, out)
            log = (out / f"{name}.log").open("w")
            logs.append(log)
            proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(proc)
            code = wait_stage(proc, processes[:2], timeout)
            if code:
                return {"status": "failed" if name == "score" and code == 1 else "error", "stage": name, "exit_code": code}
        if any(process.poll() is not None for process in processes[:2]):
            return {"status": "error", "reason": "server or stack exited unexpectedly"}
        return {"status": "passed"}
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        # Required for detached Compose/container workloads. Must be idempotent.
        if "cleanup_argv" in spec:
            with (out / "cleanup.log").open("w") as log:
                cleanup = subprocess.Popen(argv_for(spec, "cleanup", out), stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    if cleanup.wait(timeout=timeout):
                        raise RuntimeError("Explicit stack cleanup failed; inspect cleanup.log")
                finally:
                    stop(cleanup)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("suite")
    p.add_argument("--out", required=True)
    a = p.parse_args()
    suite = json.loads(Path(a.suite).read_text())
    if suite.get("reviewed") is not True or not suite.get("cases"):
        raise ValueError("Provide a reviewed nonempty suite after individual prototype gates pass")
    root = Path(a.out).resolve()
    root.mkdir(parents=True, exist_ok=False)
    results = []
    for i, case in enumerate(suite["cases"]):
        socket = Path(case["socket"])
        try:
            if os.path.lexists(socket):
                raise FileExistsError(f"Refusing a stale or active socket: {socket}")
            result = execute(case, root / f"case_{i:03d}")
        except Exception as exc:
            result = {"status": "error", "reason": str(exc)}
        results.append({"name": case["name"], **result})
        (root / "suite.json").write_text(json.dumps(results, indent=2)+"\n")
    (root / "suite.json").write_text(json.dumps(results, indent=2)+"\n")
    raise SystemExit(0 if all(row["status"] == "passed" for row in results) else 1)


if __name__ == "__main__":
    main()
