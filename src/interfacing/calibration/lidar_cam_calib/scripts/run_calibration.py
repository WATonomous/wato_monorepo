#!/usr/bin/env python3
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
"""Calibrate cameras against the LiDAR on extracted segments, several times each.

  run_calibration.py OUT_DIR [--mode creep|static] [--camera NAME ...] [--run NAME]

Needs direct_visual_lidar_calibration (run inside the lidar_cam_calib image).
Per camera, in OUT_DIR/calib/<run>/<camera>/:
  bags/      links to the selected segments (manifest: enabled, kind == mode,
             camera recorded and not in skip_cameras)
  base/      `preprocess` output, initial guess from the current URDF
  trials/    one `calibrate` run per trial:
               all             every segment
               loo_<segment>   leave that segment out (3+ segments)
               perturb_<k>     every segment, randomly perturbed initial guess
  trials.yaml  initial guess and every trial's result
Finished trials are skipped, so re-running resumes. --force redoes everything.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_cam_calib import extrinsics, toolbox  # noqa: E402
from lidar_cam_calib.config import image_topic, load_config, resolve  # noqa: E402


def say(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def select_segments(manifest: dict, camera: str, mode: str) -> list[dict]:
    return [
        s
        for s in manifest["segments"]
        if s.get("enabled", True)
        and s["kind"] == mode
        and "path" in s
        and camera in s.get("cameras", [])
        and camera not in (s.get("skip_cameras") or [])
    ]


def link_bags(out: Path, cam_dir: Path, segs: list[dict]) -> Path:
    bags = cam_dir / "bags"
    if bags.exists():
        for link in bags.iterdir():
            link.unlink()
    bags.mkdir(parents=True, exist_ok=True)
    for s in segs:
        target = out / s["path"]
        (bags / s["name"]).symlink_to(os.path.relpath(target, bags))
    return bags


def initial_guess(
    cfg: dict, camera: str, source: str, segs: list[dict], out: Path
) -> np.ndarray:
    lidar = cfg["lidar"]["frame"]
    if source == "urdf":
        graph = extrinsics.FrameGraph.from_urdf(resolve(cfg["urdf"]))
    else:  # the /tf_static recorded in the first segment
        from rclpy.serialization import deserialize_message
        from tf2_msgs.msg import TFMessage

        from lidar_cam_calib.cutting import read_tf_static

        msgs = [
            deserialize_message(d, TFMessage)
            for d, _ in read_tf_static(out / segs[0]["path"])
        ]
        graph = extrinsics.FrameGraph.from_tf_static(msgs)
    return graph.lookup(lidar, camera)


def calibrate_camera(args, cfg: dict, manifest: dict, camera: str) -> dict | None:
    c = cfg["calibration"]
    out = args.out_dir
    segs = select_segments(manifest, camera, args.mode)
    if not segs:
        say(f"{camera}: no enabled {args.mode} segments, skipping")
        return None
    cam_dir = out / "calib" / args.run / camera
    say(f"{camera}: {len(segs)} {args.mode} segments")
    selection = sorted(s["name"] for s in segs)

    # Preprocess once per selection of segments
    base = cam_dir / "base"
    sel_file = cam_dir / "selection.yaml"
    old = yaml.safe_load(sel_file.read_text()) if sel_file.exists() else None
    if args.force or old != selection or not (base / toolbox.CALIB).exists():
        bags = link_bags(out, cam_dir, segs)
        fx_fy_cx_cy = toolbox.rectified_intrinsics(
            resolve(cfg["intrinsics_dir"]) / f"{camera}.yaml"
        )
        pre = [
            str(bags),
            str(base),
            "--image_topic",
            image_topic(cfg, camera),
            "--points_topic",
            cfg["lidar"]["topic"],
            "--camera_model",
            "plumb_bob",
            "--camera_intrinsics",
            ",".join(f"{v:.6f}" for v in fx_fy_cx_cy),
            "--camera_distortion_coeffs",
            "0,0,0,0,0",
        ]
        pre += (["-d"] if args.mode == "creep" else []) + list(c["preprocess_args"])
        if not args.dry_run:
            # New preprocess output makes earlier trials stale
            shutil.rmtree(cam_dir / "trials", ignore_errors=True)
            shutil.rmtree(base, ignore_errors=True)
        say(f"{camera}: preprocess")
        rc = toolbox.run_tool(
            "preprocess", pre, cam_dir / "preprocess.log", c["headless"], args.dry_run
        )
        # preprocess sometimes exits 1 after writing everything (seen on WATcloud),
        # so judge it by its output: calib.json listing every selected segment
        done = (
            not args.dry_run
            and (base / toolbox.CALIB).exists()
            and len(toolbox.read_calib(base)["meta"]["bag_names"]) == len(segs)
        )
        if rc != 0 and not done and not args.dry_run:
            say(
                f"{camera}: preprocess FAILED (exit {rc}), see {cam_dir / 'preprocess.log'}"
            )
            return None
        if rc != 0 and done:
            say(f"{camera}: preprocess exited {rc} but wrote all outputs; continuing")
        if not args.dry_run:
            sel_file.write_text(yaml.safe_dump(selection))
    if args.dry_run:
        say(f"{camera}: dry run, stopping before calibrate")
        return None

    T_init = initial_guess(cfg, camera, args.init, segs, out)
    calib = toolbox.read_calib(base)
    calib.setdefault("results", {})["init_T_lidar_camera"] = extrinsics.vec7_from_T(
        T_init
    )
    toolbox.write_calib(base, calib)
    bag_names = calib["meta"]["bag_names"]

    trials = [("all", bag_names, T_init)]
    if c["leave_one_out"] and len(bag_names) >= 3:
        trials += [
            (f"loo_{b}", [x for x in bag_names if x != b], T_init) for b in bag_names
        ]
    rng = np.random.default_rng(c["seed"])
    for k in range(c["perturbed"]):
        T = extrinsics.perturb(
            T_init, c["perturb_translation"], c["perturb_rotation_deg"], rng
        )
        trials.append((f"perturb_{k}", bag_names, T))

    records = []
    for name, bags_used, T0 in trials:
        trial = cam_dir / "trials" / name
        result = None if args.force else toolbox.trial_result(trial)
        status, secs = "reused", 0.0
        if result is None:
            toolbox.make_trial(base, trial, bags_used, extrinsics.vec7_from_T(T0))
            say(f"{camera}: trial {name} ({len(bags_used)} segments)")
            t0 = time.time()
            rc = toolbox.run_tool(
                "calibrate",
                [str(trial), *c["calibrate_args"]],
                trial / "calibrate.log",
                c["headless"],
                timeout=c["timeout_s"],
            )
            secs = time.time() - t0
            result = toolbox.trial_result(trial)
            status = "ok" if rc == 0 and result else f"failed (exit {rc})"
        if result:
            dt, dr = extrinsics.difference(T_init, extrinsics.T_from_vec7(result))
            say(
                f"{camera}: {name}: moved {dt * 100:.1f} cm / {dr:.2f} deg from the URDF guess"
            )
        records.append(
            {
                "name": name,
                "segments": list(bags_used),
                "init_T_lidar_camera": extrinsics.vec7_from_T(T0),
                "T_lidar_camera": result,
                "status": status,
                "seconds": round(secs, 1),
            }
        )

    summary = {
        "camera": camera,
        "run": args.run,
        "mode": args.mode,
        "init_source": args.init,
        "init_T_lidar_camera": extrinsics.vec7_from_T(T_init),
        "trials": records,
    }
    (cam_dir / "trials.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False, width=200)
    )
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("out_dir", type=Path, help="folder written by extract_segments.py")
    ap.add_argument("--config", type=Path)
    ap.add_argument("--mode", choices=["creep", "static"], default="creep")
    ap.add_argument(
        "--camera", action="append", help="camera(s) to do (default: all in the config)"
    )
    ap.add_argument("--run", default="run1", help="name for this batch of results")
    ap.add_argument(
        "--init",
        choices=["urdf", "tf_static"],
        default="urdf",
        help="initial guess source",
    )
    ap.add_argument("--force", action="store_true")
    ap.add_argument(
        "--dry-run", action="store_true", help="print the toolbox commands only"
    )
    args = ap.parse_args()

    cfg = load_config(args.config)
    with open(args.out_dir / "manifest.yaml") as f:
        manifest = yaml.safe_load(f)
    cameras = args.camera or cfg["cameras"]
    failed = 0
    for cam in cameras:
        res = calibrate_camera(args, cfg, manifest, cam)
        if res and not any(t["T_lidar_camera"] for t in res["trials"]):
            failed += 1
    say(f"done; summarize with: summarize_results.py {args.out_dir} --run {args.run}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
