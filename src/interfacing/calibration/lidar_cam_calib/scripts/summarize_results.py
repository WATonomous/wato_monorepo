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
"""Summarize calibration trials and write URDF joints.

  summarize_results.py OUT_DIR [--run NAME]

Reads OUT_DIR/calib/<run>/*/trials.yaml and writes, in OUT_DIR/calib/<run>/:
  results.md            per-camera table: spread across trials, change from the
                        URDF guess, position on the roof ring, verdict
  results.yaml          the same, machine readable
  urdf_snippet.xacro    one fixed joint per camera, parent = the LiDAR frame

The estimate is the `all` trial. Spread is the largest difference between it
and any leave-one-out or perturbed trial (see the thresholds in calib.yaml).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_cam_calib import extrinsics  # noqa: E402
from lidar_cam_calib.config import load_config, resolve  # noqa: E402


def spread(T_ref: np.ndarray, results: list[list[float]]) -> tuple[float, float]:
    if not results:
        return float("nan"), float("nan")
    d = [extrinsics.difference(T_ref, extrinsics.T_from_vec7(r)) for r in results]
    return max(x[0] for x in d), max(x[1] for x in d)


def _nanmax(a: float, b: float) -> float:
    """Larger of two spreads, ignoring a missing one (NaN when there were no such trials)."""
    vals = [v for v in (a, b) if np.isfinite(v)]
    return max(vals) if vals else float("nan")


def verdict(t: float, r: float, s: dict) -> str:
    if not np.isfinite(t):
        return "no repeat trials"
    if t <= s["good_translation_m"] and r <= s["good_rotation_deg"]:
        return "good"
    if t > s["redo_translation_m"] or r > s["redo_rotation_deg"]:
        return "redo"
    return "check"


def heading_deg(T_base_cam: np.ndarray) -> float:
    z = T_base_cam[:3, 2]  # optical axis
    return float(np.degrees(np.arctan2(z[1], z[0])))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--run", default="run1")
    ap.add_argument("--config", type=Path)
    ap.add_argument(
        "--base-frame", default="base_link", help="frame for the roof-ring check"
    )
    args = ap.parse_args()

    cfg = load_config(args.config)
    lidar = cfg["lidar"]["frame"]
    run_dir = args.out_dir / "calib" / args.run
    try:
        graph = extrinsics.FrameGraph.from_urdf(resolve(cfg["urdf"]))
        T_base_lidar = graph.lookup(args.base_frame, lidar)
    except Exception as e:  # noqa: BLE001
        print(f"warning: no URDF ({e}); skipping the roof-ring check", file=sys.stderr)
        graph, T_base_lidar = None, None

    rows, joints = [], []
    for f in sorted(run_dir.glob("*/trials.yaml")):
        d = yaml.safe_load(f.read_text())
        cam = d["camera"]
        by_name = {t["name"]: t for t in d["trials"]}
        all_t = by_name.get("all", {}).get("T_lidar_camera")
        T_init = extrinsics.T_from_vec7(d["init_T_lidar_camera"])
        row = {
            "camera": cam,
            "mode": d["mode"],
            "segments": len(by_name.get("all", {}).get("segments", [])),
        }
        if not all_t:
            row["verdict"] = "failed"
            rows.append(row)
            continue
        T = extrinsics.T_from_vec7(all_t)
        loo = [
            t["T_lidar_camera"]
            for t in d["trials"]
            if t["name"].startswith("loo_") and t["T_lidar_camera"]
        ]
        per = [
            t["T_lidar_camera"]
            for t in d["trials"]
            if t["name"].startswith("perturb_") and t["T_lidar_camera"]
        ]
        lt, lr = spread(T, loo)
        pt, pr = spread(T, per)
        mt, mr = extrinsics.difference(T_init, T)
        xyz, rpy = extrinsics.xyz_rpy_from_T(T)
        row.update(
            {
                "T_lidar_camera": [round(x, 6) for x in all_t],
                "xyz": [round(float(x), 5) for x in xyz],
                "rpy": [round(float(x), 6) for x in rpy],
                "loo_spread_cm": round(lt * 100, 2),
                "loo_spread_deg": round(lr, 3),
                "perturb_spread_cm": round(pt * 100, 2),
                "perturb_spread_deg": round(pr, 3),
                "change_from_urdf_cm": round(mt * 100, 2),
                "change_from_urdf_deg": round(mr, 3),
                "verdict": verdict(_nanmax(lt, pt), _nanmax(lr, pr), cfg["summary"]),
            }
        )
        if T_base_lidar is not None:
            Tb, Tb0 = T_base_lidar @ T, T_base_lidar @ T_init
            row["ring_radius_m"] = round(float(np.hypot(*Tb[:2, 3])), 4)
            row["height_m"] = round(float(Tb[2, 3]), 4)
            row["heading_deg"] = round(heading_deg(Tb), 2)
            row["heading_change_deg"] = round(
                ((heading_deg(Tb) - heading_deg(Tb0) + 180) % 360) - 180, 2
            )
        rows.append(row)
        joints.append(
            f'  <joint name="{cam}_mount" type="fixed">\n'
            f'    <parent link="{lidar}"/>\n'
            f'    <child link="{cam}"/>\n'
            f'    <origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>\n'
            f"  </joint>"
        )

    if not rows:
        print(f"no trials.yaml under {run_dir}", file=sys.stderr)
        return 1

    s = cfg["summary"]
    lines = [
        f"# LiDAR-camera calibration: {args.run}",
        "",
        f"Estimate = the `all` trial. Spread = largest difference to a leave-one-out (LOO) or perturbed trial. "
        f"Good: <= {s['good_translation_m'] * 100:g} cm and <= {s['good_rotation_deg']:g} deg; "
        f"redo: > {s['redo_translation_m'] * 100:g} cm or > {s['redo_rotation_deg']:g} deg.",
        "",
        "| Camera | Segments | LOO spread | Perturbed spread | Change from URDF | Ring radius | Height | Heading | Verdict |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        if "xyz" not in r:
            lines.append(
                f"| {r['camera']} | {r['segments']} | | | | | | | {r['verdict']} |"
            )
            continue
        lines.append(
            f"| {r['camera']} | {r['segments']} | {r['loo_spread_cm']} cm / {r['loo_spread_deg']} deg"
            f" | {r['perturb_spread_cm']} cm / {r['perturb_spread_deg']} deg"
            f" | {r['change_from_urdf_cm']} cm / {r['change_from_urdf_deg']} deg"
            f" | {r.get('ring_radius_m', '')} m | {r.get('height_m', '')} m"
            f" | {r.get('heading_deg', '')} ({r.get('heading_change_deg', '')}) deg | {r['verdict']} |"
        )
    lines += [
        "",
        "Ring radius, height and heading are in base_link; the URDF's nominal pano ring is "
        "0.2407 m radius, 0.217 m above base_link, 45 deg apart. Heading change is relative to the URDF.",
        "",
    ]
    (run_dir / "results.md").write_text("\n".join(lines))
    (run_dir / "results.yaml").write_text(
        yaml.safe_dump({"run": args.run, "cameras": rows}, sort_keys=False)
    )
    (run_dir / "urdf_snippet.xacro").write_text(
        "<!-- Calibrated camera extrinsics (camera optical frame in the LiDAR frame) -->\n"
        + "\n".join(joints)
        + "\n"
    )
    print("\n".join(lines))
    print(f"wrote {run_dir / 'results.md'}, results.yaml, urdf_snippet.xacro")
    return 0


if __name__ == "__main__":
    sys.exit(main())
