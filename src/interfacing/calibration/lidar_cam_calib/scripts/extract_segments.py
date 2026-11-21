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
"""Find calibration segments in every bag under a folder and cut them out.

  extract_segments.py BAG_DIR OUT_DIR [--detect-only] [--force]

For each bag that has the LiDAR and at least one configured camera:
  1. estimate the car's motion (LiDAR GICP, plus IMU / CAN / odometry if recorded)
  2. find rest+creep segments (for `preprocess -d`) and parked segments (static)
  3. cut each into OUT_DIR/{creep,static}/<segment>/ with a cleaned LiDAR topic
     (NaN points dropped, per-point time rebuilt if missing)
  4. write OUT_DIR/previews/<segment>.jpg (first frame of each camera) and
     OUT_DIR/motion/<bag>.csv (the motion timeline)

OUT_DIR/manifest.yaml lists every segment. Edit `enabled` or `skip_cameras` there
after looking at the previews; re-running keeps those edits and skips bags
already processed.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_cam_calib import bag_io, cutting, motion, segments  # noqa: E402
from lidar_cam_calib.config import image_topic, info_topic, load_config  # noqa: E402

USER_FIELDS = ("enabled", "skip_cameras", "note")


def say(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_manifest(path: Path) -> dict:
    if path.exists():
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
    data.setdefault("bags", {})
    data.setdefault("segments", [])
    return data


class _PlainDumper(yaml.SafeDumper):
    """No &id001 / *id001 aliases: the manifest is meant to be edited by hand."""

    def ignore_aliases(self, data):
        return True


def save_manifest(path: Path, data: dict) -> None:
    data["segments"].sort(key=lambda s: s["name"])
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        yaml.dump(data, f, Dumper=_PlainDumper, sort_keys=False, width=200)
    tmp.replace(path)


def write_motion_csv(path: Path, tl: motion.Timeline) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cols = {
        "t": tl.t,
        "dt": tl.dt,
        "speed": tl.speed,
        "rot_rate": tl.rot_rate,
        "rest": tl.rest,
    }
    cols.update({"reliable": tl.reliable, "distance": tl.distance, **tl.extra})
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for row in zip(*cols.values()):
            w.writerow([f"{v:.4f}" if isinstance(v, float) else int(v) for v in row])


def qualify(bag: bag_io.BagInfo, cfg: dict) -> list[str]:
    """Configured cameras with images in the bag (empty if the LiDAR is missing)."""
    if cfg["lidar"]["topic"] not in bag.topics:
        return []
    return [c for c in cfg["cameras"] if bag.counts.get(image_topic(cfg, c), 0) > 0]


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--config", type=Path)
    ap.add_argument(
        "--detect-only", action="store_true", help="find segments, don't cut them"
    )
    ap.add_argument(
        "--force", action="store_true", help="redo bags already in the manifest"
    )
    ap.add_argument(
        "--bag", action="append", default=[], help="only bags whose name contains this"
    )
    args = ap.parse_args()

    cfg = load_config(args.config)
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / bag_io.OUTPUT_MARKER).touch()
    manifest_path = out / "manifest.yaml"
    manifest = load_manifest(manifest_path)
    user = {
        s["name"]: {k: s[k] for k in USER_FIELDS if k in s}
        for s in manifest["segments"]
    }

    # 1. Find bags, drop duplicates (same recording copied twice) and unusable ones
    paths = bag_io.find_bags(args.bag_dir, exclude=[out])
    say(f"{len(paths)} bags under {args.bag_dir}")
    seen, todo = {}, []
    for p in paths:
        if args.bag and not any(b in p.name for b in args.bag):
            continue
        try:
            info = bag_io.read_info(p)
        except Exception as e:  # noqa: BLE001
            say(f"  skip {p}: unreadable ({e})")
            continue
        key = (info.start_ns, info.message_count)
        if key in seen:
            say(f"  skip {p}: duplicate of {seen[key]}")
            continue
        seen[key] = p
        cams = qualify(info, cfg)
        if not cams:
            say(
                f"  skip {p.name}: no {cfg['lidar']['topic']} or no configured camera images"
            )
            continue
        say(
            f"  use  {p.name}: {info.duration_s:.0f} s, cameras {', '.join(c.replace('camera_', '') for c in cams)}"
        )
        todo.append((info, cams))

    # 2-4. Detect and cut
    lidar = cfg["lidar"]["topic"]
    for info, cams in todo:
        done = manifest["bags"].get(info.name, {})
        if done.get("processed") and not args.force and not args.detect_only:
            say(f"{info.name}: already processed, skipping (--force to redo)")
            continue
        say(f"{info.name}: estimating motion")
        tl = motion.build_timeline(motion.read_signals(info, cfg), cfg)
        write_motion_csv(out / "motion" / f"{info.name}.csv", tl)
        found = segments.detect(tl, cfg["segments"])
        say(f"{info.name}: motion from {'+'.join(tl.sources)}, {len(found)} segments")
        manifest["segments"] = [
            s for s in manifest["segments"] if s.get("bag") != info.name
        ]
        for seg in found:
            name = cutting.segment_name(info.name, seg.kind, seg.start_s)
            entry = {
                "name": name,
                "bag": info.name,
                "source": str(info.path),
                **seg.to_dict(),
            }
            entry["cameras"] = cams
            entry["motion_sources"] = tl.sources
            say(
                f"  {seg.kind:6s} {seg.start_s:7.1f}-{seg.end_s:7.1f} s  {seg.distance_m:4.1f} m"
                f"  max {seg.max_speed_mps:.2f} m/s  ({seg.end_reason})"
            )
            if not args.detect_only:
                stats = cutting.cut_segment(
                    info,
                    seg.start_s,
                    seg.end_s,
                    out / seg.kind / name,
                    lidar,
                    [image_topic(cfg, c) for c in cams],
                    [info_topic(cfg, c) for c in cams] + cfg["extra_topics"],
                    cfg["lidar"]["scan_period"],
                    preview_path=out / "previews" / f"{name}.jpg",
                )
                entry["path"] = f"{seg.kind}/{name}"
                entry["lidar_time"] = stats["lidar_time"]
                entry["lidar_points_kept"] = stats["lidar_points_kept"]
                entry["lidar_messages"] = stats["messages"].get(lidar, 0)
            entry.update({"enabled": True, "skip_cameras": []})
            entry.update(user.get(name, {}))
            manifest["segments"].append(entry)
        if not args.detect_only:
            manifest["bags"][info.name] = {
                "source": str(info.path),
                "processed": True,
                "cameras": cams,
            }
        save_manifest(manifest_path, manifest)

    kinds = [s["kind"] for s in manifest["segments"] if s.get("enabled", True)]
    say(
        f"manifest: {kinds.count('creep')} creep, {kinds.count('static')} static segments -> {manifest_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
