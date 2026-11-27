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
"""Label-free track stability metrics from a recorded /perception/detections_3D_tracked topic.

Usage:
    track_stability.py <bag_dir_or_mcap> [--topic T] [--start S] [--end E] [--json out.json]

Metrics (per class and per distance band):
  - track lifetime (median / p90 seconds) and share of short-lived (< --short-s) tracks
  - fragments/min: a new track born within --refrag-m of where a same-class track died
    less than --refrag-s ago (proxy for ID switches / dropped-then-reacquired objects)
  - class flips per track
  - position jitter: median |p[t+1] - 2 p[t] + p[t-1]| (cm); constant ego velocity cancels
  - size jitter: median frame-to-frame |d size| / size (%)
  - heading: median |d yaw| (deg) and count of ~180 deg box flips
"""

import argparse
import glob
import itertools
import json
import math
import os
import statistics
from collections import defaultdict

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

NUSCENES_CLASSES = {
    "0": "car",
    "1": "truck",
    "2": "construction_vehicle",
    "3": "bus",
    "4": "trailer",
    "5": "barrier",
    "6": "motorcycle",
    "7": "bicycle",
    "8": "pedestrian",
    "9": "traffic_cone",
}
DIST_BANDS = [(0, 20), (20, 40), (40, 1e9)]


def band_name(lo, hi):
    return f"{lo:.0f}-{hi:.0f}m" if hi < 1e9 else f"{lo:.0f}m+"


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pct(xs, p):
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))]


def med(xs):
    return statistics.median(xs) if xs else None


def read_frames(path, topic):
    """[(stamp_s, [det dict])] sorted by time, from a bag dir or a single .mcap."""
    files = (
        sorted(glob.glob(os.path.join(path, "*.mcap")))
        if os.path.isdir(path)
        else [path]
    )
    frames = []
    for f in files:
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, _, msg, m in reader.iter_decoded_messages(topics=[topic]):
                t = (
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                    or msg.log_time * 1e-9
                )
                dets = []
                for d in m.detections:
                    cls = next(
                        (
                            r.hypothesis.class_id
                            for r in d.results
                            if ":" not in r.hypothesis.class_id
                            and r.hypothesis.class_id != "linear_velocity"
                        ),
                        "?",
                    )
                    p, s = d.bbox.center.position, d.bbox.size
                    dets.append(
                        {
                            "id": d.id,
                            "cls": NUSCENES_CLASSES.get(cls, cls),
                            "score": max(
                                (r.hypothesis.score for r in d.results), default=0.0
                            ),
                            "x": p.x,
                            "y": p.y,
                            "z": p.z,
                            "yaw": yaw_of(d.bbox.center.orientation),
                            "size": (s.x, s.y, s.z),
                        }
                    )
                frames.append((t, dets))
    frames.sort(key=lambda fr: fr[0])
    return frames


def analyze(frames, args):
    tracks = defaultdict(list)  # id -> [(frame_idx, t, det)]
    for i, (t, dets) in enumerate(frames):
        for d in dets:
            tracks[d["id"]].append((i, t, d))

    duration = frames[-1][0] - frames[0][0] if len(frames) > 1 else 0.0
    rate = (len(frames) - 1) / duration if duration > 0 else float("nan")

    # Per-track summaries, bucketed by the track's majority class and median range.
    groups = defaultdict(lambda: defaultdict(list))
    births, deaths = [], []
    for tid, obs in tracks.items():
        classes = [o[2]["cls"] for o in obs]
        cls = max(set(classes), key=classes.count)
        rng = med([math.hypot(o[2]["x"], o[2]["y"]) for o in obs])
        band = next(band_name(lo, hi) for lo, hi in DIST_BANDS if lo <= rng < hi)
        life = obs[-1][1] - obs[0][1]
        flips = sum(a != b for a, b in itertools.pairwise(classes))
        births.append((obs[0][1], cls, obs[0][2]["x"], obs[0][2]["y"], tid))
        deaths.append((obs[-1][1], cls, obs[-1][2]["x"], obs[-1][2]["y"], tid))

        pos_j, size_j, yaw_d, yaw_flips = [], [], [], 0
        for k in range(1, len(obs)):
            (i0, _, a), (i1, _, b) = obs[k - 1], obs[k]
            if i1 - i0 != 1:  # only compare consecutive frames
                continue
            size_j += [
                abs(sb - sa) / max(sa, 1e-3) for sa, sb in zip(a["size"], b["size"])
            ]
            dy = abs(wrap(b["yaw"] - a["yaw"]))
            if dy > math.radians(150):
                yaw_flips += 1
            else:
                yaw_d.append(math.degrees(dy))
            if k >= 2 and obs[k - 2][0] == i0 - 1:
                z = obs[k - 2][2]
                pos_j.append(
                    math.hypot(
                        b["x"] - 2 * a["x"] + z["x"], b["y"] - 2 * a["y"] + z["y"]
                    )
                )

        for key in ("all", cls, f"{cls}@{band}", f"all@{band}"):
            g = groups[key]
            g["life"].append(life)
            g["flips"].append(flips)
            g["pos_j"] += pos_j
            g["size_j"] += size_j
            g["yaw_d"] += yaw_d
            g["yaw_flips"].append(yaw_flips)
            g["obs"].append(len(obs))

    # Fragmentation: a birth near a recent same-class death of a *different* track.
    deaths.sort()
    refrag = defaultdict(int)
    for tb, cls, x, y, tid in births:
        for td, dcls, dx, dy, did in deaths:
            if td > tb:
                break
            if (
                did != tid
                and dcls == cls
                and tb - td < args.refrag_s
                and math.hypot(x - dx, y - dy) < args.refrag_m
            ):
                refrag[cls] += 1
                refrag["all"] += 1
                break

    minutes = duration / 60 if duration > 0 else float("nan")
    rows = {}
    for key, g in groups.items():
        cls = key.split("@")[0]
        n = len(g["life"])
        rows[key] = {
            "tracks": n,
            "tracks_per_min": n / minutes,
            "life_med_s": med(g["life"]),
            "life_p90_s": pct(g["life"], 90),
            "short_frac": sum(lf < args.short_s for lf in g["life"]) / n,
            "fragments_per_min": refrag.get(cls, 0) / minutes
            if "@" not in key
            else None,
            "class_flips_per_track": sum(g["flips"]) / n,
            "pos_jitter_cm_med": None if not g["pos_j"] else 100 * med(g["pos_j"]),
            "pos_jitter_cm_p90": None if not g["pos_j"] else 100 * pct(g["pos_j"], 90),
            "size_jitter_pct_med": None if not g["size_j"] else 100 * med(g["size_j"]),
            "yaw_delta_deg_med": med(g["yaw_d"]),
            "yaw_flips_per_100_frames": 100
            * sum(g["yaw_flips"])
            / max(1, sum(g["obs"])),
        }
    return {
        "frames": len(frames),
        "duration_s": duration,
        "rate_hz": rate,
        "dets_per_frame_mean": statistics.mean(len(d) for _, d in frames)
        if frames
        else 0,
        "groups": rows,
    }


def fmt(v, nd=2):
    return "-" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def print_table(res, keys, title):
    cols = [
        ("tracks", 0),
        ("life_med_s", 2),
        ("life_p90_s", 1),
        ("short_frac", 2),
        ("fragments_per_min", 1),
        ("class_flips_per_track", 2),
        ("pos_jitter_cm_med", 1),
        ("pos_jitter_cm_p90", 1),
        ("size_jitter_pct_med", 1),
        ("yaw_delta_deg_med", 2),
        ("yaw_flips_per_100_frames", 2),
    ]
    print(f"\n### {title}\n")
    print("| group | " + " | ".join(c for c, _ in cols) + " |")
    print("|---" * (len(cols) + 1) + "|")
    for k in keys:
        r = res["groups"][k]
        print(f"| {k} | " + " | ".join(fmt(r[c], nd) for c, nd in cols) + " |")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/perception/detections_3D_tracked")
    ap.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    ap.add_argument("--end", type=float, default=None, help="seconds from bag start")
    ap.add_argument("--short-s", type=float, default=0.5)
    ap.add_argument("--refrag-s", type=float, default=1.0)
    ap.add_argument("--refrag-m", type=float, default=2.0)
    ap.add_argument(
        "--min-tracks", type=int, default=5, help="hide groups with fewer tracks"
    )
    ap.add_argument("--json")
    args = ap.parse_args()

    frames = read_frames(args.bag, args.topic)
    if not frames:
        raise SystemExit(f"no messages on {args.topic}")
    t0 = frames[0][0]
    frames = [
        f
        for f in frames
        if f[0] - t0 >= args.start and (args.end is None or f[0] - t0 <= args.end)
    ]
    res = analyze(frames, args)
    res.update(
        bag=os.path.basename(os.path.normpath(args.bag)),
        topic=args.topic,
        start_s=args.start,
        end_s=args.end,
    )

    print(
        f"{res['bag']}: {res['frames']} frames over {res['duration_s']:.0f}s ({res['rate_hz']:.1f} Hz), "
        f"{res['dets_per_frame_mean']:.1f} tracks/frame"
    )
    g = res["groups"]

    def big(k):
        return g[k]["tracks"] >= args.min_tracks

    by_class = sorted(
        (k for k in g if "@" not in k and k != "all" and big(k)),
        key=lambda k: -g[k]["tracks"],
    )
    print_table(res, ["all"] + by_class, "By class")
    print_table(
        res,
        [k for k in (f"all@{band_name(*b)}" for b in DIST_BANDS) if k in g and big(k)],
        "By distance",
    )
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(res, fh, indent=1)


if __name__ == "__main__":
    main()
