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
"""Find when each bag drives through a roundabout / parking lot / signalized intersection.

Matches GPS tracks (from extract_gps.py) against OpenStreetMap features fetched from Overpass.

Usage:
    find_scenes.py <gps_dir> <out_dir>

gps_dir holds <bag>_<n>.csv files (split bag files are merged per bag).
Writes out_dir/scenes.json and out_dir/scenes.txt; OSM features are cached in out_dir/osm.json.
"""

import csv
import glob
import itertools
import json
import math
import os
import re
import sys
import time
from collections import defaultdict

import requests

MIRRORS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
# Overpass selector per feature kind. Queried one kind at a time: the public servers often
# drop a single combined query under load.
SELECTORS = {
    "roundabout": 'way["junction"~"^(roundabout|circular)$"]',
    "parking": 'way["amenity"="parking"]',
    "signal": 'node["highway"="traffic_signals"]',
}
# A pass counts when the car's GPS comes within this many metres of the feature's centre.
RADIUS = {"roundabout": 35.0, "parking": 40.0, "signal": 25.0}


def dist_m(lat1, lon1, lat2, lon2):
    dx = math.radians(lon2 - lon1) * math.cos(math.radians(lat1)) * 6371000
    dy = math.radians(lat2 - lat1) * 6371000
    return math.hypot(dx, dy)


def load_tracks(gps_dir):
    tracks = defaultdict(list)
    for path in sorted(glob.glob(os.path.join(gps_dir, "*.csv"))):
        bag = re.sub(r"_\d+\.csv$", "", os.path.basename(path))
        with open(path) as f:
            tracks[bag] += [
                (float(r["t"]), float(r["lat"]), float(r["lon"]))
                for r in csv.DictReader(f)
                if abs(float(r["lat"])) > 1
            ]
    for bag in tracks:
        tr = sorted(tracks[bag])
        # Fixes are ~2 s apart (28 m at 50 km/h); interpolate to 0.25 s so small roundabouts aren't skipped.
        dense = []
        for a, b in itertools.pairwise(tr):
            n = max(1, int((b[0] - a[0]) / 0.25)) if b[0] - a[0] < 10 else 1
            dense += [
                tuple(a[i] + (b[i] - a[i]) * k / n for i in range(3)) for k in range(n)
            ]
        tracks[bag] = dense + tr[-1:]
    return tracks


def fetch_features(bbox, cache):
    if os.path.exists(cache):
        with open(cache) as f:
            return json.load(f)
    s, w, n, e = bbox
    features = []
    for kind, sel in SELECTORS.items():
        query = f'[out:csv(::type,::id,::lat,::lon,name;true;",")][timeout:90];{sel}({s},{w},{n},{e});out center;'
        for attempt in range(8):
            url = MIRRORS[attempt % len(MIRRORS)]
            try:
                r = requests.post(
                    url,
                    data={"data": query},
                    timeout=120,
                    headers={"User-Agent": "wato-bag-scene-finder/0.1"},
                )
                if r.status_code == 200 and r.text.startswith("@type"):
                    break
                err = f"HTTP {r.status_code}"
            except requests.RequestException as ex:
                err = str(ex)
            print(f"{kind} via {url}: {err}; retrying", file=sys.stderr)
            time.sleep(15)
        else:
            sys.exit(f"Overpass failed for {kind}")
        for row in csv.DictReader(r.text.splitlines()):
            if row["@lat"]:
                features.append(
                    {
                        "kind": kind,
                        "lat": float(row["@lat"]),
                        "lon": float(row["@lon"]),
                        "name": row["name"],
                        "osm": f"{row['@type']}/{row['@id']}",
                    }
                )
    with open(cache, "w") as f:
        json.dump(features, f)
    return features


def merge_passes(events):
    """OSM maps one physical roundabout as several ways; merge overlapping same-kind passes."""
    merged = []
    for ev in sorted(events, key=lambda ev: (ev["kind"], ev["start_s"])):
        if (
            merged
            and merged[-1]["kind"] == ev["kind"]
            and ev["start_s"] <= merged[-1]["end_s"] + 5
        ):
            merged[-1]["end_s"] = max(merged[-1]["end_s"], ev["end_s"])
            merged[-1]["name"] = merged[-1]["name"] or ev["name"]
        else:
            merged.append(dict(ev))
    return sorted(merged, key=lambda ev: ev["start_s"])


def main():
    gps_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    tracks = load_tracks(gps_dir)
    pts = [p for t in tracks.values() for p in t]
    pad = 0.005
    bbox = (
        round(min(p[1] for p in pts) - pad, 3),
        round(min(p[2] for p in pts) - pad, 3),
        round(max(p[1] for p in pts) + pad, 3),
        round(max(p[2] for p in pts) + pad, 3),
    )
    features = fetch_features(bbox, os.path.join(out_dir, "osm.json"))

    report, lines = {}, []
    for bag, tr in sorted(tracks.items()):
        t0 = tr[0][0]
        events = []
        for f in features:
            hits = [
                t
                for t, lat, lon in tr
                if dist_m(lat, lon, f["lat"], f["lon"]) < RADIUS[f["kind"]]
            ]
            if not hits:
                continue
            # Split into separate passes when there's a >20 s gap.
            start = prev = hits[0]
            for t in hits[1:] + [None]:
                if t is None or t - prev > 20:
                    events.append(
                        {
                            **f,
                            "start_s": round(start - t0, 1),
                            "end_s": round(prev - t0, 1),
                        }
                    )
                    if t is not None:
                        start = t
                if t is not None:
                    prev = t
        events = merge_passes(events)
        dur = tr[-1][0] - t0
        moved = sum(dist_m(a[1], a[2], b[1], b[2]) for a, b in itertools.pairwise(tr))
        counts = {k: sum(ev["kind"] == k for ev in events) for k in RADIUS}
        report[bag] = {
            "duration_s": round(dur),
            "distance_km": round(moved / 1000, 2),
            "counts": counts,
            "events": events,
        }
        lines.append(
            f"\n{bag}: {dur / 60:.1f} min, {moved / 1000:.1f} km, {counts['roundabout']} roundabout, "
            f"{counts['parking']} parking, {counts['signal']} signal passes"
        )
        for ev in events:
            lines.append(
                f"   {ev['kind']:<10} {ev['start_s']:>6.0f}-{ev['end_s']:<6.0f}s  {ev['name']}  "
                f"({ev['lat']:.5f},{ev['lon']:.5f})"
            )

    with open(os.path.join(out_dir, "scenes.json"), "w") as f:
        json.dump(report, f, indent=1)
    with open(os.path.join(out_dir, "scenes.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
