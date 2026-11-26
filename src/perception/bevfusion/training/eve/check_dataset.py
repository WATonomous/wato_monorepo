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
"""Check a dataset root against the Eve data contract.

Usage: python check_dataset.py ROOT [--rerun ROOT_FROM_A_SECOND_CONVERSION]
"""

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

VERSION = "v1.0-trainval"
OWN_TABLES = [
    "sensor",
    "calibrated_sensor",
    "ego_pose",
    "log",
    "scene",
    "sample",
    "sample_data",
    "map",
]
LABEL_TABLES = ["category", "attribute", "visibility", "instance", "sample_annotation"]
LIDAR = "LIDAR_TOP"
CHANNELS = [LIDAR] + [
    f"CAM_PANO_{c}" for c in ("NN", "NE", "EE", "SE", "SS", "SW", "WW", "NW")
]
SYNC_LIMIT_US = 50_000
LOCAL_FRAME_LIMIT_M = 50_000
LIDAR_ROW_BYTES = 20


def check_load(root, tables):
    errors = []
    for name in OWN_TABLES + LABEL_TABLES:
        try:
            tables[name] = json.loads((root / VERSION / f"{name}.json").read_text())
        except (OSError, ValueError) as e:
            errors.append(f"{name}.json: {e}")
    if not errors:
        try:
            from nuscenes.nuscenes import NuScenes

            NuScenes(version=VERSION, dataroot=str(root), verbose=False)
        except Exception as e:
            errors.append(f"devkit: {e!r}")
    return errors


def channel_lookup(tables):
    sensor = {row["token"]: row["channel"] for row in tables["sensor"]}
    calibrated = {
        row["token"]: sensor.get(row["sensor_token"])
        for row in tables["calibrated_sensor"]
    }
    return {
        row["token"]: calibrated.get(row["calibrated_sensor_token"])
        for row in tables["sample_data"]
    }


def check_channels(root, tables):
    errors = []
    if sorted(row["channel"] for row in tables["sensor"]) != sorted(CHANNELS):
        errors.append(
            "sensor.json does not list exactly LIDAR_TOP and the 8 CAM_PANO channels"
        )
    channel = channel_lookup(tables)
    seen = defaultdict(Counter)
    for row in tables["sample_data"]:
        if row["is_key_frame"]:
            seen[row["sample_token"]][channel[row["token"]]] += 1
    for sample in tables["sample"]:
        if seen[sample["token"]] != Counter(CHANNELS):
            errors.append(
                f"sample {sample['token']}: keyframe channels are {dict(seen[sample['token']])}"
            )
    return errors


def check_links(root, tables):
    errors = []
    by_token = {}
    for name in OWN_TABLES:
        by_token[name] = {row["token"]: row for row in tables[name]}
        if len(by_token[name]) != len(tables[name]):
            errors.append(f"{name}: duplicate tokens")
    references = [
        ("scene", "log_token", "log"),
        ("sample", "scene_token", "scene"),
        ("sample_data", "sample_token", "sample"),
        ("sample_data", "ego_pose_token", "ego_pose"),
        ("sample_data", "calibrated_sensor_token", "calibrated_sensor"),
        ("calibrated_sensor", "sensor_token", "sensor"),
    ]
    for name, field, target in references:
        for row in tables[name]:
            if row[field] not in by_token[target]:
                errors.append(f"{name} {row['token']}: {field} not in {target}")
    mapped = {t for row in tables["map"] for t in row["log_tokens"]}
    errors += [
        f"log {t}: not listed in any map row"
        for t in by_token["log"]
        if t not in mapped
    ]
    if errors:
        return errors

    samples_in_scene = Counter(row["scene_token"] for row in tables["sample"])
    for scene in tables["scene"]:
        walked, current, previous = 0, scene["first_sample_token"], ""
        while current:
            sample = by_token["sample"].get(current)
            if (
                sample is None
                or sample["prev"] != previous
                or sample["scene_token"] != scene["token"]
                or (
                    previous
                    and sample["timestamp"] <= by_token["sample"][previous]["timestamp"]
                )
            ):
                errors.append(
                    f"scene {scene['name']}: sample chain breaks at {current}"
                )
                break
            walked, previous, current = walked + 1, current, sample["next"]
        else:
            if (
                previous != scene["last_sample_token"]
                or not walked
                == scene["nbr_samples"]
                == samples_in_scene[scene["token"]]
            ):
                errors.append(
                    f"scene {scene['name']}: walked {walked} samples, nbr_samples={scene['nbr_samples']}, "
                    f"{samples_in_scene[scene['token']]} rows point to it"
                )

    channel = channel_lookup(tables)
    data = by_token["sample_data"]
    scene_of = {
        token: by_token["sample"][row["sample_token"]]["scene_token"]
        for token, row in data.items()
    }
    for token, row in data.items():
        if row["prev"] and data.get(row["prev"], {}).get("next") != token:
            errors.append(f"sample_data {token}: prev does not link back")
        if row["next"]:
            following = data.get(row["next"])
            if following is None or following["prev"] != token:
                errors.append(f"sample_data {token}: next does not link back")
            elif (
                channel[following["token"]] != channel[token]
                or scene_of[following["token"]] != scene_of[token]
                or following["timestamp"] <= row["timestamp"]
            ):
                errors.append(
                    f"sample_data {token}: next is on another channel or scene, or not later in time"
                )
    return errors


def check_files(root, tables):
    errors = []
    channel = channel_lookup(tables)
    for row in tables["sample_data"]:
        path = root / row["filename"]
        size = path.stat().st_size if path.is_file() else 0
        if size == 0:
            errors.append(f"{row['filename']}: missing or empty")
        elif channel[row["token"]] == LIDAR and size % LIDAR_ROW_BYTES:
            errors.append(
                f"{row['filename']}: size is not a multiple of {LIDAR_ROW_BYTES} bytes"
            )
    for row in tables["map"]:
        if not (root / row["filename"]).is_file():
            errors.append(f"{row['filename']}: missing")
    return errors


def check_sync(root, tables):
    errors = []
    channel = channel_lookup(tables)
    stamps = defaultdict(dict)
    for row in tables["sample_data"]:
        if row["is_key_frame"]:
            stamps[row["sample_token"]][channel[row["token"]]] = row["timestamp"]
    for sample in tables["sample"]:
        lidar = stamps[sample["token"]].get(LIDAR)
        if lidar != sample["timestamp"]:
            errors.append(
                f"sample {sample['token']}: timestamp differs from its LIDAR_TOP keyframe"
            )
            continue
        for ch, t in stamps[sample["token"]].items():
            if abs(t - lidar) > SYNC_LIMIT_US:
                errors.append(
                    f"sample {sample['token']}: {ch} is {(t - lidar) / 1000:+.0f} ms from LIDAR_TOP"
                )
    return errors


def check_units(root, tables):
    errors = []
    for name in ("sample", "sample_data", "ego_pose"):
        if any(type(row["timestamp"]) is not int for row in tables[name]):
            errors.append(f"{name}: timestamps must be integer microseconds")
    for name in ("ego_pose", "calibrated_sensor"):
        for row in tables[name]:
            if (
                len(row["translation"]) != 3
                or len(row["rotation"]) != 4
                or abs(sum(v * v for v in row["rotation"]) - 1) > 1e-6
            ):
                errors.append(f"{name} {row['token']}: bad translation or quaternion")
    if any(
        abs(v) > LOCAL_FRAME_LIMIT_M
        for row in tables["ego_pose"]
        for v in row["translation"]
    ):
        errors.append(
            f"ego_pose: translations beyond {LOCAL_FRAME_LIMIT_M} m, not a local frame"
        )
    lidar_sensors = {
        row["token"] for row in tables["sensor"] if row["channel"] == LIDAR
    }
    for row in tables["calibrated_sensor"]:
        k = row["camera_intrinsic"]
        if (
            (k != [])
            if row["sensor_token"] in lidar_sensors
            else (len(k) != 3 or any(len(r) != 3 for r in k))
        ):
            errors.append(f"calibrated_sensor {row['token']}: bad camera_intrinsic")
    return errors


def check_rerun(root, tables, other):
    errors = []
    for name in OWN_TABLES:
        second = json.loads((other / VERSION / f"{name}.json").read_text())
        if sorted(row["token"] for row in second) != sorted(
            row["token"] for row in tables[name]
        ):
            errors.append(f"{name}: tokens differ between the two runs")
        elif sorted(second, key=lambda row: row["token"]) != sorted(
            tables[name], key=lambda row: row["token"]
        ):
            errors.append(f"{name}: same tokens, different contents")
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--rerun", type=Path)
    args = parser.parse_args()

    tables, failed = {}, False
    checks = [
        ("all 13 tables load", check_load),
        ("every keyframe has 9 channels", check_channels),
        ("links are unbroken", check_links),
        ("files exist", check_files),
        ("stamps are within the sync limit", check_sync),
        ("units", check_units),
    ]
    for title, check in checks:
        errors = check(args.root, tables)
        print(f"{'FAIL' if errors else 'PASS'}  {title}")
        for line in errors[:10]:
            print(f"      {line}")
        if len(errors) > 10:
            print(f"      ... and {len(errors) - 10} more")
        failed = failed or bool(errors)
        if errors and check is check_load:
            sys.exit(1)
    if args.rerun:
        errors = check_rerun(args.root, tables, args.rerun)
        print(f"{'FAIL' if errors else 'PASS'}  tokens are unchanged on a second run")
        for line in errors:
            print(f"      {line}")
        failed = failed or bool(errors)
    else:
        print("SKIP  tokens are unchanged on a second run (pass --rerun)")
    print(
        f"{len(tables['scene'])} scenes, {len(tables['sample'])} samples, {len(tables['sample_data'])} sample_data"
    )
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
