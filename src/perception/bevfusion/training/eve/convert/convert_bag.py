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
"""Convert one Eve rosbag (MCAP) into a nuScenes-format dataset root.

Usage: python convert_bag.py BAG_DIR_OR_MCAP --out ROOT [--tables-only]
"""

import argparse
import hashlib
import json
import struct
import subprocess
from bisect import bisect_left
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from mcap.exceptions import McapError
from mcap.reader import NonSeekingReader, make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image

CAMERAS = ["NN", "NE", "EE", "SE", "SS", "SW", "WW", "NW"]
IMAGE_TOPICS = {
    f"/camera_pano_{c.lower()}/image_rect_compressed": f"CAM_PANO_{c}" for c in CAMERAS
}
INFO_TOPICS = {
    f"/camera_pano_{c.lower()}/camera_info": f"CAM_PANO_{c}" for c in CAMERAS
}
LIDAR_TOPIC = "/lidar_cc/velodyne_points"
ODOM_TOPIC = "/novatel/oem7/odom"
LIDAR = "LIDAR_TOP"
TOPIC_OF = {LIDAR: LIDAR_TOPIC, **{ch: topic for topic, ch in IMAGE_TOPICS.items()}}

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

KEYFRAME_PERIOD_US = 500_000
KEYFRAMES_PER_SCENE = 40
MIN_SCENE_KEYFRAMES = 10
SYNC_LIMIT_US = 50_000
POSE_GAP_LIMIT_US = 100_000
RETIME_STRIDE = 50
RETIME_WINDOW = 250
ORIGIN_GRID_M = 10_000
ROS_TO_NUSCENES_AXES = np.array([0.5**0.5, 0, 0, -(0.5**0.5)])
MCAP_MAGIC = b"\x89MCAP0\r\n"
POINT_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def token(*parts):
    return hashlib.md5("|".join(str(p) for p in parts).encode()).hexdigest()


def stamp_us(stamp):
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def qrotate(q, v):
    return v + 2 * np.cross(q[1:], np.cross(q[1:], v) + q[0] * v)


def lookup(tf, parent, child):
    q, t = np.array([1.0, 0, 0, 0]), np.zeros(3)
    frame = child
    while frame != parent:
        if frame not in tf:
            raise SystemExit(f"/tf_static has no path from {parent} to {child}")
        frame, step_q, step_t = tf[frame]
        q, t = qmul(step_q, q), step_t + qrotate(step_q, t)
    return q, t


def jpeg_size(data):
    i = 2
    while i < len(data):
        marker, length = data[i + 1], struct.unpack(">H", data[i + 2 : i + 4])[0]
        if 0xC0 <= marker <= 0xC3:
            height, width = struct.unpack(">HH", data[i + 5 : i + 9])
            return width, height
        i += 2 + length
    return None


def resolve(path):
    path = Path(path)
    if path.is_dir():
        return path.name, sorted(
            path.glob("*.mcap"), key=lambda p: (len(p.name), p.name)
        )
    return path.stem, [path]


def messages(files, topics):
    for path in files:
        with open(path, "rb") as f:
            f.seek(-len(MCAP_MAGIC), 2)
            closed = f.read() == MCAP_MAGIC
            f.seek(0)
            reader = (make_reader if closed else NonSeekingReader)(
                f, decoder_factories=[DecoderFactory()]
            )
            try:
                for _, channel, _, msg in reader.iter_decoded_messages(
                    topics=topics, log_time_order=False
                ):
                    yield channel.topic, msg
            except McapError:
                if closed:
                    raise
                print(
                    f"{path.name}: recording was not closed, read up to the truncated end"
                )


def read_index(files):
    stamps = defaultdict(list)
    odom, tf, cameras, frames = [], {}, {}, {}
    for topic, msg in messages(
        files, [LIDAR_TOPIC, ODOM_TOPIC, "/tf_static", *IMAGE_TOPICS, *INFO_TOPICS]
    ):
        if topic == "/tf_static":
            for tr in msg.transforms:
                r, t = tr.transform.rotation, tr.transform.translation
                tf[tr.child_frame_id] = (
                    tr.header.frame_id,
                    np.array([r.w, r.x, r.y, r.z]),
                    np.array([t.x, t.y, t.z]),
                )
        elif topic == ODOM_TOPIC:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            frames["ego"] = msg.child_frame_id
            odom.append((stamp_us(msg.header.stamp), p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        elif topic in INFO_TOPICS:
            cameras.setdefault(
                INFO_TOPICS[topic],
                {
                    "frame": msg.header.frame_id,
                    "P": np.array(msg.p).reshape(3, 4),
                    "size": (msg.width, msg.height),
                },
            )
        elif topic == LIDAR_TOPIC:
            frames[LIDAR] = msg.header.frame_id
            stamps[LIDAR].append(stamp_us(msg.header.stamp))
        else:
            stamps[IMAGE_TOPICS[topic]].append(stamp_us(msg.header.stamp))
    if not stamps[LIDAR]:
        raise SystemExit(f"no messages on {LIDAR_TOPIC}")
    if len(odom) < 2 * RETIME_STRIDE:
        raise SystemExit(f"fewer than {2 * RETIME_STRIDE} messages on {ODOM_TOPIC}")
    seen, first = np.unique(
        np.array([row[0] for row in odom], dtype=np.int64), return_index=True
    )
    pose = np.array([row[1:] for row in odom])[first]
    fresh = np.r_[True, np.any(np.diff(pose[:, :3], axis=0) != 0, axis=1)]
    return {
        "stamps": {
            ch: np.unique(np.array(stamps[ch], dtype=np.int64)) for ch in TOPIC_OF
        },
        "odom_seen": seen,
        "odom_t": retime(seen[fresh]),
        "odom": pose[fresh],
        "tf": tf,
        "cameras": cameras,
        "frames": frames,
    }


def retime(arrival):
    # Odometry is stamped on arrival, up to tens of ms late. The source is periodic, so a sample
    # cannot be newer than any later arrival minus the periods in between.
    n = np.arange(len(arrival))
    period = np.median(np.diff(arrival[::RETIME_STRIDE])) / RETIME_STRIDE
    phase = np.r_[arrival - n * period, np.full(RETIME_WINDOW - 1, np.inf)]
    stamps = np.rint(
        sliding_window_view(phase, RETIME_WINDOW).min(axis=1) + n * period
    ).astype(np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise SystemExit(f"{ODOM_TOPIC}: stamps are not increasing after retiming")
    late = (arrival - stamps) / 1000
    print(
        f"odometry: {len(arrival)} position updates, period {period / 1000:.3f} ms, "
        f"arrival delay median {np.median(late):.1f} ms, max {late.max():.1f} ms"
    )
    return stamps


def calibration(index, lidar_axes):
    q, t = lookup(index["tf"], index["frames"]["ego"], index["frames"][LIDAR])
    if lidar_axes == "nuscenes":
        q = qmul(q, ROS_TO_NUSCENES_AXES)
    rows = {
        LIDAR: {
            "translation": t.tolist(),
            "rotation": q.tolist(),
            "camera_intrinsic": [],
        }
    }
    for ch in IMAGE_TOPICS.values():
        if ch not in index["cameras"]:
            raise SystemExit(f"{ch}: no camera_info in the bag")
        q, t = lookup(
            index["tf"], index["frames"]["ego"], index["cameras"][ch]["frame"]
        )
        rows[ch] = {
            "translation": t.tolist(),
            "rotation": q.tolist(),
            "camera_intrinsic": index["cameras"][ch]["P"][:, :3].tolist(),
        }
    return hashlib.md5(json.dumps(rows, sort_keys=True).encode()).hexdigest()[:8], rows


def pose_at(index, origin, t):
    seen, odom_t, odom = index["odom_seen"], index["odom_t"], index["odom"]
    j, i = int(np.searchsorted(seen, t)), int(np.searchsorted(odom_t, t))
    if (
        j == 0
        or j == len(seen)
        or seen[j] - seen[j - 1] > POSE_GAP_LIMIT_US
        or i == 0
        or i == len(odom_t)
    ):
        return None
    f = (t - odom_t[i - 1]) / (odom_t[i] - odom_t[i - 1])
    a, b = odom[i - 1], odom[i]
    qa, qb = a[3:], b[3:] if np.dot(a[3:], b[3:]) >= 0 else -b[3:]
    q = qa + f * (qb - qa)
    return (a[:3] + f * (b[:3] - a[:3]) - origin).tolist(), (
        q / np.linalg.norm(q)
    ).tolist()


def nearest(stamps, t):
    i = int(np.searchsorted(stamps, t))
    near = stamps[max(i - 1, 0) : i + 1]
    return int(near[np.argmin(np.abs(near - t))])


def match_cameras(index, origin, t):
    if pose_at(index, origin, t) is None:
        return None, f"no_pose {LIDAR}"
    images = {}
    for ch in IMAGE_TOPICS.values():
        if not len(index["stamps"][ch]):
            return None, f"camera_missing {ch}"
        images[ch] = nearest(index["stamps"][ch], t)
        if abs(images[ch] - t) > SYNC_LIMIT_US:
            return None, f"camera_offset {ch} {(images[ch] - t) / 1000:+.0f}ms"
        if pose_at(index, origin, images[ch]) is None:
            return None, f"no_pose {ch}"
    return images, None


def build_tables(bag, index, origin, lidar_axes, location):
    dropped = Counter()

    def drop(kind, stamp, reason):
        print(f"DROP {kind} {stamp} {reason}")
        dropped[f"{kind} {reason.split()[0]}"] += 1

    lidar = index["stamps"][LIDAR]
    slot = np.rint((lidar - lidar[0]) / KEYFRAME_PERIOD_US).astype(int)
    slot_error = np.abs(lidar - lidar[0] - slot * KEYFRAME_PERIOD_US)
    scenes = defaultdict(list)
    for j in range(slot[-1] + 1):
        members = np.flatnonzero(slot == j)
        if not len(members):
            drop("keyframe", f"slot{j}", "no_lidar")
            continue
        t = int(lidar[members[np.argmin(slot_error[members])]])
        images, reason = match_cameras(index, origin, t)
        if reason:
            drop("keyframe", t, reason)
        else:
            scenes[j // KEYFRAMES_PER_SCENE].append((t, images))

    calib_id, calib = calibration(index, lidar_axes)
    log_token = token("log", bag)
    date = datetime.fromtimestamp(int(lidar[0]) / 1e6, timezone.utc).strftime(
        "%Y-%m-%d"
    )
    tables = {name: [] for name in OWN_TABLES}
    tables["log"].append(
        {
            "token": log_token,
            "logfile": bag,
            "vehicle": "eve",
            "date_captured": date,
            "location": location,
        }
    )
    for ch, row in calib.items():
        tables["sensor"].append(
            {
                "token": token("sensor", ch),
                "channel": ch,
                "modality": "lidar" if ch == LIDAR else "camera",
            }
        )
        tables["calibrated_sensor"].append(
            {
                "token": token("calibrated_sensor", calib_id, ch),
                "sensor_token": token("sensor", ch),
                **row,
            }
        )

    wanted = {}
    for k, frames in sorted(scenes.items()):
        if len(frames) < MIN_SCENE_KEYFRAMES:
            for t, _ in frames:
                drop("keyframe", t, "short_scene")
            continue
        scene_token = token("scene", bag, k)
        key_times = [t for t, _ in frames]
        sample_tokens = [token(bag, "sample", t) for t in key_times]
        tables["scene"].append(
            {
                "token": scene_token,
                "log_token": log_token,
                "nbr_samples": len(frames),
                "first_sample_token": sample_tokens[0],
                "last_sample_token": sample_tokens[-1],
                "name": f"{bag}-{k:03d}",
                "description": "",
            }
        )
        chains = defaultdict(list)
        for n, (t, images) in enumerate(frames):
            tables["sample"].append(
                {
                    "token": sample_tokens[n],
                    "timestamp": t,
                    "scene_token": scene_token,
                    "prev": sample_tokens[n - 1] if n else "",
                    "next": sample_tokens[n + 1] if n + 1 < len(frames) else "",
                }
            )
            for ch, image_t in images.items():
                chains[ch].append((image_t, sample_tokens[n], True))
        for t in lidar[(lidar >= key_times[0]) & (lidar <= key_times[-1])].tolist():
            n = bisect_left(key_times, t)
            is_key = key_times[n] == t
            if not is_key and pose_at(index, origin, t) is None:
                drop("sweep", t, "no_pose")
                continue
            chains[LIDAR].append((t, sample_tokens[n], is_key))
        for ch, chain in chains.items():
            tokens = [token(bag, TOPIC_OF[ch], t) for t, _, _ in chain]
            width, height = (0, 0) if ch == LIDAR else index["cameras"][ch]["size"]
            for n, (t, sample_token, is_key) in enumerate(chain):
                extension = "pcd.bin" if ch == LIDAR else "jpg"
                filename = f"{'samples' if is_key else 'sweeps'}/{ch}/{bag}__{ch}__{t}.{extension}"
                wanted[(ch, t)] = filename
                translation, rotation = pose_at(index, origin, t)
                tables["ego_pose"].append(
                    {
                        "token": tokens[n],
                        "timestamp": t,
                        "rotation": rotation,
                        "translation": translation,
                    }
                )
                tables["sample_data"].append(
                    {
                        "token": tokens[n],
                        "sample_token": sample_token,
                        "ego_pose_token": tokens[n],
                        "calibrated_sensor_token": token(
                            "calibrated_sensor", calib_id, ch
                        ),
                        "timestamp": t,
                        "fileformat": "pcd" if ch == LIDAR else "jpg",
                        "is_key_frame": is_key,
                        "height": height,
                        "width": width,
                        "filename": filename,
                        "prev": tokens[n - 1] if n else "",
                        "next": tokens[n + 1] if n + 1 < len(chain) else "",
                    }
                )
    stats = {
        "calibration_id": calib_id,
        "scenes": len(tables["scene"]),
        "keyframes": len(tables["sample"]),
        "sweeps": sum(not row["is_key_frame"] for row in tables["sample_data"]),
        "dropped": dict(dropped),
    }
    return tables, wanted, stats


def points(msg, lidar_axes):
    dtype = np.dtype(
        {
            "names": [f.name for f in msg.fields],
            "formats": [POINT_DTYPES[f.datatype] for f in msg.fields],
            "offsets": [f.offset for f in msg.fields],
            "itemsize": msg.point_step,
        }
    )
    raw = np.frombuffer(bytes(msg.data), dtype=dtype)
    out = np.stack(
        [raw[name] for name in ("x", "y", "z", "intensity", "ring")], axis=1
    ).astype(np.float32)
    out = out[np.isfinite(out).all(axis=1)]
    if lidar_axes == "nuscenes":
        out[:, :2] = np.stack([-out[:, 1], out[:, 0]], axis=1)
    return out


def write_files(files, wanted, root, index, lidar_axes):
    for folder in {Path(name).parent for name in wanted.values()}:
        (root / folder).mkdir(parents=True, exist_ok=True)
    missing = set(wanted)
    for topic, msg in messages(files, [LIDAR_TOPIC, *IMAGE_TOPICS]):
        ch = LIDAR if topic == LIDAR_TOPIC else IMAGE_TOPICS[topic]
        key = (ch, stamp_us(msg.header.stamp))
        if key not in missing:
            continue
        if ch == LIDAR:
            data = points(msg, lidar_axes).tobytes()
        else:
            data = bytes(msg.data)
            if jpeg_size(data) != index["cameras"][ch]["size"]:
                raise SystemExit(
                    f"{wanted[key]}: image is {jpeg_size(data)}, camera_info says "
                    f"{index['cameras'][ch]['size']}"
                )
        (root / wanted[key]).write_bytes(data)
        missing.discard(key)
    if missing:
        raise SystemExit(
            f"{len(missing)} planned files were not found on the second pass"
        )


def load(path, default):
    return json.loads(path.read_text()) if path.exists() else default


def merge_tables(root, new, wanted):
    folder = root / VERSION
    folder.mkdir(parents=True, exist_ok=True)
    old = {name: load(folder / f"{name}.json", []) for name in OWN_TABLES}
    logs = {row["token"] for row in new["log"]}
    scenes = {row["token"] for row in old["scene"] if row["log_token"] in logs}
    samples = {row["token"] for row in old["sample"] if row["scene_token"] in scenes}
    data = {
        row["token"] for row in old["sample_data"] if row["sample_token"] in samples
    }
    stale = {
        "log": logs,
        "scene": scenes,
        "sample": samples,
        "sample_data": data,
        "ego_pose": data,
    }
    keep = set(wanted.values())
    for row in old["sample_data"]:
        if row["token"] in data and row["filename"] not in keep:
            (root / row["filename"]).unlink(missing_ok=True)
    for name in OWN_TABLES:
        rows = {
            row["token"]: row
            for row in old[name]
            if row["token"] not in stale.get(name, ())
        }
        rows.update((row["token"], row) for row in new[name])
        old[name] = list(rows.values())
    old["map"] = [
        {
            "token": token("map", "empty"),
            "log_tokens": [row["token"] for row in old["log"]],
            "category": "semantic_prior",
            "filename": "maps/empty.png",
        }
    ]
    for name, rows in old.items():
        (folder / f"{name}.json").write_text(json.dumps(rows, indent=1))
    for name in LABEL_TABLES:
        if not (folder / f"{name}.json").exists():
            (folder / f"{name}.json").write_text("[]")
    (root / "maps").mkdir(exist_ok=True)
    if not (root / "maps/empty.png").exists():
        Image.new("L", (1, 1)).save(root / "maps/empty.png")


def git_sha():
    result = subprocess.run(
        ["git", "-C", str(Path(__file__).parent), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() or None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--out", required=True)
    parser.add_argument("--tables-only", action="store_true")
    parser.add_argument("--lidar-axes", choices=["nuscenes", "ros"], default="nuscenes")
    parser.add_argument("--location", default="waterloo")
    parser.add_argument("--origin", type=float, nargs=3)
    args = parser.parse_args()

    bag, files = resolve(args.bag)
    root = Path(args.out)
    index = read_index(files)
    manifest = load(
        root / "manifest.json",
        {"lidar_axes": args.lidar_axes, "origins": {}, "bags": {}},
    )
    if manifest["lidar_axes"] != args.lidar_axes:
        raise SystemExit(
            f"{root} was written with --lidar-axes {manifest['lidar_axes']}"
        )
    grid = (np.floor(index["odom"][0, :2] / ORIGIN_GRID_M) * ORIGIN_GRID_M).tolist() + [
        0.0
    ]
    origin = manifest["origins"].setdefault(args.location, args.origin or grid)
    if args.origin and args.origin != origin:
        raise SystemExit(f"{root} already uses origin {origin} for {args.location}")

    tables, wanted, stats = build_tables(
        bag, index, np.array(origin), args.lidar_axes, args.location
    )
    if not args.tables_only:
        write_files(files, wanted, root, index, args.lidar_axes)
    merge_tables(root, tables, wanted)
    manifest["bags"][bag] = {
        "files": [path.name for path in files],
        "converter_git_sha": git_sha(),
        **stats,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(json.dumps({bag: stats}, indent=1))


if __name__ == "__main__":
    main()
