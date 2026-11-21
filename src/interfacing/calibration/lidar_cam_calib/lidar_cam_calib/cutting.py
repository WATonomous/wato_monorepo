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
"""Writing one segment of a bag as a small calibration bag, plus a preview sheet."""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, PointCloud2

from . import bag_io
from .clouds import clean_cloud


def segment_name(bag_name: str, kind: str, start_s: float) -> str:
    """Folder name for a segment; no dots, since the toolbox names its files after it."""
    return f"{bag_name}__{kind}_{start_s:07.1f}s".replace(".", "p")


def read_tf_static(path: Path, max_messages: int = 20) -> list[tuple[bytes, int]]:
    """First /tf_static messages of a bag (static transforms are published at startup)."""
    reader = bag_io.open_reader(path, ["/tf_static"])
    out = []
    while reader.has_next() and len(out) < max_messages:
        _, data, stamp = reader.read_next()
        out.append((data, stamp))
    return out


def cut_segment(
    bag: bag_io.BagInfo,
    start_s: float,
    end_s: float,
    out_dir: Path,
    lidar_topic: str,
    camera_topics: list[str],
    extra_topics: list[str],
    scan_period: float,
    preview_path: Path | None = None,
) -> dict:
    """Copy [start_s, end_s] of the bag into out_dir with a cleaned LiDAR topic.

    Camera and extra topics are copied byte for byte. /tf_static from the start
    of the source bag is written at the segment start, since it is latched.
    Returns stats for the manifest.
    """
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    start_ns = bag.start_ns + int(start_s * 1e9)
    end_ns = bag.start_ns + int(end_s * 1e9)
    wanted = [lidar_topic] + [
        t for t in camera_topics + extra_topics if t in bag.topics
    ]

    reader = bag_io.open_reader(
        bag.path, [t for t in wanted if t != "/tf_static"], start_ns
    )
    meta = bag_io.topic_metadata(reader)
    writer = bag_io.open_writer(out_dir)
    for topic in wanted:
        if topic in meta:
            writer.create_topic(meta[topic])

    counts = {t: 0 for t in wanted}
    if "/tf_static" in wanted:
        for data, _ in read_tf_static(bag.path):
            writer.write("/tf_static", data, start_ns)
            counts["/tf_static"] += 1

    time_methods: dict[str, int] = {}
    first_images: dict[str, np.ndarray] = {}
    points_in = points_out = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if stamp > end_ns:
            break
        if stamp < start_ns:
            continue
        if topic == lidar_topic:
            cloud, stats = clean_cloud(
                deserialize_message(data, PointCloud2), scan_period
            )
            data = serialize_message(cloud)
            time_methods[stats["time"]] = time_methods.get(stats["time"], 0) + 1
            points_in += stats["points_in"]
            points_out += stats["points_out"]
        elif (
            preview_path is not None
            and topic in camera_topics
            and topic not in first_images
        ):
            first_images[topic] = _decode(data)
        writer.write(topic, data, stamp)
        counts[topic] += 1
    del writer  # closes the bag and writes metadata.yaml

    if preview_path is not None and first_images:
        write_preview(first_images, preview_path)
    return {
        "messages": counts,
        "lidar_time": max(time_methods, key=time_methods.get)
        if time_methods
        else "none",
        "lidar_points_kept": round(points_out / points_in, 3) if points_in else 0.0,
    }


def _decode(data: bytes):
    import cv2

    msg = deserialize_message(data, CompressedImage)
    return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)


def write_preview(images: dict[str, np.ndarray], path: Path, cols: int = 3) -> None:
    """Labeled grid of the first frame of each camera (the frame the toolbox uses)."""
    import cv2

    tiles = []
    for topic in sorted(images):
        img = images[topic]
        if img is None:
            continue
        small = cv2.resize(img, (480, 384))
        cv2.putText(
            small,
            topic.split("/")[1],
            (8, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        tiles.append(small)
    if not tiles:
        return
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i : i + cols]) for i in range(0, len(tiles), cols)]
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
